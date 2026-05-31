using Maphy.Mathematics;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public class World
    {
        private const ulong HashOffsetBasis = 14695981039346656037UL;
        private const ulong HashPrime = 1099511628211UL;

        private ulong id = 10000000;
        private ulong constraintId = 20000000;
        private ulong fixedStepCount;
        private fix fixedTimeAccumulator;
        public WorldSettings settings;
        private readonly Dictionary<ulong, Entity> entities;
        private readonly Dictionary<ulong, Rigid> rigids = new Dictionary<ulong, Rigid>();
        private readonly Dictionary<ulong, Collider> colliders = new Dictionary<ulong, Collider>();
        private readonly Dictionary<ulong, Constraint> constraints = new Dictionary<ulong, Constraint>();
        private readonly List<ulong> rigidIds = new List<ulong>();
        private readonly List<ulong> colliderIds = new List<ulong>();
        private readonly List<ulong> constraintIds = new List<ulong>();
        private readonly List<Collider> activeColliders = new List<Collider>();
        private readonly List<PhysicsIsland> islands = new List<PhysicsIsland>();
        private readonly List<PhysicsIsland> islandPool = new List<PhysicsIsland>();
        private readonly List<IslandEdge> islandEdges = new List<IslandEdge>();
        private readonly HashSet<ulong> islandVisited = new HashSet<ulong>();
        private readonly List<ulong> islandStack = new List<ulong>();
        private readonly List<ulong> colliderRemovalBuffer = new List<ulong>();
        private readonly List<ulong> constraintRemovalBuffer = new List<ulong>();
        private readonly List<BroadCollisionPairKey> collisionEventRemovalBuffer = new List<BroadCollisionPairKey>();
        private readonly List<DeferredLifecycleOperation> deferredLifecycleOperations = new List<DeferredLifecycleOperation>();
        private readonly Dictionary<BroadCollisionPairKey, CollisionEventState> activeCollisionEvents = new Dictionary<BroadCollisionPairKey, CollisionEventState>();
        private readonly Dictionary<BroadCollisionPairKey, CollisionEventState> nextCollisionEvents = new Dictionary<BroadCollisionPairKey, CollisionEventState>();
        private readonly CollisionSystem collisionSystem = new CollisionSystem();
        private int collisionCallbackDispatchDepth;
        private int lifecycleOperationDepth;
        private int lastDeferredLifecycleOperationCount;
        private int currentCallbackExceptionCount;
        private int lastCallbackExceptionCount;
        private bool isFlushingDeferredLifecycleOperations;

        public IReadOnlyDictionary<ulong, Entity> Entities => entities;
        public IReadOnlyDictionary<ulong, Rigid> Rigids => rigids;
        public IReadOnlyDictionary<ulong, Collider> Colliders => colliders;
        public IReadOnlyDictionary<ulong, Constraint> Constraints => constraints;
        public IReadOnlyList<BroadCollisionPair> BroadphasePairs => collisionSystem.BroadphasePairs;
        public IReadOnlyList<NarrowCollisionSystem.CollisionPair> CollisionPairs => collisionSystem.CollisionPairs;
        public IReadOnlyList<ContactManifold> ContactManifolds => collisionSystem.ContactManifolds;
        public IReadOnlyList<PhysicsIsland> Islands => islands;
        public fix FixedTimeAccumulator => fixedTimeAccumulator;
        public ulong FixedStepCount => fixedStepCount;
        public WorldStepStats LastStepStats { get; private set; }
        public System.Exception LastCallbackException { get; private set; }

        public static bool IsConstraintTypeImplemented(ConstraintType type)
        {
            return ConstraintRegistry.IsImplemented(type);
        }

        public World()
            : this(WorldSettings.Default)
        {
        }

        public World(WorldSettings worldSettings)
        {
            settings = worldSettings;
            entities = new Dictionary<ulong, Entity>();
        }

        public void Reserve(int rigidCapacity, int colliderCapacity, int constraintCapacity)
        {
            EnsureListCapacity(rigidIds, rigidCapacity);
            EnsureListCapacity(colliderIds, colliderCapacity);
            EnsureListCapacity(constraintIds, constraintCapacity);
            EnsureListCapacity(activeColliders, colliderCapacity);
            EnsureListCapacity(islandPool, rigidCapacity);
            EnsureListCapacity(islandStack, rigidCapacity);
            EnsureListCapacity(colliderRemovalBuffer, colliderCapacity);
            EnsureListCapacity(constraintRemovalBuffer, constraintCapacity);
        }

        public void Update()
        {
            Update(settings.timeStep);
        }

        public int Step(fix deltaTime)
        {
            return Step(deltaTime, settings.timeStep, settings.maxSubSteps);
        }

        public ulong StepAndComputeStateHash(fix deltaTime)
        {
            Step(deltaTime);
            return ComputeStateHash();
        }

        public int Step(fix deltaTime, fix fixedTimeStep, int maxSubSteps)
        {
            if (deltaTime < fix.Zero)
            {
                deltaTime = fix.Zero;
            }

            if (fixedTimeStep <= fix.Zero)
            {
                Update(deltaTime);
                fixedStepCount++;
                return deltaTime > fix.Zero ? 1 : 0;
            }

            fixedTimeAccumulator += deltaTime;
            int subSteps = 0;
            while (fixedTimeAccumulator + math.Epsilon >= fixedTimeStep
                && (maxSubSteps <= 0 || subSteps < maxSubSteps))
            {
                Update(fixedTimeStep);
                fixedTimeAccumulator -= fixedTimeStep;
                if (fixedTimeAccumulator < fix.Zero)
                {
                    fixedTimeAccumulator = fix.Zero;
                }

                fixedStepCount++;
                subSteps++;
            }

            return subSteps;
        }

        public void Update(fix deltaTime)
        {
            if (deltaTime < fix.Zero)
            {
                deltaTime = fix.Zero;
            }

            lastDeferredLifecycleOperationCount = 0;
            currentCallbackExceptionCount = 0;
            LastCallbackException = null;
            IntegrateForces(deltaTime);
            SyncColliders();
            IntegrateTransforms(deltaTime);
            SyncColliders();
            collisionSystem.Collision(GetActiveColliders(), settings.narrowPhaseAlgorithm, settings.contactManifoldSettings);
            BuildIslands();
            WakeSleepingBodiesForActiveEdges();
            SolveVelocity();
            SanitizeRigidStates(deltaTime);
            SolvePositions();
            SyncColliders();
            DispatchCollisionCallbacks();
            UpdateSleeping(deltaTime);
            SanitizeRigidStates(deltaTime);
            lastCallbackExceptionCount = currentCallbackExceptionCount;
            UpdateStepStats();
        }

        public ulong ComputeStateHash()
        {
            ulong hash = HashOffsetBasis;
            HashWorldSettings(ref hash, settings);
            Hash(ref hash, id);
            Hash(ref hash, constraintId);
            Hash(ref hash, fixedStepCount);
            Hash(ref hash, fixedTimeAccumulator);

            for (int i = 0; i < rigidIds.Count; i++)
            {
                ulong rigidId = rigidIds[i];
                if (!rigids.TryGetValue(rigidId, out Rigid rigid))
                {
                    continue;
                }

                Hash(ref hash, rigidId);
                if (entities.TryGetValue(rigidId, out Entity entity))
                {
                    Hash(ref hash, entity.translation);
                    Hash(ref hash, entity.orientation);
                    Hash(ref hash, entity.isStatic);
                }

                Hash(ref hash, (int)rigid.type);
                Hash(ref hash, rigid.force);
                Hash(ref hash, rigid.velocity);
                Hash(ref hash, rigid.acceleration);
                Hash(ref hash, rigid.torque);
                Hash(ref hash, rigid.angularVelocity);
                Hash(ref hash, rigid.angularAcceleration);
                Hash(ref hash, rigid.inertia);
                Hash(ref hash, rigid.mass);
                Hash(ref hash, rigid.useGravity);
                Hash(ref hash, rigid.autoMass);
                Hash(ref hash, rigid.autoInertia);
                Hash(ref hash, rigid.enabled);
                Hash(ref hash, rigid.allowSleep);
                Hash(ref hash, rigid.isSleeping);
                Hash(ref hash, rigid.useCCD);
                Hash(ref hash, rigid.sleepTime);
                Hash(ref hash, rigid.colliderCount);
            }

            for (int i = 0; i < colliderIds.Count; i++)
            {
                ulong colliderId = colliderIds[i];
                if (!colliders.TryGetValue(colliderId, out Collider collider))
                {
                    continue;
                }

                Hash(ref hash, collider.rigidId);
                Hash(ref hash, collider.layer);
                Hash(ref hash, collider.collisionMask);
                Hash(ref hash, collider.localCenter);
                Hash(ref hash, collider.localOrientation);
                Hash(ref hash, collider.isTrigger);
                Hash(ref hash, collider.enabled);
                Hash(ref hash, collider.material.GetDensity());
                Hash(ref hash, collider.material.GetFrictionCoefficient());
                Hash(ref hash, collider.material.GetBounciness());
                HashShape(ref hash, collider.shape);
            }

            for (int i = 0; i < constraintIds.Count; i++)
            {
                ulong currentConstraintId = constraintIds[i];
                if (!constraints.TryGetValue(currentConstraintId, out Constraint constraint))
                {
                    continue;
                }

                Hash(ref hash, constraint.id);
                Hash(ref hash, constraint.rigidId0);
                Hash(ref hash, constraint.rigidId1);
                Hash(ref hash, constraint.enabled);
                Hash(ref hash, GetConstraintTypeId(constraint));
                HashConstraint(ref hash, constraint);
            }

            return hash;
        }

        public Entity CreateEntity()
        {
            Entity entity = new Entity(id++);
            entities.Add(entity.id, entity);
            return entity;
        }

        public Rigid CreateRigid(fix3 translation, quaternion orientation)
        {
            Entity entity = new Entity(id++, translation, orientation);
            entities.Add(entity.id, entity);

            Rigid rigid = new Rigid
            {
                id = entity.id,
                type = RigidType.Dynamic,
                mass = fix._1,
                inertia = fix3.one,
                autoMass = true,
                autoInertia = true,
                enabled = true,
                useGravity = settings.enableGravity,
            };

            rigids.Add(rigid.id, rigid);
            rigidIds.Add(rigid.id);
            return rigid;
        }

        public bool TryGetEntity(ulong entityId, out Entity entity)
        {
            return entities.TryGetValue(entityId, out entity);
        }

        public bool TryGetRigid(ulong rigidId, out Rigid rigid)
        {
            return rigids.TryGetValue(rigidId, out rigid);
        }

        public bool TryGetCollider(ulong colliderId, out Collider collider)
        {
            return colliders.TryGetValue(colliderId, out collider);
        }

        public bool TryGetConstraint(ulong constraintId, out Constraint constraint)
        {
            return constraints.TryGetValue(constraintId, out constraint);
        }

        public bool TryGetDistanceConstraint(ulong constraintId, out DistanceConstraint constraint)
        {
            if (constraints.TryGetValue(constraintId, out Constraint stored) && stored is DistanceConstraint distanceConstraint)
            {
                constraint = distanceConstraint;
                return true;
            }

            constraint = default;
            return false;
        }

        public bool TryGetPointConstraint(ulong constraintId, out PointConstraint constraint)
        {
            if (constraints.TryGetValue(constraintId, out Constraint stored) && stored is PointConstraint pointConstraint)
            {
                constraint = pointConstraint;
                return true;
            }

            constraint = default;
            return false;
        }

        public bool TryGetSpringDistanceConstraint(ulong constraintId, out SpringDistanceConstraint constraint)
        {
            if (constraints.TryGetValue(constraintId, out Constraint stored) && stored is SpringDistanceConstraint springConstraint)
            {
                constraint = springConstraint;
                return true;
            }

            constraint = default;
            return false;
        }

        public bool TryGetHingeConstraint(ulong constraintId, out HingeConstraint constraint)
        {
            if (constraints.TryGetValue(constraintId, out Constraint stored) && stored is HingeConstraint hingeConstraint)
            {
                constraint = hingeConstraint;
                return true;
            }

            constraint = default;
            return false;
        }

        public bool TryGetFixedConstraint(ulong constraintId, out FixedConstraint constraint)
        {
            if (constraints.TryGetValue(constraintId, out Constraint stored) && stored is FixedConstraint fixedConstraint)
            {
                constraint = fixedConstraint;
                return true;
            }

            constraint = default;
            return false;
        }

        public bool TryGetSliderConstraint(ulong constraintId, out SliderConstraint constraint)
        {
            if (constraints.TryGetValue(constraintId, out Constraint stored) && stored is SliderConstraint sliderConstraint)
            {
                constraint = sliderConstraint;
                return true;
            }

            constraint = default;
            return false;
        }

        public bool SetTransform(ulong entityId, fix3 translation, quaternion orientation)
        {
            if (!entities.TryGetValue(entityId, out Entity entity))
            {
                return false;
            }

            entity.SetTransform(translation, orientation);
            entities[entityId] = entity;
            WakeRigidInternal(entityId);
            return true;
        }

        public bool SetTranslation(ulong entityId, fix3 translation)
        {
            if (!entities.TryGetValue(entityId, out Entity entity))
            {
                return false;
            }

            entity.translation = translation;
            entities[entityId] = entity;
            WakeRigidInternal(entityId);
            return true;
        }

        public bool SetOrientation(ulong entityId, quaternion orientation)
        {
            if (!entities.TryGetValue(entityId, out Entity entity))
            {
                return false;
            }

            entity.orientation = orientation;
            entities[entityId] = entity;
            WakeRigidInternal(entityId);
            return true;
        }

        public bool SetRigidType(ulong rigidId, RigidType type)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.type = type;
            rigid.WakeUp();
            rigids[rigidId] = rigid;

            if (entities.TryGetValue(rigidId, out Entity entity))
            {
                entity.isStatic = type == RigidType.Static;
                entities[rigidId] = entity;
            }

            return true;
        }

        public bool SetRigidEnabled(ulong rigidId, bool enabled)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            if (rigid.enabled == enabled)
            {
                return true;
            }

            if (QueueDeferredLifecycleOperation(DeferredLifecycleOperationKind.SetRigidEnabled, rigidId, enabled))
            {
                return true;
            }

            BeginLifecycleOperation();
            try
            {
                rigid.SetEnabled(enabled);
                if (enabled)
                {
                    rigid.WakeUp();
                }

                if (!enabled)
                {
                    rigid.ClearForces();
                    rigid.ClearTorques();
                    EndCollisionEventsForRigid(rigidId);
                    RemoveRigidCollidersFromCollisionSystem(rigidId);
                    ClearConstraintWarmStartForRigid(rigidId);
                }

                rigids[rigidId] = rigid;
                return true;
            }
            finally
            {
                EndLifecycleOperation();
            }
        }

        public bool SetRigidAllowSleep(ulong rigidId, bool allowSleep)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.SetAllowSleep(allowSleep);
            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetRigidCCD(ulong rigidId, bool useCCD)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.useCCD = useCCD;
            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetMass(ulong rigidId, fix mass)
        {
            if (mass < fix.Zero || !rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.mass = mass;
            rigid.autoMass = false;
            rigid.WakeUp();
            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetVelocity(ulong rigidId, fix3 velocity)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.velocity = ClampLinearVelocity(velocity);
            if (math.lengthsq(rigid.velocity) > math.Epsilon)
            {
                rigid.WakeUp();
            }

            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetAngularVelocity(ulong rigidId, fix3 angularVelocity)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.angularVelocity = ClampAngularVelocity(angularVelocity);
            if (math.lengthsq(rigid.angularVelocity) > math.Epsilon)
            {
                rigid.WakeUp();
            }

            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetAcceleration(ulong rigidId, fix3 acceleration)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.acceleration = PhysicsSafety.Sanitize(acceleration);
            if (math.lengthsq(rigid.acceleration) > math.Epsilon)
            {
                rigid.WakeUp();
            }

            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetAngularAcceleration(ulong rigidId, fix3 angularAcceleration)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.angularAcceleration = PhysicsSafety.Sanitize(angularAcceleration);
            if (math.lengthsq(rigid.angularAcceleration) > math.Epsilon)
            {
                rigid.WakeUp();
            }

            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetInertia(ulong rigidId, fix3 inertia)
        {
            if ((inertia.x < fix.Zero || inertia.y < fix.Zero || inertia.z < fix.Zero)
                || !rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.inertia = inertia;
            rigid.autoInertia = false;
            rigid.WakeUp();
            rigids[rigidId] = rigid;
            return true;
        }

        public bool AddForce(ulong rigidId, fix3 force)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            fix3 safeForce = PhysicsSafety.Sanitize(force);
            rigid.AddForce(safeForce);
            if (math.lengthsq(safeForce) > math.Epsilon)
            {
                rigid.WakeUp();
            }

            rigids[rigidId] = rigid;
            return true;
        }

        public bool AddTorque(ulong rigidId, fix3 torque)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            fix3 safeTorque = PhysicsSafety.Sanitize(torque);
            rigid.AddTorque(safeTorque);
            if (math.lengthsq(safeTorque) > math.Epsilon)
            {
                rigid.WakeUp();
            }

            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetColliderTrigger(ulong colliderId, bool isTrigger)
        {
            if (!colliders.TryGetValue(colliderId, out Collider collider))
            {
                return false;
            }

            collider.SetTrigger(isTrigger);
            return true;
        }

        public bool SetColliderEnabled(ulong colliderId, bool enabled)
        {
            if (!colliders.TryGetValue(colliderId, out Collider collider))
            {
                return false;
            }

            if (collider.enabled == enabled)
            {
                return true;
            }

            if (QueueDeferredLifecycleOperation(DeferredLifecycleOperationKind.SetColliderEnabled, colliderId, enabled))
            {
                return true;
            }

            BeginLifecycleOperation();
            try
            {
                collider.SetEnabled(enabled);
                if (enabled)
                {
                    WakeRigidInternal(collider.rigidId);
                }

                if (!enabled)
                {
                    EndCollisionEventsForCollider(colliderId);
                    collisionSystem.RemoveCollider(colliderId);
                }

                return true;
            }
            finally
            {
                EndLifecycleOperation();
            }
        }

        public bool SetColliderMaterial(ulong colliderId, Material material)
        {
            if (!colliders.TryGetValue(colliderId, out Collider collider))
            {
                return false;
            }

            collider.SetMaterial(material);
            ApplyRigidMassProperties(collider.rigidId);
            return true;
        }

        public bool SetColliderDensity(ulong colliderId, fix density)
        {
            if (!colliders.TryGetValue(colliderId, out Collider collider))
            {
                return false;
            }

            Material material = collider.material;
            material.SetDensity(density);
            collider.SetMaterial(material);
            ApplyRigidMassProperties(collider.rigidId);
            return true;
        }

        public bool SetColliderLayer(ulong colliderId, int layer)
        {
            if (!colliders.TryGetValue(colliderId, out Collider collider))
            {
                return false;
            }

            return collider.SetLayer(layer);
        }

        public bool SetColliderCollisionMask(ulong colliderId, int collisionMask)
        {
            if (!colliders.TryGetValue(colliderId, out Collider collider))
            {
                return false;
            }

            collider.SetCollisionMask(collisionMask);
            return true;
        }

        public DistanceConstraint CreateDistanceConstraint(ulong rigidId0, ulong rigidId1, fix distance)
        {
            return CreateDistanceConstraint(rigidId0, rigidId1, fix3.zero, fix3.zero, distance);
        }

        public DistanceConstraint CreateDistanceConstraint(ulong rigidId0, ulong rigidId1)
        {
            return CreateDistanceConstraint(rigidId0, rigidId1, fix3.zero, fix3.zero);
        }

        public DistanceConstraint CreateDistanceConstraint(ulong rigidId0, ulong rigidId1, fix3 localAnchor0, fix3 localAnchor1)
        {
            if (!TryComputeCurrentAnchorDistance(rigidId0, rigidId1, localAnchor0, localAnchor1, out fix distance))
            {
                return null;
            }

            return CreateDistanceConstraint(rigidId0, rigidId1, localAnchor0, localAnchor1, distance);
        }

        public DistanceConstraint CreateDistanceConstraint(ulong rigidId0, ulong rigidId1, fix3 localAnchor0, fix3 localAnchor1, fix distance)
        {
            if (rigidId0 == rigidId1 || !rigids.ContainsKey(rigidId0) || !rigids.ContainsKey(rigidId1))
            {
                return null;
            }

            DistanceConstraint constraint = new DistanceConstraint(rigidId0, rigidId1, localAnchor0, localAnchor1, distance)
            {
                id = constraintId++,
            };
            return RegisterConstraint(constraint);
        }

        public PointConstraint CreatePointConstraint(ulong rigidId0, ulong rigidId1)
        {
            return CreatePointConstraint(rigidId0, rigidId1, fix3.zero, fix3.zero);
        }

        public PointConstraint CreatePointConstraint(ulong rigidId0, ulong rigidId1, fix3 localAnchor0, fix3 localAnchor1)
        {
            if (rigidId0 == rigidId1 || !rigids.ContainsKey(rigidId0) || !rigids.ContainsKey(rigidId1))
            {
                return null;
            }

            PointConstraint constraint = new PointConstraint(rigidId0, rigidId1, localAnchor0, localAnchor1)
            {
                id = constraintId++,
            };
            return RegisterConstraint(constraint);
        }

        public SpringDistanceConstraint CreateSpringDistanceConstraint(ulong rigidId0, ulong rigidId1, fix stiffness, fix damping)
        {
            return CreateSpringDistanceConstraint(rigidId0, rigidId1, fix3.zero, fix3.zero, stiffness, damping);
        }

        public SpringDistanceConstraint CreateSpringDistanceConstraint(
            ulong rigidId0,
            ulong rigidId1,
            fix3 localAnchor0,
            fix3 localAnchor1,
            fix stiffness,
            fix damping)
        {
            if (!TryComputeCurrentAnchorDistance(rigidId0, rigidId1, localAnchor0, localAnchor1, out fix distance))
            {
                return null;
            }

            return CreateSpringDistanceConstraint(rigidId0, rigidId1, localAnchor0, localAnchor1, distance, stiffness, damping);
        }

        public SpringDistanceConstraint CreateSpringDistanceConstraint(
            ulong rigidId0,
            ulong rigidId1,
            fix3 localAnchor0,
            fix3 localAnchor1,
            fix restDistance,
            fix stiffness,
            fix damping)
        {
            if (rigidId0 == rigidId1 || !rigids.ContainsKey(rigidId0) || !rigids.ContainsKey(rigidId1))
            {
                return null;
            }

            SpringDistanceConstraint constraint = new SpringDistanceConstraint(
                rigidId0,
                rigidId1,
                localAnchor0,
                localAnchor1,
                restDistance,
                stiffness,
                damping)
            {
                id = constraintId++,
            };
            return RegisterConstraint(constraint);
        }

        public HingeConstraint CreateHingeConstraint(ulong rigidId0, ulong rigidId1, fix3 localAnchor0, fix3 localAnchor1, fix3 localAxis0, fix3 localAxis1)
        {
            if (rigidId0 == rigidId1 || !rigids.ContainsKey(rigidId0) || !rigids.ContainsKey(rigidId1))
            {
                return null;
            }

            HingeConstraint constraint = new HingeConstraint(rigidId0, rigidId1, localAnchor0, localAnchor1, localAxis0, localAxis1)
            {
                id = constraintId++,
            };
            return RegisterConstraint(constraint);
        }

        public FixedConstraint CreateFixedConstraint(ulong rigidId0, ulong rigidId1)
        {
            return CreateFixedConstraint(rigidId0, rigidId1, fix3.zero, fix3.zero);
        }

        public FixedConstraint CreateFixedConstraint(ulong rigidId0, ulong rigidId1, fix3 localAnchor0, fix3 localAnchor1)
        {
            if (rigidId0 == rigidId1
                || !rigids.ContainsKey(rigidId0)
                || !rigids.ContainsKey(rigidId1)
                || !TryComputeCurrentRelativeOrientation(rigidId0, rigidId1, out quaternion targetLocalRotation))
            {
                return null;
            }

            FixedConstraint constraint = new FixedConstraint(rigidId0, rigidId1, localAnchor0, localAnchor1, targetLocalRotation)
            {
                id = constraintId++,
            };
            return RegisterConstraint(constraint);
        }

        public SliderConstraint CreateSliderConstraint(ulong rigidId0, ulong rigidId1, fix3 localAxis0, fix3 localAxis1)
        {
            return CreateSliderConstraint(rigidId0, rigidId1, fix3.zero, fix3.zero, localAxis0, localAxis1);
        }

        public SliderConstraint CreateSliderConstraint(
            ulong rigidId0,
            ulong rigidId1,
            fix3 localAnchor0,
            fix3 localAnchor1,
            fix3 localAxis0,
            fix3 localAxis1)
        {
            if (rigidId0 == rigidId1 || !rigids.ContainsKey(rigidId0) || !rigids.ContainsKey(rigidId1))
            {
                return null;
            }

            SliderConstraint constraint = new SliderConstraint(rigidId0, rigidId1, localAnchor0, localAnchor1, localAxis0, localAxis1)
            {
                id = constraintId++,
            };
            return RegisterConstraint(constraint);
        }

        public bool SetConstraintEnabled(ulong constraintId, bool enabled)
        {
            if (!constraints.TryGetValue(constraintId, out Constraint constraint))
            {
                return false;
            }

            if (QueueDeferredLifecycleOperation(DeferredLifecycleOperationKind.SetConstraintEnabled, constraintId, enabled))
            {
                return true;
            }

            BeginLifecycleOperation();
            try
            {
                constraint.SetEnabled(enabled);
                if (enabled)
                {
                    WakeRigidInternal(constraint.rigidId0);
                    WakeRigidInternal(constraint.rigidId1);
                }

                return true;
            }
            finally
            {
                EndLifecycleOperation();
            }
        }

        public bool RemoveConstraint(ulong constraintId)
        {
            if (!constraints.TryGetValue(constraintId, out Constraint constraint))
            {
                return false;
            }

            if (QueueDeferredLifecycleOperation(DeferredLifecycleOperationKind.RemoveConstraint, constraintId, false))
            {
                return true;
            }

            BeginLifecycleOperation();
            try
            {
                if (!constraints.Remove(constraintId))
                {
                    return false;
                }

                constraintIds.Remove(constraintId);
                WakeRigidInternal(constraint.rigidId0);
                WakeRigidInternal(constraint.rigidId1);
                return true;
            }
            finally
            {
                EndLifecycleOperation();
            }
        }

        private T RegisterConstraint<T>(T constraint)
            where T : Constraint
        {
            constraints.Add(constraint.id, constraint);
            constraintIds.Add(constraint.id);
            WakeRigidInternal(constraint.rigidId0);
            WakeRigidInternal(constraint.rigidId1);
            return constraint;
        }

        public Collider AddAABBCollider(ulong rigidId, fix3 center, fix3 size)
        {
            Collider collider = new Collider();
            collider.AddAABBCollider(rigidId, center, size);
            return RegisterCollider(collider);
        }

        public Collider AddOBBCollider(ulong rigidId, fix3 center, fix3 size, quaternion rotation)
        {
            Collider collider = new Collider();
            collider.AddOBBCollider(rigidId, center, size, rotation);
            return RegisterCollider(collider);
        }

        public Collider AddSphereCollider(ulong rigidId, fix3 center, fix radius)
        {
            Collider collider = new Collider();
            collider.AddSphereCollider(rigidId, center, radius);
            return RegisterCollider(collider);
        }

        public Collider AddCapsuleCollider(ulong rigidId, fix3 center, fix radius, fix height, quaternion rotation)
        {
            Collider collider = new Collider();
            collider.AddCapsuleCollider(rigidId, center, radius, height, rotation);
            return RegisterCollider(collider);
        }

        public bool RemoveCollider(ulong colliderId)
        {
            if (!colliders.ContainsKey(colliderId))
            {
                return false;
            }

            if (QueueDeferredLifecycleOperation(DeferredLifecycleOperationKind.RemoveCollider, colliderId, false))
            {
                return true;
            }

            return RemoveColliderInternal(colliderId, true);
        }

        public bool RemoveRigid(ulong rigidId)
        {
            if (!rigids.ContainsKey(rigidId))
            {
                return false;
            }

            if (QueueDeferredLifecycleOperation(DeferredLifecycleOperationKind.RemoveRigid, rigidId, false))
            {
                return true;
            }

            BeginLifecycleOperation();
            try
            {
                colliderRemovalBuffer.Clear();
                for (int i = 0; i < colliderIds.Count; i++)
                {
                    ulong colliderId = colliderIds[i];
                    if (colliders.TryGetValue(colliderId, out Collider collider) && collider.rigidId == rigidId)
                    {
                        colliderRemovalBuffer.Add(colliderId);
                    }
                }

                for (int i = 0; i < colliderRemovalBuffer.Count; i++)
                {
                    RemoveColliderInternal(colliderRemovalBuffer[i], true);
                }

                colliderRemovalBuffer.Clear();
                RemoveConstraintsForRigid(rigidId);
                rigids.Remove(rigidId);
                rigidIds.Remove(rigidId);
                entities.Remove(rigidId);
                return true;
            }
            finally
            {
                EndLifecycleOperation();
            }
        }

        public bool TestCollision(Collider a, Collider b)
        {
            return collisionSystem.TestCollision(a, b);
        }

        public bool TryGetCollision(Collider a, Collider b, out CollisionInfo collision)
        {
            return collisionSystem.TryGetCollision(a, b, settings.narrowPhaseAlgorithm, out collision);
        }

        public void QueryAABB(AABB bounds, List<Collider> results)
        {
            QueryAABB(bounds, Collider.AllLayers, results);
        }

        public void QueryAABB(AABB bounds, int layerMask, List<Collider> results)
        {
            SyncColliders();
            collisionSystem.QueryAABB(GetActiveColliders(), bounds, layerMask, results);
        }

        public bool Raycast(Ray ray, out RaycastHit hitInfo, fix maxDistance)
        {
            return Raycast(ray, out hitInfo, maxDistance, Collider.AllLayers);
        }

        public bool Raycast(Ray ray, out RaycastHit hitInfo, fix maxDistance, int layerMask)
        {
            SyncColliders();
            return collisionSystem.Raycast(GetActiveColliders(), ray, maxDistance, layerMask, out hitInfo);
        }

        public bool Raycast(fix3 origin, fix3 direction, out RaycastHit hitInfo, fix maxDistance)
        {
            return Raycast(new Ray(origin, direction), out hitInfo, maxDistance);
        }

        public bool Raycast(fix3 origin, fix3 direction, out RaycastHit hitInfo, fix maxDistance, int layerMask)
        {
            return Raycast(new Ray(origin, direction), out hitInfo, maxDistance, layerMask);
        }

        private void IntegrateForces(fix deltaTime)
        {
            for (int i = 0; i < rigidIds.Count; i++)
            {
                ulong rigidId = rigidIds[i];
                if (!rigids.TryGetValue(rigidId, out Rigid rigid))
                {
                    continue;
                }

                if (!rigid.IsAwakeDynamic)
                {
                    if (!rigid.IsDynamic)
                    {
                        rigid.ClearForces();
                        rigid.ClearTorques();
                    }

                    rigids[rigidId] = rigid;
                    continue;
                }

                fix3 acceleration = rigid.acceleration + rigid.force * rigid.inverseMass;
                if (rigid.useGravity && settings.enableGravity)
                {
                    acceleration += new fix3(fix.Zero, settings.gravity, fix.Zero);
                }

                rigid.velocity += acceleration * deltaTime;
                rigid.angularVelocity += (rigid.angularAcceleration + rigid.torque * rigid.inverseInertia) * deltaTime;
                ClampRigidMotion(ref rigid, deltaTime);
                rigid.ClearForces();
                rigid.ClearTorques();
                rigids[rigidId] = rigid;
            }
        }

        private void IntegrateTransforms(fix deltaTime)
        {
            for (int i = 0; i < rigidIds.Count; i++)
            {
                ulong rigidId = rigidIds[i];
                if (!rigids.TryGetValue(rigidId, out Rigid rigid))
                {
                    continue;
                }

                if (!rigid.IsAwakeDynamic && !rigid.IsKinematic)
                {
                    continue;
                }

                if (!entities.TryGetValue(rigidId, out Entity entity))
                {
                    continue;
                }

                ClampRigidMotion(ref rigid, deltaTime);
                fix integrationTime = IntegrateTranslationWithCCD(rigidId, ref rigid, ref entity, deltaTime);
                entity.orientation = IntegrateOrientation(entity.orientation, rigid.angularVelocity, integrationTime);
                entities[rigidId] = entity;
                rigids[rigidId] = rigid;
            }
        }

        private fix IntegrateTranslationWithCCD(ulong rigidId, ref Rigid rigid, ref Entity entity, fix deltaTime)
        {
            if (math.lengthsq(rigid.velocity) <= math.Epsilon)
            {
                return deltaTime;
            }

            fix remainingTime = deltaTime;
            fix integratedTime = fix.Zero;
            int maxIterations = settings.ccdMaxIterations > 0 ? settings.ccdMaxIterations : 1;

            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                if (remainingTime <= fix.Zero || math.lengthsq(rigid.velocity) <= math.Epsilon)
                {
                    break;
                }

                fix3 translationDelta = rigid.velocity * remainingTime;
                if (!TryComputeTimeOfImpact(rigid, translationDelta, remainingTime, out ContinuousHit hit))
                {
                    entity.translation += translationDelta;
                    integratedTime += remainingTime;
                    break;
                }

                fix distance = math.length(translationDelta);
                fix safeFraction = hit.fraction;
                if (distance > math.Epsilon && settings.ccdSkin > fix.Zero)
                {
                    safeFraction = math.max(fix.Zero, safeFraction - settings.ccdSkin / distance);
                }

                entity.translation += translationDelta * safeFraction;
                fix consumedTime = remainingTime * safeFraction;
                integratedTime += consumedTime;

                fix normalVelocity = math.dot(rigid.velocity, hit.normal);
                if (normalVelocity > fix.Zero)
                {
                    rigid.velocity -= hit.normal * normalVelocity;
                }

                remainingTime -= consumedTime;
                if (remainingTime <= math.Epsilon || safeFraction <= math.Epsilon || math.lengthsq(rigid.velocity) <= math.Epsilon)
                {
                    break;
                }

                entities[rigidId] = entity;
                rigids[rigidId] = rigid;
                SyncRigidColliders(rigidId);
            }

            return integratedTime;
        }

        private static quaternion IntegrateOrientation(quaternion orientation, fix3 angularVelocity, fix deltaTime)
        {
            fix speedSq = math.lengthsq(angularVelocity);
            if (deltaTime <= fix.Zero || speedSq <= math.Epsilon)
            {
                return orientation;
            }

            fix speed = math.sqrt(speedSq);
            fix angle = speed * deltaTime;
            if (math.abs(angle) <= math.Epsilon)
            {
                return orientation;
            }

            quaternion deltaRotation = quaternion.AxisAngle(angularVelocity / speed, angle);
            return quaternion.normalize(deltaRotation * orientation);
        }

        private void ClampRigidMotion(ref Rigid rigid, fix deltaTime)
        {
            rigid.velocity = ClampLinearVelocityForStep(rigid.velocity, deltaTime);
            rigid.angularVelocity = ClampAngularVelocityForStep(rigid.angularVelocity, deltaTime);
            rigid.acceleration = PhysicsSafety.Sanitize(rigid.acceleration);
            rigid.angularAcceleration = PhysicsSafety.Sanitize(rigid.angularAcceleration);
            rigid.force = PhysicsSafety.Sanitize(rigid.force);
            rigid.torque = PhysicsSafety.Sanitize(rigid.torque);
        }

        private void SanitizeRigidStates(fix deltaTime)
        {
            for (int i = 0; i < rigidIds.Count; i++)
            {
                ulong rigidId = rigidIds[i];
                if (!rigids.TryGetValue(rigidId, out Rigid rigid))
                {
                    continue;
                }

                ClampRigidMotion(ref rigid, deltaTime);
                rigids[rigidId] = rigid;
            }
        }

        private fix3 ClampLinearVelocity(fix3 velocity)
        {
            return PhysicsSafety.ClampVectorMagnitude(velocity, settings.maxLinearVelocity);
        }

        private fix3 ClampAngularVelocity(fix3 angularVelocity)
        {
            return PhysicsSafety.ClampVectorMagnitude(angularVelocity, settings.maxAngularVelocity);
        }

        private fix3 ClampLinearVelocityForStep(fix3 velocity, fix deltaTime)
        {
            fix maxVelocity = settings.maxLinearVelocity;
            if (settings.maxTranslationPerStep > fix.Zero && deltaTime > fix.Zero)
            {
                fix stepVelocity = settings.maxTranslationPerStep / deltaTime;
                maxVelocity = maxVelocity > fix.Zero ? math.min(maxVelocity, stepVelocity) : stepVelocity;
            }

            return PhysicsSafety.ClampVectorMagnitude(velocity, maxVelocity);
        }

        private fix3 ClampAngularVelocityForStep(fix3 angularVelocity, fix deltaTime)
        {
            fix maxVelocity = settings.maxAngularVelocity;
            if (settings.maxRotationPerStep > fix.Zero && deltaTime > fix.Zero)
            {
                fix stepVelocity = settings.maxRotationPerStep / deltaTime;
                maxVelocity = maxVelocity > fix.Zero ? math.min(maxVelocity, stepVelocity) : stepVelocity;
            }

            return PhysicsSafety.ClampVectorMagnitude(angularVelocity, maxVelocity);
        }

        private bool TryComputeTimeOfImpact(Rigid rigid, fix3 translationDelta, fix deltaTime, out ContinuousHit hit)
        {
            hit = default;
            if (!settings.enableCCD
                || rigid == null
                || !rigid.useCCD
                || !rigid.IsAwakeDynamic
                || math.lengthsq(translationDelta) <= math.Epsilon)
            {
                return false;
            }

            fix minVelocity = math.max(fix.Zero, settings.ccdMinVelocity);
            if (math.lengthsq(rigid.velocity) < minVelocity * minVelocity)
            {
                return false;
            }

            bool found = false;
            fix bestFraction = fix.One;
            fix3 bestNormal = fix3.zero;
            ulong bestColliderId = ulong.MaxValue;

            for (int i = 0; i < rigid.ColliderIds.Count; i++)
            {
                ulong movingColliderId = rigid.ColliderIds[i];
                if (!colliders.TryGetValue(movingColliderId, out Collider movingCollider)
                    || !IsColliderActive(movingCollider)
                    || movingCollider.isTrigger)
                {
                    continue;
                }

                AABB movingBounds = Physics.ComputeBounds(movingCollider.shape);
                for (int j = 0; j < colliderIds.Count; j++)
                {
                    ulong targetColliderId = colliderIds[j];
                    if (targetColliderId == movingColliderId
                        || !colliders.TryGetValue(targetColliderId, out Collider targetCollider)
                        || !IsColliderActive(targetCollider)
                        || targetCollider.rigidId == rigid.id
                        || targetCollider.isTrigger
                        || !movingCollider.CanCollideWith(targetCollider)
                        || (!settings.enableDynamicCCD && IsAwakeDynamicRigid(targetCollider.rigidId)))
                    {
                        continue;
                    }

                    fix3 targetTranslationDelta = GetContinuousTargetDelta(targetCollider.rigidId, deltaTime);
                    fix3 relativeTranslationDelta = translationDelta - targetTranslationDelta;
                    if (math.lengthsq(relativeTranslationDelta) <= math.Epsilon
                        || !TryComputeColliderTimeOfImpact(movingCollider, targetCollider, movingBounds, relativeTranslationDelta, out fix fraction, out fix3 normal))
                    {
                        continue;
                    }

                    if (fraction > bestFraction || (fraction == bestFraction && targetColliderId >= bestColliderId))
                    {
                        continue;
                    }

                    found = true;
                    bestFraction = fraction;
                    bestNormal = normal;
                    bestColliderId = targetColliderId;
                }
            }

            if (!found)
            {
                return false;
            }

            hit = new ContinuousHit(bestFraction, bestNormal, bestColliderId);
            return true;
        }

        private fix3 GetContinuousTargetDelta(ulong targetRigidId, fix deltaTime)
        {
            if (deltaTime <= fix.Zero || !rigids.TryGetValue(targetRigidId, out Rigid targetRigid))
            {
                return fix3.zero;
            }

            if (targetRigid.IsKinematic || (settings.enableDynamicCCD && targetRigid.IsAwakeDynamic))
            {
                return targetRigid.velocity * deltaTime;
            }

            return fix3.zero;
        }

        private static bool TryComputeColliderTimeOfImpact(
            Collider movingCollider,
            Collider targetCollider,
            AABB movingBounds,
            fix3 translationDelta,
            out fix fraction,
            out fix3 normal)
        {
            if (Physics.TryShapeCast(movingCollider.shape, targetCollider.shape, translationDelta, out ShapeCastHit shapeCastHit))
            {
                fraction = shapeCastHit.fraction;
                normal = shapeCastHit.normal;
                return true;
            }

            AABB targetBounds = Physics.ComputeBounds(targetCollider.shape);
            if (Physics.TryAABBCast(movingBounds, targetBounds, translationDelta, out ShapeCastHit boundsCastHit))
            {
                fraction = boundsCastHit.fraction;
                normal = boundsCastHit.normal;
                return true;
            }

            fraction = fix.Zero;
            normal = fix3.zero;
            return false;
        }

        private bool IsAwakeDynamicRigid(ulong rigidId)
        {
            return rigids.TryGetValue(rigidId, out Rigid rigid) && rigid.IsAwakeDynamic;
        }

        private void SolveVelocity()
        {
            int solverIterations = settings.solverIterations > 0 ? settings.solverIterations : 1;
            IReadOnlyList<ContactManifold> manifolds = collisionSystem.ContactManifolds;
            SolverContext context = CreateSolverContext();
            WarmStartContacts(manifolds, context);
            WarmStartConstraints(context);
            for (int iteration = 0; iteration < solverIterations; iteration++)
            {
                for (int i = 0; i < manifolds.Count; i++)
                {
                    ContactManifold manifold = manifolds[i];
                    if (manifold.isTrigger)
                    {
                        continue;
                    }

                    for (int j = 0; j < manifold.contactCount; j++)
                    {
                        ref ContactPoint point = ref manifold.GetPointRef(j);
                        ResolveContactVelocity(manifold, ref point, context);
                    }
                }

                SolveConstraintVelocity(context);
            }
        }

        private void WarmStartContacts(IReadOnlyList<ContactManifold> manifolds, SolverContext context)
        {
            if (context.warmStartScale <= fix.Zero)
            {
                return;
            }

            for (int i = 0; i < manifolds.Count; i++)
            {
                ContactManifold manifold = manifolds[i];
                if (manifold.isTrigger)
                {
                    continue;
                }

                for (int j = 0; j < manifold.contactCount; j++)
                {
                    ContactPoint point = manifold[j];
                    if (point.normalImpulse <= fix.Zero
                        && math.abs(point.tangentImpulse0) <= math.Epsilon
                        && math.abs(point.tangentImpulse1) <= math.Epsilon)
                    {
                        continue;
                    }

                    bool hasRigid0 = context.TryGetRigidMassData(manifold.rigidId0, out Rigid rigid0, out fix inverseMass0);
                    bool hasRigid1 = context.TryGetRigidMassData(manifold.rigidId1, out Rigid rigid1, out fix inverseMass1);
                    if (!hasRigid0 && !hasRigid1)
                    {
                        continue;
                    }

                    fix3 r0 = hasRigid0 ? point.position - context.GetEntityPosition(rigid0.id) : fix3.zero;
                    fix3 r1 = hasRigid1 ? point.position - context.GetEntityPosition(rigid1.id) : fix3.zero;
                    fix3 impulse = manifold.normal * (point.normalImpulse * context.warmStartScale);
                    if (math.lengthsq(point.tangent0) > math.Epsilon)
                    {
                        impulse += point.tangent0 * (point.tangentImpulse0 * context.warmStartScale);
                    }

                    if (math.lengthsq(point.tangent1) > math.Epsilon)
                    {
                        impulse += point.tangent1 * (point.tangentImpulse1 * context.warmStartScale);
                    }

                    context.ApplyImpulse(hasRigid0, rigid0, inverseMass0, r0, hasRigid1, rigid1, inverseMass1, r1, impulse);
                }
            }
        }

        private void WarmStartConstraints(SolverContext context)
        {
            for (int i = 0; i < constraintIds.Count; i++)
            {
                ulong currentConstraintId = constraintIds[i];
                if (constraints.TryGetValue(currentConstraintId, out Constraint constraint))
                {
                    constraint.WarmStart(context);
                }
            }
        }

        private void SolveConstraintVelocity(SolverContext context)
        {
            for (int i = 0; i < constraintIds.Count; i++)
            {
                ulong currentConstraintId = constraintIds[i];
                if (constraints.TryGetValue(currentConstraintId, out Constraint constraint))
                {
                    constraint.SolveVelocity(context);
                }
            }
        }

        private void ResolveContactVelocity(ContactManifold manifold, ref ContactPoint point, SolverContext context)
        {
            bool hasRigid0 = context.TryGetRigidMassData(manifold.rigidId0, out Rigid rigid0, out fix inverseMass0);
            bool hasRigid1 = context.TryGetRigidMassData(manifold.rigidId1, out Rigid rigid1, out fix inverseMass1);
            if (!hasRigid0 && !hasRigid1)
            {
                return;
            }

            fix3 r0 = hasRigid0 ? point.position - context.GetEntityPosition(rigid0.id) : fix3.zero;
            fix3 r1 = hasRigid1 ? point.position - context.GetEntityPosition(rigid1.id) : fix3.zero;
            fix3 velocity0 = hasRigid0 ? SolverContext.GetVelocityAtPoint(rigid0, r0) : fix3.zero;
            fix3 velocity1 = hasRigid1 ? SolverContext.GetVelocityAtPoint(rigid1, r1) : fix3.zero;
            fix3 relativeVelocity = velocity1 - velocity0;
            fix normalVelocity = math.dot(relativeVelocity, manifold.normal);
            if (normalVelocity > fix.Zero && point.lifetime <= 1)
            {
                return;
            }

            fix effectiveMass = inverseMass0 + inverseMass1
                + SolverContext.GetAngularEffectiveMass(rigid0, r0, manifold.normal)
                + SolverContext.GetAngularEffectiveMass(rigid1, r1, manifold.normal);
            if (effectiveMass <= fix.Zero)
            {
                return;
            }

            fix restitutionThreshold = math.max(fix.Zero, settings.restitutionVelocityThreshold);
            fix restitution = normalVelocity < -restitutionThreshold ? GetCombinedRestitution(manifold) : fix.Zero;
            fix normalImpulseDelta = -(fix.One + restitution) * normalVelocity / effectiveMass;
            fix previousNormalImpulse = point.normalImpulse;
            point.normalImpulse = math.max(fix.Zero, previousNormalImpulse + normalImpulseDelta);
            fix maxContactImpulse = math.max(fix.Zero, settings.maxContactImpulse);
            if (maxContactImpulse > fix.Zero)
            {
                point.normalImpulse = math.min(point.normalImpulse, maxContactImpulse);
            }

            fix normalImpulseMagnitude = point.normalImpulse - previousNormalImpulse;
            fix3 impulse = manifold.normal * normalImpulseMagnitude;

            context.ApplyImpulse(hasRigid0, rigid0, inverseMass0, r0, hasRigid1, rigid1, inverseMass1, r1, impulse);

            ResolveContactFriction(manifold, ref point, hasRigid0, rigid0, inverseMass0, r0, hasRigid1, rigid1, inverseMass1, r1, context);

            if (hasRigid0 && inverseMass0 > fix.Zero)
            {
                rigids[rigid0.id] = rigid0;
            }

            if (hasRigid1 && inverseMass1 > fix.Zero)
            {
                rigids[rigid1.id] = rigid1;
            }
        }

        private void ResolveContactFriction(
            ContactManifold manifold,
            ref ContactPoint point,
            bool hasRigid0,
            Rigid rigid0,
            fix inverseMass0,
            fix3 r0,
            bool hasRigid1,
            Rigid rigid1,
            fix inverseMass1,
            fix3 r1,
            SolverContext context)
        {
            fix friction = GetCombinedFriction(manifold);
            if (friction <= fix.Zero || point.normalImpulse <= fix.Zero)
            {
                return;
            }

            fix3 velocity0 = hasRigid0 ? SolverContext.GetVelocityAtPoint(rigid0, r0) : fix3.zero;
            fix3 velocity1 = hasRigid1 ? SolverContext.GetVelocityAtPoint(rigid1, r1) : fix3.zero;
            fix3 relativeVelocity = velocity1 - velocity0;
            fix normalVelocity = math.dot(relativeVelocity, manifold.normal);
            fix3 tangentVelocity = relativeVelocity - manifold.normal * normalVelocity;
            fix tangentSpeedSq = math.lengthsq(tangentVelocity);
            if (tangentSpeedSq <= math.Epsilon)
            {
                return;
            }

            fix tangentSpeed = math.sqrt(tangentSpeedSq);
            fix3 tangent = tangentVelocity / tangentSpeed;
            if (math.lengthsq(point.tangent0) <= math.Epsilon || math.dot(point.tangent0, tangent) < fix._0_5)
            {
                point.tangentImpulse0 = fix.Zero;
            }

            point.tangent0 = tangent;
            fix effectiveMass = inverseMass0 + inverseMass1
                + SolverContext.GetAngularEffectiveMass(rigid0, r0, tangent)
                + SolverContext.GetAngularEffectiveMass(rigid1, r1, tangent);
            if (effectiveMass <= fix.Zero)
            {
                return;
            }

            fix frictionDelta = -math.dot(relativeVelocity, tangent) / effectiveMass;
            fix maxFriction = point.normalImpulse * friction;
            fix maxFrictionImpulse = math.max(fix.Zero, settings.maxFrictionImpulse);
            if (maxFrictionImpulse > fix.Zero)
            {
                maxFriction = math.min(maxFriction, maxFrictionImpulse);
            }

            fix previousTangentImpulse = point.tangentImpulse0;
            point.tangentImpulse0 = math.clamp(previousTangentImpulse + frictionDelta, -maxFriction, maxFriction);
            fix frictionMagnitude = point.tangentImpulse0 - previousTangentImpulse;
            fix3 frictionImpulse = tangent * frictionMagnitude;

            context.ApplyImpulse(hasRigid0, rigid0, inverseMass0, r0, hasRigid1, rigid1, inverseMass1, r1, frictionImpulse);
        }

        private fix GetCombinedRestitution(ContactManifold manifold)
        {
            fix restitution = math.max(fix.Zero, settings.restitution);
            if (colliders.TryGetValue(manifold.colliderId0, out Collider collider0))
            {
                restitution = math.max(restitution, collider0.material.GetBounciness());
            }

            if (colliders.TryGetValue(manifold.colliderId1, out Collider collider1))
            {
                restitution = math.max(restitution, collider1.material.GetBounciness());
            }

            return math.clamp(restitution, fix.Zero, fix.One);
        }

        private fix GetCombinedFriction(ContactManifold manifold)
        {
            fix friction = math.max(fix.Zero, settings.friction);
            bool hasMaterial = false;
            fix materialFriction = fix.Zero;

            if (colliders.TryGetValue(manifold.colliderId0, out Collider collider0))
            {
                materialFriction += collider0.material.GetFrictionCoefficient();
                hasMaterial = true;
            }

            if (colliders.TryGetValue(manifold.colliderId1, out Collider collider1))
            {
                materialFriction += collider1.material.GetFrictionCoefficient();
                hasMaterial = true;
            }

            if (hasMaterial)
            {
                materialFriction = materialFriction * fix._0_5;
                friction = math.max(friction, materialFriction);
            }

            return math.max(fix.Zero, friction);
        }

        private void SolvePositions()
        {
            SolverContext context = CreateSolverContext();
            int positionIterations = settings.positionIterations > 0 ? settings.positionIterations : 1;
            for (int iteration = 0; iteration < positionIterations; iteration++)
            {
                CorrectContactPositions(context);
                SolveConstraintPositions(context);
            }
        }

        private void CorrectContactPositions(SolverContext context)
        {
            IReadOnlyList<ContactManifold> manifolds = collisionSystem.ContactManifolds;
            for (int i = 0; i < manifolds.Count; i++)
            {
                ContactManifold manifold = manifolds[i];
                if (manifold.isTrigger)
                {
                    continue;
                }

                bool hasRigid0 = context.TryGetRigidMassData(manifold.rigidId0, out Rigid rigid0, out fix inverseMass0);
                bool hasRigid1 = context.TryGetRigidMassData(manifold.rigidId1, out Rigid rigid1, out fix inverseMass1);
                if (!hasRigid0 && !hasRigid1)
                {
                    continue;
                }

                fix inverseMassSum = inverseMass0 + inverseMass1;
                if (inverseMassSum <= fix.Zero)
                {
                    continue;
                }

                for (int j = 0; j < manifold.contactCount; j++)
                {
                    ContactPoint point = manifold[j];
                    fix penetration = point.penetrationDepth - settings.penetrationSlop;
                    if (penetration <= fix.Zero)
                    {
                        continue;
                    }

                    fix correctionMagnitude = penetration * settings.positionCorrectionPercent / inverseMassSum / manifold.contactCount;
                    fix maxCorrection = math.max(fix.Zero, settings.maxPositionCorrection);
                    if (maxCorrection > fix.Zero)
                    {
                        correctionMagnitude = math.min(correctionMagnitude, maxCorrection);
                    }

                    fix3 correction = manifold.normal * correctionMagnitude;
                    if (hasRigid0 && inverseMass0 > fix.Zero)
                    {
                        context.TranslateEntity(rigid0.id, -correction * inverseMass0);
                    }

                    if (hasRigid1 && inverseMass1 > fix.Zero)
                    {
                        context.TranslateEntity(rigid1.id, correction * inverseMass1);
                    }
                }
            }
        }

        private void SolveConstraintPositions(SolverContext context)
        {
            for (int i = 0; i < constraintIds.Count; i++)
            {
                ulong currentConstraintId = constraintIds[i];
                if (constraints.TryGetValue(currentConstraintId, out Constraint constraint))
                {
                    constraint.SolvePosition(context);
                }
            }
        }

        private SolverContext CreateSolverContext()
        {
            return new SolverContext(rigids, entities, settings.warmStartScale);
        }

        private void UpdateStepStats()
        {
            LastStepStats = new WorldStepStats(
                activeColliders.Count,
                collisionSystem.BroadphasePairs.Count,
                collisionSystem.BroadphaseTreeProxyCount,
                collisionSystem.BroadphaseTreeHeight,
                collisionSystem.BroadphaseTreeMaxBalance,
                collisionSystem.CollisionPairs.Count,
                collisionSystem.ContactManifolds.Count,
                islands.Count,
                settings.solverIterations > 0 ? settings.solverIterations : 1,
                settings.positionIterations > 0 ? settings.positionIterations : 1,
                lastDeferredLifecycleOperationCount,
                lastCallbackExceptionCount);
        }

        private static void EnsureListCapacity<T>(List<T> list, int capacity)
        {
            if (capacity > list.Capacity)
            {
                list.Capacity = capacity;
            }
        }

        private void BuildIslands()
        {
            islands.Clear();
            islandEdges.Clear();
            islandVisited.Clear();
            islandStack.Clear();

            AddCollisionIslandEdges();
            AddConstraintIslandEdges();

            for (int i = 0; i < rigidIds.Count; i++)
            {
                ulong rigidId = rigidIds[i];
                if (islandVisited.Contains(rigidId)
                    || !rigids.TryGetValue(rigidId, out Rigid rigid)
                    || !rigid.IsDynamic)
                {
                    continue;
                }

                PhysicsIsland island = RentIsland();
                islandVisited.Add(rigidId);
                islandStack.Add(rigidId);

                while (islandStack.Count > 0)
                {
                    int last = islandStack.Count - 1;
                    ulong currentRigidId = islandStack[last];
                    islandStack.RemoveAt(last);
                    island.AddRigid(currentRigidId);

                    for (int edgeIndex = 0; edgeIndex < islandEdges.Count; edgeIndex++)
                    {
                        IslandEdge edge = islandEdges[edgeIndex];
                        if (edge.rigidId0 == currentRigidId)
                        {
                            PushDynamicIslandRigid(edge.rigidId1);
                        }
                        else if (edge.rigidId1 == currentRigidId)
                        {
                            PushDynamicIslandRigid(edge.rigidId0);
                        }
                    }
                }

                island.sleeping = AllIslandBodiesSleeping(island);
                islands.Add(island);
            }
        }

        private PhysicsIsland RentIsland()
        {
            int index = islands.Count;
            while (islandPool.Count <= index)
            {
                islandPool.Add(new PhysicsIsland());
            }

            PhysicsIsland island = islandPool[index];
            island.Clear();
            return island;
        }

        private void AddCollisionIslandEdges()
        {
            IReadOnlyList<ContactManifold> manifolds = collisionSystem.ContactManifolds;
            for (int i = 0; i < manifolds.Count; i++)
            {
                ContactManifold manifold = manifolds[i];
                if (manifold.isTrigger)
                {
                    continue;
                }

                TryAddIslandEdge(manifold.rigidId0, manifold.rigidId1);
            }
        }

        private void AddConstraintIslandEdges()
        {
            for (int i = 0; i < constraintIds.Count; i++)
            {
                ulong currentConstraintId = constraintIds[i];
                if (constraints.TryGetValue(currentConstraintId, out Constraint constraint) && constraint.enabled)
                {
                    TryAddIslandEdge(constraint.rigidId0, constraint.rigidId1);
                }
            }
        }

        private void TryAddIslandEdge(ulong rigidId0, ulong rigidId1)
        {
            if (rigidId0 == 0 || rigidId1 == 0 || rigidId0 == rigidId1)
            {
                return;
            }

            if (!rigids.TryGetValue(rigidId0, out Rigid rigid0)
                || !rigids.TryGetValue(rigidId1, out Rigid rigid1)
                || !rigid0.enabled
                || !rigid1.enabled
                || (!rigid0.IsDynamic && !rigid1.IsDynamic))
            {
                return;
            }

            islandEdges.Add(new IslandEdge(rigidId0, rigidId1));
        }

        private void PushDynamicIslandRigid(ulong rigidId)
        {
            if (islandVisited.Contains(rigidId)
                || !rigids.TryGetValue(rigidId, out Rigid rigid)
                || !rigid.IsDynamic)
            {
                return;
            }

            islandVisited.Add(rigidId);
            islandStack.Add(rigidId);
        }

        private void WakeSleepingBodiesForActiveEdges()
        {
            for (int i = 0; i < rigidIds.Count; i++)
            {
                ulong rigidId = rigidIds[i];
                if (rigids.TryGetValue(rigidId, out Rigid rigid) && rigid.IsDynamic && rigid.isSleeping && HasWakeState(rigid))
                {
                    WakeRigidInternal(rigidId);
                }
            }

            bool changed;
            do
            {
                changed = false;
                for (int i = 0; i < islandEdges.Count; i++)
                {
                    IslandEdge edge = islandEdges[i];
                    changed |= TryWakeSleepingNeighbor(edge.rigidId0, edge.rigidId1);
                    changed |= TryWakeSleepingNeighbor(edge.rigidId1, edge.rigidId0);
                }
            }
            while (changed);
        }

        private bool TryWakeSleepingNeighbor(ulong sleepingRigidId, ulong neighborRigidId)
        {
            if (!rigids.TryGetValue(sleepingRigidId, out Rigid sleepingRigid)
                || !sleepingRigid.IsDynamic
                || !sleepingRigid.isSleeping
                || !rigids.TryGetValue(neighborRigidId, out Rigid neighbor)
                || !ShouldWakeSleepingNeighbor(neighbor))
            {
                return false;
            }

            sleepingRigid.WakeUp();
            rigids[sleepingRigidId] = sleepingRigid;
            return true;
        }

        private static bool ShouldWakeSleepingNeighbor(Rigid neighbor)
        {
            if (neighbor == null || !neighbor.enabled)
            {
                return false;
            }

            if (neighbor.IsDynamic)
            {
                return !neighbor.isSleeping;
            }

            return neighbor.IsKinematic && HasMotion(neighbor);
        }

        private static bool HasWakeState(Rigid rigid)
        {
            return HasMotion(rigid)
                || math.lengthsq(rigid.force) > math.Epsilon
                || math.lengthsq(rigid.torque) > math.Epsilon;
        }

        private static bool HasMotion(Rigid rigid)
        {
            return math.lengthsq(rigid.velocity) > math.Epsilon
                || math.lengthsq(rigid.angularVelocity) > math.Epsilon;
        }

        private void UpdateSleeping(fix deltaTime)
        {
            if (!settings.enableSleeping)
            {
                WakeAllSleepingBodies();
                return;
            }

            fix sleepDuration = math.max(fix.Zero, settings.sleepTime);
            for (int i = 0; i < islands.Count; i++)
            {
                PhysicsIsland island = islands[i];
                if (AllIslandBodiesSleeping(island))
                {
                    island.sleeping = true;
                    continue;
                }

                bool canSleep = CanIslandSleep(island);
                if (!canSleep)
                {
                    WakeIsland(island);
                    island.sleeping = false;
                    continue;
                }

                bool shouldSleep = true;
                for (int j = 0; j < island.RigidIds.Count; j++)
                {
                    ulong rigidId = island.RigidIds[j];
                    if (!rigids.TryGetValue(rigidId, out Rigid rigid) || !rigid.IsDynamic)
                    {
                        continue;
                    }

                    rigid.sleepTime += deltaTime;
                    if (rigid.sleepTime < sleepDuration)
                    {
                        shouldSleep = false;
                    }

                    rigids[rigidId] = rigid;
                }

                if (shouldSleep)
                {
                    SleepIsland(island);
                    island.sleeping = true;
                }
            }
        }

        private bool CanIslandSleep(PhysicsIsland island)
        {
            for (int i = 0; i < island.RigidIds.Count; i++)
            {
                ulong rigidId = island.RigidIds[i];
                if (!rigids.TryGetValue(rigidId, out Rigid rigid) || !rigid.IsDynamic)
                {
                    continue;
                }

                if (!CanRigidSleep(rigid))
                {
                    return false;
                }
            }

            return true;
        }

        private bool CanRigidSleep(Rigid rigid)
        {
            if (!rigid.allowSleep)
            {
                return false;
            }

            fix linearThreshold = math.max(fix.Zero, settings.linearSleepThreshold);
            fix angularThreshold = math.max(fix.Zero, settings.angularSleepThreshold);
            return math.lengthsq(rigid.velocity) <= linearThreshold * linearThreshold
                && math.lengthsq(rigid.angularVelocity) <= angularThreshold * angularThreshold
                && math.lengthsq(rigid.force) <= math.Epsilon
                && math.lengthsq(rigid.torque) <= math.Epsilon;
        }

        private bool AllIslandBodiesSleeping(PhysicsIsland island)
        {
            if (island.RigidIds.Count == 0)
            {
                return false;
            }

            for (int i = 0; i < island.RigidIds.Count; i++)
            {
                ulong rigidId = island.RigidIds[i];
                if (rigids.TryGetValue(rigidId, out Rigid rigid) && rigid.IsDynamic && !rigid.isSleeping)
                {
                    return false;
                }
            }

            return true;
        }

        private void SleepIsland(PhysicsIsland island)
        {
            for (int i = 0; i < island.RigidIds.Count; i++)
            {
                ulong rigidId = island.RigidIds[i];
                if (rigids.TryGetValue(rigidId, out Rigid rigid) && rigid.IsDynamic)
                {
                    rigid.Sleep();
                    rigids[rigidId] = rigid;
                }
            }
        }

        private void WakeIsland(PhysicsIsland island)
        {
            for (int i = 0; i < island.RigidIds.Count; i++)
            {
                WakeRigidInternal(island.RigidIds[i]);
            }
        }

        private void WakeAllSleepingBodies()
        {
            for (int i = 0; i < rigidIds.Count; i++)
            {
                ulong rigidId = rigidIds[i];
                if (rigids.TryGetValue(rigidId, out Rigid rigid) && rigid.isSleeping)
                {
                    rigid.WakeUp();
                    rigids[rigidId] = rigid;
                }
            }
        }

        private void WakeRigidInternal(ulong rigidId)
        {
            if (rigids.TryGetValue(rigidId, out Rigid rigid) && rigid.IsDynamic)
            {
                rigid.WakeUp();
                rigids[rigidId] = rigid;
            }
        }

        private static int GetConstraintTypeId(Constraint constraint)
        {
            return constraint == null ? 0 : (int)constraint.Type;
        }

        private static void HashConstraint(ref ulong hash, Constraint constraint)
        {
            switch (constraint)
            {
                case DistanceConstraint distanceConstraint:
                    Hash(ref hash, distanceConstraint.localAnchor0);
                    Hash(ref hash, distanceConstraint.localAnchor1);
                    Hash(ref hash, distanceConstraint.distance);
                    break;
                case PointConstraint pointConstraint:
                    Hash(ref hash, pointConstraint.localAnchor0);
                    Hash(ref hash, pointConstraint.localAnchor1);
                    break;
                case SpringDistanceConstraint springConstraint:
                    Hash(ref hash, springConstraint.localAnchor0);
                    Hash(ref hash, springConstraint.localAnchor1);
                    Hash(ref hash, springConstraint.restDistance);
                    Hash(ref hash, springConstraint.stiffness);
                    Hash(ref hash, springConstraint.damping);
                    break;
                case HingeConstraint hingeConstraint:
                    Hash(ref hash, hingeConstraint.localAnchor0);
                    Hash(ref hash, hingeConstraint.localAnchor1);
                    Hash(ref hash, hingeConstraint.localAxis0);
                    Hash(ref hash, hingeConstraint.localAxis1);
                    Hash(ref hash, hingeConstraint.angularStiffness);
                    Hash(ref hash, hingeConstraint.angularDamping);
                    break;
                case FixedConstraint fixedConstraint:
                    Hash(ref hash, fixedConstraint.localAnchor0);
                    Hash(ref hash, fixedConstraint.localAnchor1);
                    Hash(ref hash, fixedConstraint.targetLocalRotation);
                    Hash(ref hash, fixedConstraint.angularStiffness);
                    Hash(ref hash, fixedConstraint.angularDamping);
                    break;
                case SliderConstraint sliderConstraint:
                    Hash(ref hash, sliderConstraint.localAnchor0);
                    Hash(ref hash, sliderConstraint.localAnchor1);
                    Hash(ref hash, sliderConstraint.localAxis0);
                    Hash(ref hash, sliderConstraint.localAxis1);
                    Hash(ref hash, sliderConstraint.linearStiffness);
                    Hash(ref hash, sliderConstraint.linearDamping);
                    Hash(ref hash, sliderConstraint.angularStiffness);
                    Hash(ref hash, sliderConstraint.angularDamping);
                    break;
            }
        }

        private static void HashWorldSettings(ref ulong hash, WorldSettings worldSettings)
        {
            Hash(ref hash, worldSettings.enableGravity);
            Hash(ref hash, worldSettings.gravity);
            Hash(ref hash, worldSettings.timeStep);
            Hash(ref hash, worldSettings.restitution);
            Hash(ref hash, worldSettings.restitutionVelocityThreshold);
            Hash(ref hash, worldSettings.friction);
            Hash(ref hash, worldSettings.warmStartScale);
            Hash(ref hash, worldSettings.penetrationSlop);
            Hash(ref hash, worldSettings.positionCorrectionPercent);
            Hash(ref hash, worldSettings.maxPositionCorrection);
            Hash(ref hash, worldSettings.maxLinearVelocity);
            Hash(ref hash, worldSettings.maxAngularVelocity);
            Hash(ref hash, worldSettings.maxTranslationPerStep);
            Hash(ref hash, worldSettings.maxRotationPerStep);
            Hash(ref hash, worldSettings.maxContactImpulse);
            Hash(ref hash, worldSettings.maxFrictionImpulse);
            Hash(ref hash, worldSettings.solverIterations);
            Hash(ref hash, worldSettings.positionIterations);
            Hash(ref hash, worldSettings.enableSleeping);
            Hash(ref hash, worldSettings.linearSleepThreshold);
            Hash(ref hash, worldSettings.angularSleepThreshold);
            Hash(ref hash, worldSettings.sleepTime);
            Hash(ref hash, worldSettings.enableCCD);
            Hash(ref hash, worldSettings.enableDynamicCCD);
            Hash(ref hash, worldSettings.ccdMinVelocity);
            Hash(ref hash, worldSettings.ccdSkin);
            Hash(ref hash, worldSettings.ccdMaxIterations);
            Hash(ref hash, (int)worldSettings.narrowPhaseAlgorithm);
            Hash(ref hash, worldSettings.contactManifoldSettings.normalPersistenceDot);
            Hash(ref hash, worldSettings.contactManifoldSettings.anchorMatchDistance);
            Hash(ref hash, worldSettings.contactManifoldSettings.positionMatchDistance);
            Hash(ref hash, worldSettings.contactManifoldSettings.staleFrameLimit);
            Hash(ref hash, worldSettings.maxSubSteps);
            Hash(ref hash, worldSettings.deferLifecycleChangesDuringCallbacks);
            Hash(ref hash, worldSettings.catchCallbackExceptions);
        }

        private static void HashShape(ref ulong hash, Shape shape)
        {
            if (shape == null)
            {
                Hash(ref hash, 0);
                return;
            }

            Hash(ref hash, (int)shape.Type);
            switch (shape.Type)
            {
                case ShapeType.AABB:
                    AABB aabb = (AABB)shape;
                    Hash(ref hash, aabb.center);
                    Hash(ref hash, aabb.extents);
                    break;
                case ShapeType.OBB:
                    OBB obb = (OBB)shape;
                    Hash(ref hash, obb.center);
                    Hash(ref hash, obb.extents);
                    Hash(ref hash, obb.orientation);
                    break;
                case ShapeType.Sphere:
                    Sphere sphere = (Sphere)shape;
                    Hash(ref hash, sphere.Center);
                    Hash(ref hash, sphere.Radius);
                    break;
                case ShapeType.Capsule:
                    Capsule capsule = (Capsule)shape;
                    Hash(ref hash, capsule.Center);
                    Hash(ref hash, capsule.Radius);
                    Hash(ref hash, capsule.Height);
                    Hash(ref hash, capsule.Orientation);
                    break;
            }
        }

        private static void Hash(ref ulong hash, bool value)
        {
            Hash(ref hash, value ? 1UL : 0UL);
        }

        private static void Hash(ref ulong hash, int value)
        {
            Hash(ref hash, unchecked((ulong)value));
        }

        private static void Hash(ref ulong hash, ulong value)
        {
            unchecked
            {
                for (int i = 0; i < 8; i++)
                {
                    hash ^= value & 0xffUL;
                    hash *= HashPrime;
                    value >>= 8;
                }
            }
        }

        private static void Hash(ref ulong hash, fix value)
        {
            Hash(ref hash, unchecked((ulong)value.value));
        }

        private static void Hash(ref ulong hash, fix3 value)
        {
            Hash(ref hash, value.x);
            Hash(ref hash, value.y);
            Hash(ref hash, value.z);
        }

        private static void Hash(ref ulong hash, quaternion value)
        {
            Hash(ref hash, value.value.x);
            Hash(ref hash, value.value.y);
            Hash(ref hash, value.value.z);
            Hash(ref hash, value.value.w);
        }

        private void DispatchCollisionCallbacks()
        {
            BeginCollisionCallbackDispatch();
            try
            {
                nextCollisionEvents.Clear();
                IReadOnlyList<NarrowCollisionSystem.CollisionPair> pairs = collisionSystem.CollisionPairs;
                for (int i = 0; i < pairs.Count; i++)
                {
                    NarrowCollisionSystem.CollisionPair pair = pairs[i];
                    bool isTrigger = pair.collider0.isTrigger || pair.collider1.isTrigger;
                    CollisionEventState state = CreateCollisionEventState(pair, isTrigger);

                    if (activeCollisionEvents.TryGetValue(pair.key, out CollisionEventState previous))
                    {
                        if (previous.isTrigger != isTrigger)
                        {
                            DispatchCollisionEvent(previous, CollisionEventPhase.Exit);
                            DispatchCollisionEvent(state, CollisionEventPhase.Enter);
                        }
                        else
                        {
                            DispatchCollisionEvent(state, CollisionEventPhase.Stay);
                        }
                    }
                    else
                    {
                        DispatchCollisionEvent(state, CollisionEventPhase.Enter);
                    }

                    nextCollisionEvents[pair.key] = state;
                }

                foreach (KeyValuePair<BroadCollisionPairKey, CollisionEventState> item in activeCollisionEvents)
                {
                    if (!nextCollisionEvents.ContainsKey(item.Key))
                    {
                        DispatchCollisionEvent(item.Value, CollisionEventPhase.Exit);
                    }
                }

                activeCollisionEvents.Clear();
                foreach (KeyValuePair<BroadCollisionPairKey, CollisionEventState> item in nextCollisionEvents)
                {
                    activeCollisionEvents.Add(item.Key, item.Value);
                }

                nextCollisionEvents.Clear();
            }
            finally
            {
                EndCollisionCallbackDispatch();
            }
        }

        private bool QueueDeferredLifecycleOperation(DeferredLifecycleOperationKind kind, ulong targetId, bool enabled)
        {
            if (!settings.deferLifecycleChangesDuringCallbacks
                || collisionCallbackDispatchDepth <= 0)
            {
                return false;
            }

            DeferredLifecycleOperation operation = new DeferredLifecycleOperation(kind, targetId, enabled);
            for (int i = 0; i < deferredLifecycleOperations.Count; i++)
            {
                if (deferredLifecycleOperations[i].Equals(operation))
                {
                    return true;
                }
            }

            deferredLifecycleOperations.Add(operation);
            return true;
        }

        private void BeginCollisionCallbackDispatch()
        {
            collisionCallbackDispatchDepth++;
        }

        private void EndCollisionCallbackDispatch()
        {
            collisionCallbackDispatchDepth--;
            if (collisionCallbackDispatchDepth > 0)
            {
                return;
            }

            collisionCallbackDispatchDepth = 0;
            FlushDeferredLifecycleOperationsIfSafe();
        }

        private void BeginLifecycleOperation()
        {
            lifecycleOperationDepth++;
        }

        private void EndLifecycleOperation()
        {
            lifecycleOperationDepth--;
            if (lifecycleOperationDepth < 0)
            {
                lifecycleOperationDepth = 0;
            }

            FlushDeferredLifecycleOperationsIfSafe();
        }

        private void FlushDeferredLifecycleOperationsIfSafe()
        {
            if (collisionCallbackDispatchDepth == 0 && lifecycleOperationDepth == 0 && !isFlushingDeferredLifecycleOperations)
            {
                FlushDeferredLifecycleOperations();
            }
        }

        private void FlushDeferredLifecycleOperations()
        {
            if (deferredLifecycleOperations.Count == 0)
            {
                return;
            }

            isFlushingDeferredLifecycleOperations = true;
            try
            {
                for (int i = 0; i < deferredLifecycleOperations.Count; i++)
                {
                    DeferredLifecycleOperation operation = deferredLifecycleOperations[i];
                    switch (operation.kind)
                    {
                        case DeferredLifecycleOperationKind.SetRigidEnabled:
                            SetRigidEnabled(operation.targetId, operation.enabled);
                            break;
                        case DeferredLifecycleOperationKind.SetColliderEnabled:
                            SetColliderEnabled(operation.targetId, operation.enabled);
                            break;
                        case DeferredLifecycleOperationKind.SetConstraintEnabled:
                            SetConstraintEnabled(operation.targetId, operation.enabled);
                            break;
                        case DeferredLifecycleOperationKind.RemoveConstraint:
                            RemoveConstraint(operation.targetId);
                            break;
                        case DeferredLifecycleOperationKind.RemoveCollider:
                            RemoveCollider(operation.targetId);
                            break;
                        case DeferredLifecycleOperationKind.RemoveRigid:
                            RemoveRigid(operation.targetId);
                            break;
                    }
                }
            }
            finally
            {
                lastDeferredLifecycleOperationCount = deferredLifecycleOperations.Count;
                deferredLifecycleOperations.Clear();
                isFlushingDeferredLifecycleOperations = false;
            }
        }

        private CollisionEventState CreateCollisionEventState(NarrowCollisionSystem.CollisionPair pair, bool isTrigger)
        {
            CollisionInfo collision = pair.collision;
            collision.isTrigger = isTrigger;

            rigids.TryGetValue(pair.rigidId0, out Rigid rigid0);
            rigids.TryGetValue(pair.rigidId1, out Rigid rigid1);
            return new CollisionEventState(pair.collider0, pair.collider1, rigid0, rigid1, collision, isTrigger);
        }

        private void DispatchCollisionEvent(CollisionEventState state, CollisionEventPhase phase)
        {
            CollisionInfo collision0 = state.collision;
            collision0.isTrigger = state.isTrigger;
            CollisionInfo collision1 = collision0.Flipped();

            switch (phase)
            {
                case CollisionEventPhase.Enter:
                    DispatchColliderEnter(state.collider0, collision0);
                    DispatchColliderEnter(state.collider1, collision1);
                    DispatchRigidEnter(state.rigid0, collision0);
                    DispatchRigidEnter(state.rigid1, collision1);
                    break;
                case CollisionEventPhase.Stay:
                    DispatchColliderStay(state.collider0, collision0);
                    DispatchColliderStay(state.collider1, collision1);
                    DispatchRigidStay(state.rigid0, collision0);
                    DispatchRigidStay(state.rigid1, collision1);
                    break;
                case CollisionEventPhase.Exit:
                    DispatchColliderExit(state.collider0, collision0);
                    DispatchColliderExit(state.collider1, collision1);
                    DispatchRigidExit(state.rigid0, collision0);
                    DispatchRigidExit(state.rigid1, collision1);
                    break;
            }
        }

        private void DispatchColliderEnter(Collider collider, CollisionInfo collision)
        {
            if (collider == null)
            {
                return;
            }

            if (!settings.catchCallbackExceptions)
            {
                collider.DispatchCollisionEnter(collision);
                return;
            }

            try
            {
                collider.DispatchCollisionEnter(collision);
            }
            catch (System.Exception exception)
            {
                RecordCallbackException(exception);
            }
        }

        private void DispatchColliderStay(Collider collider, CollisionInfo collision)
        {
            if (collider == null)
            {
                return;
            }

            if (!settings.catchCallbackExceptions)
            {
                collider.DispatchCollisionStay(collision);
                return;
            }

            try
            {
                collider.DispatchCollisionStay(collision);
            }
            catch (System.Exception exception)
            {
                RecordCallbackException(exception);
            }
        }

        private void DispatchColliderExit(Collider collider, CollisionInfo collision)
        {
            if (collider == null)
            {
                return;
            }

            if (!settings.catchCallbackExceptions)
            {
                collider.DispatchCollisionExit(collision);
                return;
            }

            try
            {
                collider.DispatchCollisionExit(collision);
            }
            catch (System.Exception exception)
            {
                RecordCallbackException(exception);
            }
        }

        private void DispatchRigidEnter(Rigid rigid, CollisionInfo collision)
        {
            if (rigid == null)
            {
                return;
            }

            if (!settings.catchCallbackExceptions)
            {
                rigid.DispatchCollisionEnter(collision);
                return;
            }

            try
            {
                rigid.DispatchCollisionEnter(collision);
            }
            catch (System.Exception exception)
            {
                RecordCallbackException(exception);
            }
        }

        private void DispatchRigidStay(Rigid rigid, CollisionInfo collision)
        {
            if (rigid == null)
            {
                return;
            }

            if (!settings.catchCallbackExceptions)
            {
                rigid.DispatchCollisionStay(collision);
                return;
            }

            try
            {
                rigid.DispatchCollisionStay(collision);
            }
            catch (System.Exception exception)
            {
                RecordCallbackException(exception);
            }
        }

        private void DispatchRigidExit(Rigid rigid, CollisionInfo collision)
        {
            if (rigid == null)
            {
                return;
            }

            if (!settings.catchCallbackExceptions)
            {
                rigid.DispatchCollisionExit(collision);
                return;
            }

            try
            {
                rigid.DispatchCollisionExit(collision);
            }
            catch (System.Exception exception)
            {
                RecordCallbackException(exception);
            }
        }

        private void RecordCallbackException(System.Exception exception)
        {
            currentCallbackExceptionCount++;
            if (LastCallbackException == null)
            {
                LastCallbackException = exception;
            }
        }

        private void EndCollisionEventsForCollider(ulong colliderId)
        {
            BeginCollisionCallbackDispatch();
            try
            {
                collisionEventRemovalBuffer.Clear();
                foreach (KeyValuePair<BroadCollisionPairKey, CollisionEventState> item in activeCollisionEvents)
                {
                    if (item.Value.ContainsCollider(colliderId))
                    {
                        DispatchCollisionEvent(item.Value, CollisionEventPhase.Exit);
                        collisionEventRemovalBuffer.Add(item.Key);
                    }
                }

                for (int i = 0; i < collisionEventRemovalBuffer.Count; i++)
                {
                    activeCollisionEvents.Remove(collisionEventRemovalBuffer[i]);
                    nextCollisionEvents.Remove(collisionEventRemovalBuffer[i]);
                }

                collisionEventRemovalBuffer.Clear();
            }
            finally
            {
                EndCollisionCallbackDispatch();
            }
        }

        private void EndCollisionEventsForRigid(ulong rigidId)
        {
            BeginCollisionCallbackDispatch();
            try
            {
                collisionEventRemovalBuffer.Clear();
                foreach (KeyValuePair<BroadCollisionPairKey, CollisionEventState> item in activeCollisionEvents)
                {
                    if (item.Value.ContainsRigid(rigidId))
                    {
                        DispatchCollisionEvent(item.Value, CollisionEventPhase.Exit);
                        collisionEventRemovalBuffer.Add(item.Key);
                    }
                }

                for (int i = 0; i < collisionEventRemovalBuffer.Count; i++)
                {
                    activeCollisionEvents.Remove(collisionEventRemovalBuffer[i]);
                    nextCollisionEvents.Remove(collisionEventRemovalBuffer[i]);
                }

                collisionEventRemovalBuffer.Clear();
            }
            finally
            {
                EndCollisionCallbackDispatch();
            }
        }

        private bool RemoveColliderInternal(ulong colliderId, bool dispatchExit)
        {
            BeginLifecycleOperation();
            try
            {
                if (!colliders.TryGetValue(colliderId, out Collider collider))
                {
                    return false;
                }

                if (dispatchExit)
                {
                    EndCollisionEventsForCollider(colliderId);
                }

                colliders.Remove(colliderId);
                colliderIds.Remove(colliderId);
                collisionSystem.RemoveCollider(colliderId);

                if (rigids.TryGetValue(collider.rigidId, out Rigid rigid))
                {
                    rigid.RemoveCollider(colliderId);
                    rigid.WakeUp();
                    rigids[rigid.id] = rigid;
                    ApplyRigidMassProperties(rigid.id);
                }

                return true;
            }
            finally
            {
                EndLifecycleOperation();
            }
        }

        private void RemoveRigidCollidersFromCollisionSystem(ulong rigidId)
        {
            for (int i = 0; i < colliderIds.Count; i++)
            {
                ulong colliderId = colliderIds[i];
                if (colliders.TryGetValue(colliderId, out Collider collider) && collider.rigidId == rigidId)
                {
                    collisionSystem.RemoveCollider(colliderId);
                }
            }
        }

        private void RemoveConstraintsForRigid(ulong rigidId)
        {
            constraintRemovalBuffer.Clear();
            for (int i = 0; i < constraintIds.Count; i++)
            {
                ulong currentConstraintId = constraintIds[i];
                if (constraints.TryGetValue(currentConstraintId, out Constraint constraint) && constraint.ContainsRigid(rigidId))
                {
                    constraintRemovalBuffer.Add(currentConstraintId);
                }
            }

            for (int i = 0; i < constraintRemovalBuffer.Count; i++)
            {
                RemoveConstraint(constraintRemovalBuffer[i]);
            }

            constraintRemovalBuffer.Clear();
        }

        private void ClearConstraintWarmStartForRigid(ulong rigidId)
        {
            for (int i = 0; i < constraintIds.Count; i++)
            {
                ulong currentConstraintId = constraintIds[i];
                if (constraints.TryGetValue(currentConstraintId, out Constraint constraint) && constraint.ContainsRigid(rigidId))
                {
                    constraint.ClearWarmStart();
                }
            }
        }

        private bool TryComputeCurrentAnchorDistance(ulong rigidId0, ulong rigidId1, fix3 localAnchor0, fix3 localAnchor1, out fix distance)
        {
            distance = fix.Zero;
            if (rigidId0 == rigidId1
                || !entities.TryGetValue(rigidId0, out Entity entity0)
                || !entities.TryGetValue(rigidId1, out Entity entity1))
            {
                return false;
            }

            fix3 worldAnchor0 = entity0.translation + entity0.orientation * localAnchor0;
            fix3 worldAnchor1 = entity1.translation + entity1.orientation * localAnchor1;
            distance = math.distance(worldAnchor0, worldAnchor1);
            return true;
        }

        private bool TryComputeCurrentRelativeOrientation(ulong rigidId0, ulong rigidId1, out quaternion targetLocalRotation)
        {
            targetLocalRotation = quaternion.identity;
            if (rigidId0 == rigidId1
                || !entities.TryGetValue(rigidId0, out Entity entity0)
                || !entities.TryGetValue(rigidId1, out Entity entity1))
            {
                return false;
            }

            targetLocalRotation = quaternion.normalize(quaternion.conjugate(entity0.orientation) * entity1.orientation);
            return true;
        }

        private IReadOnlyList<Collider> GetActiveColliders()
        {
            activeColliders.Clear();
            for (int i = 0; i < colliderIds.Count; i++)
            {
                ulong colliderId = colliderIds[i];
                if (colliders.TryGetValue(colliderId, out Collider collider) && IsColliderActive(collider))
                {
                    activeColliders.Add(collider);
                }
            }

            return activeColliders;
        }

        private bool IsColliderActive(Collider collider)
        {
            if (collider == null || !collider.enabled || collider.shape == null)
            {
                return false;
            }

            return !rigids.TryGetValue(collider.rigidId, out Rigid rigid) || rigid.enabled;
        }

        private enum CollisionEventPhase
        {
            Enter,
            Stay,
            Exit
        }

        private enum DeferredLifecycleOperationKind
        {
            SetRigidEnabled,
            SetColliderEnabled,
            SetConstraintEnabled,
            RemoveConstraint,
            RemoveCollider,
            RemoveRigid
        }

        private readonly struct DeferredLifecycleOperation
        {
            public readonly DeferredLifecycleOperationKind kind;
            public readonly ulong targetId;
            public readonly bool enabled;

            public DeferredLifecycleOperation(DeferredLifecycleOperationKind kind, ulong targetId, bool enabled)
            {
                this.kind = kind;
                this.targetId = targetId;
                this.enabled = enabled;
            }

            public bool Equals(DeferredLifecycleOperation other)
            {
                return kind == other.kind && targetId == other.targetId && enabled == other.enabled;
            }
        }

        private readonly struct IslandEdge
        {
            public readonly ulong rigidId0;
            public readonly ulong rigidId1;

            public IslandEdge(ulong rigidId0, ulong rigidId1)
            {
                this.rigidId0 = rigidId0;
                this.rigidId1 = rigidId1;
            }
        }

        private readonly struct ContinuousHit
        {
            public readonly fix fraction;
            public readonly fix3 normal;
            public readonly ulong colliderId;

            public ContinuousHit(fix fraction, fix3 normal, ulong colliderId)
            {
                this.fraction = fraction;
                this.normal = normal;
                this.colliderId = colliderId;
            }
        }

        private readonly struct CollisionEventState
        {
            public readonly Collider collider0;
            public readonly Collider collider1;
            public readonly Rigid rigid0;
            public readonly Rigid rigid1;
            public readonly CollisionInfo collision;
            public readonly bool isTrigger;

            public CollisionEventState(Collider collider0, Collider collider1, Rigid rigid0, Rigid rigid1, CollisionInfo collision, bool isTrigger)
            {
                this.collider0 = collider0;
                this.collider1 = collider1;
                this.rigid0 = rigid0;
                this.rigid1 = rigid1;
                this.collision = collision;
                this.isTrigger = isTrigger;
            }

            public bool ContainsCollider(ulong colliderId)
            {
                return (collider0 != null && collider0.id == colliderId)
                    || (collider1 != null && collider1.id == colliderId);
            }

            public bool ContainsRigid(ulong rigidId)
            {
                return (rigid0 != null && rigid0.id == rigidId)
                    || (rigid1 != null && rigid1.id == rigidId);
            }
        }

        private Collider RegisterCollider(Collider collider)
        {
            colliders[collider.id] = collider;
            if (!colliderIds.Contains(collider.id))
            {
                colliderIds.Add(collider.id);
            }

            if (rigids.TryGetValue(collider.rigidId, out Rigid rigid))
            {
                rigid.AddCollider(collider.id);
                rigid.WakeUp();
                rigids[rigid.id] = rigid;

                if (entities.TryGetValue(rigid.id, out Entity entity))
                {
                    collider.SyncTransform(entity.translation, entity.orientation);
                    colliders[collider.id] = collider;
                }

                ApplyRigidMassProperties(rigid.id);
            }

            return collider;
        }

        private void ApplyRigidMassProperties(ulong rigidId)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return;
            }

            if (!rigid.autoMass && !rigid.autoInertia)
            {
                return;
            }

            fix totalMass = fix.Zero;
            fix3 totalInertia = fix3.zero;
            bool hasMassProperties = false;

            for (int i = 0; i < colliderIds.Count; i++)
            {
                ulong colliderId = colliderIds[i];
                if (!colliders.TryGetValue(colliderId, out Collider collider)
                    || collider.rigidId != rigidId
                    || collider.shape == null)
                {
                    continue;
                }

                MassProperties massProperties = Physics.ComputeMassProperties(collider.shape, collider.material.GetDensity());
                if (massProperties.mass <= fix.Zero)
                {
                    continue;
                }

                hasMassProperties = true;
                totalMass += massProperties.mass;
                totalInertia += ApplyParallelAxis(massProperties, collider.localCenter);
            }

            if (!hasMassProperties)
            {
                return;
            }

            if (rigid.autoMass)
            {
                rigid.mass = totalMass;
            }

            if (rigid.autoInertia)
            {
                rigid.inertia = totalInertia;
            }

            rigids[rigid.id] = rigid;
        }

        private static fix3 ApplyParallelAxis(MassProperties massProperties, fix3 offset)
        {
            fix xSq = offset.x * offset.x;
            fix ySq = offset.y * offset.y;
            fix zSq = offset.z * offset.z;
            return massProperties.inertia + new fix3(
                massProperties.mass * (ySq + zSq),
                massProperties.mass * (xSq + zSq),
                massProperties.mass * (xSq + ySq));
        }

        private void SyncColliders()
        {
            for (int i = 0; i < colliderIds.Count; i++)
            {
                ulong colliderId = colliderIds[i];
                if (!colliders.TryGetValue(colliderId, out Collider collider))
                {
                    continue;
                }

                if (!entities.TryGetValue(collider.rigidId, out Entity entity))
                {
                    continue;
                }

                collider.SyncTransform(entity.translation, entity.orientation);
                colliders[colliderId] = collider;
            }
        }

        private void SyncRigidColliders(ulong rigidId)
        {
            if (!entities.TryGetValue(rigidId, out Entity entity)
                || !rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return;
            }

            for (int i = 0; i < rigid.ColliderIds.Count; i++)
            {
                ulong colliderId = rigid.ColliderIds[i];
                if (!colliders.TryGetValue(colliderId, out Collider collider))
                {
                    continue;
                }

                collider.SyncTransform(entity.translation, entity.orientation);
                colliders[colliderId] = collider;
            }
        }
    }
}
