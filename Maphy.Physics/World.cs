using Maphy.Mathematics;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public class World
    {
        private ulong id = 10000000;
        private ulong constraintId = 20000000;
        public WorldSettings settings;
        private readonly Dictionary<ulong, Entity> entities;
        private readonly Dictionary<ulong, Rigid> rigids = new Dictionary<ulong, Rigid>();
        private readonly Dictionary<ulong, Collider> colliders = new Dictionary<ulong, Collider>();
        private readonly Dictionary<ulong, Constraint> constraints = new Dictionary<ulong, Constraint>();
        private readonly List<ulong> rigidIds = new List<ulong>();
        private readonly List<ulong> colliderIds = new List<ulong>();
        private readonly List<ulong> constraintIds = new List<ulong>();
        private readonly List<Collider> activeColliders = new List<Collider>();
        private readonly List<ulong> colliderRemovalBuffer = new List<ulong>();
        private readonly List<ulong> constraintRemovalBuffer = new List<ulong>();
        private readonly List<BroadCollisionPairKey> collisionEventRemovalBuffer = new List<BroadCollisionPairKey>();
        private readonly Dictionary<BroadCollisionPairKey, CollisionEventState> activeCollisionEvents = new Dictionary<BroadCollisionPairKey, CollisionEventState>();
        private readonly Dictionary<BroadCollisionPairKey, CollisionEventState> nextCollisionEvents = new Dictionary<BroadCollisionPairKey, CollisionEventState>();
        private readonly CollisionSystem collisionSystem = new CollisionSystem();

        public IReadOnlyDictionary<ulong, Entity> Entities => entities;
        public IReadOnlyDictionary<ulong, Rigid> Rigids => rigids;
        public IReadOnlyDictionary<ulong, Collider> Colliders => colliders;
        public IReadOnlyDictionary<ulong, Constraint> Constraints => constraints;
        public IReadOnlyList<BroadCollisionPair> BroadphasePairs => collisionSystem.BroadphasePairs;
        public IReadOnlyList<NarrowCollisionSystem.CollisionPair> CollisionPairs => collisionSystem.CollisionPairs;
        public IReadOnlyList<ContactManifold> ContactManifolds => collisionSystem.ContactManifolds;

        public World()
            : this(WorldSettings.Default)
        {
        }

        public World(WorldSettings worldSettings)
        {
            settings = worldSettings;
            entities = new Dictionary<ulong, Entity>();
        }

        public void Update()
        {
            Update(settings.timeStep);
        }

        public void Update(fix deltaTime)
        {
            if (deltaTime < fix.Zero)
            {
                deltaTime = fix.Zero;
            }

            IntegrateForces(deltaTime);
            IntegrateTransforms(deltaTime);
            SyncColliders();
            collisionSystem.Collision(GetActiveColliders());
            ResolveContacts();
            CorrectPositions();
            CorrectConstraints();
            SyncColliders();
            DispatchCollisionCallbacks();
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

        public bool SetTransform(ulong entityId, fix3 translation, quaternion orientation)
        {
            if (!entities.TryGetValue(entityId, out Entity entity))
            {
                return false;
            }

            entity.SetTransform(translation, orientation);
            entities[entityId] = entity;
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
            return true;
        }

        public bool SetRigidType(ulong rigidId, RigidType type)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.type = type;
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

            rigid.SetEnabled(enabled);
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

        public bool SetMass(ulong rigidId, fix mass)
        {
            if (mass < fix.Zero || !rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.mass = mass;
            rigid.autoMass = false;
            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetVelocity(ulong rigidId, fix3 velocity)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.velocity = velocity;
            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetAngularVelocity(ulong rigidId, fix3 angularVelocity)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.angularVelocity = angularVelocity;
            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetAcceleration(ulong rigidId, fix3 acceleration)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.acceleration = acceleration;
            rigids[rigidId] = rigid;
            return true;
        }

        public bool SetAngularAcceleration(ulong rigidId, fix3 angularAcceleration)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.angularAcceleration = angularAcceleration;
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
            rigids[rigidId] = rigid;
            return true;
        }

        public bool AddForce(ulong rigidId, fix3 force)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.AddForce(force);
            rigids[rigidId] = rigid;
            return true;
        }

        public bool AddTorque(ulong rigidId, fix3 torque)
        {
            if (!rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.AddTorque(torque);
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

            collider.SetEnabled(enabled);
            if (!enabled)
            {
                EndCollisionEventsForCollider(colliderId);
                collisionSystem.RemoveCollider(colliderId);
            }

            return true;
        }

        public bool SetColliderMaterial(ulong colliderId, Material material)
        {
            if (!colliders.TryGetValue(colliderId, out Collider collider))
            {
                return false;
            }

            collider.SetMaterial(material);
            ApplyColliderMassProperties(collider);
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
            ApplyColliderMassProperties(collider);
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
            constraints.Add(constraint.id, constraint);
            constraintIds.Add(constraint.id);
            return constraint;
        }

        public bool SetConstraintEnabled(ulong constraintId, bool enabled)
        {
            if (!constraints.TryGetValue(constraintId, out Constraint constraint))
            {
                return false;
            }

            constraint.SetEnabled(enabled);
            return true;
        }

        public bool RemoveConstraint(ulong constraintId)
        {
            if (!constraints.Remove(constraintId))
            {
                return false;
            }

            constraintIds.Remove(constraintId);
            return true;
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
            return RemoveColliderInternal(colliderId, true);
        }

        public bool RemoveRigid(ulong rigidId)
        {
            if (!rigids.ContainsKey(rigidId))
            {
                return false;
            }

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

        public bool TestCollision(Collider a, Collider b)
        {
            return collisionSystem.TestCollision(a, b);
        }

        public bool TryGetCollision(Collider a, Collider b, out CollisionInfo collision)
        {
            return collisionSystem.TryGetCollision(a, b, out collision);
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

                if (!rigid.IsDynamic)
                {
                    rigid.ClearForces();
                    rigid.ClearTorques();
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

                if (!rigid.IsDynamic && !rigid.IsKinematic)
                {
                    continue;
                }

                if (!entities.TryGetValue(rigidId, out Entity entity))
                {
                    continue;
                }

                entity.translation += rigid.velocity * deltaTime;
                entity.orientation = IntegrateOrientation(entity.orientation, rigid.angularVelocity, deltaTime);
                entities[rigidId] = entity;
            }
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

        private void ResolveContacts()
        {
            int solverIterations = settings.solverIterations > 0 ? settings.solverIterations : 1;
            IReadOnlyList<ContactManifold> manifolds = collisionSystem.ContactManifolds;
            WarmStartContacts(manifolds);
            WarmStartConstraints();
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
                        ResolveContactVelocity(manifold, ref point);
                    }
                }

                ResolveConstraintsVelocity();
            }
        }

        private void WarmStartContacts(IReadOnlyList<ContactManifold> manifolds)
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
                    ContactPoint point = manifold[j];
                    if (point.normalImpulse <= fix.Zero
                        && math.abs(point.tangentImpulse0) <= math.Epsilon
                        && math.abs(point.tangentImpulse1) <= math.Epsilon)
                    {
                        continue;
                    }

                    bool hasRigid0 = TryGetRigidMassData(manifold.rigidId0, out Rigid rigid0, out fix inverseMass0);
                    bool hasRigid1 = TryGetRigidMassData(manifold.rigidId1, out Rigid rigid1, out fix inverseMass1);
                    if (!hasRigid0 && !hasRigid1)
                    {
                        continue;
                    }

                    fix3 r0 = hasRigid0 ? point.position - GetEntityPosition(rigid0.id) : fix3.zero;
                    fix3 r1 = hasRigid1 ? point.position - GetEntityPosition(rigid1.id) : fix3.zero;
                    fix3 impulse = manifold.normal * point.normalImpulse;
                    if (math.lengthsq(point.tangent0) > math.Epsilon)
                    {
                        impulse += point.tangent0 * point.tangentImpulse0;
                    }

                    if (math.lengthsq(point.tangent1) > math.Epsilon)
                    {
                        impulse += point.tangent1 * point.tangentImpulse1;
                    }

                    ApplyImpulse(hasRigid0, rigid0, inverseMass0, r0, hasRigid1, rigid1, inverseMass1, r1, impulse);
                }
            }
        }

        private void WarmStartConstraints()
        {
            for (int i = 0; i < constraintIds.Count; i++)
            {
                ulong currentConstraintId = constraintIds[i];
                if (!constraints.TryGetValue(currentConstraintId, out Constraint constraint)
                    || constraint.accumulatedImpulse == fix.Zero)
                {
                    continue;
                }

                if (constraint is DistanceConstraint distanceConstraint
                    && TryGetDistanceConstraintSolveData(distanceConstraint, out DistanceConstraintSolveData data))
                {
                    ApplyImpulse(
                        true,
                        data.rigid0,
                        data.inverseMass0,
                        data.relativeAnchor0,
                        true,
                        data.rigid1,
                        data.inverseMass1,
                        data.relativeAnchor1,
                        data.axis * distanceConstraint.accumulatedImpulse);
                }
                else
                {
                    constraint.ClearWarmStart();
                }
            }
        }

        private void ResolveConstraintsVelocity()
        {
            for (int i = 0; i < constraintIds.Count; i++)
            {
                ulong currentConstraintId = constraintIds[i];
                if (!constraints.TryGetValue(currentConstraintId, out Constraint constraint))
                {
                    continue;
                }

                if (constraint is DistanceConstraint distanceConstraint)
                {
                    ResolveDistanceConstraintVelocity(distanceConstraint);
                }
            }
        }

        private void ResolveDistanceConstraintVelocity(DistanceConstraint constraint)
        {
            if (!TryGetDistanceConstraintSolveData(constraint, out DistanceConstraintSolveData data))
            {
                constraint.ClearWarmStart();
                return;
            }

            fix effectiveMass = data.inverseMass0 + data.inverseMass1
                + GetAngularEffectiveMass(data.rigid0, data.relativeAnchor0, data.axis)
                + GetAngularEffectiveMass(data.rigid1, data.relativeAnchor1, data.axis);
            if (effectiveMass <= fix.Zero)
            {
                constraint.ClearWarmStart();
                return;
            }

            fix3 velocity0 = GetVelocityAtPoint(data.rigid0, data.relativeAnchor0);
            fix3 velocity1 = GetVelocityAtPoint(data.rigid1, data.relativeAnchor1);
            fix constraintVelocity = math.dot(velocity1 - velocity0, data.axis);
            fix impulseDelta = -constraintVelocity / effectiveMass;
            constraint.accumulatedImpulse += impulseDelta;

            ApplyImpulse(
                true,
                data.rigid0,
                data.inverseMass0,
                data.relativeAnchor0,
                true,
                data.rigid1,
                data.inverseMass1,
                data.relativeAnchor1,
                data.axis * impulseDelta);
        }

        private void ResolveContactVelocity(ContactManifold manifold, ref ContactPoint point)
        {
            bool hasRigid0 = TryGetRigidMassData(manifold.rigidId0, out Rigid rigid0, out fix inverseMass0);
            bool hasRigid1 = TryGetRigidMassData(manifold.rigidId1, out Rigid rigid1, out fix inverseMass1);
            if (!hasRigid0 && !hasRigid1)
            {
                return;
            }

            fix3 r0 = hasRigid0 ? point.position - GetEntityPosition(rigid0.id) : fix3.zero;
            fix3 r1 = hasRigid1 ? point.position - GetEntityPosition(rigid1.id) : fix3.zero;
            fix3 velocity0 = hasRigid0 ? GetVelocityAtPoint(rigid0, r0) : fix3.zero;
            fix3 velocity1 = hasRigid1 ? GetVelocityAtPoint(rigid1, r1) : fix3.zero;
            fix3 relativeVelocity = velocity1 - velocity0;
            fix normalVelocity = math.dot(relativeVelocity, manifold.normal);
            if (normalVelocity > fix.Zero && point.lifetime <= 1)
            {
                return;
            }

            fix effectiveMass = inverseMass0 + inverseMass1
                + GetAngularEffectiveMass(rigid0, r0, manifold.normal)
                + GetAngularEffectiveMass(rigid1, r1, manifold.normal);
            if (effectiveMass <= fix.Zero)
            {
                return;
            }

            fix restitution = normalVelocity < -math.Epsilon ? GetCombinedRestitution(manifold) : fix.Zero;
            fix normalImpulseDelta = -(fix.One + restitution) * normalVelocity / effectiveMass;
            fix previousNormalImpulse = point.normalImpulse;
            point.normalImpulse = math.max(fix.Zero, previousNormalImpulse + normalImpulseDelta);
            fix normalImpulseMagnitude = point.normalImpulse - previousNormalImpulse;
            fix3 impulse = manifold.normal * normalImpulseMagnitude;

            ApplyImpulse(hasRigid0, rigid0, inverseMass0, r0, hasRigid1, rigid1, inverseMass1, r1, impulse);

            ResolveContactFriction(manifold, ref point, hasRigid0, rigid0, inverseMass0, r0, hasRigid1, rigid1, inverseMass1, r1);

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
            fix3 r1)
        {
            fix friction = GetCombinedFriction(manifold);
            if (friction <= fix.Zero || point.normalImpulse <= fix.Zero)
            {
                return;
            }

            fix3 velocity0 = hasRigid0 ? GetVelocityAtPoint(rigid0, r0) : fix3.zero;
            fix3 velocity1 = hasRigid1 ? GetVelocityAtPoint(rigid1, r1) : fix3.zero;
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
                + GetAngularEffectiveMass(rigid0, r0, tangent)
                + GetAngularEffectiveMass(rigid1, r1, tangent);
            if (effectiveMass <= fix.Zero)
            {
                return;
            }

            fix frictionDelta = -math.dot(relativeVelocity, tangent) / effectiveMass;
            fix maxFriction = point.normalImpulse * friction;
            fix previousTangentImpulse = point.tangentImpulse0;
            point.tangentImpulse0 = math.clamp(previousTangentImpulse + frictionDelta, -maxFriction, maxFriction);
            fix frictionMagnitude = point.tangentImpulse0 - previousTangentImpulse;
            fix3 frictionImpulse = tangent * frictionMagnitude;

            ApplyImpulse(hasRigid0, rigid0, inverseMass0, r0, hasRigid1, rigid1, inverseMass1, r1, frictionImpulse);
        }

        private void ApplyImpulse(
            bool hasRigid0,
            Rigid rigid0,
            fix inverseMass0,
            fix3 r0,
            bool hasRigid1,
            Rigid rigid1,
            fix inverseMass1,
            fix3 r1,
            fix3 impulse)
        {
            if (impulse == fix3.zero)
            {
                return;
            }

            if (hasRigid0 && inverseMass0 > fix.Zero)
            {
                rigid0.velocity -= impulse * inverseMass0;
                rigid0.angularVelocity -= rigid0.inverseInertia * math.cross(r0, impulse);
                rigids[rigid0.id] = rigid0;
            }

            if (hasRigid1 && inverseMass1 > fix.Zero)
            {
                rigid1.velocity += impulse * inverseMass1;
                rigid1.angularVelocity += rigid1.inverseInertia * math.cross(r1, impulse);
                rigids[rigid1.id] = rigid1;
            }
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

        private fix3 GetEntityPosition(ulong entityId)
        {
            return entities.TryGetValue(entityId, out Entity entity) ? entity.translation : fix3.zero;
        }

        private static fix3 GetVelocityAtPoint(Rigid rigid, fix3 relativePoint)
        {
            return rigid.velocity + math.cross(rigid.angularVelocity, relativePoint);
        }

        private static fix GetAngularEffectiveMass(Rigid rigid, fix3 relativePoint, fix3 direction)
        {
            if (rigid == null || !rigid.IsDynamic)
            {
                return fix.Zero;
            }

            fix3 angularVelocityPerImpulse = rigid.inverseInertia * math.cross(relativePoint, direction);
            return math.dot(math.cross(angularVelocityPerImpulse, relativePoint), direction);
        }

        private void CorrectPositions()
        {
            IReadOnlyList<ContactManifold> manifolds = collisionSystem.ContactManifolds;
            for (int i = 0; i < manifolds.Count; i++)
            {
                ContactManifold manifold = manifolds[i];
                if (manifold.isTrigger)
                {
                    continue;
                }

                bool hasRigid0 = TryGetRigidMassData(manifold.rigidId0, out Rigid rigid0, out fix inverseMass0);
                bool hasRigid1 = TryGetRigidMassData(manifold.rigidId1, out Rigid rigid1, out fix inverseMass1);
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

                    fix3 correction = manifold.normal * (penetration * settings.positionCorrectionPercent / inverseMassSum / manifold.contactCount);
                    if (hasRigid0 && inverseMass0 > fix.Zero)
                    {
                        TranslateEntity(rigid0.id, -correction * inverseMass0);
                    }

                    if (hasRigid1 && inverseMass1 > fix.Zero)
                    {
                        TranslateEntity(rigid1.id, correction * inverseMass1);
                    }
                }
            }
        }

        private void CorrectConstraints()
        {
            for (int i = 0; i < constraintIds.Count; i++)
            {
                ulong currentConstraintId = constraintIds[i];
                if (!constraints.TryGetValue(currentConstraintId, out Constraint constraint))
                {
                    continue;
                }

                if (constraint is DistanceConstraint distanceConstraint)
                {
                    CorrectDistanceConstraint(distanceConstraint);
                }
            }
        }

        private void CorrectDistanceConstraint(DistanceConstraint constraint)
        {
            if (!TryGetDistanceConstraintSolveData(constraint, out DistanceConstraintSolveData data))
            {
                constraint.ClearWarmStart();
                return;
            }

            fix error = data.currentDistance - constraint.distance;
            if (math.abs(error) <= math.Epsilon)
            {
                return;
            }

            fix inverseMassSum = data.inverseMass0 + data.inverseMass1;
            if (inverseMassSum <= fix.Zero)
            {
                constraint.ClearWarmStart();
                return;
            }

            fix3 correction = data.axis * (error / inverseMassSum);
            if (data.inverseMass0 > fix.Zero)
            {
                TranslateEntity(data.rigid0.id, correction * data.inverseMass0);
            }

            if (data.inverseMass1 > fix.Zero)
            {
                TranslateEntity(data.rigid1.id, -correction * data.inverseMass1);
            }
        }

        private bool TryGetDistanceConstraintSolveData(DistanceConstraint constraint, out DistanceConstraintSolveData data)
        {
            data = default;
            if (!IsConstraintActive(constraint)
                || !rigids.TryGetValue(constraint.rigidId0, out Rigid rigid0)
                || !rigids.TryGetValue(constraint.rigidId1, out Rigid rigid1)
                || !entities.TryGetValue(constraint.rigidId0, out Entity entity0)
                || !entities.TryGetValue(constraint.rigidId1, out Entity entity1))
            {
                return false;
            }

            fix inverseMass0 = rigid0.inverseMass;
            fix inverseMass1 = rigid1.inverseMass;
            if (inverseMass0 + inverseMass1 <= fix.Zero)
            {
                return false;
            }

            fix3 relativeAnchor0 = entity0.orientation * constraint.localAnchor0;
            fix3 relativeAnchor1 = entity1.orientation * constraint.localAnchor1;
            fix3 worldAnchor0 = entity0.translation + relativeAnchor0;
            fix3 worldAnchor1 = entity1.translation + relativeAnchor1;
            fix3 delta = worldAnchor1 - worldAnchor0;
            fix distanceSq = math.lengthsq(delta);
            if (distanceSq <= math.Epsilon)
            {
                return false;
            }

            fix currentDistance = math.sqrt(distanceSq);
            data = new DistanceConstraintSolveData(
                rigid0,
                rigid1,
                inverseMass0,
                inverseMass1,
                relativeAnchor0,
                relativeAnchor1,
                delta / currentDistance,
                currentDistance);
            return true;
        }

        private bool IsConstraintActive(Constraint constraint)
        {
            if (constraint == null || !constraint.enabled)
            {
                return false;
            }

            return rigids.TryGetValue(constraint.rigidId0, out Rigid rigid0)
                && rigids.TryGetValue(constraint.rigidId1, out Rigid rigid1)
                && rigid0.enabled
                && rigid1.enabled;
        }

        private bool TryGetRigidMassData(ulong rigidId, out Rigid rigid, out fix inverseMass)
        {
            if (rigids.TryGetValue(rigidId, out rigid))
            {
                inverseMass = rigid.inverseMass;
                return true;
            }

            rigid = default;
            inverseMass = fix.Zero;
            return false;
        }

        private void TranslateEntity(ulong entityId, fix3 delta)
        {
            if (!entities.TryGetValue(entityId, out Entity entity))
            {
                return;
            }

            entity.translation += delta;
            entities[entityId] = entity;
        }

        private void DispatchCollisionCallbacks()
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
                    state.collider0?.DispatchCollisionEnter(collision0);
                    state.collider1?.DispatchCollisionEnter(collision1);
                    state.rigid0?.DispatchCollisionEnter(collision0);
                    state.rigid1?.DispatchCollisionEnter(collision1);
                    break;
                case CollisionEventPhase.Stay:
                    state.collider0?.DispatchCollisionStay(collision0);
                    state.collider1?.DispatchCollisionStay(collision1);
                    state.rigid0?.DispatchCollisionStay(collision0);
                    state.rigid1?.DispatchCollisionStay(collision1);
                    break;
                case CollisionEventPhase.Exit:
                    state.collider0?.DispatchCollisionExit(collision0);
                    state.collider1?.DispatchCollisionExit(collision1);
                    state.rigid0?.DispatchCollisionExit(collision0);
                    state.rigid1?.DispatchCollisionExit(collision1);
                    break;
            }
        }

        private void EndCollisionEventsForCollider(ulong colliderId)
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

        private void EndCollisionEventsForRigid(ulong rigidId)
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

        private bool RemoveColliderInternal(ulong colliderId, bool dispatchExit)
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

            if (rigids.TryGetValue(collider.rigidId, out Rigid rigid) && rigid.colliderId == colliderId)
            {
                rigid.SetCollider(0);
                rigids[rigid.id] = rigid;
            }

            return true;
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

        private readonly struct DistanceConstraintSolveData
        {
            public readonly Rigid rigid0;
            public readonly Rigid rigid1;
            public readonly fix inverseMass0;
            public readonly fix inverseMass1;
            public readonly fix3 relativeAnchor0;
            public readonly fix3 relativeAnchor1;
            public readonly fix3 axis;
            public readonly fix currentDistance;

            public DistanceConstraintSolveData(
                Rigid rigid0,
                Rigid rigid1,
                fix inverseMass0,
                fix inverseMass1,
                fix3 relativeAnchor0,
                fix3 relativeAnchor1,
                fix3 axis,
                fix currentDistance)
            {
                this.rigid0 = rigid0;
                this.rigid1 = rigid1;
                this.inverseMass0 = inverseMass0;
                this.inverseMass1 = inverseMass1;
                this.relativeAnchor0 = relativeAnchor0;
                this.relativeAnchor1 = relativeAnchor1;
                this.axis = axis;
                this.currentDistance = currentDistance;
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
                rigid.SetCollider(collider.id);
                rigids[rigid.id] = rigid;

                if (entities.TryGetValue(rigid.id, out Entity entity))
                {
                    collider.SyncTransform(entity.translation, entity.orientation);
                    colliders[collider.id] = collider;
                }

                ApplyColliderMassProperties(collider);
            }

            return collider;
        }

        private void ApplyColliderMassProperties(Collider collider)
        {
            if (collider.shape == null || !rigids.TryGetValue(collider.rigidId, out Rigid rigid))
            {
                return;
            }

            if (!rigid.autoMass && !rigid.autoInertia)
            {
                return;
            }

            MassProperties massProperties = Physics.ComputeMassProperties(collider.shape, collider.material.GetDensity());
            if (rigid.autoMass)
            {
                rigid.mass = massProperties.mass;
            }

            if (rigid.autoInertia)
            {
                rigid.inertia = massProperties.inertia;
            }

            rigids[rigid.id] = rigid;
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
    }
}
