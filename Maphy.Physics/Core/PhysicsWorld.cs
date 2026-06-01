using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    /// <summary>
    /// Array-backed physics world for hot deterministic paths. It is intentionally kept
    /// independent from the object-friendly World class, so the two layers can be read
    /// side by side while the core gradually absorbs solver and narrowphase behavior.
    /// </summary>
    public sealed class PhysicsWorld
    {
        private const ulong HashOffsetBasis = 14695981039346656037UL;
        private const ulong HashPrime = 1099511628211UL;

        private PhysicsWorldSettings settings;
        private PhysicsWorldBody[] bodies;
        private int[] bodyVersions;
        private bool[] bodyAlive;
        private bool[] bodyDirty;
        private int bodyHighWaterMark;
        private int activeBodyCount;

        private PhysicsWorldCollider[] colliders;
        private int[] colliderVersions;
        private bool[] colliderAlive;
        private bool[] colliderDirty;
        private bool[] colliderBroadphaseDirty;
        private int colliderHighWaterMark;
        private int activeColliderCount;
        private readonly PhysicsWorldAABBTree broadphaseTree;

        private PhysicsWorldPair[] pairs;
        private int pairCount;
        private bool pairOverflowed;
        private bool broadphaseTreeOverflowed;
        private int bodySyncCount;
        private int colliderSyncCount;
        private int broadphaseMovedProxyCount;
        private int broadphaseCandidateCount;
        private int broadphaseFilteredCandidateCount;
        private PhysicsWorldManifold[] manifolds;
        private PhysicsWorldManifold[] manifoldScratch;
        private int contactManifoldCount;
        private bool contactOverflowed;
        private int narrowPhaseTestCount;
        private int contactManifoldNewCount;
        private int contactManifoldReusedCount;
        private int contactManifoldDroppedCount;
        private int solverContactPointCount;
        private int contactFrameIndex;
        private bool queryOverflowed;
        private bool raycastOverflowed;
        private bool shapeCastOverflowed;
        private fix fixedTimeAccumulator;
        private ulong fixedStepCount;
        private int[] islandParent;
        private int[] islandRank;
        private int[] islandBodyCounts;
        private int[] islandSleepingBodyCounts;
        private int[] islandRootToId;
        private int[] bodyIslandIds;
        private int islandCount;
        private int sleepingIslandCount;
        private PhysicsWorldStepStats lastStepStats;

        public PhysicsWorld()
            : this(PhysicsWorldSettings.Default)
        {
        }

        public PhysicsWorld(PhysicsWorldSettings settings)
        {
            this.settings = NormalizeSettings(settings);
            bodies = Array.Empty<PhysicsWorldBody>();
            bodyVersions = Array.Empty<int>();
            bodyAlive = Array.Empty<bool>();
            bodyDirty = Array.Empty<bool>();
            colliders = Array.Empty<PhysicsWorldCollider>();
            colliderVersions = Array.Empty<int>();
            colliderAlive = Array.Empty<bool>();
            colliderDirty = Array.Empty<bool>();
            colliderBroadphaseDirty = Array.Empty<bool>();
            broadphaseTree = new PhysicsWorldAABBTree();
            pairs = Array.Empty<PhysicsWorldPair>();
            manifolds = Array.Empty<PhysicsWorldManifold>();
            manifoldScratch = Array.Empty<PhysicsWorldManifold>();
            islandParent = Array.Empty<int>();
            islandRank = Array.Empty<int>();
            islandBodyCounts = Array.Empty<int>();
            islandSleepingBodyCounts = Array.Empty<int>();
            islandRootToId = Array.Empty<int>();
            bodyIslandIds = Array.Empty<int>();
        }

        public PhysicsWorldSettings Settings
        {
            get { return settings; }
            set { settings = NormalizeSettings(value); }
        }

        public int BodyCapacity => bodies.Length;
        public int ColliderCapacity => colliders.Length;
        public int PairCapacity => pairs.Length;
        public int ContactManifoldCapacity => manifolds.Length;
        public PhysicsWorldCapacity Capacity =>
            new PhysicsWorldCapacity(bodies.Length, colliders.Length, pairs.Length, manifolds.Length);
        public PhysicsWorldMemoryBudget EstimatedMemoryBudget => Capacity.EstimateMemoryBudget();
        public int ActiveBodyCount => activeBodyCount;
        public int ActiveColliderCount => activeColliderCount;
        public int BroadphaseTreeProxyCount => broadphaseTree.ProxyCount;
        public int BroadphaseTreeHeight => broadphaseTree.Height;
        public int BroadphaseTreeMaxBalance => broadphaseTree.MaxBalance;
        public bool BroadphaseTreeOverflowed => broadphaseTreeOverflowed;
        public int PairCount => pairCount;
        public bool PairOverflowed => pairOverflowed;
        public int ContactManifoldCount => contactManifoldCount;
        public bool ContactOverflowed => contactOverflowed;
        public bool QueryOverflowed => queryOverflowed;
        public bool RaycastOverflowed => raycastOverflowed;
        public bool ShapeCastOverflowed => shapeCastOverflowed;
        public fix FixedTimeAccumulator => fixedTimeAccumulator;
        public ulong FixedStepCount => fixedStepCount;
        public int IslandCount => islandCount;
        public int SleepingIslandCount => sleepingIslandCount;
        public PhysicsWorldStepStats LastStepStats => lastStepStats;

        /// <summary>
        /// Reserves storage used by the no-GC core. Update, Step and Query never resize
        /// these arrays; creation returns false when capacity is exhausted.
        /// </summary>
        public void Reserve(int bodyCapacity, int colliderCapacity, int pairCapacity)
        {
            Reserve(bodyCapacity, colliderCapacity, pairCapacity, pairCapacity);
        }

        public void Reserve(PhysicsWorldCapacity capacity)
        {
            capacity = capacity.Normalize();
            Reserve(
                capacity.bodyCapacity,
                capacity.colliderCapacity,
                capacity.pairCapacity,
                capacity.contactManifoldCapacity);
        }

        /// <summary>
        /// Reserves storage for bodies, colliders, broadphase pairs and contact
        /// manifolds. Reserve may allocate; simulation methods keep to these buffers.
        /// </summary>
        public void Reserve(int bodyCapacity, int colliderCapacity, int pairCapacity, int contactManifoldCapacity)
        {
            EnsureCapacity(ref bodies, math.max(bodyCapacity, bodyHighWaterMark));
            EnsureCapacity(ref bodyVersions, bodies.Length);
            EnsureCapacity(ref bodyAlive, bodies.Length);
            EnsureCapacity(ref bodyDirty, bodies.Length);
            EnsureCapacity(ref islandParent, bodies.Length);
            EnsureCapacity(ref islandRank, bodies.Length);
            EnsureCapacity(ref islandBodyCounts, bodies.Length);
            EnsureCapacity(ref islandSleepingBodyCounts, bodies.Length);
            EnsureCapacity(ref islandRootToId, bodies.Length);
            EnsureCapacity(ref bodyIslandIds, bodies.Length);
            EnsureCapacity(ref colliders, math.max(colliderCapacity, colliderHighWaterMark));
            EnsureCapacity(ref colliderVersions, colliders.Length);
            EnsureCapacity(ref colliderAlive, colliders.Length);
            EnsureCapacity(ref colliderDirty, colliders.Length);
            EnsureCapacity(ref colliderBroadphaseDirty, colliders.Length);
            broadphaseTree.Reserve(colliders.Length);
            EnsureCapacity(ref pairs, math.max(pairCapacity, pairCount));
            EnsureCapacity(ref manifolds, math.max(contactManifoldCapacity, contactManifoldCount));
            EnsureCapacity(ref manifoldScratch, manifolds.Length);

            // Box/OBB contact clipping uses thread-local scratch arrays in Physics.
            // Warming them here keeps the first simulated box contact out of the GC path.
            Physics.ReserveContactScratch();
        }

        public bool CreateBody(fix3 position, quaternion rotation, RigidType type, out BodyHandle handle)
        {
            handle = BodyHandle.Invalid;
            int index = FindFreeBodySlot();
            if (index < 0)
            {
                return false;
            }

            int version = NextVersion(bodyVersions[index]);
            handle = new BodyHandle(index, version);
            bodyVersions[index] = version;
            bodyAlive[index] = true;
            bodyDirty[index] = false;
            bodies[index] = new PhysicsWorldBody
            {
                handle = handle,
                type = type,
                position = position,
                rotation = rotation,
                velocity = fix3.zero,
                angularVelocity = fix3.zero,
                mass = fix.One,
                inertia = fix3.one,
                autoMass = true,
                autoInertia = true,
                enabled = true,
                useGravity = settings.enableGravity,
                allowSleep = true,
                isSleeping = false,
                useCCD = true,
                sleepTime = fix.Zero,
            };

            if (index == bodyHighWaterMark)
            {
                bodyHighWaterMark++;
            }

            activeBodyCount++;
            return true;
        }

        public bool DestroyBody(BodyHandle handle)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            int index = handle.index;
            bodyAlive[index] = false;
            bodyDirty[index] = false;
            bodyVersions[index] = NextVersion(bodyVersions[index]);
            bodies[index] = default;
            activeBodyCount--;

            for (int i = 0; i < colliderHighWaterMark; i++)
            {
                if (colliderAlive[i] && colliders[i].body == handle)
                {
                    DestroyColliderAt(i);
                }
            }

            ClearCollisionCaches();
            return true;
        }

        public bool SetBodyEnabled(BodyHandle handle, bool enabled)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.enabled = enabled;
            if (enabled)
            {
                WakeBody(ref body);
            }
            else
            {
                SleepBody(ref body, clearVelocity: false);
            }

            bodies[handle.index] = body;
            if (enabled)
            {
                MarkBodyDirty(handle);
            }

            if (!enabled)
            {
                RemoveBodyBroadphaseProxies(handle);
            }

            ClearCollisionCaches();
            return true;
        }

        public bool SetBodyType(BodyHandle handle, RigidType type)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.type = type;
            WakeBody(ref body);
            bodies[handle.index] = body;
            ClearCollisionCaches();
            return true;
        }

        public bool SetBodyGravity(BodyHandle handle, bool useGravity)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.useGravity = useGravity;
            bodies[handle.index] = body;
            return true;
        }

        public bool SetBodyMass(BodyHandle handle, fix mass)
        {
            if (!IsBodyAlive(handle) || mass < fix.Zero)
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.mass = mass;
            body.autoMass = false;
            WakeBody(ref body);
            bodies[handle.index] = body;
            return true;
        }

        public bool SetBodyInertia(BodyHandle handle, fix3 inertia)
        {
            if (!IsBodyAlive(handle) || inertia.x < fix.Zero || inertia.y < fix.Zero || inertia.z < fix.Zero)
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.inertia = inertia;
            body.autoInertia = false;
            WakeBody(ref body);
            bodies[handle.index] = body;
            return true;
        }

        public bool SetBodyAutoMass(BodyHandle handle, bool autoMass)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.autoMass = autoMass;
            WakeBody(ref body);
            bodies[handle.index] = body;
            if (autoMass)
            {
                ApplyBodyMassProperties(handle);
            }

            return true;
        }

        public bool SetBodyAutoInertia(BodyHandle handle, bool autoInertia)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.autoInertia = autoInertia;
            WakeBody(ref body);
            bodies[handle.index] = body;
            if (autoInertia)
            {
                ApplyBodyMassProperties(handle);
            }

            return true;
        }

        public bool SetBodyAutoMassProperties(BodyHandle handle, bool autoMass, bool autoInertia)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.autoMass = autoMass;
            body.autoInertia = autoInertia;
            WakeBody(ref body);
            bodies[handle.index] = body;
            if (autoMass || autoInertia)
            {
                ApplyBodyMassProperties(handle);
            }

            return true;
        }

        public bool SetBodyVelocity(BodyHandle handle, fix3 velocity)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.velocity = ClampLinearVelocity(PhysicsSafety.Sanitize(velocity));
            if (math.lengthsq(body.velocity) > math.Epsilon)
            {
                WakeBody(ref body);
            }

            bodies[handle.index] = body;
            return true;
        }

        public bool SetBodyAngularVelocity(BodyHandle handle, fix3 angularVelocity)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.angularVelocity = ClampAngularVelocity(PhysicsSafety.Sanitize(angularVelocity));
            if (math.lengthsq(body.angularVelocity) > math.Epsilon)
            {
                WakeBody(ref body);
            }

            bodies[handle.index] = body;
            return true;
        }

        public bool SetBodyTransform(BodyHandle handle, fix3 position, quaternion rotation)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.position = position;
            body.rotation = rotation;
            WakeBody(ref body);
            bodies[handle.index] = body;
            MarkBodyDirty(handle);
            ClearCollisionCaches();
            return true;
        }

        public bool SetBodyAllowSleep(BodyHandle handle, bool allowSleep)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.allowSleep = allowSleep;
            if (!allowSleep)
            {
                WakeBody(ref body);
            }

            bodies[handle.index] = body;
            return true;
        }

        public bool SetBodyCCD(BodyHandle handle, bool useCCD)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            body.useCCD = useCCD;
            bodies[handle.index] = body;
            return true;
        }

        public bool WakeBody(BodyHandle handle)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            WakeBody(ref body);
            bodies[handle.index] = body;
            return true;
        }

        public bool SleepBody(BodyHandle handle)
        {
            if (!IsBodyAlive(handle))
            {
                return false;
            }

            PhysicsWorldBody body = bodies[handle.index];
            SleepBody(ref body, clearVelocity: true);
            bodies[handle.index] = body;
            return true;
        }

        public bool TryGetBody(BodyHandle handle, out PhysicsWorldBody body)
        {
            if (!IsBodyAlive(handle))
            {
                body = default;
                return false;
            }

            body = bodies[handle.index];
            return true;
        }

        public bool TryGetBodyIslandId(BodyHandle handle, out int islandId)
        {
            if (!IsBodyAlive(handle) || handle.index >= bodyIslandIds.Length)
            {
                islandId = -1;
                return false;
            }

            islandId = bodyIslandIds[handle.index];
            return islandId >= 0;
        }

        public bool AddAABB(BodyHandle body, fix3 center, fix3 size, out ColliderHandle handle)
        {
            return AddCollider(body, PhysicsShapeData.AABB(center, size), out handle);
        }

        public bool AddOBB(BodyHandle body, fix3 center, fix3 size, quaternion rotation, out ColliderHandle handle)
        {
            return AddCollider(body, PhysicsShapeData.OBB(center, size, rotation), out handle);
        }

        public bool AddSphere(BodyHandle body, fix3 center, fix radius, out ColliderHandle handle)
        {
            return AddCollider(body, PhysicsShapeData.Sphere(center, radius), out handle);
        }

        public bool AddCapsule(BodyHandle body, fix3 center, fix radius, fix height, quaternion rotation, out ColliderHandle handle)
        {
            return AddCollider(body, PhysicsShapeData.Capsule(center, radius, height, rotation), out handle);
        }

        public bool AddCollider(BodyHandle body, PhysicsShapeData localShape, out ColliderHandle handle)
        {
            handle = ColliderHandle.Invalid;
            if (!IsBodyAlive(body))
            {
                return false;
            }

            int index = FindFreeColliderSlot();
            if (index < 0)
            {
                return false;
            }

            int version = NextVersion(colliderVersions[index]);
            handle = new ColliderHandle(index, version);
            PhysicsWorldBody owner = bodies[body.index];
            PhysicsShapeData worldShape = localShape.Transform(owner.position, owner.rotation);

            colliderVersions[index] = version;
            colliderAlive[index] = true;
            colliderDirty[index] = false;
            colliderBroadphaseDirty[index] = true;
            colliders[index] = new PhysicsWorldCollider
            {
                handle = handle,
                body = body,
                layer = Collider.DefaultLayer,
                collisionMask = Collider.AllLayers,
                enabled = true,
                isTrigger = false,
                material = Material.Default,
                localShape = localShape,
                worldShape = worldShape,
                bounds = worldShape.ComputeBounds(),
            };

            if (index == colliderHighWaterMark)
            {
                colliderHighWaterMark++;
            }

            activeColliderCount++;
            ApplyBodyMassProperties(body);
            ClearCollisionCaches();
            return true;
        }

        public bool DestroyCollider(ColliderHandle handle)
        {
            if (!IsColliderAlive(handle))
            {
                return false;
            }

            DestroyColliderAt(handle.index);
            ClearCollisionCaches();
            return true;
        }

        public bool SetColliderEnabled(ColliderHandle handle, bool enabled)
        {
            if (!IsColliderAlive(handle))
            {
                return false;
            }

            PhysicsWorldCollider collider = colliders[handle.index];
            collider.enabled = enabled;
            colliders[handle.index] = collider;
            if (enabled)
            {
                MarkColliderBroadphaseDirty(handle.index);
            }
            else
            {
                broadphaseTree.RemoveProxy(handle);
                colliderBroadphaseDirty[handle.index] = false;
            }

            ClearCollisionCaches();
            return true;
        }

        public bool SetColliderLayer(ColliderHandle handle, int layer)
        {
            if (!IsColliderAlive(handle) || layer < Collider.MinLayer || layer > Collider.MaxLayer)
            {
                return false;
            }

            PhysicsWorldCollider collider = colliders[handle.index];
            collider.layer = layer;
            colliders[handle.index] = collider;
            ClearCollisionCaches();
            return true;
        }

        public bool SetColliderCollisionMask(ColliderHandle handle, int collisionMask)
        {
            if (!IsColliderAlive(handle))
            {
                return false;
            }

            PhysicsWorldCollider collider = colliders[handle.index];
            collider.collisionMask = collisionMask;
            colliders[handle.index] = collider;
            ClearCollisionCaches();
            return true;
        }

        public bool SetColliderTrigger(ColliderHandle handle, bool isTrigger)
        {
            if (!IsColliderAlive(handle))
            {
                return false;
            }

            PhysicsWorldCollider collider = colliders[handle.index];
            collider.isTrigger = isTrigger;
            colliders[handle.index] = collider;
            ClearContacts();
            return true;
        }

        public bool SetColliderMaterial(ColliderHandle handle, Material material)
        {
            if (!IsColliderAlive(handle))
            {
                return false;
            }

            PhysicsWorldCollider collider = colliders[handle.index];
            collider.material = material;
            colliders[handle.index] = collider;
            ApplyBodyMassProperties(collider.body);
            return true;
        }

        public bool SetColliderDensity(ColliderHandle handle, fix density)
        {
            if (!IsColliderAlive(handle))
            {
                return false;
            }

            PhysicsWorldCollider collider = colliders[handle.index];
            Material material = collider.material;
            material.SetDensity(density);
            collider.material = material;
            colliders[handle.index] = collider;
            ApplyBodyMassProperties(collider.body);
            return true;
        }

        public bool TryGetCollider(ColliderHandle handle, out PhysicsWorldCollider collider)
        {
            if (!IsColliderAlive(handle))
            {
                collider = default;
                return false;
            }

            SyncDirtyColliders();
            collider = colliders[handle.index];
            return true;
        }

        public bool TryGetColliderBounds(ColliderHandle handle, out AABB bounds)
        {
            if (!IsColliderAlive(handle))
            {
                bounds = default;
                return false;
            }

            SyncDirtyColliders();
            bounds = colliders[handle.index].bounds;
            return true;
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

            ResetSyncStats();
            SyncDirtyColliders();
            IntegrateBodies(deltaTime);
            SyncDirtyColliders();
            BuildPairs();
            BuildContacts();
            BuildIslands();
            WakeSleepingBodiesForActiveContacts();
            SolveVelocity(deltaTime);
            SolvePositions();
            SanitizeBodies(deltaTime);
            SyncDirtyColliders();
            SyncBroadphaseTree();
            UpdateSleeping(deltaTime);
            SanitizeBodies(deltaTime);
            UpdateStepStats();
        }

        public int Step(fix deltaTime)
        {
            return Step(deltaTime, settings.timeStep, settings.maxSubSteps);
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

        public ulong StepAndComputeStateHash(fix deltaTime)
        {
            Step(deltaTime);
            return ComputeStateHash();
        }

        public int QueryAABBNonAlloc(AABB bounds, ColliderHandle[] results)
        {
            return QueryAABBNonAlloc(bounds, Collider.AllLayers, results);
        }

        public int QueryAABBNonAlloc(AABB bounds, int layerMask, ColliderHandle[] results)
        {
            queryOverflowed = false;
            if (results == null)
            {
                return 0;
            }

            SyncDirtyColliders();
            SyncBroadphaseTree();
            int count = 0;
            broadphaseTree.BeginQuery(bounds);
            while (broadphaseTree.TryGetNext(out ColliderHandle candidateHandle, out AABB candidateBounds))
            {
                if (!IsColliderAlive(candidateHandle) || !IsColliderQueryable(candidateHandle.index, layerMask))
                {
                    continue;
                }

                if (Physics.IsOverlap(candidateBounds, bounds))
                {
                    if (count < results.Length)
                    {
                        results[count++] = candidateHandle;
                    }
                    else
                    {
                        queryOverflowed = true;
                    }
                }
            }

            broadphaseTreeOverflowed = broadphaseTree.Overflowed;
            queryOverflowed |= broadphaseTreeOverflowed;
            SortColliderHandles(results, count);
            return count;
        }

        public int RaycastNonAlloc(Ray ray, fix maxDistance, PhysicsWorldRaycastHit[] results)
        {
            return RaycastNonAlloc(ray, maxDistance, Collider.AllLayers, results);
        }

        public int RaycastNonAlloc(Ray ray, fix maxDistance, int layerMask, PhysicsWorldRaycastHit[] results)
        {
            raycastOverflowed = false;
            if (results == null || maxDistance < fix.Zero || !Physics.TryNormalizeRay(ray, out Ray normalizedRay))
            {
                return 0;
            }

            SyncDirtyColliders();
            SyncBroadphaseTree();
            int count = 0;
            broadphaseTree.BeginRaycast(normalizedRay, maxDistance);
            while (broadphaseTree.TryGetNextRaycast(out ColliderHandle candidateHandle, out AABB candidateBounds))
            {
                if (!IsColliderAlive(candidateHandle) || !IsColliderQueryable(candidateHandle.index, layerMask))
                {
                    continue;
                }

                PhysicsWorldCollider collider = colliders[candidateHandle.index];
                if (!Physics.TryRaycast(collider.worldShape, normalizedRay, maxDistance, out RaycastHit hitInfo))
                {
                    continue;
                }

                AddRaycastHit(
                    results,
                    ref count,
                    ref raycastOverflowed,
                    new PhysicsWorldRaycastHit
                    {
                        collider = candidateHandle,
                        body = collider.body,
                        distance = hitInfo.distance,
                        point = hitInfo.point,
                        normal = hitInfo.normal,
                        bounds = candidateBounds,
                    });
            }

            broadphaseTreeOverflowed = broadphaseTree.Overflowed;
            raycastOverflowed |= broadphaseTreeOverflowed;
            return count;
        }

        public int ShapeCastNonAlloc(PhysicsShapeData movingShape, fix3 delta, PhysicsWorldShapeCastHit[] results)
        {
            return ShapeCastNonAlloc(movingShape, delta, Collider.AllLayers, results);
        }

        public int ShapeCastNonAlloc(
            PhysicsShapeData movingShape,
            fix3 delta,
            int layerMask,
            PhysicsWorldShapeCastHit[] results)
        {
            return ShapeCastNonAlloc(movingShape, delta, layerMask, ColliderHandle.Invalid, BodyHandle.Invalid, results);
        }

        public int ShapeCastNonAlloc(ColliderHandle movingCollider, fix3 delta, PhysicsWorldShapeCastHit[] results)
        {
            return ShapeCastNonAlloc(movingCollider, delta, Collider.AllLayers, results);
        }

        public int ShapeCastNonAlloc(
            ColliderHandle movingCollider,
            fix3 delta,
            int layerMask,
            PhysicsWorldShapeCastHit[] results)
        {
            if (!IsColliderAlive(movingCollider))
            {
                shapeCastOverflowed = false;
                return 0;
            }

            SyncDirtyColliders();
            PhysicsWorldCollider collider = colliders[movingCollider.index];
            return ShapeCastNonAlloc(collider.worldShape, delta, layerMask, movingCollider, collider.body, results);
        }

        public bool TryGetPair(int index, out PhysicsWorldPair pair)
        {
            if (index < 0 || index >= pairCount)
            {
                pair = default;
                return false;
            }

            pair = pairs[index];
            return true;
        }

        public bool TryGetContactManifold(int index, out PhysicsWorldManifold manifold)
        {
            if (index < 0 || index >= contactManifoldCount)
            {
                manifold = default;
                return false;
            }

            manifold = manifolds[index];
            return true;
        }

        public ulong ComputeStateHash()
        {
            ulong hash = HashOffsetBasis;
            Hash(ref hash, settings.enableGravity);
            Hash(ref hash, settings.gravity);
            Hash(ref hash, settings.timeStep);
            Hash(ref hash, settings.maxSubSteps);
            Hash(ref hash, settings.restitution);
            Hash(ref hash, settings.restitutionVelocityThreshold);
            Hash(ref hash, settings.friction);
            Hash(ref hash, settings.warmStartScale);
            Hash(ref hash, settings.penetrationSlop);
            Hash(ref hash, settings.positionCorrectionPercent);
            Hash(ref hash, settings.maxPositionCorrection);
            Hash(ref hash, settings.maxContactImpulse);
            Hash(ref hash, settings.maxFrictionImpulse);
            Hash(ref hash, settings.contactVelocityBiasFactor);
            Hash(ref hash, settings.maxContactBiasVelocity);
            Hash(ref hash, settings.maxLinearVelocity);
            Hash(ref hash, settings.maxAngularVelocity);
            Hash(ref hash, settings.maxTranslationPerStep);
            Hash(ref hash, settings.maxRotationPerStep);
            Hash(ref hash, settings.solverIterations);
            Hash(ref hash, settings.positionIterations);
            Hash(ref hash, settings.enableSleeping);
            Hash(ref hash, settings.linearSleepThreshold);
            Hash(ref hash, settings.angularSleepThreshold);
            Hash(ref hash, settings.sleepTime);
            Hash(ref hash, settings.enableCCD);
            Hash(ref hash, settings.enableDynamicCCD);
            Hash(ref hash, settings.ccdMinVelocity);
            Hash(ref hash, settings.ccdSkin);
            Hash(ref hash, settings.ccdMaxIterations);
            Hash(ref hash, (int)settings.narrowPhaseAlgorithm);
            Hash(ref hash, settings.contactManifoldSettings.normalPersistenceDot);
            Hash(ref hash, settings.contactManifoldSettings.anchorMatchDistance);
            Hash(ref hash, settings.contactManifoldSettings.positionMatchDistance);
            Hash(ref hash, settings.contactManifoldSettings.staleFrameLimit);
            Hash(ref hash, fixedStepCount);
            Hash(ref hash, fixedTimeAccumulator);

            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i])
                {
                    continue;
                }

                PhysicsWorldBody body = bodies[i];
                Hash(ref hash, body.handle.Index);
                Hash(ref hash, body.handle.Version);
                Hash(ref hash, (int)body.type);
                Hash(ref hash, body.position);
                Hash(ref hash, body.rotation);
                Hash(ref hash, body.velocity);
                Hash(ref hash, body.angularVelocity);
                Hash(ref hash, body.mass);
                Hash(ref hash, body.inertia);
                Hash(ref hash, body.autoMass);
                Hash(ref hash, body.autoInertia);
                Hash(ref hash, body.enabled);
                Hash(ref hash, body.useGravity);
                Hash(ref hash, body.allowSleep);
                Hash(ref hash, body.isSleeping);
                Hash(ref hash, body.useCCD);
                Hash(ref hash, body.sleepTime);
            }

            for (int i = 0; i < colliderHighWaterMark; i++)
            {
                if (!colliderAlive[i])
                {
                    continue;
                }

                PhysicsWorldCollider collider = colliders[i];
                Hash(ref hash, collider.handle.Index);
                Hash(ref hash, collider.handle.Version);
                Hash(ref hash, collider.body.Index);
                Hash(ref hash, collider.body.Version);
                Hash(ref hash, collider.layer);
                Hash(ref hash, collider.collisionMask);
                Hash(ref hash, collider.enabled);
                Hash(ref hash, collider.isTrigger);
                Hash(ref hash, collider.material.GetDensity());
                Hash(ref hash, collider.material.GetFrictionCoefficient());
                Hash(ref hash, collider.material.GetBounciness());
                Hash(ref hash, collider.localShape);
            }

            Hash(ref hash, islandCount);
            Hash(ref hash, sleepingIslandCount);
            Hash(ref hash, pairCount);
            for (int i = 0; i < pairCount; i++)
            {
                Hash(ref hash, pairs[i].collider0.Index);
                Hash(ref hash, pairs[i].collider0.Version);
                Hash(ref hash, pairs[i].collider1.Index);
                Hash(ref hash, pairs[i].collider1.Version);
            }

            Hash(ref hash, contactManifoldCount);
            for (int i = 0; i < contactManifoldCount; i++)
            {
                Hash(ref hash, manifolds[i]);
            }

            return hash;
        }

        private void IntegrateBodies(fix deltaTime)
        {
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i])
                {
                    continue;
                }

                PhysicsWorldBody body = bodies[i];
                fix3 originalPosition = body.position;
                quaternion originalRotation = body.rotation;
                if (!body.enabled)
                {
                    continue;
                }

                if (body.IsAwakeDynamic)
                {
                    if (body.useGravity && settings.enableGravity)
                    {
                        body.velocity += settings.gravity * deltaTime;
                    }

                    ClampBodyMotion(ref body, deltaTime);
                    fix integrationTime = IntegrateTranslationWithCCD(i, ref body, deltaTime);
                    body.rotation = IntegrateOrientation(body.rotation, body.angularVelocity, integrationTime);
                }
                else if (body.type == RigidType.Kinematic)
                {
                    ClampBodyMotion(ref body, deltaTime);
                    body.position += body.velocity * deltaTime;
                    body.rotation = IntegrateOrientation(body.rotation, body.angularVelocity, deltaTime);
                }

                bodies[i] = body;
                if (body.position != originalPosition || body.rotation != originalRotation)
                {
                    MarkBodyDirtyAt(i);
                }
            }
        }

        private fix IntegrateTranslationWithCCD(int bodyIndex, ref PhysicsWorldBody body, fix deltaTime)
        {
            if (!ShouldUseCCD(body, deltaTime))
            {
                body.position += body.velocity * deltaTime;
                return deltaTime;
            }

            fix remainingTime = deltaTime;
            fix integratedTime = fix.Zero;
            int maxIterations = settings.ccdMaxIterations > 0 ? settings.ccdMaxIterations : 1;

            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                if (remainingTime <= fix.Zero || math.lengthsq(body.velocity) <= math.Epsilon)
                {
                    break;
                }

                fix3 translationDelta = body.velocity * remainingTime;
                if (!TryComputeTimeOfImpact(body, translationDelta, remainingTime, out ContinuousHit hit))
                {
                    body.position += translationDelta;
                    integratedTime += remainingTime;
                    break;
                }

                fix distance = math.length(translationDelta);
                fix safeFraction = hit.fraction;
                if (distance > math.Epsilon && settings.ccdSkin > fix.Zero)
                {
                    safeFraction = math.max(fix.Zero, safeFraction - settings.ccdSkin / distance);
                }

                body.position += translationDelta * safeFraction;
                fix consumedTime = remainingTime * safeFraction;
                integratedTime += consumedTime;

                fix normalVelocity = math.dot(body.velocity, hit.normal);
                if (normalVelocity > fix.Zero)
                {
                    body.velocity -= hit.normal * normalVelocity;
                }

                bodies[bodyIndex] = body;
                MarkBodyDirtyAt(bodyIndex);
                SyncDirtyColliders();

                remainingTime -= consumedTime;
                if (remainingTime <= math.Epsilon
                    || safeFraction <= math.Epsilon
                    || math.lengthsq(body.velocity) <= math.Epsilon)
                {
                    break;
                }

            }

            return integratedTime;
        }

        private bool ShouldUseCCD(PhysicsWorldBody body, fix deltaTime)
        {
            if (!settings.enableCCD
                || !body.useCCD
                || !body.IsAwakeDynamic
                || deltaTime <= fix.Zero
                || math.lengthsq(body.velocity) <= math.Epsilon)
            {
                return false;
            }

            fix minVelocity = math.max(fix.Zero, settings.ccdMinVelocity);
            return math.lengthsq(body.velocity) >= minVelocity * minVelocity;
        }

        private bool TryComputeTimeOfImpact(
            PhysicsWorldBody movingBody,
            fix3 translationDelta,
            fix deltaTime,
            out ContinuousHit hit)
        {
            hit = default;
            bool found = false;
            fix bestFraction = fix.One;
            fix3 bestNormal = fix3.zero;
            int bestColliderIndex = int.MaxValue;

            for (int i = 0; i < colliderHighWaterMark; i++)
            {
                if (!IsColliderActive(i)
                    || colliders[i].body != movingBody.handle
                    || colliders[i].isTrigger)
                {
                    continue;
                }

                PhysicsWorldCollider movingCollider = colliders[i];
                PhysicsShapeData movingShape = movingCollider.localShape.Transform(movingBody.position, movingBody.rotation);
                AABB movingBounds = movingShape.ComputeBounds();

                for (int j = 0; j < colliderHighWaterMark; j++)
                {
                    if (j == i
                        || !IsColliderActive(j)
                        || colliders[j].body == movingBody.handle
                        || colliders[j].isTrigger
                        || !CanCollide(movingCollider, colliders[j])
                        || (!settings.enableDynamicCCD && IsAwakeDynamicBody(colliders[j].body)))
                    {
                        continue;
                    }

                    fix3 targetDelta = GetContinuousTargetDelta(colliders[j].body, deltaTime);
                    fix3 relativeDelta = translationDelta - targetDelta;
                    if (math.lengthsq(relativeDelta) <= math.Epsilon
                        || !TryComputeColliderTimeOfImpact(
                            movingShape,
                            colliders[j].worldShape,
                            movingBounds,
                            colliders[j].bounds,
                            relativeDelta,
                            out fix fraction,
                            out fix3 normal))
                    {
                        continue;
                    }

                    ColliderHandle targetHandle = colliders[j].handle;
                    if (fraction > bestFraction || (fraction == bestFraction && targetHandle.Index >= bestColliderIndex))
                    {
                        continue;
                    }

                    found = true;
                    bestFraction = fraction;
                    bestNormal = normal;
                    bestColliderIndex = targetHandle.Index;
                }
            }

            if (!found)
            {
                return false;
            }

            hit = new ContinuousHit(bestFraction, bestNormal);
            return true;
        }

        private fix3 GetContinuousTargetDelta(BodyHandle targetBody, fix deltaTime)
        {
            if (deltaTime <= fix.Zero || !IsBodyAlive(targetBody))
            {
                return fix3.zero;
            }

            PhysicsWorldBody body = bodies[targetBody.index];
            if (body.IsKinematic || (settings.enableDynamicCCD && body.IsAwakeDynamic))
            {
                return body.velocity * deltaTime;
            }

            return fix3.zero;
        }

        private static bool TryComputeColliderTimeOfImpact(
            PhysicsShapeData movingShape,
            PhysicsShapeData targetShape,
            AABB movingBounds,
            AABB targetBounds,
            fix3 translationDelta,
            out fix fraction,
            out fix3 normal)
        {
            if (Physics.TryShapeCast(movingShape, targetShape, translationDelta, out ShapeCastHit shapeCastHit))
            {
                fraction = shapeCastHit.fraction;
                normal = shapeCastHit.normal;
                return true;
            }

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

        private void SyncDirtyColliders()
        {
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyDirty[i])
                {
                    continue;
                }

                bodyDirty[i] = false;
                if (!bodyAlive[i])
                {
                    continue;
                }

                bodySyncCount++;
                for (int j = 0; j < colliderHighWaterMark; j++)
                {
                    if (colliderAlive[j] && colliders[j].body.index == i)
                    {
                        colliderDirty[j] = true;
                    }
                }
            }

            for (int i = 0; i < colliderHighWaterMark; i++)
            {
                if (!colliderDirty[i])
                {
                    continue;
                }

                colliderDirty[i] = false;
                if (!colliderAlive[i])
                {
                    colliderBroadphaseDirty[i] = false;
                    continue;
                }

                SyncColliderAt(i);
                colliderBroadphaseDirty[i] = true;
                colliderSyncCount++;
            }
        }

        private void SyncColliderAt(int index)
        {
            PhysicsWorldCollider collider = colliders[index];
            if (!IsBodyAlive(collider.body))
            {
                return;
            }

            PhysicsWorldBody body = bodies[collider.body.index];
            collider.worldShape = collider.localShape.Transform(body.position, body.rotation);
            collider.bounds = collider.worldShape.ComputeBounds();
            colliders[index] = collider;
        }

        private void BuildPairs()
        {
            pairCount = 0;
            pairOverflowed = false;
            broadphaseCandidateCount = 0;
            broadphaseFilteredCandidateCount = 0;
            SyncBroadphaseTree();

            for (int i = 0; i < colliderHighWaterMark; i++)
            {
                if (!IsColliderActive(i))
                {
                    continue;
                }

                broadphaseTree.BeginQuery(colliders[i].bounds);
                while (broadphaseTree.TryGetNext(out ColliderHandle candidateHandle, out AABB candidateBounds))
                {
                    broadphaseCandidateCount++;
                    int j = candidateHandle.index;
                    if (j <= i || !IsColliderAlive(candidateHandle) || !IsColliderActive(j) || !CanCollide(colliders[i], colliders[j]))
                    {
                        broadphaseFilteredCandidateCount++;
                        continue;
                    }

                    if (!Physics.IsOverlap(colliders[i].bounds, candidateBounds))
                    {
                        broadphaseFilteredCandidateCount++;
                        continue;
                    }

                    if (pairCount >= pairs.Length)
                    {
                        pairOverflowed = true;
                        continue;
                    }

                    pairs[pairCount++] = new PhysicsWorldPair(
                        colliders[i].handle,
                        colliders[j].handle,
                        colliders[i].body,
                        colliders[j].body,
                        colliders[i].bounds,
                        colliders[j].bounds);
                }
            }

            broadphaseTreeOverflowed = broadphaseTree.Overflowed;
            pairOverflowed |= broadphaseTreeOverflowed;
            SortPairs();
        }

        private void SyncBroadphaseTree()
        {
            broadphaseTree.ResetOverflow();
            for (int i = 0; i < colliderHighWaterMark; i++)
            {
                if (!IsColliderActive(i))
                {
                    colliderBroadphaseDirty[i] = false;
                    continue;
                }

                if (!colliderBroadphaseDirty[i])
                {
                    continue;
                }

                colliderBroadphaseDirty[i] = false;
                if (!broadphaseTree.MoveProxy(colliders[i].handle, colliders[i].bounds))
                {
                    break;
                }

                broadphaseMovedProxyCount++;
            }

            broadphaseTreeOverflowed = broadphaseTree.Overflowed;
        }

        private void SortPairs()
        {
            for (int i = 1; i < pairCount; i++)
            {
                PhysicsWorldPair item = pairs[i];
                int j = i - 1;
                while (j >= 0 && ComparePairs(pairs[j], item) > 0)
                {
                    pairs[j + 1] = pairs[j];
                    j--;
                }

                pairs[j + 1] = item;
            }
        }

        private static int ComparePairs(PhysicsWorldPair a, PhysicsWorldPair b)
        {
            int first = a.collider0.Index.CompareTo(b.collider0.Index);
            return first != 0 ? first : a.collider1.Index.CompareTo(b.collider1.Index);
        }

        private int ShapeCastNonAlloc(
            PhysicsShapeData movingShape,
            fix3 delta,
            int layerMask,
            ColliderHandle excludedCollider,
            BodyHandle excludedBody,
            PhysicsWorldShapeCastHit[] results)
        {
            shapeCastOverflowed = false;
            if (results == null)
            {
                return 0;
            }

            SyncDirtyColliders();
            SyncBroadphaseTree();
            int count = 0;
            AABB sweptBounds = ComputeSweptBounds(movingShape.ComputeBounds(), delta);
            broadphaseTree.BeginQuery(sweptBounds);
            while (broadphaseTree.TryGetNext(out ColliderHandle candidateHandle, out AABB candidateBounds))
            {
                if (!IsColliderAlive(candidateHandle) || !IsColliderQueryable(candidateHandle.index, layerMask))
                {
                    continue;
                }

                PhysicsWorldCollider collider = colliders[candidateHandle.index];
                if (candidateHandle == excludedCollider || (excludedBody.IsValid && collider.body == excludedBody))
                {
                    continue;
                }

                if (!Physics.TryShapeCast(movingShape, collider.worldShape, delta, out ShapeCastHit hit))
                {
                    continue;
                }

                AddShapeCastHit(
                    results,
                    ref count,
                    ref shapeCastOverflowed,
                    new PhysicsWorldShapeCastHit
                    {
                        collider = candidateHandle,
                        body = collider.body,
                        fraction = hit.fraction,
                        point = hit.point,
                        normal = hit.normal,
                        bounds = candidateBounds,
                    });
            }

            broadphaseTreeOverflowed = broadphaseTree.Overflowed;
            shapeCastOverflowed |= broadphaseTreeOverflowed;
            return count;
        }

        private static void SortColliderHandles(ColliderHandle[] handles, int count)
        {
            for (int i = 1; i < count; i++)
            {
                ColliderHandle item = handles[i];
                int j = i - 1;
                while (j >= 0 && handles[j].Index > item.Index)
                {
                    handles[j + 1] = handles[j];
                    j--;
                }

                handles[j + 1] = item;
            }
        }

        private static void AddRaycastHit(
            PhysicsWorldRaycastHit[] results,
            ref int count,
            ref bool overflowed,
            PhysicsWorldRaycastHit hit)
        {
            if (results.Length == 0)
            {
                overflowed = true;
                return;
            }

            if (count < results.Length)
            {
                results[count++] = hit;
                SortLastRaycastHit(results, count - 1);
                return;
            }

            overflowed = true;
            if (CompareRaycastHits(hit, results[count - 1]) >= 0)
            {
                return;
            }

            results[count - 1] = hit;
            SortLastRaycastHit(results, count - 1);
        }

        private static void AddShapeCastHit(
            PhysicsWorldShapeCastHit[] results,
            ref int count,
            ref bool overflowed,
            PhysicsWorldShapeCastHit hit)
        {
            if (results.Length == 0)
            {
                overflowed = true;
                return;
            }

            if (count < results.Length)
            {
                results[count++] = hit;
                SortLastShapeCastHit(results, count - 1);
                return;
            }

            overflowed = true;
            if (CompareShapeCastHits(hit, results[count - 1]) >= 0)
            {
                return;
            }

            results[count - 1] = hit;
            SortLastShapeCastHit(results, count - 1);
        }

        private static void SortLastRaycastHit(PhysicsWorldRaycastHit[] results, int index)
        {
            PhysicsWorldRaycastHit item = results[index];
            int j = index - 1;
            while (j >= 0 && CompareRaycastHits(results[j], item) > 0)
            {
                results[j + 1] = results[j];
                j--;
            }

            results[j + 1] = item;
        }

        private static void SortLastShapeCastHit(PhysicsWorldShapeCastHit[] results, int index)
        {
            PhysicsWorldShapeCastHit item = results[index];
            int j = index - 1;
            while (j >= 0 && CompareShapeCastHits(results[j], item) > 0)
            {
                results[j + 1] = results[j];
                j--;
            }

            results[j + 1] = item;
        }

        private static int CompareRaycastHits(PhysicsWorldRaycastHit a, PhysicsWorldRaycastHit b)
        {
            int distance = a.distance.CompareTo(b.distance);
            return distance != 0 ? distance : a.collider.Index.CompareTo(b.collider.Index);
        }

        private static int CompareShapeCastHits(PhysicsWorldShapeCastHit a, PhysicsWorldShapeCastHit b)
        {
            int fraction = a.fraction.CompareTo(b.fraction);
            return fraction != 0 ? fraction : a.collider.Index.CompareTo(b.collider.Index);
        }

        private static AABB ComputeSweptBounds(AABB bounds, fix3 delta)
        {
            fix3 deltaMin = math.min(fix3.zero, delta);
            fix3 deltaMax = math.max(fix3.zero, delta);
            return AABB.FromMinMax(bounds.min + deltaMin, bounds.max + deltaMax);
        }

        private void BuildContacts()
        {
            contactFrameIndex++;
            contactOverflowed = false;
            narrowPhaseTestCount = 0;
            contactManifoldNewCount = 0;
            contactManifoldReusedCount = 0;
            contactManifoldDroppedCount = 0;
            int nextCount = 0;

            for (int i = 0; i < pairCount; i++)
            {
                PhysicsWorldPair pair = pairs[i];
                PhysicsWorldCollider collider0 = colliders[pair.collider0.index];
                PhysicsWorldCollider collider1 = colliders[pair.collider1.index];
                narrowPhaseTestCount++;

                if (!Physics.TryComputeContact(
                    collider0.worldShape,
                    collider1.worldShape,
                    settings.narrowPhaseAlgorithm,
                    out CollisionInfo collision))
                {
                    continue;
                }

                if (nextCount >= manifoldScratch.Length)
                {
                    contactOverflowed = true;
                    contactManifoldDroppedCount++;
                    continue;
                }

                bool hasPrevious = TryFindPreviousManifold(pair, out PhysicsWorldManifold previous);
                if (hasPrevious)
                {
                    contactManifoldReusedCount++;
                }
                else
                {
                    contactManifoldNewCount++;
                }

                StabilizeCollisionNormal(collider0.bounds, collider1.bounds, previous, hasPrevious, ref collision);
                manifoldScratch[nextCount].Update(
                    pair,
                    collision,
                    collider0.isTrigger || collider1.isTrigger,
                    contactFrameIndex,
                    settings.contactManifoldSettings,
                    previous,
                    hasPrevious);
                nextCount++;
            }

            PhysicsWorldManifold[] previousManifolds = manifolds;
            manifolds = manifoldScratch;
            manifoldScratch = previousManifolds;
            contactManifoldCount = nextCount;
        }

        private void StabilizeCollisionNormal(
            AABB bounds0,
            AABB bounds1,
            PhysicsWorldManifold previous,
            bool hasPrevious,
            ref CollisionInfo collision)
        {
            if (!hasPrevious
                || previous.contactCount <= 0
                || math.lengthsq(previous.normal) <= math.Epsilon
                || math.dot(previous.normal, collision.normal) >= settings.contactManifoldSettings.normalPersistenceDot)
            {
                return;
            }

            if (!TryProjectAABBPenetration(bounds0, bounds1, previous.normal, out fix previousAxisPenetration))
            {
                return;
            }

            fix normalSlop = math.max(settings.penetrationSlop, fix._0_01) * fix._4;
            if (previousAxisPenetration > collision.penetrationDepth + normalSlop)
            {
                return;
            }

            collision.normal = previous.normal;
            collision.penetrationDepth = previousAxisPenetration;
        }

        private void BuildIslands()
        {
            islandCount = 0;
            sleepingIslandCount = 0;

            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                bodyIslandIds[i] = -1;
                islandRootToId[i] = -1;
                islandBodyCounts[i] = 0;
                islandSleepingBodyCounts[i] = 0;

                if (bodyAlive[i] && bodies[i].IsDynamic)
                {
                    islandParent[i] = i;
                    islandRank[i] = 0;
                }
                else
                {
                    islandParent[i] = -1;
                    islandRank[i] = 0;
                }
            }

            for (int i = 0; i < contactManifoldCount; i++)
            {
                PhysicsWorldManifold manifold = manifolds[i];
                if (manifold.isTrigger)
                {
                    continue;
                }

                TryUnionIslandBodies(manifold.body0, manifold.body1);
            }

            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i] || !bodies[i].IsDynamic)
                {
                    continue;
                }

                int root = FindIslandRoot(i);
                int id = islandRootToId[root];
                if (id < 0)
                {
                    id = islandCount++;
                    islandRootToId[root] = id;
                }

                bodyIslandIds[i] = id;
                islandBodyCounts[id]++;
                if (bodies[i].isSleeping)
                {
                    islandSleepingBodyCounts[id]++;
                }
            }

            for (int i = 0; i < islandCount; i++)
            {
                if (islandBodyCounts[i] > 0 && islandBodyCounts[i] == islandSleepingBodyCounts[i])
                {
                    sleepingIslandCount++;
                }
            }
        }

        private void TryUnionIslandBodies(BodyHandle body0, BodyHandle body1)
        {
            if (!IsBodyAlive(body0) || !IsBodyAlive(body1) || body0 == body1)
            {
                return;
            }

            PhysicsWorldBody rigid0 = bodies[body0.index];
            PhysicsWorldBody rigid1 = bodies[body1.index];
            if (!rigid0.enabled || !rigid1.enabled || (!rigid0.IsDynamic && !rigid1.IsDynamic))
            {
                return;
            }

            if (rigid0.IsDynamic && rigid1.IsDynamic)
            {
                UnionIslandRoots(body0.index, body1.index);
            }
        }

        private void WakeSleepingBodiesForActiveContacts()
        {
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i])
                {
                    continue;
                }

                PhysicsWorldBody body = bodies[i];
                if (body.IsDynamic && body.isSleeping && HasWakeMotion(body))
                {
                    WakeBody(ref body);
                    bodies[i] = body;
                }
            }

            bool changed;
            do
            {
                changed = false;
                for (int i = 0; i < contactManifoldCount; i++)
                {
                    PhysicsWorldManifold manifold = manifolds[i];
                    if (manifold.isTrigger)
                    {
                        continue;
                    }

                    changed |= TryWakeSleepingNeighbor(manifold.body0, manifold.body1);
                    changed |= TryWakeSleepingNeighbor(manifold.body1, manifold.body0);
                }
            }
            while (changed);
        }

        private bool TryWakeSleepingNeighbor(BodyHandle sleepingBodyHandle, BodyHandle neighborHandle)
        {
            if (!IsBodyAlive(sleepingBodyHandle)
                || !IsBodyAlive(neighborHandle)
                || !bodies[sleepingBodyHandle.index].IsDynamic
                || !bodies[sleepingBodyHandle.index].isSleeping
                || !ShouldWakeSleepingNeighbor(bodies[neighborHandle.index]))
            {
                return false;
            }

            PhysicsWorldBody sleepingBody = bodies[sleepingBodyHandle.index];
            WakeBody(ref sleepingBody);
            bodies[sleepingBodyHandle.index] = sleepingBody;
            return true;
        }

        private void UpdateSleeping(fix deltaTime)
        {
            if (!settings.enableSleeping)
            {
                WakeAllSleepingBodies();
                return;
            }

            fix sleepDuration = math.max(fix.Zero, settings.sleepTime);
            for (int island = 0; island < islandCount; island++)
            {
                if (!CanIslandSleep(island))
                {
                    WakeIsland(island);
                    continue;
                }

                bool shouldSleep = true;
                for (int i = 0; i < bodyHighWaterMark; i++)
                {
                    if (!bodyAlive[i] || bodyIslandIds[i] != island || !bodies[i].IsDynamic)
                    {
                        continue;
                    }

                    PhysicsWorldBody body = bodies[i];
                    body.sleepTime += deltaTime;
                    if (body.sleepTime < sleepDuration)
                    {
                        shouldSleep = false;
                    }

                    bodies[i] = body;
                }

                if (shouldSleep)
                {
                    SleepIsland(island);
                }
            }

            RecountSleepingIslands();
        }

        private bool CanIslandSleep(int island)
        {
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i] || bodyIslandIds[i] != island || !bodies[i].IsDynamic)
                {
                    continue;
                }

                if (!CanBodySleep(bodies[i]))
                {
                    return false;
                }
            }

            return true;
        }

        private bool CanBodySleep(PhysicsWorldBody body)
        {
            if (!body.allowSleep)
            {
                return false;
            }

            fix linearThreshold = math.max(fix.Zero, settings.linearSleepThreshold);
            fix angularThreshold = math.max(fix.Zero, settings.angularSleepThreshold);
            return math.lengthsq(body.velocity) <= linearThreshold * linearThreshold
                && math.lengthsq(body.angularVelocity) <= angularThreshold * angularThreshold;
        }

        private void SleepIsland(int island)
        {
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i] || bodyIslandIds[i] != island)
                {
                    continue;
                }

                PhysicsWorldBody body = bodies[i];
                SleepBody(ref body, clearVelocity: true);
                bodies[i] = body;
            }
        }

        private void WakeIsland(int island)
        {
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i] || bodyIslandIds[i] != island)
                {
                    continue;
                }

                PhysicsWorldBody body = bodies[i];
                WakeBody(ref body);
                bodies[i] = body;
            }
        }

        private void WakeAllSleepingBodies()
        {
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i] || !bodies[i].isSleeping)
                {
                    continue;
                }

                PhysicsWorldBody body = bodies[i];
                WakeBody(ref body);
                bodies[i] = body;
            }

            RecountSleepingIslands();
        }

        private void RecountSleepingIslands()
        {
            for (int i = 0; i < islandCount; i++)
            {
                islandSleepingBodyCounts[i] = 0;
            }

            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i] || bodyIslandIds[i] < 0 || !bodies[i].isSleeping)
                {
                    continue;
                }

                islandSleepingBodyCounts[bodyIslandIds[i]]++;
            }

            sleepingIslandCount = 0;
            for (int i = 0; i < islandCount; i++)
            {
                if (islandBodyCounts[i] > 0 && islandBodyCounts[i] == islandSleepingBodyCounts[i])
                {
                    sleepingIslandCount++;
                }
            }
        }

        private void SolveVelocity(fix deltaTime)
        {
            WarmStartContacts();
            solverContactPointCount = CountSolverContactPoints();
            int solverIterations = settings.solverIterations > 0 ? settings.solverIterations : 1;
            for (int iteration = 0; iteration < solverIterations; iteration++)
            {
                for (int i = 0; i < contactManifoldCount; i++)
                {
                    ref PhysicsWorldManifold manifold = ref manifolds[i];
                    if (manifold.isTrigger)
                    {
                        continue;
                    }

                    for (int j = 0; j < manifold.contactCount; j++)
                    {
                        PhysicsWorldContactPoint point = manifold[j];
                        ResolveContactVelocity(ref manifold, ref point, deltaTime);
                        manifold.SetPointForSolver(j, point);
                    }
                }
            }
        }

        private void WarmStartContacts()
        {
            fix warmStartScale = math.clamp(settings.warmStartScale, fix.Zero, fix.One);
            if (warmStartScale <= fix.Zero)
            {
                return;
            }

            for (int i = 0; i < contactManifoldCount; i++)
            {
                ref PhysicsWorldManifold manifold = ref manifolds[i];
                if (manifold.isTrigger)
                {
                    continue;
                }

                for (int j = 0; j < manifold.contactCount; j++)
                {
                    PhysicsWorldContactPoint point = manifold[j];
                    if (point.normalImpulse <= fix.Zero
                        && math.abs(point.tangentImpulse0) <= math.Epsilon
                        && math.abs(point.tangentImpulse1) <= math.Epsilon)
                    {
                        continue;
                    }

                    bool hasBody0 = TryGetBodyForSolver(manifold.body0, out PhysicsWorldBody body0, out fix inverseMass0);
                    bool hasBody1 = TryGetBodyForSolver(manifold.body1, out PhysicsWorldBody body1, out fix inverseMass1);
                    if (!hasBody0 && !hasBody1)
                    {
                        continue;
                    }

                    fix3 r0 = hasBody0 ? point.position - body0.position : fix3.zero;
                    fix3 r1 = hasBody1 ? point.position - body1.position : fix3.zero;
                    fix3 impulse = manifold.normal * (point.normalImpulse * warmStartScale);
                    if (math.lengthsq(point.tangent0) > math.Epsilon)
                    {
                        impulse += point.tangent0 * (point.tangentImpulse0 * warmStartScale);
                    }

                    if (math.lengthsq(point.tangent1) > math.Epsilon)
                    {
                        impulse += point.tangent1 * (point.tangentImpulse1 * warmStartScale);
                    }

                    ApplyImpulse(
                        hasBody0,
                        manifold.body0,
                        ref body0,
                        inverseMass0,
                        r0,
                        hasBody1,
                        manifold.body1,
                        ref body1,
                        inverseMass1,
                        r1,
                        impulse);
                }
            }
        }

        private void ResolveContactVelocity(
            ref PhysicsWorldManifold manifold,
            ref PhysicsWorldContactPoint point,
            fix deltaTime)
        {
            bool hasBody0 = TryGetBodyForSolver(manifold.body0, out PhysicsWorldBody body0, out fix inverseMass0);
            bool hasBody1 = TryGetBodyForSolver(manifold.body1, out PhysicsWorldBody body1, out fix inverseMass1);
            if (!hasBody0 && !hasBody1)
            {
                return;
            }

            fix3 r0 = hasBody0 ? point.position - body0.position : fix3.zero;
            fix3 r1 = hasBody1 ? point.position - body1.position : fix3.zero;
            fix3 velocity0 = hasBody0 ? GetVelocityAtPoint(body0, r0) : fix3.zero;
            fix3 velocity1 = hasBody1 ? GetVelocityAtPoint(body1, r1) : fix3.zero;
            fix3 relativeVelocity = velocity1 - velocity0;
            fix normalVelocity = math.dot(relativeVelocity, manifold.normal);

            fix effectiveMass = inverseMass0 + inverseMass1
                + GetAngularEffectiveMass(body0, r0, manifold.normal)
                + GetAngularEffectiveMass(body1, r1, manifold.normal);
            if (effectiveMass <= fix.Zero)
            {
                return;
            }

            fix targetNormalVelocity = ComputeContactTargetNormalVelocity(manifold, point, normalVelocity, deltaTime);
            if (normalVelocity >= targetNormalVelocity && point.lifetime <= 1)
            {
                return;
            }

            fix normalImpulseDelta = (targetNormalVelocity - normalVelocity) / effectiveMass;
            fix previousNormalImpulse = point.normalImpulse;
            point.normalImpulse = math.max(fix.Zero, previousNormalImpulse + normalImpulseDelta);
            fix maxContactImpulse = math.max(fix.Zero, settings.maxContactImpulse);
            if (maxContactImpulse > fix.Zero)
            {
                point.normalImpulse = math.min(point.normalImpulse, maxContactImpulse);
            }

            fix3 impulse = manifold.normal * (point.normalImpulse - previousNormalImpulse);
            ApplyImpulse(
                hasBody0,
                manifold.body0,
                ref body0,
                inverseMass0,
                r0,
                hasBody1,
                manifold.body1,
                ref body1,
                inverseMass1,
                r1,
                impulse);

            ResolveContactFriction(
                ref manifold,
                ref point,
                hasBody0,
                manifold.body0,
                ref body0,
                inverseMass0,
                r0,
                hasBody1,
                manifold.body1,
                ref body1,
                inverseMass1,
                r1);
        }

        private fix ComputeContactTargetNormalVelocity(
            PhysicsWorldManifold manifold,
            PhysicsWorldContactPoint point,
            fix normalVelocity,
            fix deltaTime)
        {
            fix targetVelocity = fix.Zero;
            fix restitutionThreshold = math.max(fix.Zero, settings.restitutionVelocityThreshold);
            if (normalVelocity < -restitutionThreshold)
            {
                targetVelocity = -GetCombinedRestitution(manifold) * normalVelocity;
            }

            fix biasFactor = math.max(fix.Zero, settings.contactVelocityBiasFactor);
            fix penetration = point.penetrationDepth - settings.penetrationSlop;
            if (biasFactor > fix.Zero && penetration > fix.Zero && deltaTime > fix.Zero)
            {
                fix biasVelocity = penetration * biasFactor / deltaTime;
                fix maxBiasVelocity = math.max(fix.Zero, settings.maxContactBiasVelocity);
                if (maxBiasVelocity > fix.Zero)
                {
                    biasVelocity = math.min(biasVelocity, maxBiasVelocity);
                }

                targetVelocity = math.max(targetVelocity, biasVelocity);
            }

            return targetVelocity;
        }

        private void ResolveContactFriction(
            ref PhysicsWorldManifold manifold,
            ref PhysicsWorldContactPoint point,
            bool hasBody0,
            BodyHandle handle0,
            ref PhysicsWorldBody body0,
            fix inverseMass0,
            fix3 r0,
            bool hasBody1,
            BodyHandle handle1,
            ref PhysicsWorldBody body1,
            fix inverseMass1,
            fix3 r1)
        {
            fix friction = GetCombinedFriction(manifold);
            if (friction <= fix.Zero || point.normalImpulse <= fix.Zero)
            {
                return;
            }

            fix3 velocity0 = hasBody0 ? GetVelocityAtPoint(body0, r0) : fix3.zero;
            fix3 velocity1 = hasBody1 ? GetVelocityAtPoint(body1, r1) : fix3.zero;
            fix3 relativeVelocity = velocity1 - velocity0;
            fix normalVelocity = math.dot(relativeVelocity, manifold.normal);
            fix3 tangentVelocity = relativeVelocity - manifold.normal * normalVelocity;
            BuildContactTangentBasis(manifold.normal, out fix3 tangent0, out fix3 tangent1);

            if (math.lengthsq(point.tangent0) <= math.Epsilon
                || math.dot(point.tangent0, tangent0) < fix._0_5
                || math.dot(point.tangent1, tangent1) < fix._0_5)
            {
                point.tangentImpulse0 = fix.Zero;
                point.tangentImpulse1 = fix.Zero;
            }

            point.tangent0 = tangent0;
            point.tangent1 = tangent1;

            if (math.lengthsq(tangentVelocity) <= math.Epsilon
                && math.abs(point.tangentImpulse0) <= math.Epsilon
                && math.abs(point.tangentImpulse1) <= math.Epsilon)
            {
                return;
            }

            fix effectiveMass0 = inverseMass0 + inverseMass1
                + GetAngularEffectiveMass(body0, r0, tangent0)
                + GetAngularEffectiveMass(body1, r1, tangent0);
            fix effectiveMass1 = inverseMass0 + inverseMass1
                + GetAngularEffectiveMass(body0, r0, tangent1)
                + GetAngularEffectiveMass(body1, r1, tangent1);
            if (effectiveMass0 <= fix.Zero && effectiveMass1 <= fix.Zero)
            {
                return;
            }

            fix frictionDelta0 = effectiveMass0 > fix.Zero ? -math.dot(relativeVelocity, tangent0) / effectiveMass0 : fix.Zero;
            fix frictionDelta1 = effectiveMass1 > fix.Zero ? -math.dot(relativeVelocity, tangent1) / effectiveMass1 : fix.Zero;
            fix maxFriction = point.normalImpulse * friction;
            fix maxFrictionImpulse = math.max(fix.Zero, settings.maxFrictionImpulse);
            if (maxFrictionImpulse > fix.Zero)
            {
                maxFriction = math.min(maxFriction, maxFrictionImpulse);
            }

            fix previousTangentImpulse0 = point.tangentImpulse0;
            fix previousTangentImpulse1 = point.tangentImpulse1;
            fix nextTangentImpulse0 = previousTangentImpulse0 + frictionDelta0;
            fix nextTangentImpulse1 = previousTangentImpulse1 + frictionDelta1;
            ClampFrictionCircle(ref nextTangentImpulse0, ref nextTangentImpulse1, maxFriction);

            point.tangentImpulse0 = nextTangentImpulse0;
            point.tangentImpulse1 = nextTangentImpulse1;
            fix3 frictionImpulse = tangent0 * (nextTangentImpulse0 - previousTangentImpulse0)
                + tangent1 * (nextTangentImpulse1 - previousTangentImpulse1);

            ApplyImpulse(
                hasBody0,
                handle0,
                ref body0,
                inverseMass0,
                r0,
                hasBody1,
                handle1,
                ref body1,
                inverseMass1,
                r1,
                frictionImpulse);
        }

        private fix GetCombinedRestitution(PhysicsWorldManifold manifold)
        {
            fix restitution = math.max(fix.Zero, settings.restitution);
            if (IsColliderAlive(manifold.collider0))
            {
                restitution = math.max(restitution, colliders[manifold.collider0.index].material.GetBounciness());
            }

            if (IsColliderAlive(manifold.collider1))
            {
                restitution = math.max(restitution, colliders[manifold.collider1.index].material.GetBounciness());
            }

            return math.clamp(restitution, fix.Zero, fix.One);
        }

        private fix GetCombinedFriction(PhysicsWorldManifold manifold)
        {
            fix friction = math.max(fix.Zero, settings.friction);
            bool hasMaterial = false;
            fix materialFriction = fix.Zero;

            if (IsColliderAlive(manifold.collider0))
            {
                materialFriction += colliders[manifold.collider0.index].material.GetFrictionCoefficient();
                hasMaterial = true;
            }

            if (IsColliderAlive(manifold.collider1))
            {
                materialFriction += colliders[manifold.collider1.index].material.GetFrictionCoefficient();
                hasMaterial = true;
            }

            if (hasMaterial)
            {
                materialFriction *= fix._0_5;
                friction = math.max(friction, materialFriction);
            }

            return math.max(fix.Zero, friction);
        }

        private void SolvePositions()
        {
            int positionIterations = settings.positionIterations > 0 ? settings.positionIterations : 1;
            for (int iteration = 0; iteration < positionIterations; iteration++)
            {
                CorrectContactPositions();
            }
        }

        private void CorrectContactPositions()
        {
            for (int i = 0; i < contactManifoldCount; i++)
            {
                PhysicsWorldManifold manifold = manifolds[i];
                if (manifold.isTrigger)
                {
                    continue;
                }

                bool hasBody0 = TryGetBodyForSolver(manifold.body0, out PhysicsWorldBody body0, out fix inverseMass0);
                bool hasBody1 = TryGetBodyForSolver(manifold.body1, out PhysicsWorldBody body1, out fix inverseMass1);
                fix3 originalPosition0 = body0.position;
                fix3 originalPosition1 = body1.position;
                if (!hasBody0 && !hasBody1)
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
                    PhysicsWorldContactPoint point = manifold[j];
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
                    if (hasBody0 && inverseMass0 > fix.Zero)
                    {
                        body0.position -= correction * inverseMass0;
                    }

                    if (hasBody1 && inverseMass1 > fix.Zero)
                    {
                        body1.position += correction * inverseMass1;
                    }
                }

                if (hasBody0)
                {
                    bodies[manifold.body0.index] = body0;
                    if (body0.position != originalPosition0)
                    {
                        MarkBodyDirty(manifold.body0);
                    }
                }

                if (hasBody1)
                {
                    bodies[manifold.body1.index] = body1;
                    if (body1.position != originalPosition1)
                    {
                        MarkBodyDirty(manifold.body1);
                    }
                }
            }
        }

        private bool TryGetBodyForSolver(BodyHandle handle, out PhysicsWorldBody body, out fix inverseMass)
        {
            if (!IsBodyAlive(handle))
            {
                body = default;
                inverseMass = fix.Zero;
                return false;
            }

            body = bodies[handle.index];
            inverseMass = body.inverseMass;
            return body.enabled;
        }

        private void ApplyImpulse(
            bool hasBody0,
            BodyHandle handle0,
            ref PhysicsWorldBody body0,
            fix inverseMass0,
            fix3 relativePoint0,
            bool hasBody1,
            BodyHandle handle1,
            ref PhysicsWorldBody body1,
            fix inverseMass1,
            fix3 relativePoint1,
            fix3 impulse)
        {
            if (impulse == fix3.zero)
            {
                return;
            }

            if (hasBody0 && inverseMass0 > fix.Zero)
            {
                body0.velocity -= impulse * inverseMass0;
                body0.angularVelocity -= body0.inverseInertia * math.cross(relativePoint0, impulse);
                bodies[handle0.index] = body0;
            }

            if (hasBody1 && inverseMass1 > fix.Zero)
            {
                body1.velocity += impulse * inverseMass1;
                body1.angularVelocity += body1.inverseInertia * math.cross(relativePoint1, impulse);
                bodies[handle1.index] = body1;
            }
        }

        private void SanitizeBodies(fix deltaTime)
        {
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i])
                {
                    continue;
                }

                PhysicsWorldBody body = bodies[i];
                fix3 originalPosition = body.position;
                quaternion originalRotation = body.rotation;
                body.position = PhysicsSafety.Sanitize(body.position);
                body.velocity = PhysicsSafety.Sanitize(body.velocity);
                body.angularVelocity = PhysicsSafety.Sanitize(body.angularVelocity);
                body.mass = PhysicsSafety.Sanitize(body.mass);
                body.inertia = PhysicsSafety.Sanitize(body.inertia);
                body.sleepTime = PhysicsSafety.Sanitize(body.sleepTime);
                if (body.mass < fix.Zero)
                {
                    body.mass = fix.Zero;
                }

                body.inertia = new fix3(
                    math.max(fix.Zero, body.inertia.x),
                    math.max(fix.Zero, body.inertia.y),
                    math.max(fix.Zero, body.inertia.z));
                if (body.sleepTime < fix.Zero)
                {
                    body.sleepTime = fix.Zero;
                }

                ClampBodyMotion(ref body, deltaTime);
                bodies[i] = body;
                if (body.position != originalPosition || body.rotation != originalRotation)
                {
                    MarkBodyDirtyAt(i);
                }
            }
        }

        private bool TryFindPreviousManifold(PhysicsWorldPair pair, out PhysicsWorldManifold manifold)
        {
            for (int i = 0; i < contactManifoldCount; i++)
            {
                PhysicsWorldManifold candidate = manifolds[i];
                if (candidate.collider0 == pair.collider0 && candidate.collider1 == pair.collider1)
                {
                    manifold = candidate;
                    return true;
                }
            }

            manifold = default;
            return false;
        }

        private void UpdateStepStats()
        {
            CountBodyStateStats(out int sleepingBodyCount, out int awakeDynamicBodyCount);
            lastStepStats = new PhysicsWorldStepStats(
                activeBodyCount,
                activeColliderCount,
                bodies.Length,
                colliders.Length,
                pairs.Length,
                manifolds.Length,
                bodySyncCount,
                colliderSyncCount,
                broadphaseTree.ProxyCount,
                broadphaseTree.Height,
                broadphaseTree.MaxBalance,
                broadphaseTreeOverflowed,
                broadphaseMovedProxyCount,
                broadphaseCandidateCount,
                broadphaseFilteredCandidateCount,
                pairCount,
                pairOverflowed,
                narrowPhaseTestCount,
                contactManifoldCount,
                contactManifoldNewCount,
                contactManifoldReusedCount,
                contactManifoldDroppedCount,
                contactOverflowed,
                solverContactPointCount,
                islandCount,
                sleepingIslandCount,
                sleepingBodyCount,
                awakeDynamicBodyCount,
                settings.solverIterations > 0 ? settings.solverIterations : 1,
                settings.positionIterations > 0 ? settings.positionIterations : 1,
                fixedStepCount);
        }

        private int CountSolverContactPoints()
        {
            int count = 0;
            for (int i = 0; i < contactManifoldCount; i++)
            {
                if (!manifolds[i].isTrigger)
                {
                    count += manifolds[i].contactCount;
                }
            }

            return count;
        }

        private void CountBodyStateStats(out int sleepingBodyCount, out int awakeDynamicBodyCount)
        {
            sleepingBodyCount = 0;
            awakeDynamicBodyCount = 0;
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i])
                {
                    continue;
                }

                PhysicsWorldBody body = bodies[i];
                if (body.isSleeping)
                {
                    sleepingBodyCount++;
                }

                if (body.IsAwakeDynamic)
                {
                    awakeDynamicBodyCount++;
                }
            }
        }

        private void ResetSyncStats()
        {
            bodySyncCount = 0;
            colliderSyncCount = 0;
            broadphaseMovedProxyCount = 0;
        }

        private void MarkBodyDirty(BodyHandle handle)
        {
            if (handle.index >= 0 && handle.index < bodyDirty.Length)
            {
                bodyDirty[handle.index] = true;
            }
        }

        private void MarkBodyDirtyAt(int index)
        {
            if (index >= 0 && index < bodyDirty.Length)
            {
                bodyDirty[index] = true;
            }
        }

        private void MarkColliderBroadphaseDirty(int index)
        {
            if (index >= 0 && index < colliderBroadphaseDirty.Length)
            {
                colliderBroadphaseDirty[index] = true;
            }
        }

        private int FindIslandRoot(int index)
        {
            int parent = islandParent[index];
            if (parent < 0 || parent == index)
            {
                return index;
            }

            int root = FindIslandRoot(parent);
            islandParent[index] = root;
            return root;
        }

        private void UnionIslandRoots(int index0, int index1)
        {
            int root0 = FindIslandRoot(index0);
            int root1 = FindIslandRoot(index1);
            if (root0 == root1)
            {
                return;
            }

            if (islandRank[root0] < islandRank[root1])
            {
                islandParent[root0] = root1;
            }
            else if (islandRank[root0] > islandRank[root1])
            {
                islandParent[root1] = root0;
            }
            else
            {
                islandParent[root1] = root0;
                islandRank[root0]++;
            }
        }

        private void ApplyBodyMassProperties(BodyHandle handle)
        {
            if (!IsBodyAlive(handle))
            {
                return;
            }

            PhysicsWorldBody body = bodies[handle.index];
            if (!body.autoMass && !body.autoInertia)
            {
                return;
            }

            fix totalMass = fix.Zero;
            fix3 totalInertia = fix3.zero;
            bool hasMassProperties = false;

            for (int i = 0; i < colliderHighWaterMark; i++)
            {
                if (!colliderAlive[i] || colliders[i].body != handle)
                {
                    continue;
                }

                PhysicsWorldCollider collider = colliders[i];
                MassProperties massProperties = Physics.ComputeMassProperties(collider.localShape, collider.material.GetDensity());
                if (massProperties.mass <= fix.Zero)
                {
                    continue;
                }

                hasMassProperties = true;
                totalMass += massProperties.mass;
                totalInertia += ApplyParallelAxis(massProperties, collider.localShape.GetLocalCenter());
            }

            if (!hasMassProperties)
            {
                return;
            }

            if (body.autoMass)
            {
                body.mass = totalMass;
            }

            if (body.autoInertia)
            {
                body.inertia = totalInertia;
            }

            WakeBody(ref body);
            bodies[handle.index] = body;
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

        private void WakeBody(ref PhysicsWorldBody body)
        {
            body.isSleeping = false;
            body.sleepTime = fix.Zero;
        }

        private void SleepBody(ref PhysicsWorldBody body, bool clearVelocity)
        {
            if (!body.IsDynamic || !body.allowSleep)
            {
                return;
            }

            body.isSleeping = true;
            body.sleepTime = fix.Zero;
            if (clearVelocity)
            {
                body.velocity = fix3.zero;
                body.angularVelocity = fix3.zero;
            }
        }

        private static bool HasWakeMotion(PhysicsWorldBody body)
        {
            return math.lengthsq(body.velocity) > math.Epsilon
                || math.lengthsq(body.angularVelocity) > math.Epsilon;
        }

        private static bool ShouldWakeSleepingNeighbor(PhysicsWorldBody neighbor)
        {
            if (!neighbor.enabled)
            {
                return false;
            }

            if (neighbor.IsDynamic)
            {
                return !neighbor.isSleeping;
            }

            return neighbor.IsKinematic && HasWakeMotion(neighbor);
        }

        private bool IsAwakeDynamicBody(BodyHandle handle)
        {
            return IsBodyAlive(handle) && bodies[handle.index].IsAwakeDynamic;
        }

        private void ClampBodyMotion(ref PhysicsWorldBody body, fix deltaTime)
        {
            body.velocity = ClampLinearVelocityForStep(body.velocity, deltaTime);
            body.angularVelocity = ClampAngularVelocityForStep(body.angularVelocity, deltaTime);
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

        private void ClearCollisionCaches()
        {
            broadphaseTreeOverflowed = false;
            pairCount = 0;
            pairOverflowed = false;
            ClearContacts();
        }

        private void ClearContacts()
        {
            contactManifoldCount = 0;
            contactOverflowed = false;
        }

        private void RemoveBodyBroadphaseProxies(BodyHandle body)
        {
            for (int i = 0; i < colliderHighWaterMark; i++)
            {
                if (colliderAlive[i] && colliders[i].body == body)
                {
                    broadphaseTree.RemoveProxy(colliders[i].handle);
                    colliderBroadphaseDirty[i] = false;
                }
            }
        }

        private bool IsColliderActive(int index)
        {
            if (!colliderAlive[index])
            {
                return false;
            }

            PhysicsWorldCollider collider = colliders[index];
            return collider.enabled && IsBodyEnabled(collider.body);
        }

        private bool IsColliderQueryable(int index, int layerMask)
        {
            if (!IsColliderActive(index))
            {
                return false;
            }

            return IsLayerEnabled(layerMask, colliders[index].layer);
        }

        private bool IsBodyEnabled(BodyHandle handle)
        {
            return IsBodyAlive(handle) && bodies[handle.index].enabled;
        }

        private bool IsBodyAlive(BodyHandle handle)
        {
            return handle.index >= 0
                && handle.index < bodyHighWaterMark
                && bodyAlive[handle.index]
                && bodyVersions[handle.index] == handle.version;
        }

        private bool IsColliderAlive(ColliderHandle handle)
        {
            return handle.index >= 0
                && handle.index < colliderHighWaterMark
                && colliderAlive[handle.index]
                && colliderVersions[handle.index] == handle.version;
        }

        private int FindFreeBodySlot()
        {
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                if (!bodyAlive[i])
                {
                    return i;
                }
            }

            return bodyHighWaterMark < bodies.Length ? bodyHighWaterMark : -1;
        }

        private int FindFreeColliderSlot()
        {
            for (int i = 0; i < colliderHighWaterMark; i++)
            {
                if (!colliderAlive[i])
                {
                    return i;
                }
            }

            return colliderHighWaterMark < colliders.Length ? colliderHighWaterMark : -1;
        }

        private void DestroyColliderAt(int index)
        {
            BodyHandle owner = colliders[index].body;
            broadphaseTree.RemoveProxy(colliders[index].handle);
            colliderAlive[index] = false;
            colliderDirty[index] = false;
            colliderBroadphaseDirty[index] = false;
            colliderVersions[index] = NextVersion(colliderVersions[index]);
            colliders[index] = default;
            activeColliderCount--;
            ApplyBodyMassProperties(owner);
        }

        private static bool CanCollide(PhysicsWorldCollider a, PhysicsWorldCollider b)
        {
            return a.handle != b.handle
                && a.body != b.body
                && IsLayerEnabled(a.collisionMask, b.layer)
                && IsLayerEnabled(b.collisionMask, a.layer);
        }

        private static bool IsLayerEnabled(int mask, int layer)
        {
            return (mask & Collider.GetLayerBit(layer)) != 0;
        }

        private static fix3 GetVelocityAtPoint(PhysicsWorldBody body, fix3 relativePoint)
        {
            return body.velocity + math.cross(body.angularVelocity, relativePoint);
        }

        private static fix GetAngularEffectiveMass(PhysicsWorldBody body, fix3 relativePoint, fix3 direction)
        {
            if (!body.IsDynamic)
            {
                return fix.Zero;
            }

            fix3 angularVelocityPerImpulse = body.inverseInertia * math.cross(relativePoint, direction);
            return math.dot(math.cross(angularVelocityPerImpulse, relativePoint), direction);
        }

        private static void BuildContactTangentBasis(fix3 normal, out fix3 tangent0, out fix3 tangent1)
        {
            fix3 reference = math.abs(normal.x) < fix._0_75 ? fix3.right : fix3.up;
            tangent0 = PhysicsSafety.SafeNormalize(math.cross(reference, normal), fix3.forward);
            tangent1 = PhysicsSafety.SafeNormalize(math.cross(normal, tangent0), fix3.up);
        }

        private static void ClampFrictionCircle(ref fix impulse0, ref fix impulse1, fix maxImpulse)
        {
            if (maxImpulse <= fix.Zero)
            {
                impulse0 = fix.Zero;
                impulse1 = fix.Zero;
                return;
            }

            fix magnitudeSq = impulse0 * impulse0 + impulse1 * impulse1;
            fix maxSq = maxImpulse * maxImpulse;
            if (magnitudeSq <= maxSq || magnitudeSq <= math.Epsilon)
            {
                return;
            }

            fix scale = maxImpulse / math.sqrt(magnitudeSq);
            impulse0 *= scale;
            impulse1 *= scale;
        }

        private static bool TryProjectAABBPenetration(AABB a, AABB b, fix3 axis, out fix penetration)
        {
            axis = PhysicsSafety.SafeNormalize(axis, fix3.up);
            fix center0 = math.dot(a.center, axis);
            fix center1 = math.dot(b.center, axis);
            fix radius0 = math.dot(math.abs(axis), a.extents);
            fix radius1 = math.dot(math.abs(axis), b.extents);
            fix min0 = center0 - radius0;
            fix max0 = center0 + radius0;
            fix min1 = center1 - radius1;
            fix max1 = center1 + radius1;
            penetration = math.min(max0, max1) - math.max(min0, min1);
            return penetration >= fix.Zero;
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

        private static PhysicsWorldSettings NormalizeSettings(PhysicsWorldSettings value)
        {
            if (value.timeStep <= fix.Zero)
            {
                value.timeStep = fix.One / 60;
            }

            if (value.maxSubSteps < 0)
            {
                value.maxSubSteps = 0;
            }

            if (value.solverIterations < 0)
            {
                value.solverIterations = 0;
            }

            if (value.positionIterations < 0)
            {
                value.positionIterations = 0;
            }

            if (value.maxLinearVelocity < fix.Zero)
            {
                value.maxLinearVelocity = fix.Zero;
            }

            if (value.maxAngularVelocity < fix.Zero)
            {
                value.maxAngularVelocity = fix.Zero;
            }

            if (value.maxTranslationPerStep < fix.Zero)
            {
                value.maxTranslationPerStep = fix.Zero;
            }

            if (value.maxRotationPerStep < fix.Zero)
            {
                value.maxRotationPerStep = fix.Zero;
            }

            if (value.linearSleepThreshold < fix.Zero)
            {
                value.linearSleepThreshold = fix.Zero;
            }

            if (value.angularSleepThreshold < fix.Zero)
            {
                value.angularSleepThreshold = fix.Zero;
            }

            if (value.sleepTime < fix.Zero)
            {
                value.sleepTime = fix.Zero;
            }

            if (value.ccdMinVelocity < fix.Zero)
            {
                value.ccdMinVelocity = fix.Zero;
            }

            if (value.ccdSkin < fix.Zero)
            {
                value.ccdSkin = fix.Zero;
            }

            if (value.ccdMaxIterations < 0)
            {
                value.ccdMaxIterations = 0;
            }

            return value;
        }

        private static int NextVersion(int version)
        {
            version++;
            return version > 0 ? version : 1;
        }

        private static void EnsureCapacity<T>(ref T[] array, int capacity)
        {
            if (capacity > array.Length)
            {
                Array.Resize(ref array, capacity);
            }
        }

        private static void Hash(ref ulong hash, bool value)
        {
            Hash(ref hash, value ? 1 : 0);
        }

        private static void Hash(ref ulong hash, int value)
        {
            unchecked
            {
                Hash(ref hash, (ulong)(uint)value);
            }
        }

        private static void Hash(ref ulong hash, ulong value)
        {
            unchecked
            {
                for (int i = 0; i < 8; i++)
                {
                    hash ^= (byte)(value >> (i * 8));
                    hash *= HashPrime;
                }
            }
        }

        private static void Hash(ref ulong hash, long value)
        {
            unchecked
            {
                Hash(ref hash, (ulong)value);
            }
        }

        private static void Hash(ref ulong hash, fix value)
        {
            Hash(ref hash, value.value);
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

        private static void Hash(ref ulong hash, PhysicsShapeData shape)
        {
            Hash(ref hash, (int)shape.type);
            switch (shape.type)
            {
                case ShapeType.AABB:
                    Hash(ref hash, shape.aabb.center);
                    Hash(ref hash, shape.aabb.extents);
                    break;
                case ShapeType.OBB:
                    Hash(ref hash, shape.obb.center);
                    Hash(ref hash, shape.obb.extents);
                    Hash(ref hash, shape.obb.orientation);
                    break;
                case ShapeType.Sphere:
                    Hash(ref hash, shape.sphere.Center);
                    Hash(ref hash, shape.sphere.Radius);
                    break;
                case ShapeType.Capsule:
                    Hash(ref hash, shape.capsule.Center);
                    Hash(ref hash, shape.capsule.Radius);
                    Hash(ref hash, shape.capsule.Height);
                    Hash(ref hash, shape.capsule.Orientation);
                    break;
            }
        }

        private static void Hash(ref ulong hash, PhysicsWorldManifold manifold)
        {
            Hash(ref hash, manifold.collider0.Index);
            Hash(ref hash, manifold.collider0.Version);
            Hash(ref hash, manifold.collider1.Index);
            Hash(ref hash, manifold.collider1.Version);
            Hash(ref hash, manifold.body0.Index);
            Hash(ref hash, manifold.body0.Version);
            Hash(ref hash, manifold.body1.Index);
            Hash(ref hash, manifold.body1.Version);
            Hash(ref hash, manifold.normal);
            Hash(ref hash, manifold.contactCount);
            Hash(ref hash, manifold.isTrigger);

            for (int i = 0; i < manifold.contactCount; i++)
            {
                Hash(ref hash, manifold[i]);
            }
        }

        private static void Hash(ref ulong hash, PhysicsWorldContactPoint point)
        {
            Hash(ref hash, point.position);
            Hash(ref hash, point.pointOnCollider0);
            Hash(ref hash, point.pointOnCollider1);
            Hash(ref hash, point.penetrationDepth);
            Hash(ref hash, point.normalImpulse);
            Hash(ref hash, point.tangentImpulse0);
            Hash(ref hash, point.tangentImpulse1);
            Hash(ref hash, point.tangent0);
            Hash(ref hash, point.tangent1);
            Hash(ref hash, point.featureId);
            Hash(ref hash, point.lifetime);
        }

        private readonly struct ContinuousHit
        {
            public readonly fix fraction;
            public readonly fix3 normal;

            public ContinuousHit(fix fraction, fix3 normal)
            {
                this.fraction = fraction;
                this.normal = normal;
            }
        }
    }
}
