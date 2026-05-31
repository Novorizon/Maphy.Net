using Maphy.Mathematics;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public class World
    {
        private ulong id = 10000000;
        public WorldSettings settings;
        private readonly Dictionary<ulong, Entity> entities;
        private readonly Dictionary<ulong, Rigid> rigids = new Dictionary<ulong, Rigid>();
        private readonly Dictionary<ulong, Collider> colliders = new Dictionary<ulong, Collider>();
        private readonly List<ulong> rigidIds = new List<ulong>();
        private readonly List<ulong> colliderIds = new List<ulong>();
        private readonly CollisionSystem collisionSystem = new CollisionSystem();

        public IReadOnlyDictionary<ulong, Entity> Entities => entities;
        public IReadOnlyDictionary<ulong, Rigid> Rigids => rigids;
        public IReadOnlyDictionary<ulong, Collider> Colliders => colliders;
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
            collisionSystem.Collision(colliders.Values);
            ResolveContacts();
            CorrectPositions();
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

        public bool SetMass(ulong rigidId, fix mass)
        {
            if (mass < fix.Zero || !rigids.TryGetValue(rigidId, out Rigid rigid))
            {
                return false;
            }

            rigid.mass = mass;
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

        public bool SetColliderTrigger(ulong colliderId, bool isTrigger)
        {
            if (!colliders.TryGetValue(colliderId, out Collider collider))
            {
                return false;
            }

            collider.SetTrigger(isTrigger);
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
            if (!colliders.Remove(colliderId))
            {
                return false;
            }

            colliderIds.Remove(colliderId);
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

        private void IntegrateForces(fix deltaTime)
        {
            for (int i = 0; i < rigidIds.Count; i++)
            {
                ulong rigidId = rigidIds[i];
                if (!rigids.TryGetValue(rigidId, out Rigid rigid))
                {
                    continue;
                }

                if (!rigid.IsDynamic || rigid.inverseMass == fix.Zero)
                {
                    rigid.ClearForces();
                    rigids[rigidId] = rigid;
                    continue;
                }

                fix3 acceleration = rigid.acceleration + rigid.force * rigid.inverseMass;
                if (rigid.useGravity && settings.enableGravity)
                {
                    acceleration += new fix3(fix.Zero, settings.gravity, fix.Zero);
                }

                rigid.velocity += acceleration * deltaTime;
                rigid.ClearForces();
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
                entities[rigidId] = entity;
            }
        }

        private void ResolveContacts()
        {
            int solverIterations = settings.solverIterations > 0 ? settings.solverIterations : 1;
            IReadOnlyList<ContactManifold> manifolds = collisionSystem.ContactManifolds;
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
                        ResolveContactVelocity(manifold);
                    }
                }
            }
        }

        private void ResolveContactVelocity(ContactManifold manifold)
        {
            bool hasRigid0 = TryGetRigidMassData(manifold.rigidId0, out Rigid rigid0, out fix inverseMass0);
            bool hasRigid1 = TryGetRigidMassData(manifold.rigidId1, out Rigid rigid1, out fix inverseMass1);
            if (!hasRigid0 && !hasRigid1)
            {
                return;
            }

            fix inverseMassSum = inverseMass0 + inverseMass1;
            if (inverseMassSum <= fix.Zero)
            {
                return;
            }

            fix3 velocity0 = hasRigid0 ? rigid0.velocity : fix3.zero;
            fix3 velocity1 = hasRigid1 ? rigid1.velocity : fix3.zero;
            fix3 relativeVelocity = velocity1 - velocity0;
            fix normalVelocity = math.dot(relativeVelocity, manifold.normal);
            if (normalVelocity > fix.Zero)
            {
                return;
            }

            fix normalImpulseMagnitude = -(fix.One + settings.restitution) * normalVelocity / inverseMassSum / manifold.contactCount;
            fix3 impulse = manifold.normal * normalImpulseMagnitude;

            if (hasRigid0 && inverseMass0 > fix.Zero)
            {
                rigid0.velocity -= impulse * inverseMass0;
            }

            if (hasRigid1 && inverseMass1 > fix.Zero)
            {
                rigid1.velocity += impulse * inverseMass1;
            }

            ResolveContactFriction(manifold, hasRigid0, rigid0, inverseMass0, hasRigid1, rigid1, inverseMass1, inverseMassSum, normalImpulseMagnitude);

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
            bool hasRigid0,
            Rigid rigid0,
            fix inverseMass0,
            bool hasRigid1,
            Rigid rigid1,
            fix inverseMass1,
            fix inverseMassSum,
            fix normalImpulseMagnitude)
        {
            fix friction = math.max(fix.Zero, settings.friction);
            if (friction <= fix.Zero || normalImpulseMagnitude <= fix.Zero)
            {
                return;
            }

            fix3 velocity0 = hasRigid0 ? rigid0.velocity : fix3.zero;
            fix3 velocity1 = hasRigid1 ? rigid1.velocity : fix3.zero;
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
            fix frictionMagnitude = -math.dot(relativeVelocity, tangent) / inverseMassSum / manifold.contactCount;
            fix maxFriction = normalImpulseMagnitude * friction;
            frictionMagnitude = math.clamp(frictionMagnitude, -maxFriction, maxFriction);
            fix3 frictionImpulse = tangent * frictionMagnitude;

            if (hasRigid0 && inverseMass0 > fix.Zero)
            {
                rigid0.velocity -= frictionImpulse * inverseMass0;
            }

            if (hasRigid1 && inverseMass1 > fix.Zero)
            {
                rigid1.velocity += frictionImpulse * inverseMass1;
            }
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

                    fix3 correction = manifold.normal * (penetration * settings.positionCorrectionPercent / inverseMassSum);
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
            IReadOnlyList<NarrowCollisionSystem.CollisionPair> pairs = collisionSystem.CollisionPairs;
            for (int i = 0; i < pairs.Count; i++)
            {
                NarrowCollisionSystem.CollisionPair pair = pairs[i];
                CollisionInfo collision0 = pair.collision;
                CollisionInfo collision1 = collision0.Flipped();

                pair.collider0.collision = collision0;
                pair.collider1.collision = collision1;

                if (rigids.TryGetValue(pair.rigidId0, out Rigid rigid0))
                {
                    rigid0.Listener(collision0);
                }

                if (rigids.TryGetValue(pair.rigidId1, out Rigid rigid1))
                {
                    rigid1.Listener(collision1);
                }
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
            }

            return collider;
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
