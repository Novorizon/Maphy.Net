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
        private readonly List<ulong> colliderIds = new List<ulong>();
        private readonly CollisionSystem collisionSystem = new CollisionSystem();

        public IReadOnlyDictionary<ulong, Entity> Entities => entities;
        public IReadOnlyDictionary<ulong, Rigid> Rigids => rigids;
        public IReadOnlyDictionary<ulong, Collider> Colliders => colliders;
        public IReadOnlyList<BroadCollisionPair> BroadphasePairs => collisionSystem.BroadphasePairs;
        public IReadOnlyList<NarrowCollisionSystem.CollisionPair> CollisionPairs => collisionSystem.CollisionPairs;

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
            SyncColliders();
            collisionSystem.Collision(colliders.Values);
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
