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
            return colliders.Remove(colliderId);
        }

        public bool TestCollision(Collider a, Collider b)
        {
            return collisionSystem.TestCollision(a, b);
        }

        private Collider RegisterCollider(Collider collider)
        {
            colliders[collider.id] = collider;

            if (rigids.TryGetValue(collider.rigidId, out Rigid rigid))
            {
                rigid.SetCollider(collider.id);
                rigids[rigid.id] = rigid;
            }

            return collider;
        }
    }
}
