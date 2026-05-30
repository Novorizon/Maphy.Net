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
            CollisionSystem.Collision();
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
    }
}