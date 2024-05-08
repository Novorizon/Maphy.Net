
using Maphy.Mathematics;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public class EntityManager
    {
        public static ulong id = 1000000;
        private static readonly Dictionary<ulong, Entity> datas = new Dictionary<ulong, Entity>();
        public static ulong GenerateId()
        {
            return id++;
        }


        public Entity CreateEntity()
        {
            Entity entity = new Entity(id++);
            return entity;
        }

        public static Entity CreateEntity(fix3 position, quaternion rotation)
        {
            Entity entity = new Entity(id++, position, rotation);
            return entity;
        }

        public static Entity GetData(ulong id)
        {
            if (datas.ContainsKey(id))
            {
                return datas[id];
            }

            return Entity.Default;
        }

        public static void SetData(Entity entity)
        {
            if (datas.ContainsKey(entity.id))
            {
                datas[entity.id] = entity;
            }
            else
            {
                datas.Add(entity.id, entity);
            }
        }


        public static void Remove(ulong id)
        {
            if (datas.ContainsKey(id))
            {
                datas.Remove(id);
            }
        }


        public static void Remove(Shape a)
        {
            if (datas.ContainsKey(a.Id))
            {
                datas.Remove(a.Id);
            }
        }


        public static Entity AddRigid(fix3 position, quaternion rotation)
        {
            Entity entity = new Entity(id++, position, rotation);
            return entity;
        }
    }
}