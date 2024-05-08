
using Maphy.Mathematics;
using Maphy.Tree;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public class World
    {
        ulong id = 10000000;
        public WorldSettings settings;
        Dictionary<ulong, Entity> entities;
        Dictionary<int, Collider> colliders = new Dictionary<int, Collider>();

        //������

        public World()
        {
            settings = new WorldSettings();
            entities=new Dictionary<ulong, Entity>();
        }


        public World(WorldSettings worldSettings)
        {
            settings = worldSettings;
            entities = new Dictionary<ulong, Entity>();
        }


        //����
        public void Update()
        {
            //���� �������õ��ٶ�
            //���� �����ܵ�����
            //���� ����ĺ��ٶ�
            //���� �����λ��

            CollisionSystem.Collision();

            // Լ��=>���¸����λ��=>��ײ�ص�

        }

        //����Entity
        public void UpdateEntity()
        {
            int length=entities.Count;
            for (int i = 0; i < length; i++)
            {

            }
        }



        ////���³�����
        //public void UpdateScene()
        //{
        //}

        public Entity CreateEntity()
        {
            Entity entity = new Entity(id++);
            return entity;
        }

        public ref Rigid CreateRigid(fix3 translation,quaternion orientation)
        {
            //Entity entity = new Entity(id++);

            Rigid rigid = RigidManager.CreateRigid();
            rigid.AddBoxCollider(id,translation,orientation);

            
            return ref rigid;
        }


        //public Entity CreateBox(fix3 position, quaternion rotation, fix3 center, fix3 size)
        //{
        //    Box box = new Box(center, size);

        //    Entity entity = new Entity(id++, position, rotation, box);
        //    entities.Add(entity.id, entity);
        //    return entity;
        //}
    }
}
