using System.Collections;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public class CollisionSystem
    {
        public static List<BroadCollisionPair> pairs =new List<BroadCollisionPair>();
        public static void Collision()
        {
            //������ �ּ��=>Ǳ����ײ��
            pairs = BroadCollisionSystem.Collision();

            //Narrow=>��ײ��
            NarrowCollisionSystem.Collision();
        }

        public static bool TestCollision(Collider a,Collider b)
        {
            if (!BroadCollisionSystem.IsBroadCollision(a, b))
                return false;

            //1 ͨ��narrow phase���
            return NarrowCollisionSystem.Collision(a.shape,b.shape);



            //2 ͨ�������ײ����û��,��������Լ����׼�� ��ΪԼ����û�и�����ײ��
            return false;
        }
    }
}
