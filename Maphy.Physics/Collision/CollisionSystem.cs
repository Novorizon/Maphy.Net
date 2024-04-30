using System.Collections;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public class CollisionSystem
    {
        public static List<BroadCollisionPair> pairs =new List<BroadCollisionPair>();
        public static void Collision()
        {
            //场景树 粗检测=>潜在碰撞对
            pairs = BroadCollisionSystem.Collision();

            //Narrow=>碰撞对
            NarrowCollisionSystem.Collision();
        }

        public static bool TestCollision(Collider a,Collider b)
        {
            if (!BroadCollisionSystem.IsBroadCollision(a, b))
                return false;

            //1 通过narrow phase检测
            return NarrowCollisionSystem.Collision(a.shape,b.shape);



            //2 通过检查碰撞对有没有,但可能在约束后不准了 因为约束后没有更新碰撞对
            return false;
        }
    }
}
