using System.Collections.Generic;

namespace Maphy.Physics
{
    public static class CollisionSystem
    {
        public static List<BroadCollisionPair> pairs = new List<BroadCollisionPair>();

        public static void Collision()
        {
            pairs = BroadCollisionSystem.Collision();
            NarrowCollisionSystem.Collision();
        }

        public static bool TestCollision(Collider a, Collider b)
        {
            if (!BroadCollisionSystem.IsBroadCollision(a, b))
            {
                return false;
            }

            return NarrowCollisionSystem.Collision(a.shape, b.shape);
        }
    }
}