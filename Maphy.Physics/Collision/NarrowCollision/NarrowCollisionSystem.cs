using System.Collections.Generic;

namespace Maphy.Physics
{
    public sealed class NarrowCollisionSystem
    {
        private readonly List<CollisionPair> collisionPairs = new List<CollisionPair>();

        public IReadOnlyList<CollisionPair> CollisionPairs => collisionPairs;

        public struct CollisionPair
        {
            public Collider collider0;
            public Collider collider1;

            public CollisionPair(Collider collider0, Collider collider1)
            {
                this.collider0 = collider0;
                this.collider1 = collider1;
            }
        }

        public IReadOnlyList<CollisionPair> Collision(IEnumerable<BroadCollisionPair> pairs)
        {
            collisionPairs.Clear();

            foreach (BroadCollisionPair pair in pairs)
            {
                if (Test(pair.collider0.shape, pair.collider1.shape))
                {
                    collisionPairs.Add(new CollisionPair(pair.collider0, pair.collider1));
                }
            }

            return collisionPairs;
        }

        public static bool Test(Shape shape0, Shape shape1)
        {
            return Physics.Overlaps(shape0, shape1);
        }
    }
}
