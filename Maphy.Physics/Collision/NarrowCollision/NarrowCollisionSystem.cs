using System.Collections.Generic;

namespace Maphy.Physics
{
    public sealed class NarrowCollisionSystem
    {
        private readonly List<CollisionPair> collisionPairs = new List<CollisionPair>();

        public IReadOnlyList<CollisionPair> CollisionPairs => collisionPairs;

        public readonly struct CollisionPair
        {
            public readonly BroadCollisionPairKey key;
            public readonly ulong colliderId0;
            public readonly ulong colliderId1;
            public readonly ulong rigidId0;
            public readonly ulong rigidId1;
            public readonly Collider collider0;
            public readonly Collider collider1;
            public readonly CollisionInfo collision;

            public CollisionPair(BroadCollisionPair pair, CollisionInfo collision)
            {
                key = pair.key;
                colliderId0 = pair.colliderId0;
                colliderId1 = pair.colliderId1;
                rigidId0 = pair.rigidId0;
                rigidId1 = pair.rigidId1;
                collider0 = pair.collider0;
                collider1 = pair.collider1;
                this.collision = collision;
            }
        }

        public IReadOnlyList<CollisionPair> Collision(IEnumerable<BroadCollisionPair> pairs)
        {
            collisionPairs.Clear();

            foreach (BroadCollisionPair pair in pairs)
            {
                if (Physics.TryComputeContact(pair, out CollisionInfo collision))
                {
                    collisionPairs.Add(new CollisionPair(pair, collision));
                }
            }

            return collisionPairs;
        }

        public bool RemoveCollider(ulong colliderId)
        {
            return collisionPairs.RemoveAll(pair => pair.colliderId0 == colliderId || pair.colliderId1 == colliderId) > 0;
        }

        public static bool Test(Shape shape0, Shape shape1)
        {
            return Physics.Overlaps(shape0, shape1);
        }

        public static bool TryComputeContact(Shape shape0, Shape shape1, out CollisionInfo collision)
        {
            return Physics.TryComputeContact(shape0, shape1, out collision);
        }
    }
}
