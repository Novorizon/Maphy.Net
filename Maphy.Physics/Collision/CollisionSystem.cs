using System.Collections.Generic;

namespace Maphy.Physics
{
    public sealed class CollisionSystem
    {
        private readonly BroadCollisionSystem broadphase = new BroadCollisionSystem();
        private readonly NarrowCollisionSystem narrowphase = new NarrowCollisionSystem();

        public IReadOnlyList<BroadCollisionPair> BroadphasePairs => broadphase.Pairs;
        public IReadOnlyList<NarrowCollisionSystem.CollisionPair> CollisionPairs => narrowphase.CollisionPairs;

        public void Collision(IEnumerable<Collider> colliders)
        {
            IReadOnlyList<BroadCollisionPair> pairs = broadphase.Collision(colliders);
            narrowphase.Collision(pairs);
        }

        public bool TestCollision(Collider a, Collider b)
        {
            if (!broadphase.IsBroadCollision(a, b))
            {
                return false;
            }

            return NarrowCollisionSystem.Test(a.shape, b.shape);
        }
    }
}
