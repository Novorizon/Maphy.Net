using System.Collections.Generic;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public sealed class CollisionSystem
    {
        private readonly BroadCollisionSystem broadphase = new BroadCollisionSystem();
        private readonly NarrowCollisionSystem narrowphase = new NarrowCollisionSystem();
        private readonly ContactManifoldCache contactCache = new ContactManifoldCache();

        public IReadOnlyList<BroadCollisionPair> BroadphasePairs => broadphase.Pairs;
        public IReadOnlyList<NarrowCollisionSystem.CollisionPair> CollisionPairs => narrowphase.CollisionPairs;
        public IReadOnlyList<ContactManifold> ContactManifolds => contactCache.ActiveManifolds;

        public void Collision(IEnumerable<Collider> colliders)
        {
            IReadOnlyList<BroadCollisionPair> pairs = broadphase.Collision(colliders);
            narrowphase.Collision(pairs);
            contactCache.Update(narrowphase.CollisionPairs);
        }

        public void QueryAABB(IEnumerable<Collider> colliders, AABB bounds, List<Collider> results)
        {
            broadphase.QueryAABB(colliders, bounds, results);
        }

        public bool Raycast(IEnumerable<Collider> colliders, Ray ray, fix maxDistance, out RaycastHit hitInfo)
        {
            return broadphase.Raycast(colliders, ray, maxDistance, out hitInfo);
        }

        public bool TestCollision(Collider a, Collider b)
        {
            if (!broadphase.IsBroadCollision(a, b))
            {
                return false;
            }

            return NarrowCollisionSystem.Test(a.shape, b.shape);
        }

        public bool TryGetCollision(Collider a, Collider b, out CollisionInfo collision)
        {
            collision = default;
            if (a.shape == null || b.shape == null)
            {
                return false;
            }

            BroadphaseProxy proxy0 = new BroadphaseProxy(a, Physics.ComputeBounds(a.shape));
            BroadphaseProxy proxy1 = new BroadphaseProxy(b, Physics.ComputeBounds(b.shape));
            if (!broadphase.IsBroadCollision(proxy0, proxy1))
            {
                return false;
            }

            return Physics.TryComputeContact(new BroadCollisionPair(proxy0, proxy1), out collision);
        }
    }
}
