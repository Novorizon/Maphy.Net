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
        public int BroadphaseProxyCount => broadphase.Proxies.Count;
        public int BroadphaseTreeProxyCount => broadphase.TreeProxyCount;
        public int BroadphaseTreeHeight => broadphase.TreeHeight;
        public int BroadphaseTreeMaxBalance => broadphase.TreeMaxBalance;

        public void Collision(IEnumerable<Collider> colliders)
        {
            Collision(colliders, NarrowPhaseAlgorithm.Auto);
        }

        public void Collision(IEnumerable<Collider> colliders, NarrowPhaseAlgorithm algorithm)
        {
            Collision(colliders, algorithm, ContactManifoldSettings.Default);
        }

        public void Collision(IReadOnlyList<Collider> colliders, NarrowPhaseAlgorithm algorithm, ContactManifoldSettings manifoldSettings)
        {
            IReadOnlyList<BroadCollisionPair> pairs = broadphase.Collision(colliders);
            IReadOnlyList<NarrowCollisionSystem.CollisionPair> collisionPairs = narrowphase.Collision(pairs, algorithm);
            contactCache.Update(collisionPairs, manifoldSettings);
        }

        public void Collision(IEnumerable<Collider> colliders, NarrowPhaseAlgorithm algorithm, ContactManifoldSettings manifoldSettings)
        {
            IReadOnlyList<BroadCollisionPair> pairs = broadphase.Collision(colliders);
            IReadOnlyList<NarrowCollisionSystem.CollisionPair> collisionPairs = narrowphase.Collision(pairs, algorithm);
            contactCache.Update(collisionPairs, manifoldSettings);
        }

        public bool RemoveCollider(ulong colliderId)
        {
            bool removed = broadphase.RemoveCollider(colliderId);
            removed |= narrowphase.RemoveCollider(colliderId);
            removed |= contactCache.RemoveCollider(colliderId);
            return removed;
        }

        public void QueryAABB(IEnumerable<Collider> colliders, AABB bounds, List<Collider> results)
        {
            broadphase.QueryAABB(colliders, bounds, results);
        }

        public void QueryAABB(IEnumerable<Collider> colliders, AABB bounds, int layerMask, List<Collider> results)
        {
            broadphase.QueryAABB(colliders, bounds, layerMask, results);
        }

        public bool Raycast(IEnumerable<Collider> colliders, Ray ray, fix maxDistance, out RaycastHit hitInfo)
        {
            return broadphase.Raycast(colliders, ray, maxDistance, out hitInfo);
        }

        public bool Raycast(IEnumerable<Collider> colliders, Ray ray, fix maxDistance, int layerMask, out RaycastHit hitInfo)
        {
            return broadphase.Raycast(colliders, ray, maxDistance, layerMask, out hitInfo);
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
            return TryGetCollision(a, b, NarrowPhaseAlgorithm.Auto, out collision);
        }

        public bool TryGetCollision(Collider a, Collider b, NarrowPhaseAlgorithm algorithm, out CollisionInfo collision)
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

            return Physics.TryComputeContact(new BroadCollisionPair(proxy0, proxy1), algorithm, out collision);
        }
    }
}
