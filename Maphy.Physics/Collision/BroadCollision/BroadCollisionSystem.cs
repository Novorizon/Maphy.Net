using System;
using System.Collections.Generic;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public readonly struct BroadphaseProxy
    {
        public readonly ulong colliderId;
        public readonly ulong rigidId;
        public readonly Collider collider;
        public readonly AABB bounds;

        public BroadphaseProxy(Collider collider, AABB bounds)
        {
            colliderId = collider.id;
            rigidId = collider.rigidId;
            this.collider = collider;
            this.bounds = bounds;
        }
    }

    public readonly struct BroadCollisionPairKey : IEquatable<BroadCollisionPairKey>
    {
        public readonly ulong colliderId0;
        public readonly ulong colliderId1;

        public BroadCollisionPairKey(ulong colliderId0, ulong colliderId1)
        {
            if (colliderId0 <= colliderId1)
            {
                this.colliderId0 = colliderId0;
                this.colliderId1 = colliderId1;
            }
            else
            {
                this.colliderId0 = colliderId1;
                this.colliderId1 = colliderId0;
            }
        }

        public bool Equals(BroadCollisionPairKey other)
        {
            return colliderId0 == other.colliderId0 && colliderId1 == other.colliderId1;
        }

        public override bool Equals(object obj)
        {
            return obj is BroadCollisionPairKey other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + colliderId0.GetHashCode();
                hash = hash * 31 + colliderId1.GetHashCode();
                return hash;
            }
        }

        public static bool operator ==(BroadCollisionPairKey left, BroadCollisionPairKey right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(BroadCollisionPairKey left, BroadCollisionPairKey right)
        {
            return !left.Equals(right);
        }
    }

    public readonly struct BroadCollisionPair
    {
        public readonly BroadCollisionPairKey key;
        public readonly ulong colliderId0;
        public readonly ulong colliderId1;
        public readonly ulong rigidId0;
        public readonly ulong rigidId1;
        public readonly Collider collider0;
        public readonly Collider collider1;
        public readonly AABB bounds0;
        public readonly AABB bounds1;

        public BroadCollisionPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            if (proxy0.colliderId <= proxy1.colliderId)
            {
                key = new BroadCollisionPairKey(proxy0.colliderId, proxy1.colliderId);
                colliderId0 = proxy0.colliderId;
                colliderId1 = proxy1.colliderId;
                rigidId0 = proxy0.rigidId;
                rigidId1 = proxy1.rigidId;
                collider0 = proxy0.collider;
                collider1 = proxy1.collider;
                bounds0 = proxy0.bounds;
                bounds1 = proxy1.bounds;
            }
            else
            {
                key = new BroadCollisionPairKey(proxy1.colliderId, proxy0.colliderId);
                colliderId0 = proxy1.colliderId;
                colliderId1 = proxy0.colliderId;
                rigidId0 = proxy1.rigidId;
                rigidId1 = proxy0.rigidId;
                collider0 = proxy1.collider;
                collider1 = proxy0.collider;
                bounds0 = proxy1.bounds;
                bounds1 = proxy0.bounds;
            }
        }
    }

    public sealed class BroadCollisionSystem
    {
        private readonly DynamicAABBTree tree = new DynamicAABBTree();
        private readonly List<BroadphaseProxy> proxies = new List<BroadphaseProxy>();
        private readonly List<BroadCollisionPair> broadCollisionPairs = new List<BroadCollisionPair>();
        private readonly Dictionary<ulong, BroadphaseProxy> proxiesById = new Dictionary<ulong, BroadphaseProxy>();
        private readonly HashSet<ulong> activeColliderIds = new HashSet<ulong>();
        private readonly List<ulong> staleColliderIds = new List<ulong>();
        private readonly List<BroadphaseProxy> queryResults = new List<BroadphaseProxy>();
        private readonly HashSet<BroadCollisionPairKey> pairKeys = new HashSet<BroadCollisionPairKey>();
        private readonly List<BroadCollisionPairKey> sortedPairKeys = new List<BroadCollisionPairKey>();

        public IReadOnlyList<BroadphaseProxy> Proxies => proxies;
        public IReadOnlyList<BroadCollisionPair> Pairs => broadCollisionPairs;
        public int TreeProxyCount => tree.ProxyCount;

        public void Clear()
        {
            tree.Clear();
            proxies.Clear();
            broadCollisionPairs.Clear();
            proxiesById.Clear();
            activeColliderIds.Clear();
            staleColliderIds.Clear();
            queryResults.Clear();
            pairKeys.Clear();
            sortedPairKeys.Clear();
        }

        public IReadOnlyList<BroadCollisionPair> Collision(IEnumerable<Collider> colliders)
        {
            SyncProxies(colliders);
            broadCollisionPairs.Clear();
            pairKeys.Clear();
            sortedPairKeys.Clear();

            for (int i = 0; i < proxies.Count; i++)
            {
                BroadphaseProxy proxy = proxies[i];
                queryResults.Clear();
                tree.Query(proxy.bounds, queryResults);

                for (int j = 0; j < queryResults.Count; j++)
                {
                    BroadphaseProxy other = queryResults[j];
                    if (IsBroadCollision(proxy, other))
                    {
                        pairKeys.Add(new BroadCollisionPairKey(proxy.colliderId, other.colliderId));
                    }
                }
            }

            sortedPairKeys.AddRange(pairKeys);
            sortedPairKeys.Sort(ComparePairKeys);
            for (int i = 0; i < sortedPairKeys.Count; i++)
            {
                BroadCollisionPairKey key = sortedPairKeys[i];
                if (proxiesById.TryGetValue(key.colliderId0, out BroadphaseProxy proxy0)
                    && proxiesById.TryGetValue(key.colliderId1, out BroadphaseProxy proxy1))
                {
                    broadCollisionPairs.Add(new BroadCollisionPair(proxy0, proxy1));
                }
            }

            return broadCollisionPairs;
        }

        public void QueryAABB(IEnumerable<Collider> colliders, AABB bounds, List<Collider> results)
        {
            QueryAABB(colliders, bounds, Collider.AllLayers, results);
        }

        public void QueryAABB(IEnumerable<Collider> colliders, AABB bounds, int layerMask, List<Collider> results)
        {
            if (results == null)
            {
                return;
            }

            SyncProxies(colliders);
            results.Clear();
            queryResults.Clear();
            tree.Query(bounds, queryResults);
            for (int i = 0; i < queryResults.Count; i++)
            {
                BroadphaseProxy proxy = queryResults[i];
                if (proxy.collider.IsInLayerMask(layerMask) && Physics.IsOverlap(proxy.bounds, bounds))
                {
                    results.Add(proxy.collider);
                }
            }
        }

        public bool Raycast(IEnumerable<Collider> colliders, Ray ray, fix maxDistance, out RaycastHit hitInfo)
        {
            return Raycast(colliders, ray, maxDistance, Collider.AllLayers, out hitInfo);
        }

        public bool Raycast(IEnumerable<Collider> colliders, Ray ray, fix maxDistance, int layerMask, out RaycastHit hitInfo)
        {
            hitInfo = default;
            if (maxDistance < fix.Zero)
            {
                return false;
            }

            SyncProxies(colliders);
            queryResults.Clear();
            tree.Raycast(ray, maxDistance, queryResults);

            bool hasHit = false;
            fix bestDistance = maxDistance;
            ulong bestColliderId = ulong.MaxValue;
            for (int i = 0; i < queryResults.Count; i++)
            {
                BroadphaseProxy proxy = queryResults[i];
                if (!proxy.collider.IsInLayerMask(layerMask))
                {
                    continue;
                }

                if (!Physics.TryRaycast(proxy.collider.shape, ray, maxDistance, out RaycastHit candidate))
                {
                    continue;
                }

                if (candidate.distance > bestDistance)
                {
                    continue;
                }

                if (candidate.distance == bestDistance && proxy.colliderId >= bestColliderId)
                {
                    continue;
                }

                candidate.id = proxy.colliderId <= long.MaxValue ? (long)proxy.colliderId : long.MaxValue;
                candidate.colliderId = proxy.colliderId;
                candidate.rigidId = proxy.rigidId;
                candidate.collider = proxy.collider;
                hitInfo = candidate;
                hasHit = true;
                bestDistance = candidate.distance;
                bestColliderId = proxy.colliderId;
            }

            return hasHit;
        }

        public bool IsBroadCollision(Collider a, Collider b)
        {
            if (a.id == b.id || a.shape == null || b.shape == null)
            {
                return false;
            }

            return IsBroadCollision(
                new BroadphaseProxy(a, Physics.ComputeBounds(a.shape)),
                new BroadphaseProxy(b, Physics.ComputeBounds(b.shape)));
        }

        public bool IsBroadCollision(BroadphaseProxy a, BroadphaseProxy b)
        {
            if (a.colliderId == b.colliderId)
            {
                return false;
            }

            if (a.rigidId != 0 && a.rigidId == b.rigidId)
            {
                return false;
            }

            if (!a.collider.CanCollideWith(b.collider))
            {
                return false;
            }

            return Physics.IsOverlap(a.bounds, b.bounds);
        }

        private void SyncProxies(IEnumerable<Collider> colliders)
        {
            proxies.Clear();
            proxiesById.Clear();
            activeColliderIds.Clear();

            foreach (Collider collider in colliders)
            {
                if (collider.shape != null)
                {
                    BroadphaseProxy proxy = new BroadphaseProxy(collider, Physics.ComputeBounds(collider.shape));
                    proxies.Add(proxy);
                    proxiesById[proxy.colliderId] = proxy;
                    activeColliderIds.Add(proxy.colliderId);
                    tree.MoveProxy(proxy);
                }
            }

            tree.RemoveExcept(activeColliderIds, staleColliderIds);
        }

        private static int ComparePairKeys(BroadCollisionPairKey a, BroadCollisionPairKey b)
        {
            int first = a.colliderId0.CompareTo(b.colliderId0);
            return first != 0 ? first : a.colliderId1.CompareTo(b.colliderId1);
        }
    }
}
