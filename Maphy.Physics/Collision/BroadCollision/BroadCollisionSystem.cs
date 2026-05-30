using System;
using System.Collections.Generic;

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
        private readonly List<BroadphaseProxy> proxies = new List<BroadphaseProxy>();
        private readonly List<BroadCollisionPair> broadCollisionPairs = new List<BroadCollisionPair>();

        public IReadOnlyList<BroadphaseProxy> Proxies => proxies;
        public IReadOnlyList<BroadCollisionPair> Pairs => broadCollisionPairs;

        public void Clear()
        {
            proxies.Clear();
            broadCollisionPairs.Clear();
        }

        public IReadOnlyList<BroadCollisionPair> Collision(IEnumerable<Collider> colliders)
        {
            proxies.Clear();
            broadCollisionPairs.Clear();

            foreach (Collider collider in colliders)
            {
                if (collider.shape != null)
                {
                    proxies.Add(new BroadphaseProxy(collider, Physics.ComputeBounds(collider.shape)));
                }
            }

            for (int i = 0; i < proxies.Count; i++)
            {
                for (int j = i + 1; j < proxies.Count; j++)
                {
                    if (IsBroadCollision(proxies[i], proxies[j]))
                    {
                        broadCollisionPairs.Add(new BroadCollisionPair(proxies[i], proxies[j]));
                    }
                }
            }

            return broadCollisionPairs;
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

            return Physics.IsOverlap(a.bounds, b.bounds);
        }
    }
}
