using System.Collections.Generic;

namespace Maphy.Physics
{
    public struct BroadCollisionPair
    {
        public Collider collider0;
        public Collider collider1;

        public BroadCollisionPair(Collider collider0, Collider collider1)
        {
            this.collider0 = collider0;
            this.collider1 = collider1;
        }
    }

    public static class BroadCollisionSystem
    {
        private static readonly List<Collider> colliders = new List<Collider>();
        private static readonly List<BroadCollisionPair> broadCollisionPairs = new List<BroadCollisionPair>();

        public static IReadOnlyList<Collider> Colliders => colliders;

        public static void Clear()
        {
            colliders.Clear();
            broadCollisionPairs.Clear();
        }

        public static void Register(Collider collider)
        {
            for (int i = 0; i < colliders.Count; i++)
            {
                if (colliders[i].id == collider.id)
                {
                    colliders[i] = collider;
                    return;
                }
            }

            colliders.Add(collider);
        }

        public static bool Unregister(ulong colliderId)
        {
            for (int i = 0; i < colliders.Count; i++)
            {
                if (colliders[i].id == colliderId)
                {
                    colliders.RemoveAt(i);
                    return true;
                }
            }

            return false;
        }

        public static void SetColliders(IEnumerable<Collider> source)
        {
            colliders.Clear();
            colliders.AddRange(source);
        }

        public static List<BroadCollisionPair> Collision()
        {
            broadCollisionPairs.Clear();

            for (int i = 0; i < colliders.Count; i++)
            {
                for (int j = i + 1; j < colliders.Count; j++)
                {
                    if (IsBroadCollision(colliders[i], colliders[j]))
                    {
                        broadCollisionPairs.Add(new BroadCollisionPair(colliders[i], colliders[j]));
                    }
                }
            }

            return broadCollisionPairs;
        }

        public static bool IsBroadCollision(Collider a, Collider b)
        {
            if (a.id == b.id || a.shape == null || b.shape == null)
            {
                return false;
            }

            return Physics.IsOverlap(Physics.ComputeBounds(a.shape), Physics.ComputeBounds(b.shape));
        }
    }
}