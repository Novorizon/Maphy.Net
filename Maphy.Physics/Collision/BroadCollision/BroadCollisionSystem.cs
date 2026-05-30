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

    public sealed class BroadCollisionSystem
    {
        private readonly List<Collider> colliderBuffer = new List<Collider>();
        private readonly List<BroadCollisionPair> broadCollisionPairs = new List<BroadCollisionPair>();

        public IReadOnlyList<BroadCollisionPair> Pairs => broadCollisionPairs;

        public void Clear()
        {
            colliderBuffer.Clear();
            broadCollisionPairs.Clear();
        }

        public IReadOnlyList<BroadCollisionPair> Collision(IEnumerable<Collider> colliders)
        {
            colliderBuffer.Clear();
            colliderBuffer.AddRange(colliders);
            broadCollisionPairs.Clear();

            for (int i = 0; i < colliderBuffer.Count; i++)
            {
                for (int j = i + 1; j < colliderBuffer.Count; j++)
                {
                    if (IsBroadCollision(colliderBuffer[i], colliderBuffer[j]))
                    {
                        broadCollisionPairs.Add(new BroadCollisionPair(colliderBuffer[i], colliderBuffer[j]));
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

            return Physics.IsOverlap(Physics.ComputeBounds(a.shape), Physics.ComputeBounds(b.shape));
        }
    }
}
