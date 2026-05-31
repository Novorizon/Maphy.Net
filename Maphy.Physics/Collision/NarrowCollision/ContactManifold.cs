using System.Collections.Generic;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct ContactPoint
    {
        public fix3 position;
        public fix3 pointOnCollider0;
        public fix3 pointOnCollider1;
        public fix penetrationDepth;
        public fix normalImpulse;
        public fix tangentImpulse0;
        public fix tangentImpulse1;
        public fix3 tangent0;
        public fix3 tangent1;
        public int lifetime;

        public ContactPoint(
            fix3 position,
            fix3 pointOnCollider0,
            fix3 pointOnCollider1,
            fix penetrationDepth,
            ContactPoint previous,
            bool preserveImpulse)
        {
            this.position = position;
            this.pointOnCollider0 = pointOnCollider0;
            this.pointOnCollider1 = pointOnCollider1;
            this.penetrationDepth = penetrationDepth;
            normalImpulse = preserveImpulse ? previous.normalImpulse : fix.Zero;
            tangentImpulse0 = preserveImpulse ? previous.tangentImpulse0 : fix.Zero;
            tangentImpulse1 = preserveImpulse ? previous.tangentImpulse1 : fix.Zero;
            tangent0 = preserveImpulse ? previous.tangent0 : fix3.zero;
            tangent1 = preserveImpulse ? previous.tangent1 : fix3.zero;
            lifetime = preserveImpulse ? previous.lifetime + 1 : 1;
        }
    }

    public sealed class ContactManifold
    {
        public const int MaxContactPoints = 4;

        private ContactPoint point0;
        private ContactPoint point1;
        private ContactPoint point2;
        private ContactPoint point3;

        public BroadCollisionPairKey key { get; private set; }
        public ulong colliderId0 { get; private set; }
        public ulong colliderId1 { get; private set; }
        public ulong rigidId0 { get; private set; }
        public ulong rigidId1 { get; private set; }
        public fix3 normal { get; private set; }
        public int contactCount { get; private set; }
        public int lastUpdatedFrame { get; private set; }
        public bool isTrigger { get; private set; }

        public ContactPoint this[int index]
        {
            get
            {
                if ((uint)index >= (uint)contactCount)
                {
                    throw new System.IndexOutOfRangeException();
                }

                switch (index)
                {
                    case 0:
                        return point0;
                    case 1:
                        return point1;
                    case 2:
                        return point2;
                    default:
                        return point3;
                }
            }
        }

        public void Update(CollisionInfo collision, bool isTrigger, int frameIndex)
        {
            key = collision.key;
            colliderId0 = collision.id;
            colliderId1 = collision.otherId;
            rigidId0 = collision.rigidId;
            rigidId1 = collision.otherRigidId;
            normal = collision.normal;
            lastUpdatedFrame = frameIndex;
            this.isTrigger = isTrigger;

            int newContactCount = System.Math.Min(collision.contactCount, MaxContactPoints);
            for (int i = 0; i < newContactCount; i++)
            {
                CollisionContact contact = collision[i];
                bool preserveImpulse = TryFindPersistentPoint(contact.position, out ContactPoint previous);
                SetPoint(
                    i,
                    new ContactPoint(
                        contact.position,
                        contact.pointOnCollider0,
                        contact.pointOnCollider1,
                        contact.penetrationDepth,
                        previous,
                        preserveImpulse));
            }

            contactCount = newContactCount;
            ClearUnusedPoints(newContactCount);
        }

        internal ref ContactPoint GetPointRef(int index)
        {
            if ((uint)index >= (uint)contactCount)
            {
                throw new System.IndexOutOfRangeException();
            }

            switch (index)
            {
                case 0:
                    return ref point0;
                case 1:
                    return ref point1;
                case 2:
                    return ref point2;
                default:
                    return ref point3;
            }
        }

        private void SetPoint(int index, ContactPoint point)
        {
            switch (index)
            {
                case 0:
                    point0 = point;
                    break;
                case 1:
                    point1 = point;
                    break;
                case 2:
                    point2 = point;
                    break;
                default:
                    point3 = point;
                    break;
            }
        }

        private void ClearUnusedPoints(int usedCount)
        {
            if (usedCount <= 0)
            {
                point0 = default;
            }

            if (usedCount <= 1)
            {
                point1 = default;
            }

            if (usedCount <= 2)
            {
                point2 = default;
            }

            if (usedCount <= 3)
            {
                point3 = default;
            }
        }

        private bool TryFindPersistentPoint(fix3 position, out ContactPoint point)
        {
            fix maxDistanceSq = fix._0_01 * fix._0_01;
            for (int i = 0; i < contactCount; i++)
            {
                ContactPoint candidate = this[i];
                if (math.distancesq(candidate.position, position) <= maxDistanceSq)
                {
                    point = candidate;
                    return true;
                }
            }

            point = default;
            return false;
        }
    }

    internal sealed class ContactManifoldCache
    {
        private const int StaleFrameLimit = 1;

        private readonly Dictionary<BroadCollisionPairKey, ContactManifold> manifoldsByKey = new Dictionary<BroadCollisionPairKey, ContactManifold>();
        private readonly List<ContactManifold> activeManifolds = new List<ContactManifold>();
        private readonly List<BroadCollisionPairKey> staleKeys = new List<BroadCollisionPairKey>();
        private int frameIndex;

        public IReadOnlyList<ContactManifold> ActiveManifolds => activeManifolds;
        public int FrameIndex => frameIndex;

        public void Clear()
        {
            manifoldsByKey.Clear();
            activeManifolds.Clear();
            staleKeys.Clear();
            frameIndex = 0;
        }

        public IReadOnlyList<ContactManifold> Update(IEnumerable<NarrowCollisionSystem.CollisionPair> collisionPairs)
        {
            frameIndex++;
            activeManifolds.Clear();

            foreach (NarrowCollisionSystem.CollisionPair pair in collisionPairs)
            {
                if (!pair.collision.hasContact)
                {
                    continue;
                }

                if (!manifoldsByKey.TryGetValue(pair.key, out ContactManifold manifold))
                {
                    manifold = new ContactManifold();
                    manifoldsByKey.Add(pair.key, manifold);
                }

                manifold.Update(pair.collision, pair.collider0.isTrigger || pair.collider1.isTrigger, frameIndex);
                activeManifolds.Add(manifold);
            }

            RemoveStaleManifolds();
            return activeManifolds;
        }

        private void RemoveStaleManifolds()
        {
            staleKeys.Clear();
            foreach (KeyValuePair<BroadCollisionPairKey, ContactManifold> item in manifoldsByKey)
            {
                if (frameIndex - item.Value.lastUpdatedFrame > StaleFrameLimit)
                {
                    staleKeys.Add(item.Key);
                }
            }

            for (int i = 0; i < staleKeys.Count; i++)
            {
                manifoldsByKey.Remove(staleKeys[i]);
            }
        }
    }
}
