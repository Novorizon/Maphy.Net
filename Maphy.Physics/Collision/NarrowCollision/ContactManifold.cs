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

        public void Update(CollisionInfo collision, int frameIndex)
        {
            bool preserveImpulse = TryFindPersistentPoint(collision.contactPoint, out ContactPoint previous);

            key = collision.key;
            colliderId0 = collision.id;
            colliderId1 = collision.otherId;
            rigidId0 = collision.rigidId;
            rigidId1 = collision.otherRigidId;
            normal = collision.normal;
            contactCount = 1;
            lastUpdatedFrame = frameIndex;

            point0 = new ContactPoint(
                collision.contactPoint,
                collision.contactPoint1,
                collision.contactPoint2,
                collision.penetrationDepth,
                previous,
                preserveImpulse);
            point1 = default;
            point2 = default;
            point3 = default;
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

                manifold.Update(pair.collision, frameIndex);
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
