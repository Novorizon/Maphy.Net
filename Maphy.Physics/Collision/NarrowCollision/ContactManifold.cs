using System.Collections.Generic;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct ContactManifoldSettings
    {
        public fix normalPersistenceDot;
        public fix anchorMatchDistance;
        public fix positionMatchDistance;
        public int staleFrameLimit;

        public static ContactManifoldSettings Default
        {
            get
            {
                return new ContactManifoldSettings
                {
                    normalPersistenceDot = fix._0_5,
                    anchorMatchDistance = fix._0_02,
                    positionMatchDistance = fix._0_02,
                    staleFrameLimit = 1,
                };
            }
        }
    }

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
        public int featureId;
        public int lifetime;

        public ContactPoint(
            fix3 position,
            fix3 pointOnCollider0,
            fix3 pointOnCollider1,
            fix penetrationDepth,
            int featureId,
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
            this.featureId = featureId;
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
            Update(collision, isTrigger, frameIndex, ContactManifoldSettings.Default);
        }

        public void Update(CollisionInfo collision, bool isTrigger, int frameIndex, ContactManifoldSettings settings)
        {
            fix3 previousNormal = normal;
            fix normalPersistenceDot = math.clamp(settings.normalPersistenceDot, fix.Zero, fix.One);
            bool canPreserveImpulse = contactCount > 0 && math.dot(previousNormal, collision.normal) >= normalPersistenceDot;

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
                ContactPoint previous = default;
                bool preserveImpulse = canPreserveImpulse && TryFindPersistentPoint(contact, settings, out previous);
                SetPoint(
                    i,
                    new ContactPoint(
                        contact.position,
                        contact.pointOnCollider0,
                        contact.pointOnCollider1,
                        contact.penetrationDepth,
                        contact.featureId,
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

        private bool TryFindPersistentPoint(CollisionContact contact, ContactManifoldSettings settings, out ContactPoint point)
        {
            fix anchorMatchDistance = math.max(fix.Zero, settings.anchorMatchDistance);
            fix positionMatchDistance = math.max(fix.Zero, settings.positionMatchDistance);
            fix maxAnchorDistanceSq = anchorMatchDistance * anchorMatchDistance;
            fix maxPositionDistanceSq = positionMatchDistance * positionMatchDistance;
            fix bestScore = fix.Max;
            bool found = false;
            point = default;

            for (int i = 0; i < contactCount; i++)
            {
                ContactPoint candidate = this[i];
                if (contact.featureId != 0 && candidate.featureId == contact.featureId)
                {
                    point = candidate;
                    return true;
                }

                fix anchorDistanceSq = math.distancesq(candidate.pointOnCollider0, contact.pointOnCollider0)
                    + math.distancesq(candidate.pointOnCollider1, contact.pointOnCollider1);
                fix positionDistanceSq = math.distancesq(candidate.position, contact.position);
                if (anchorDistanceSq <= maxAnchorDistanceSq || positionDistanceSq <= maxPositionDistanceSq)
                {
                    fix score = math.min(anchorDistanceSq, positionDistanceSq);
                    if (found && score >= bestScore)
                    {
                        continue;
                    }

                    found = true;
                    bestScore = score;
                    point = candidate;
                }
            }

            return found;
        }
    }

    internal sealed class ContactManifoldCache
    {
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

        public bool RemoveCollider(ulong colliderId)
        {
            staleKeys.Clear();
            foreach (KeyValuePair<BroadCollisionPairKey, ContactManifold> item in manifoldsByKey)
            {
                if (item.Value.colliderId0 == colliderId || item.Value.colliderId1 == colliderId)
                {
                    staleKeys.Add(item.Key);
                }
            }

            for (int i = 0; i < staleKeys.Count; i++)
            {
                manifoldsByKey.Remove(staleKeys[i]);
            }

            bool removedActive = activeManifolds.RemoveAll(manifold => manifold.colliderId0 == colliderId || manifold.colliderId1 == colliderId) > 0;
            bool removedCached = staleKeys.Count > 0;
            staleKeys.Clear();
            return removedActive || removedCached;
        }

        public IReadOnlyList<ContactManifold> Update(IEnumerable<NarrowCollisionSystem.CollisionPair> collisionPairs)
        {
            return Update(collisionPairs, ContactManifoldSettings.Default);
        }

        public IReadOnlyList<ContactManifold> Update(IReadOnlyList<NarrowCollisionSystem.CollisionPair> collisionPairs, ContactManifoldSettings settings)
        {
            frameIndex++;
            activeManifolds.Clear();

            for (int i = 0; i < collisionPairs.Count; i++)
            {
                NarrowCollisionSystem.CollisionPair pair = collisionPairs[i];
                if (!pair.collision.hasContact)
                {
                    continue;
                }

                if (!manifoldsByKey.TryGetValue(pair.key, out ContactManifold manifold))
                {
                    manifold = new ContactManifold();
                    manifoldsByKey.Add(pair.key, manifold);
                }

                manifold.Update(pair.collision, pair.collider0.isTrigger || pair.collider1.isTrigger, frameIndex, settings);
                activeManifolds.Add(manifold);
            }

            RemoveStaleManifolds(settings);
            return activeManifolds;
        }

        public IReadOnlyList<ContactManifold> Update(IEnumerable<NarrowCollisionSystem.CollisionPair> collisionPairs, ContactManifoldSettings settings)
        {
            if (collisionPairs is IReadOnlyList<NarrowCollisionSystem.CollisionPair> list)
            {
                return Update(list, settings);
            }

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

                manifold.Update(pair.collision, pair.collider0.isTrigger || pair.collider1.isTrigger, frameIndex, settings);
                activeManifolds.Add(manifold);
            }

            RemoveStaleManifolds(settings);
            return activeManifolds;
        }

        private void RemoveStaleManifolds(ContactManifoldSettings settings)
        {
            int staleFrameLimit = settings.staleFrameLimit >= 0 ? settings.staleFrameLimit : 1;
            staleKeys.Clear();
            foreach (KeyValuePair<BroadCollisionPairKey, ContactManifold> item in manifoldsByKey)
            {
                if (frameIndex - item.Value.lastUpdatedFrame > staleFrameLimit)
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
