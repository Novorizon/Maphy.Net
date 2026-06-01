using Maphy.Mathematics;

namespace Maphy.Physics
{
    /// <summary>
    /// Value manifold for PhysicsWorld. It mirrors ContactManifold behavior, but avoids
    /// heap objects and dictionary lookup by keeping all state in preallocated arrays.
    /// </summary>
    public struct PhysicsWorldManifold
    {
        public const int MaxContactPoints = 4;

        private PhysicsWorldContactPoint point0;
        private PhysicsWorldContactPoint point1;
        private PhysicsWorldContactPoint point2;
        private PhysicsWorldContactPoint point3;

        public ColliderHandle collider0;
        public ColliderHandle collider1;
        public BodyHandle body0;
        public BodyHandle body1;
        public fix3 normal;
        public int contactCount;
        public int lastUpdatedFrame;
        public bool isTrigger;

        public PhysicsWorldContactPoint this[int index]
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

        internal void SetPointForSolver(int index, PhysicsWorldContactPoint point)
        {
            if ((uint)index >= (uint)contactCount)
            {
                throw new System.IndexOutOfRangeException();
            }

            SetPoint(index, point);
        }

        public void Update(
            PhysicsWorldPair pair,
            CollisionInfo collision,
            bool isTrigger,
            int frameIndex,
            ContactManifoldSettings settings,
            PhysicsWorldManifold previous,
            bool hasPrevious)
        {
            fix normalPersistenceDot = math.clamp(settings.normalPersistenceDot, fix.Zero, fix.One);
            bool canPreserveImpulse = hasPrevious
                && previous.contactCount > 0
                && math.dot(previous.normal, collision.normal) >= normalPersistenceDot;

            collider0 = pair.collider0;
            collider1 = pair.collider1;
            body0 = pair.body0;
            body1 = pair.body1;
            normal = collision.normal;
            lastUpdatedFrame = frameIndex;
            this.isTrigger = isTrigger;

            int newContactCount = collision.contactCount < MaxContactPoints
                ? collision.contactCount
                : MaxContactPoints;

            for (int i = 0; i < newContactCount; i++)
            {
                CollisionContact contact = collision[i];
                PhysicsWorldContactPoint previousPoint = default;
                bool preserveImpulse = canPreserveImpulse
                    && previous.TryFindPersistentPoint(contact, settings, out previousPoint);
                SetPoint(i, new PhysicsWorldContactPoint(contact, previousPoint, preserveImpulse));
            }

            contactCount = newContactCount;
            ClearUnusedPoints(newContactCount);
        }

        private bool TryFindPersistentPoint(
            CollisionContact contact,
            ContactManifoldSettings settings,
            out PhysicsWorldContactPoint point)
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
                PhysicsWorldContactPoint candidate = this[i];
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

        private void SetPoint(int index, PhysicsWorldContactPoint point)
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
    }
}
