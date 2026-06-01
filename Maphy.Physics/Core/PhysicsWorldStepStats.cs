namespace Maphy.Physics
{
    /// <summary>
    /// Value snapshot of the last PhysicsWorld step. It is returned by value and does
    /// not expose any object-layer collections, so reading it is allocation-free.
    /// </summary>
    public readonly struct PhysicsWorldStepStats
    {
        public readonly int activeBodyCount;
        public readonly int activeColliderCount;
        public readonly int bodyCapacity;
        public readonly int colliderCapacity;
        public readonly int pairCapacity;
        public readonly int contactManifoldCapacity;
        public readonly int bodySyncCount;
        public readonly int colliderSyncCount;
        public readonly int broadphaseProxyCount;
        public readonly int broadphaseTreeHeight;
        public readonly int broadphaseTreeMaxBalance;
        public readonly bool broadphaseTreeOverflowed;
        public readonly int broadphaseMovedProxyCount;
        public readonly int broadphaseCandidateCount;
        public readonly int broadphaseFilteredCandidateCount;
        public readonly int pairCount;
        public readonly bool pairOverflowed;
        public readonly int narrowPhaseTestCount;
        public readonly int contactManifoldCount;
        public readonly int contactManifoldNewCount;
        public readonly int contactManifoldReusedCount;
        public readonly int contactManifoldDroppedCount;
        public readonly bool contactOverflowed;
        public readonly int solverContactPointCount;
        public readonly int islandCount;
        public readonly int sleepingIslandCount;
        public readonly int sleepingBodyCount;
        public readonly int awakeDynamicBodyCount;
        public readonly int solverIterations;
        public readonly int positionIterations;
        public readonly ulong fixedStepCount;
        public bool HasOverflow =>
            broadphaseTreeOverflowed || pairOverflowed || contactOverflowed;

        public PhysicsWorldStepStats(
            int activeBodyCount,
            int activeColliderCount,
            int bodyCapacity,
            int colliderCapacity,
            int pairCapacity,
            int contactManifoldCapacity,
            int bodySyncCount,
            int colliderSyncCount,
            int broadphaseProxyCount,
            int broadphaseTreeHeight,
            int broadphaseTreeMaxBalance,
            bool broadphaseTreeOverflowed,
            int broadphaseMovedProxyCount,
            int broadphaseCandidateCount,
            int broadphaseFilteredCandidateCount,
            int pairCount,
            bool pairOverflowed,
            int narrowPhaseTestCount,
            int contactManifoldCount,
            int contactManifoldNewCount,
            int contactManifoldReusedCount,
            int contactManifoldDroppedCount,
            bool contactOverflowed,
            int solverContactPointCount,
            int islandCount,
            int sleepingIslandCount,
            int sleepingBodyCount,
            int awakeDynamicBodyCount,
            int solverIterations,
            int positionIterations,
            ulong fixedStepCount)
        {
            this.activeBodyCount = activeBodyCount;
            this.activeColliderCount = activeColliderCount;
            this.bodyCapacity = bodyCapacity;
            this.colliderCapacity = colliderCapacity;
            this.pairCapacity = pairCapacity;
            this.contactManifoldCapacity = contactManifoldCapacity;
            this.bodySyncCount = bodySyncCount;
            this.colliderSyncCount = colliderSyncCount;
            this.broadphaseProxyCount = broadphaseProxyCount;
            this.broadphaseTreeHeight = broadphaseTreeHeight;
            this.broadphaseTreeMaxBalance = broadphaseTreeMaxBalance;
            this.broadphaseTreeOverflowed = broadphaseTreeOverflowed;
            this.broadphaseMovedProxyCount = broadphaseMovedProxyCount;
            this.broadphaseCandidateCount = broadphaseCandidateCount;
            this.broadphaseFilteredCandidateCount = broadphaseFilteredCandidateCount;
            this.pairCount = pairCount;
            this.pairOverflowed = pairOverflowed;
            this.narrowPhaseTestCount = narrowPhaseTestCount;
            this.contactManifoldCount = contactManifoldCount;
            this.contactManifoldNewCount = contactManifoldNewCount;
            this.contactManifoldReusedCount = contactManifoldReusedCount;
            this.contactManifoldDroppedCount = contactManifoldDroppedCount;
            this.contactOverflowed = contactOverflowed;
            this.solverContactPointCount = solverContactPointCount;
            this.islandCount = islandCount;
            this.sleepingIslandCount = sleepingIslandCount;
            this.sleepingBodyCount = sleepingBodyCount;
            this.awakeDynamicBodyCount = awakeDynamicBodyCount;
            this.solverIterations = solverIterations;
            this.positionIterations = positionIterations;
            this.fixedStepCount = fixedStepCount;
        }
    }
}
