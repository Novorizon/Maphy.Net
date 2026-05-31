namespace Maphy.Physics
{
    public readonly struct WorldStepStats
    {
        public readonly int activeColliderCount;
        public readonly int broadphasePairCount;
        public readonly int broadphaseProxyCount;
        public readonly int broadphaseTreeHeight;
        public readonly int broadphaseTreeMaxBalance;
        public readonly int collisionPairCount;
        public readonly int contactManifoldCount;
        public readonly int islandCount;
        public readonly int solverIterations;
        public readonly int positionIterations;
        public readonly int deferredLifecycleOperationCount;
        public readonly int callbackExceptionCount;

        public WorldStepStats(
            int activeColliderCount,
            int broadphasePairCount,
            int broadphaseProxyCount,
            int broadphaseTreeHeight,
            int broadphaseTreeMaxBalance,
            int collisionPairCount,
            int contactManifoldCount,
            int islandCount,
            int solverIterations,
            int positionIterations,
            int deferredLifecycleOperationCount,
            int callbackExceptionCount)
        {
            this.activeColliderCount = activeColliderCount;
            this.broadphasePairCount = broadphasePairCount;
            this.broadphaseProxyCount = broadphaseProxyCount;
            this.broadphaseTreeHeight = broadphaseTreeHeight;
            this.broadphaseTreeMaxBalance = broadphaseTreeMaxBalance;
            this.collisionPairCount = collisionPairCount;
            this.contactManifoldCount = contactManifoldCount;
            this.islandCount = islandCount;
            this.solverIterations = solverIterations;
            this.positionIterations = positionIterations;
            this.deferredLifecycleOperationCount = deferredLifecycleOperationCount;
            this.callbackExceptionCount = callbackExceptionCount;
        }
    }
}
