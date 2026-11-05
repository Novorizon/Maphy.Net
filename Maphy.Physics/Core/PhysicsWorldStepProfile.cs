namespace Maphy.Physics
{
    /// <summary>Optional wall-clock diagnostics. Never used by simulation or replay hashing.</summary>
    public readonly struct PhysicsWorldStepProfile
    {
        public readonly long syncTicks, ccdPreparationTicks, integrationTicks, pairTicks, contactTicks;
        public readonly long islandTicks, velocityTicks, positionTicks, finalizeTicks, statsTicks;
        public readonly long sleepTicks, sanitizationTicks;
        public readonly int updateCount;
        public long TotalTicks => syncTicks + ccdPreparationTicks + integrationTicks + pairTicks + contactTicks
            + islandTicks + velocityTicks + positionTicks + finalizeTicks + statsTicks;

        internal PhysicsWorldStepProfile(long sync, long ccd, long integration, long pairs, long contacts,
            long islands, long velocity, long position, long finalize, long stats, int updates,
            long sleep = 0, long sanitization = 0)
        {
            syncTicks = sync; ccdPreparationTicks = ccd; integrationTicks = integration;
            pairTicks = pairs; contactTicks = contacts; islandTicks = islands;
            velocityTicks = velocity; positionTicks = position; finalizeTicks = finalize;
            statsTicks = stats; updateCount = updates;
            sleepTicks = sleep; sanitizationTicks = sanitization;
        }

        public static PhysicsWorldStepProfile operator +(PhysicsWorldStepProfile a, PhysicsWorldStepProfile b)
        {
            return new PhysicsWorldStepProfile(a.syncTicks + b.syncTicks, a.ccdPreparationTicks + b.ccdPreparationTicks,
                a.integrationTicks + b.integrationTicks, a.pairTicks + b.pairTicks, a.contactTicks + b.contactTicks,
                a.islandTicks + b.islandTicks, a.velocityTicks + b.velocityTicks, a.positionTicks + b.positionTicks,
                a.finalizeTicks + b.finalizeTicks, a.statsTicks + b.statsTicks, a.updateCount + b.updateCount,
                a.sleepTicks + b.sleepTicks, a.sanitizationTicks + b.sanitizationTicks);
        }
    }
}
