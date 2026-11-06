namespace Maphy.Physics
{
    /// <summary>Substep-aggregated CCD work counters. Fallback scans include the forced reference mode.</summary>
    public readonly struct PhysicsWorldCCDStats
    {
        public readonly long queries, candidates, shapeTests, fallbackScans, fallbackCandidates, movingColliders;
        public readonly int proxyCount;
        public readonly bool indexOverflowed;
        public readonly long proxiesCreated, proxiesUpdated, proxiesReused, proxiesRemoved, indexRebuilds;
        internal PhysicsWorldCCDStats(long queries, long candidates, long tests, long scans, long scanned,
            long moving, int proxies, bool overflowed, long created = 0, long updated = 0,
            long reused = 0, long removed = 0, long rebuilds = 0)
        {
            this.queries = queries; this.candidates = candidates; shapeTests = tests;
            fallbackScans = scans; fallbackCandidates = scanned; movingColliders = moving;
            proxyCount = proxies; indexOverflowed = overflowed;
            proxiesCreated = created; proxiesUpdated = updated; proxiesReused = reused;
            proxiesRemoved = removed; indexRebuilds = rebuilds;
        }
        public static PhysicsWorldCCDStats operator +(PhysicsWorldCCDStats a, PhysicsWorldCCDStats b)
        {
            return new PhysicsWorldCCDStats(a.queries + b.queries, a.candidates + b.candidates,
                a.shapeTests + b.shapeTests, a.fallbackScans + b.fallbackScans,
                a.fallbackCandidates + b.fallbackCandidates, a.movingColliders + b.movingColliders,
                b.proxyCount, a.indexOverflowed || b.indexOverflowed, a.proxiesCreated + b.proxiesCreated,
                a.proxiesUpdated + b.proxiesUpdated, a.proxiesReused + b.proxiesReused,
                a.proxiesRemoved + b.proxiesRemoved, a.indexRebuilds + b.indexRebuilds);
        }
    }
}
