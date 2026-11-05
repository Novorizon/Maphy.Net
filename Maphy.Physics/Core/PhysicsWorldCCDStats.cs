namespace Maphy.Physics
{
    /// <summary>Substep-aggregated CCD work counters. Fallback scans include the forced reference mode.</summary>
    public readonly struct PhysicsWorldCCDStats
    {
        public readonly long queries, candidates, shapeTests, fallbackScans, fallbackCandidates, movingColliders;
        public readonly int proxyCount;
        public readonly bool indexOverflowed;
        internal PhysicsWorldCCDStats(long queries, long candidates, long tests, long scans, long scanned,
            long moving, int proxies, bool overflowed)
        {
            this.queries = queries; this.candidates = candidates; shapeTests = tests;
            fallbackScans = scans; fallbackCandidates = scanned; movingColliders = moving;
            proxyCount = proxies; indexOverflowed = overflowed;
        }
        public static PhysicsWorldCCDStats operator +(PhysicsWorldCCDStats a, PhysicsWorldCCDStats b)
        {
            return new PhysicsWorldCCDStats(a.queries + b.queries, a.candidates + b.candidates,
                a.shapeTests + b.shapeTests, a.fallbackScans + b.fallbackScans,
                a.fallbackCandidates + b.fallbackCandidates, a.movingColliders + b.movingColliders,
                b.proxyCount, a.indexOverflowed || b.indexOverflowed);
        }
    }
}
