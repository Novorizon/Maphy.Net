# CCD candidate pruning and phase profiling

This report records the first indexed version. See [CCD-reuse.md](CCD-reuse.md)
for the subsequent retained-index and island-sleep optimization (38.393 to
15.094 ms/update in its paired run).

## Reproducible paired observation

The current full-scan reference and indexed backend were run sequentially on
.NET 8.0.427 / Linux x64, Release, with `DOTNET_TieredCompilation=0`, eight warmup
updates, 30 measured updates, and two recreated-world repeats. Phase profiling
was enabled for both. These are host-dependent observations, not Unity results.

| Median per update | Reference scan | Indexed CCD |
| --- | ---: | ---: |
| Simulation | 788.113 ms | 38.754 ms |
| CCD preparation | 0.000 ms | 13.919 ms |
| Integration, including CCD and hit-time synchronization | 773.936 ms | 10.233 ms |
| Other synchronization | 0.369 ms | 0.389 ms |
| Ordinary pair generation | 2.120 ms | 1.821 ms |
| Finalization, sleeping and sanitization | 11.477 ms | 12.189 ms |
| Enumerated targets per update | 2,000,000 | 2,000 |
| Shape TOI tests per update | 1,499,500 | 1,000 |
| Measured simulation/hash allocation | 0 / 0 bytes | 0 / 0 bytes |

The paired median ratio is **20.34x**. Both repeats and both backends produce
state hash `3500359475767577299`, without index/capacity overflow or fallback in
the indexed case. There are 1,000 moving spheres and 1,000 wall bodies.

The reference backend shares the new body/collider ownership index and dirty
marking path; it disables CCD spatial indexing and target-velocity caching.
It is an equivalence oracle, not a byte-for-byte copy of the older implementation.
The earlier 667 ms report used different JIT/warmup conditions, so do not compute
a before/after ratio from that number. The short original-profile capture is
retained only as diagnostic evidence: over 95% of its simulation time was spent
inside integration/CCD.

```bash
DOTNET_TieredCompilation=0 dotnet run --project Maphy.Net.Benchmarks~/Maphy.Net.Benchmarks.csproj -c Release -- --scenario CCD --count 1000 --warmup 8 --iterations 30 --repeat 2 --profile --brute-force-ccd --out /tmp/ccd-reference.csv
DOTNET_TieredCompilation=0 dotnet run --project Maphy.Net.Benchmarks~/Maphy.Net.Benchmarks.csproj -c Release -- --scenario CCD --count 1000 --warmup 8 --iterations 30 --repeat 2 --profile --out /tmp/ccd-indexed.csv
python Tools/check_benchmarks.py /tmp/ccd-reference.csv
python Tools/check_benchmarks.py /tmp/ccd-indexed.csv
python Tools/compare_ccd_benchmarks.py /tmp/ccd-reference.csv /tmp/ccd-indexed.csv
```

Matching CSVs are checked in with `ccd-reference` and `ccd-indexed` suffixes.
The comparison script gates on matching setup/state, allocations, capacity and
candidate-test behavior. It reports timing without a hardware-dependent speed
threshold.

## Acceleration contract

- Bodies own a sorted, intrusive collider-index chain. Creation, removal, handle
  reuse and Reserve growth preserve the ascending collider order used previously.
  Dirty-body synchronization uses this chain instead of scanning all colliders.
- A separate reserved AABB tree indexes target **swept** envelopes over the full
  integration horizon, including kinematic and eligible dynamic target motion.
  Querying only present target bounds would miss fast crossing targets.
- Cached target velocities and envelopes are refreshed as sequential integration
  updates bodies and synchronizes colliders. This preserves the reference path's
  exact target snapshots and multiplication/rounding semantics for remaining time.
- Earliest fraction and then target collider index remain the hit-selection rule.
  The spatial query does not replace exact shape casts or the existing AABB cast
  fallback. Envelope padding is only conservative candidate filtering.
- If a leaf, moving envelope or aggregate tree span cannot be represented safely,
  or the index overflows, candidate selection falls back to the reference scan.
  Diagnostics expose the fallback; missing candidates are not silently dropped.

`EnableCCDBroadphase` defaults to true; setting it false selects the reference
scan for diagnostics. It does not change replay hashing or intended physical
behavior. Regression tests compare both paths for 12 mixed-shape seeds over 160
frames each, including moving targets, gravity, compound colliders, trigger/mask
filters, multiple CCD iterations, body disabling, destruction, index reuse and
capacity growth. Additional cases cover equal-fraction target ties, numerical
fallback, no-allocation sparse pruning and profiling aggregation. The frozen
1,239-row runtime corpus is unchanged.

That version allocates the secondary tree, collider chains and target-velocity
cache up front. The capacity memory estimate includes them; for 2,000 reserved
bodies/colliders it increases by approximately 736,000 bytes versus the previous
estimate, with unchanged pair/manifold capacities. No allocation is deferred
into CCD updates.
The current version adds another 8 bytes per reserved body for island member
lists; see CCD-reuse.md for the updated allocation contract.

## Optional phase timing

```csharp
world.EnableProfiling = true; // default false
world.Step(fix.One / 60);
PhysicsWorldStepProfile phases = world.LastStepProfile;
PhysicsWorldCCDStats ccd = world.LastCCDStats;
```

Profiles cover synchronization, CCD preparation, integration, pair generation,
contacts, island/wake processing, velocity solve, position solve, finalization,
and step statistics. Ticks use `Stopwatch.Frequency`. `Step` sums all executed
substeps. Positive fixed stepping clears diagnostics if it executes no update;
disabled fixed stepping reports its direct Update, even for zero delta.
`Update` reports that single update. Disabled profiling performs no timestamp reads. Timers and CCD
counters never enter solver decisions or replay hashes.

CCD counters sum the executed substeps, while proxyCount describes the last
integration index. fallbackScans includes explicitly selecting the reference
mode; indexOverflowed reports capacity/query overflow separately from numerical
envelope fallback. With positive fixed stepping, a Step executing no updates
reports zero counters rather than a stale previous update.

CSV phase `*Ms` columns are totals over measured iterations. Divide by Iterations
for a per-update average. Integration includes hit-time synchronization; the
separate SyncMs column covers synchronization outside integration. Benchmark
scenario reset/setup calls inside the step action remain in overall simulation
time but outside these internal phases. State hashing is still measured separately.

## Remaining hotspots

The indexed workload remains above a 16.7 ms frame budget on this host. Its next
targets are reusing/refitting the CCD index instead of rebuilding all proxies,
avoiding unnecessary sanitization of unchanged bodies, and reducing repeated
hit-time dirty-array scans. Dense scenes may still produce large candidate sets;
the sparse-scene speed ratio should not be generalized to them. Actual Unity
2022.3 Mono/IL2CPP execution still requires the licensed environment described in
VALIDATION.md.
