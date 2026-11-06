# Reproducible physics measurements

The original dated 100/1,000-object CSVs below precede CCD candidate pruning.
See [CCD.md](CCD.md) for the new indexed backend, phase timings and paired results.
The following [index-reuse report](CCD-reuse.md) records the next improvement to
15.094 ms/update and the linear island-sleep path.
The [dirty synchronization report](CCD-sync.md) records the next change, with a
new same-host pinned pair of 12.564 to 8.235 ms/update and retained unpinned data.
The [dense/solver follow-up](DenseContacts.md) adds per-frame latency and rolling
trace gates; its independent 1,000-body pairs are 95.125 to 59.884 ms for dense
contacts, 24.238 to 20.076 ms for stacking, and 7.219 to 6.792 ms for CCD.

The CSV separates **simulation** time/allocation from **state hashing**. The old
ElapsedMs field included both and should not be compared directly with the new
simulation columns. Values use invariant culture; timings include timestamp and
allocation-counter instrumentation and depend on JIT, hardware, and scheduling.

```bash
dotnet run --project Maphy.Net.Benchmarks~/Maphy.Net.Benchmarks.csproj -c Release -- --quick --iterations 30 --repeat 2 --out /tmp/quick.csv
python Tools/check_benchmarks.py /tmp/quick.csv
dotnet run --project Maphy.Net.Benchmarks~/Maphy.Net.Benchmarks.csproj -c Release -- --count 100 --iterations 30 --warmup 60 --repeat 2 --out /tmp/100.csv
dotnet run --project Maphy.Net.Benchmarks~/Maphy.Net.Benchmarks.csproj -c Release -- --count 1000 --iterations 30 --warmup 60 --repeat 2 --out /tmp/1000.csv
```

Scenarios cover static queries, moving broadphase, stacked boxes, sleeping
bodies, repeated CCD, and densely contacting boxes. `--count` selects dynamic
scenario objects, not total bodies: StackSolver adds a floor and CCD adds one
wall per moving body. Warmup defaults to 60 steps. Every repeat recreates the
world, so repeated hashes should match. Run the allocation/capacity checker on
the output; a timing result with overflow is not a valid complete-scene result.

## Allocation diagnosis and repair

On .NET 8 Linux, the original StackSolver/16 measurement allocated **12,768
bytes**. Per-stage instrumentation traced it to IntegrateBodies. Explicitly
initializing the trigonometric lookup class allocated exactly 12,768 bytes and
reduced that stage's subsequent allocations to zero. The first nonzero rotation
occurred after the original eight-step warmup, so warming only initial contacts
missed the lazy table initialization.

`PhysicsWorld.Reserve` now initializes those tables along with contact scratch.
A fresh-process first-rotation regression runs before the rest of the test suite
and requires zero bytes; extending benchmark warmup alone is not the fix.
Reserve should run on the simulation thread because contact scratch is
thread-local. Worlds are not safe for concurrent mutation.

Persistent manifold lookup also copied and scanned every previous manifold for
each candidate pair. Manifolds are emitted in sorted pair order, and lifecycle
changes clear the cache; lookup now searches that immutable previous-step order
with binary search and copies only a match. The frozen per-frame corpus guards
against a change in replay behavior.

The checked-in CSVs are observations from .NET 8.0.427, Linux x64 in the execution
environment, with no CPU affinity or isolated host. They demonstrate allocation,
capacity, and repeat consistency for those scenes, not frame-rate guarantees for
Unity or a comparison with another engine. Capacity-exhaustion behavior is
covered by the core's regression tests separately.

Observed median simulation milliseconds per iteration across two runs (30
measured steps after 60 warmup steps):

| Scenario | 100 objects | 1,000 objects |
| --- | ---: | ---: |
| StaticQuery | 0.838 | 2.858 |
| DynamicBroadphase | 0.701 | 5.204 |
| StackSolver | 6.809 | 35.596 |
| SleepingBodies | 0.236 | 6.010 |
| CCD | 8.460 | 667.438 |
| DenseContacts | 18.574 | 124.120 |

All 24 observations have zero measured simulation/hash allocation, no overflow,
and matching hashes between repeats. See the two dated CSVs for each sample and
its contact/candidate counts. The 1,000-object CCD result remains a substantial
bottleneck: core TOI selection scans collider targets for each moving collider.
Subsequent swept candidate pruning and its measured limits are described in
[CCD.md](CCD.md). These original results do not establish real-time viability.
The unoptimized
1,000-object full run was stopped after several minutes without a complete CSV;
there is no valid before/after speedup ratio to report.
