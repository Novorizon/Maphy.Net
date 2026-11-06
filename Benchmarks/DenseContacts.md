# Dense contacts, stacked bodies and remaining CCD work

This pass keeps solver iterations, fixed-point arithmetic, contact order and
simulation results unchanged. The before revision is the instrumentation-only
commit `9424bac2fb7a500634ac3fbae0bfecc567786a18`; the after revision contains this
report. The instrumentation commit preserves the preceding physics algorithms
while adding the identical benchmark and phase diagnostics used on both sides.

## Measurement conditions and results

.NET 8 / SDK 8.0.427, Linux x64, Release, tiered compilation disabled, profiling
enabled, eight warmup iterations and 100 measured iterations per fresh process.
Four before/after pairs alternate which version starts first. Both processes
use the same allowed CPU (CPU 4 here); no builds ran during measurement.

| Median repeat mean, ms/iteration | Before | After | Ratio |
| --- | ---: | ---: | ---: |
| DenseContacts / 100 | 7.310 | 4.548 | 1.61x |
| DenseContacts / 1,000 | 95.125 | 59.884 | 1.59x |
| StackSolver / 100 | 3.373 | 2.847 | 1.18x |
| StackSolver / 1,000 | 24.238 | 20.076 | 1.21x |
| CCD / 100 movers plus 100 walls | 0.643 | 0.581 | 1.11x |
| CCD / 1,000 movers plus 1,000 walls | 7.219 | 6.792 | 1.06x |

The dense and stacked 1,000-body observations reduce mean elapsed time by 37.0%
and 17.2%, respectively. Every paired final hash **and rolling per-frame trace
hash** matches. All 48 observations have zero measured simulation/hash allocation,
no measured-frame overflow, and identical before/after contact/pair ranges.

| Single-iteration latency, ms; median of four repeat percentiles | P50 before → after | P95 before → after | P99 before → after |
| --- | ---: | ---: | ---: |
| DenseContacts / 100 | 7.014 → 4.332 | 9.178 → 5.994 | 12.477 → 7.367 |
| DenseContacts / 1,000 | 93.267 → 57.996 | 109.145 → 66.754 | 133.887 → 76.353 |
| StackSolver / 100 | 3.407 → 2.954 | 5.636 → 4.954 | 10.906 → 8.511 |
| StackSolver / 1,000 | 24.423 → 20.175 | 31.244 → 26.612 | 37.371 → 31.611 |
| CCD / 100 | 0.575 → 0.540 | 0.992 → 0.838 | 1.526 → 0.970 |
| CCD / 1,000 | 6.898 → 6.550 | 8.867 → 8.550 | 9.947 → 9.896 |

Percentiles use nearest rank within each 100-sample repeat, then the table takes
the median across repeats; these are not pooled percentiles. Samples time one
harness call including its public setters and optional profiling, excluding
state hashing. Samples are allocated before measurement and sorted after it.
No frame CSV or console output is written inside the measured loop.

The fixtures evolve naturally after eight warmups. DenseContacts has 795 contact
manifolds at 100 bodies and 9,500–9,996 at 1,000. StackSolver ranges from 14–100
and 957–1,000, respectively. These are workload scaling tests, not a fully settled
stack or a solver-quality certification. CPU pinning does not isolate a cloud
host, and 400 samples per case do not establish production tail bounds.

Raw files: `2026-11-06-dense-100-before.csv`, `2026-11-06-dense-100-after.csv`,
`2026-11-06-dense-1000-before.csv`, `2026-11-06-dense-1000-after.csv`.

## Changes selected from the profile

| Median phase cost per 1,000-body iteration | Before | After |
| --- | ---: | ---: |
| DenseContacts: contact generation | 63.156 ms | 30.128 ms |
| DenseContacts: velocity solving, including reciprocal preparation | 21.546 ms | 19.545 ms |
| StackSolver: contact generation | 5.935 ms | 2.713 ms |
| StackSolver: velocity solving, including reciprocal preparation | 10.912 ms | 10.006 ms |
| CCD: complete TOI query | 2.290 ms | 2.268 ms |
| CCD: per-candidate shape/AABB casts, nested inside query | 0.388 ms | 0.374 ms |
| CCD: proxy envelope updates across preparation/integration | 1.165 ms | 0.853 ms |

Contact generation dominates the dense baseline; velocity solving dominates the
stacked baseline. An exactly canonical pair of axis-aligned boxes now tests
three distinct SAT axes instead of repeatedly projecting the same face/cross
axes. The shortcut preserves x/y/z order, strict penetration-tolerance ties,
reference-face choice, polygon clipping, feature IDs and contact insertion order.
Rotated boxes, invalid/negative extents and unrepresentable intermediate results
retain the full general SAT path. This shared implementation serves both APIs.

The solver reserves inverse-mass and diagonal inverse-inertia arrays, preparing
them after wake propagation. These properties stay constant through warm start,
velocity iterations and position iterations; only the repeated reciprocal
calculation is removed. Preparation cost is included in VelocitySolveMs. The
cache clears before sanitization/sleep handling and is rebuilt each update, so
mass, material, type, sleep and lifecycle changes are observed. It adds **32 bytes
per reserved body**, excluding headers. `EnableSolverMassCache=false` selects
live reciprocals for differential testing and is excluded from replay hashes.

CCD shape changes already refresh their proxies during the same dirty flush.
RefreshCCDBody now skips recalculating those envelopes if cached target velocity
has not changed. Gravity, clamping and other velocity changes still refresh them.
The sparse 1,000-mover fixture retains 2,000 candidates, 1,000 shape tests and
2,000 tight-envelope updates; redundant reused-envelope calls fall from 3,999
to 1,999. Sequential target snapshots and numerical fallback remain unchanged.

`ccdQueryTicks`, `ccdShapeTicks` and `ccdProxyTicks` are optional nested timings
in LastStepProfile. They aggregate fixed substeps and are excluded from TotalTicks
and replay hashing. Shape time is contained in query time; proxy time spans
main phases. Do not add them to the primary phase totals. The profile shows that
per-candidate cast math is a small part of remaining CCD cost in this fixture; moving-shape
preparation, tree traversal/filtering and other integration work merit investigation
before changing the cast algorithms.

## Reproduction and gates

The repository projects use .NET 9. To reproduce the .NET 8 adapters used here,
create clean before/after worktrees and put this `Measurement.csproj` in each:

```xml
<Project Sdk="Microsoft.NET.Sdk">
  <PropertyGroup>
    <OutputType>Exe</OutputType><TargetFramework>net8.0</TargetFramework>
    <AllowUnsafeBlocks>true</AllowUnsafeBlocks>
    <EnableDefaultCompileItems>false</EnableDefaultCompileItems><NoWarn>CS8981</NoWarn>
  </PropertyGroup>
  <ItemGroup>
    <Compile Include="Maphy.Mathematics/**/*.cs"/><Compile Include="Maphy.Physics/**/*.cs"/>
    <Compile Include="Maphy.Tree/**/*.cs"/><Compile Include="Maphy.Net.Benchmarks~/Program.cs"/>
  </ItemGroup>
</Project>
```

```bash
git worktree add --detach /tmp/maphy-dense-before 9424bac2fb7a500634ac3fbae0bfecc567786a18
git worktree add --detach /tmp/maphy-dense-after HEAD
# Write the Measurement.csproj above into both worktree roots.
dotnet build /tmp/maphy-dense-before/Measurement.csproj -c Release -o /tmp/maphy-dense-before-bin
dotnet build /tmp/maphy-dense-after/Measurement.csproj -c Release -o /tmp/maphy-dense-after-bin
for count in 100 1000; do
  python Tools/measure_paired_benchmarks.py --before /tmp/maphy-dense-before-bin/Measurement.dll --after /tmp/maphy-dense-after-bin/Measurement.dll --out-prefix /tmp/dense-$count --scenario DenseContacts,StackSolver,CCD --count $count --iterations 100 --repeat 4 --cpu 4
  python Tools/check_benchmarks.py /tmp/dense-$count-before.csv
  python Tools/check_benchmarks.py /tmp/dense-$count-after.csv
  python Tools/compare_ccd_benchmarks.py /tmp/dense-$count-before.csv /tmp/dense-$count-after.csv --same-backend
done
```

Choose an allowed CPU on the host, or omit `--cpu` outside Linux. Both revisions
must use the same runtime and build options; timings from earlier sessions or
different instrumented harnesses should not be combined into one speedup ratio.

Both arithmetic backends pass **147 test groups** and the existing **1,239-row**
corpus, with unchanged numeric/rolling physics fingerprints. New coverage checks
4,269 frozen box-pair results against the original 15-axis SAT, including anchors,
features, tolerance ties, touching, degeneracy and invalid-range fallback. A
cached/live solver replay spans 4 seeds × 96 frames, material/mass/inertia changes,
sleep/wake, type changes, reserve growth, slot reuse and diagnostic-mode switching.
Existing CCD differential and snapshot replays remain unchanged.

The .NET Standard 2.1 build has zero warnings/errors. All 18 repeated quick
measurements pass, as do the 1,000-body CI-sized dense/stack workloads and indexed
versus reference CCD comparison. CI checks now also reject any measured-frame
overflow, divergent rolling traces, malformed latency/range summaries and nested
timing inconsistencies; wall-clock speed remains a reported observation.

Dense 1,000-body contact generation (~30 ms) and velocity solving (~20 ms) remain
the main costs. Further split contact geometry/clipping/manifold work and solver
constraint preparation before the next optimization. Actual Unity 2022.3
Mono/IL2CPP execution still needs a licensed environment; these measurements
establish no Unity frame-rate guarantee.
