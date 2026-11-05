# CCD index reuse and linear island sleep processing

This follow-up targets the preparation and finalization costs exposed by phase
profiling. It retains exact contact/CCD calculations and the frozen replay corpus.

## Same-host before/after measurement

Both versions ran on .NET 8 using SDK 8.0.427 / Linux x64, Release,
`DOTNET_TieredCompilation=0`, profiling enabled, eight warmup updates, 30 measured
updates and two recreated-world repeats. The before source was copied before
these optimizations, after adding only sleep/sanitization subphase timers. The
after version uses the same scenario, input sequence and phase instrumentation.

| Median per update | Before | After |
| --- | ---: | ---: |
| CCD / 1,000 movers plus 1,000 walls: simulation | 38.393 ms | 15.094 ms |
| CCD: index preparation | 14.371 ms | 0.715 ms |
| CCD: finalization total | 10.888 ms | 0.553 ms |
| CCD: sleep processing | 10.344 ms | 0.098 ms |
| CCD: sanitization | 0.543 ms | 0.455 ms |
| SleepingBodies / 1,000 bodies: simulation | 5.794 ms | 1.655 ms |
| SleepingBodies: sleep processing | 4.698 ms | 0.102 ms |

The same-host median simulation ratios are **2.54x** for CCD and **3.50x** for
SleepingBodies. All four after observations have zero measured simulation/hash
allocation, no overflow and matching repeat/before state hashes. The warmed CCD
update creates/removes/rebuilds zero proxies, updates 2,000 tight envelopes and
reuses 3,999 unchanged envelopes. Updates within retained fat bounds do not
require reinserting tree nodes.

The raw files are `2026-11-05-ccd-reuse-before.csv` and
`2026-11-05-ccd-reuse-after.csv`. These are measurements on a non-isolated host,
not worst-frame bounds, Unity results or a general 60 FPS guarantee.

```bash
DOTNET_TieredCompilation=0 dotnet run --project Maphy.Net.Benchmarks~/Maphy.Net.Benchmarks.csproj -c Release -- --scenario CCD,SleepingBodies --count 1000 --warmup 8 --iterations 30 --repeat 2 --profile --out /tmp/reuse-after.csv
python Tools/check_benchmarks.py /tmp/reuse-after.csv
python Tools/compare_ccd_benchmarks.py Benchmarks/2026-11-05-ccd-reuse-before.csv /tmp/reuse-after.csv --same-backend
```

The comparison gates on setup, state hashes, allocations and capacity, reporting
timing per scenario without a hardware-dependent speed threshold. CI also checks
that the warmed sparse CCD benchmark actually reuses proxies.

## Retained CCD index

PrepareCCD reconciles existing proxy handles with current collider versions,
enabled state and trigger flags, removes stale leaves, and refits changed swept
envelopes. Unchanged tight envelopes use a direct reuse path. Static leaves and
the tree's retained fat bounds survive between updates and Reserve growth.

The numeric guard includes retained fat bounds before applying current changes,
so intermediate old/new unions remain representable. A large teleport can make
that historical union invalid while the current scene alone fits; preparation
retries one clean rebuild in this case. A genuinely unrepresentable current scene
still uses the reference scan. This recovery is bounded, reserved and allocation
free. CCD disabled/reference intervals and later re-enabling also reconcile stale
handles before any indexed query.

`LastCCDStats` adds proxiesCreated, proxiesUpdated, proxiesReused, proxiesRemoved,
and indexRebuilds counters. They sum executed substeps and are excluded from
replay hashes. Counts describe successful operations, not only tree reinsertions.
Dense or highly changing scenes may retain broad historical bounds and return
extra candidates; the sparse benchmark ratio does not describe that workload.

## Island member lists

The finalization split showed that sleep handling dominated; sanitization was
about half a millisecond in the CCD workload. Previously each island's eligibility,
timer update and sleep/wake action scanned the entire body high-water range.

BuildIslands now prepares reserved member chains in ascending body order.
Eligibility, time accumulation and sleep/wake visit those members directly. The
membership work across all islands becomes linear in body membership instead of
island-count times body-count. Sleep thresholds, timer arithmetic, wake behavior
and sanitization remain the same. Two int arrays add **8 bytes per reserved body**
to the memory estimate (16,000 bytes for the 2,000-body CCD benchmark).

`SleepMs` and `SanitizeMs` split FinalizeMs; they are subphases, so do not add them
again when summing overall phase totals. Their tick fields likewise partition
finalizeTicks in LastStepProfile. For positive fixed stepping, a call executing no
substeps clears diagnostics. With fixed stepping disabled, Step reports the direct
Update it performs, including a zero-delta Update.

## Validation and next targets

Both arithmetic backends pass 143 test groups. Added cases cover proxy reuse,
version replacement, trigger/body enable changes, growth, reference-mode gaps,
changing motion horizons, large-teleport rebuild recovery, and 1,000 independent
islands sleeping/waking with body deletion, reuse and type changes. The 1,239-row
corpus and existing rolling replay hashes are unchanged.

The next dominant target is repeated whole-array dirty synchronization inside CCD
hit processing, followed by dense-contact generation/solving. More workloads and
tail-latency measurements are needed before making frame-budget claims. Actual
Unity 2022.3 Mono/IL2CPP execution still requires a licensed environment.
