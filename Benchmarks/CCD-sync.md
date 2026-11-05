# Sparse dirty synchronization during CCD

Every CCD hit previously flushed pending transforms by scanning the entire body
and collider high-water ranges. Repeated hits made this bookkeeping quadratic
even after the candidate and index-reuse improvements in [CCD-reuse.md](CCD-reuse.md).

The dirty flags now use reserved hierarchical bitsets. A flush takes nonempty
64-index words and visits their set bits in ascending order. Body membership
still marks every owned collider, then all collider updates run in global
collider-index order. Pending transforms from earlier integrated bodies are
flushed at exactly the same CCD hit as before. This preserves sequential target
snapshots, compound ordering, tree refits and numerical fallback behavior.

Marks deduplicate without a queue capacity or sorting pass. Deletion and slot
reuse remove stale marks; Reserve preserves pending leaf bits and reconstructs
summary levels. Only Reserve allocates. At 2,000 reserved bodies and 2,000
colliders, the two dirty flag buffers shrink from 4,000 to 528 bytes, excluding
object/array headers. The memory-budget guide includes rounded hierarchy buffers.

## Paired measurements

Before is commit `9e081a666e9f433ffa216ef2a8b905c19590db37`; after is the revision
containing this report. Both use the same benchmark harness and scenario inputs,
.NET 8 / SDK 8.0.427, Linux x64, Release, disabled tiered compilation and profiling
enabled. No builds ran during measurement. Six before/after pairs alternated
which version started first. Each fresh process used eight warmup updates and
100 measured updates, pinned to the same allowed CPU (CPU 4 on this host).

| Median per measured iteration | Before | After |
| --- | ---: | ---: |
| CCD / 1,000 movers plus 1,000 walls | 12.564 ms | 8.235 ms |
| CCD: integration, including in-hit synchronization | 9.124 ms | 4.777 ms |
| CCD: top-level synchronization | 0.376 ms | 0.401 ms |
| DynamicBroadphase / 1,000 | 3.074 ms | 2.942 ms |
| StaticQuery / 1,000 | 2.232 ms | 2.153 ms |
| SleepingBodies / 1,000 | 1.028 ms | 1.039 ms |

CCD's observed median ratio is **1.53x**, a **34.5%** reduction in total elapsed
time. After repeat means range from 7.898 to 8.632 ms; before ranges from 11.935 to
21.090 ms. This range describes repeat means, not individual-frame tail latency.
Dense motion's top-level synchronization is 0.224 to 0.223 ms in
DynamicBroadphase; batching summary removal avoids walking the hierarchy for
every dirty object. Other scenarios do not establish a general speedup.

The raw files are `2026-11-05-ccd-sync-before.csv` and
`2026-11-05-ccd-sync-after.csv` (24 measurements each). All paired setup/state
hashes match, with zero measured simulation/hash allocation, no capacity overflow,
and unchanged CCD candidate/proxy work counts. SyncMs covers top-level flushes;
hit-triggered flushes remain part of IntegrationMs.

Scheduling still matters. The earlier **final-code unpinned** six-pair batch,
with eight warmups and 30 measured iterations, recorded CCD medians of
17.685 to 21.658 ms, including wide variation in unchanged phases. Those raw
files are retained as `2026-11-05-ccd-sync-unpinned-before.csv` and
`2026-11-05-ccd-sync-unpinned-after.csv`. Pinning is not CPU isolation. These
observations do not support a hardware-independent speed threshold or a frame
budget guarantee, and should not be combined with ratios from earlier sessions.

## Reproduction

Build both revisions with identical SDK/runtime and compilation options, then
run `Tools/measure_paired_benchmarks.py` against the preserved DLLs. The checked-in
projects target .NET 9. To reproduce the .NET 8 source adapters used here, create
two clean worktrees (run from the after revision):

```bash
git worktree add --detach /tmp/maphy-sync-before 9e081a666e9f433ffa216ef2a8b905c19590db37
git worktree add --detach /tmp/maphy-sync-after HEAD
python - <<'PY'
from pathlib import Path
project = '''<Project Sdk="Microsoft.NET.Sdk">
<PropertyGroup><OutputType>Exe</OutputType><TargetFramework>net8.0</TargetFramework>
<AllowUnsafeBlocks>true</AllowUnsafeBlocks><EnableDefaultCompileItems>false</EnableDefaultCompileItems>
<NoWarn>CS8981</NoWarn></PropertyGroup><ItemGroup>
<Compile Include="Maphy.Mathematics/**/*.cs"/><Compile Include="Maphy.Physics/**/*.cs"/>
<Compile Include="Maphy.Tree/**/*.cs"/><Compile Include="Maphy.Net.Benchmarks~/Program.cs"/>
</ItemGroup></Project>'''
for name in ('before', 'after'):
    Path('/tmp/maphy-sync-' + name, 'Measurement.csproj').write_text(project)
PY
dotnet build /tmp/maphy-sync-before/Measurement.csproj -c Release -o /tmp/maphy-sync-before-bin
dotnet build /tmp/maphy-sync-after/Measurement.csproj -c Release -o /tmp/maphy-sync-after-bin
python Tools/measure_paired_benchmarks.py --before /tmp/maphy-sync-before-bin/Measurement.dll --after /tmp/maphy-sync-after-bin/Measurement.dll --out-prefix /tmp/ccd-sync --iterations 100 --cpu 4
python Tools/check_benchmarks.py /tmp/ccd-sync-before.csv
python Tools/check_benchmarks.py /tmp/ccd-sync-after.csv
python Tools/compare_ccd_benchmarks.py /tmp/ccd-sync-before.csv /tmp/ccd-sync-after.csv --same-backend
```

Choose an allowed CPU on the measurement machine; omit `--cpu` for unpinned or
non-Linux execution. The runner disables tiered compilation for both binaries,
recreates each process, records execution order, and leaves the scenario harness
and hashes unchanged. Timings are reported rather than used as CI thresholds.

## Validation and next work

Both arithmetic backends pass 145 test groups. New integration cases cover
duplicate/reverse marks, bits 63/64 and 4095/4096, pending growth from one word to
three hierarchy levels, disabled/trigger colliders, body deletion and collider
slot reuse into a different compound, empty/repopulated summaries, and allocation
checks. A 64-frame compound CCD replay with earlier non-CCD/kinematic movers,
multiple hits, growth and lifecycle changes matches the old full-scan dirty
synchronizer's frozen fingerprint `72FF59AB79CD3A51` for indexed and reference CCD.

The existing 1,239-row corpus, numeric and rolling physics fingerprints remain
unchanged. .NET Standard 2.1 builds without warnings/errors; repeated profiled
quick benchmarks pass allocation, capacity, phase-total and index-reuse gates.
Actual Unity 2022.3 Mono/IL2CPP execution remains pending a licensed runner.

The quadratic dirty scan is removed. Next, profile dense contact generation and
solving, and the remaining CCD TOI/proxy-refit cost. Measure frame distributions
and representative scenes before choosing the next algorithm change.
