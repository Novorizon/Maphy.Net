# Runtime validation

The target Unity line is **2022.3**. The repository contains an executable corpus,
not a claim that Unity has already passed it. A licensed Editor and Linux Mono /
IL2CPP build support are required to complete real player validation.

## Gates and current evidence

| Gate | Evidence |
| --- | --- |
| Scalar input conversion | Exact rational oracles for 4,000 float and double bit patterns plus boundary cases; decimal bounds, subnormals, signed zero, infinities, and NaN. |
| Arithmetic backend agreement | Full tests and frozen corpus checked using Int128 and forced-portable arithmetic. |
| .NET operating systems | Numeric CI runs on Linux, Windows, and macOS. Consult the current workflow result for observed status. |
| Unity Mono / IL2CPP | Project generator and player builders target 2022.3; execution remains pending a licensed runner. |

`Validation~/expected.csv` freezes 1,239 rows: explicitly specified float/double/decimal
inputs, raw arithmetic boundaries, and 1,000 **per-frame** hashes for both worlds.
The replay exercises primitive shapes, rotation, a distance joint, repeated CCD,
sleep, body disable/enable, destruction, and handle reuse. Do not regenerate this
file merely to make a failing test pass; inspect the first differing row and
review any intended simulation change.

```bash
dotnet run --project Validation~/Maphy.Validation.csproj -c Release -- --out /tmp/maphy-native.csv
python Tools/compare_corpus.py Validation~/expected.csv /tmp/maphy-native.csv
dotnet run --project Validation~/Maphy.Validation.csproj -c Release -p:MaphyForcePortable=true -- --out /tmp/maphy-portable.csv
python Tools/compare_corpus.py Validation~/expected.csv /tmp/maphy-portable.csv
```

The comparison ignores platform newline encoding and an optional UTF-8 BOM.
Every data row must otherwise match exactly. It rejects empty/truncated output
and identifies the first mismatching row or frame.

## Real Unity 2022.3 players

Use the exact patch installed on the machine. The generator refuses an existing
output directory and copies only the runtime sources, adapters, corpus, and
build scripts into a new project; it does not alter an existing game project.

```bash
python3 Tools/prepare_unity_validation.py --output /tmp/maphy-unity --unity-version 2022.3.62f1
"$UNITY_EDITOR" -batchmode -nographics -quit -projectPath /tmp/maphy-unity -executeMethod MaphyValidationBuild.Mono -logFile /tmp/mono-editor.log --build-output /tmp/maphy-mono/player
/tmp/maphy-mono/player -batchmode -nographics -logFile /tmp/mono-player.log --corpus-out /tmp/mono.csv
python3 Tools/compare_corpus.py Validation~/expected.csv /tmp/mono.csv
```

Repeat the build with `MaphyValidationBuild.IL2CPP` and a different player output
directory, then run and compare that player as well. The players also check
Unity `Vector3` conversion and the Transform/body adapter round trip before
writing a successful corpus. Both builds select the .NET Standard API profile.

The manual `Unity 2022.3 runtime validation` workflow performs these commands on
a Linux self-hosted runner labeled `unity-2022.3`. Supply the installed Editor
path and exact version. The runner must already have an active license, Linux
build modules, and the C++ toolchain needed by IL2CPP. Editor/player logs and
corpora are uploaded even on failure. A failure or an unexecuted workflow is not
a determinism pass; record exact Editor patch, backend, OS, and architecture
with the successful artifacts.

The manual workflow must first exist on the repository's default branch to be
offered by GitHub's dispatch UI. Until this draft is merged, use the local CLI
commands above on a licensed machine; adding the file does not run Unity.

## Remaining limits

The corpus is a regression gate, not a proof over all possible scenes or all
platforms. Optional Unity.Mathematics adapters and additional player targets need
their own jobs. Trigonometric approximation accuracy and exhaustive matrix /
quaternion parity remain separate numeric validation work.
