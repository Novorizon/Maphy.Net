#!/usr/bin/env python3
"""Create an isolated Unity 2022.3 player project from the current checkout."""
import argparse
import json
import shutil
from pathlib import Path

def prepare(destination, version):
    if not version.startswith("2022.3."):
        raise ValueError("This validation project targets Unity 2022.3")
    root = Path(__file__).resolve().parents[1]
    destination = Path(destination).resolve()
    if destination.exists():
        raise ValueError("Output already exists; choose a new project directory")
    assets = destination / "Assets"
    core = assets / "Maphy"
    core.mkdir(parents=True)
    for name in ("Maphy.Mathematics", "Maphy.Physics", "Maphy.Tree", "Maphy.Extension"):
        shutil.copytree(root / name, core / name, ignore=shutil.ignore_patterns("*.meta"))
    shutil.copy2(root / "Maphy.Net.asmdef", core / "Maphy.Net.asmdef")
    validation = assets / "Validation"
    shutil.copytree(root / "Tools" / "Unity~", validation)
    shutil.copy2(root / "Validation~" / "DeterminismCorpus.cs", validation / "DeterminismCorpus.cs")
    (destination / "Packages").mkdir()
    (destination / "Packages" / "manifest.json").write_text(json.dumps({"dependencies": {}}, indent=2))
    settings = destination / "ProjectSettings"
    settings.mkdir()
    (settings / "ProjectVersion.txt").write_text(f"m_EditorVersion: {version}\n")
    print(destination)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--unity-version", required=True, help="Exact installed patch version, e.g. 2022.3.62f1")
    args = parser.parse_args()
    prepare(args.output, args.unity_version)
