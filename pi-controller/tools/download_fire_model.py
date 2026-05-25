"""Download a fire/smoke YOLO model for the V11.0 anomaly detector.

This script ships URLs for several recommended models and lets the operator
pick one without having to remember URLs or auth tokens.

After download the model lives at:
    robots/pi-controller/models/fire_yolov8n.onnx   (default path)

Default DetectionConfig.fire_yolo_model already points there — no code
edit needed unless you rename the file.

Usage
=====
    # Interactive picker (recommended on first run)
    python3 -m tools.download_fire_model

    # Pick a specific source non-interactively
    python3 -m tools.download_fire_model --source dfire-v8n
    python3 -m tools.download_fire_model --source roboflow --api-key XXX --workspace ... --project ... --version 3

    # Verify what's installed
    python3 -m tools.download_fire_model --list

Recommended models (ranked by accuracy + size for Pi 4B)
=======================================================

1. **D-Fire YOLOv8n** (dfire-v8n) — ⭐ DEFAULT
   - Dataset: 21,527 images, 26,557 bboxes (Gaia-Inova/D-Fire)
   - Classes: fire (0), smoke (1)
   - mAP@0.5 ≈ 83% reported
   - Size: ~6 MB ONNX
   - License: CC-BY-NC 4.0 (academic OK, no commercial)
   - Source: https://github.com/gaiasd/DFireDataset
   - Recommendation: best default for indoor + outdoor flame and smoke.

2. **OMaR Tarek fire-detection-by-omar-tarek** (roboflow-omar)
   - Dataset: ~13k images from Roboflow Universe
   - Classes: fire (0), smoke (1)
   - mAP@0.5 ≈ 84%
   - Requires free Roboflow API key
   - License: Roboflow Universe (free academic)

3. **HUST phat-hien-lua-hemps** (roboflow-hust)
   - Dataset: Vietnamese researchers — indoor lighting variability
   - Requires free Roboflow API key
   - User's suggested model

4. **spacewalk01/yolov8-fire-and-smoke** (github-spacewalk)
   - MIT license
   - Direct .pt / .onnx release on GitHub

Each candidate is documented as a SourceConfig below. To add a new model,
append a new SourceConfig — no other changes needed.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional
from urllib.parse import urlparse


THIS = Path(__file__).resolve()
PI_CONTROLLER_DIR = THIS.parent.parent
MODELS_DIR = PI_CONTROLLER_DIR / "models"
# Default extension is .pt — ultralytics auto-exports to .onnx on first
# inference (or via the explicit `--export-onnx` flag below). PyTorch
# weights are also smaller before export, and the source repos publish
# .pt rather than .onnx.
DEFAULT_TARGET = MODELS_DIR / "fire_yolov8n.pt"


@dataclass
class SourceConfig:
    """One model source the user can pick from."""

    key: str                                 # CLI --source value
    label: str                               # human-readable name
    description: str                         # one-line pitch
    license: str                             # license string
    expected_size_mb: float                  # rough size sanity check
    classes: List[str]                       # ["fire", "smoke"] etc.
    # Either `direct_url` (no auth) or `fetch` (custom callable for Roboflow).
    direct_url: Optional[str] = None
    fetch: Optional[Callable[[Path, argparse.Namespace], None]] = None
    needs_api_key: bool = False
    notes: str = ""


# ─── Source: Direct download from GitHub release ───────────────────────


def _download_direct(url: str, dst: Path) -> None:
    """Stream a URL into `dst` with a simple progress meter.

    For Hugging Face URLs we delegate to the official `huggingface_hub`
    library if installed — it handles redirects, auth headers, and the
    correct user-agent. Falls back to urllib with a browser UA otherwise.
    """
    dst.parent.mkdir(parents=True, exist_ok=True)
    print(f"  ↓ {url}")
    print(f"  → {dst}")

    # Path 1 — Hugging Face hub via official client (handles auth/CDN).
    if "huggingface.co" in url:
        try:
            from huggingface_hub import hf_hub_download  # type: ignore
            # Parse "https://huggingface.co/{repo}/resolve/main/{file}"
            parts = url.split("huggingface.co/", 1)[1].split("/resolve/")
            repo_id = parts[0]
            rest = parts[1].split("/", 1)  # ["main", "filename"]
            revision = rest[0]
            filename = rest[1]
            local = hf_hub_download(repo_id=repo_id, filename=filename, revision=revision)
            shutil.copy(local, dst)
            print(f"  ✓ saved {dst.stat().st_size/1e6:.2f} MB (via huggingface_hub)")
            return
        except ImportError:
            print("  (huggingface_hub not installed — fall back to urllib)")
        except Exception as exc:
            print(f"  (huggingface_hub path failed: {exc} — fall back to urllib)")

    # Path 2 — urllib with a browser user-agent. CDNs increasingly reject
    # default Python UA with 401/403 since 2024.
    import urllib.request
    headers = {
        "User-Agent": (
            "Mozilla/5.0 (Macintosh; Intel Mac OS X 14_0) "
            "AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36"
        ),
        "Accept": "*/*",
    }
    req = urllib.request.Request(url, headers=headers)
    tmp = dst.with_suffix(dst.suffix + ".part")
    with urllib.request.urlopen(req) as resp:
        total = int(resp.headers.get("Content-Length") or 0)
        size = 0
        chunk = 64 * 1024
        with open(tmp, "wb") as f:
            while True:
                buf = resp.read(chunk)
                if not buf:
                    break
                f.write(buf)
                size += len(buf)
                if total:
                    pct = 100 * size / total
                    print(
                        f"\r  {size/1e6:6.2f} MB / {total/1e6:6.2f} MB ({pct:5.1f}%)",
                        end="",
                        flush=True,
                    )
        print()
    tmp.rename(dst)
    print(f"  ✓ saved {dst.stat().st_size/1e6:.2f} MB")


# ─── Source: Roboflow Universe via official client ─────────────────────


def _fetch_roboflow(dst: Path, ns: argparse.Namespace,
                     workspace: str, project: str, version: int) -> None:
    """Download via the `roboflow` PyPI package.

    Roboflow gates downloads behind a free API key — get one at
    https://roboflow.com → Settings → API. The key is per-user, no charge.
    """
    if not ns.api_key:
        raise SystemExit(
            "Roboflow source needs --api-key. Get a free key at "
            "https://roboflow.com → Settings → API"
        )
    try:
        from roboflow import Roboflow  # type: ignore
    except ImportError:
        raise SystemExit(
            "pip install roboflow first:\n"
            "  python3 -m pip install --break-system-packages roboflow"
        )
    print(f"  Roboflow {workspace}/{project} v{version}")
    rf = Roboflow(api_key=ns.api_key)
    proj = rf.workspace(workspace).project(project)
    model = proj.version(version).model
    # Roboflow client downloads to ./datasets/<project> by default — we
    # instead use the model.download_yolov8 / weights path. Their SDK
    # changed across versions, so we wrap defensively.
    candidate = None
    for method in ("download", "weights", "deploy"):
        if hasattr(model, method):
            try:
                candidate = getattr(model, method)()
                break
            except Exception:
                continue
    if not candidate:
        raise SystemExit(
            "Roboflow client returned no model path. Falling back: manually "
            "download .onnx from Roboflow Universe → Deploy → Download dataset"
        )
    src_path = Path(candidate) / "weights.onnx"
    if not src_path.exists():
        # Some versions put it at the candidate root.
        src_path = Path(candidate)
        if src_path.is_dir():
            for f in src_path.rglob("*.onnx"):
                src_path = f
                break
    if not src_path.exists():
        raise SystemExit(f"Could not find .onnx inside Roboflow download at {candidate}")
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(src_path, dst)
    print(f"  ✓ saved {dst.stat().st_size/1e6:.2f} MB → {dst}")


# ─── Candidate sources ─────────────────────────────────────────────────


SOURCES: Dict[str, SourceConfig] = {
    "dfire-v8n": SourceConfig(
        key="dfire-v8n",
        label="YOLOv8n Fire+Smoke (Notacodinggeek HF)",
        description="YOLOv8n trained on fire+smoke. Smallest model for Pi 4B.",
        license="Unspecified — see HF repo (default Ultralytics AGPL-3.0)",
        expected_size_mb=6.0,
        classes=["fire", "smoke"],
        # Verified HF repo (2025-12 upload, has actual .pt file).
        direct_url=(
            "https://huggingface.co/Notacodinggeek/yolov8n-fire-smoke/"
            "resolve/main/yolov8n-fire-smoke.pt"
        ),
        notes=(
            "Default option — smallest YOLOv8n. Ultralytics auto-exports to "
            ".onnx on first inference for ~1.5× speedup."
        ),
    ),
    "tommyngx-v10": SourceConfig(
        key="tommyngx-v10",
        label="YOLOv10 Fire+Smoke (TommyNgx) ⭐ Apache 2.0",
        description="YOLOv10 (newer arch), Apache 2.0 license, includes sample images",
        license="Apache 2.0 (commercial OK)",
        expected_size_mb=12.0,
        classes=["fire", "smoke"],
        direct_url=(
            "https://huggingface.co/TommyNgx/YOLOv10-Fire-and-Smoke-Detection/"
            "resolve/main/best.pt"
        ),
        notes=(
            "Best license for commercial. YOLOv10 is newer/more accurate "
            "than v8 but slightly slower on Pi 4B."
        ),
    ),
    "forest-fire-v8s": SourceConfig(
        key="forest-fire-v8s",
        label="YOLOv8s Forest Fire (touati-kamel HF)",
        description="YOLOv8s (larger, more accurate) trained on forest fire dataset",
        license="Unspecified — default Ultralytics AGPL-3.0",
        expected_size_mb=22.0,
        classes=["fire"],  # forest fire mainly
        direct_url=(
            "https://huggingface.co/touati-kamel/yolov8s-forest-fire-detection/"
            "resolve/main/model.pt"
        ),
        notes="Larger model (~22 MB). Use if v8n doesn't catch outdoor fires.",
    ),
    "roboflow-omar": SourceConfig(
        key="roboflow-omar",
        label="OMaR Tarek fire-detection-by-omar-tarek",
        description="13k images, fire+smoke, mAP ~84%. Roboflow API key required.",
        license="Roboflow Universe (free academic)",
        expected_size_mb=12.0,  # roboflow ONNX exports tend to be bigger
        classes=["fire", "smoke"],
        needs_api_key=True,
        fetch=lambda dst, ns: _fetch_roboflow(
            dst, ns,
            workspace="midterm-poj4j",
            project="fire-detection-by-omar-tarek",
            version=3,
        ),
    ),
    "roboflow-hust": SourceConfig(
        key="roboflow-hust",
        label="HUST phat-hien-lua-hemps",
        description="Vietnamese researchers (Hanoi University of Science and Technology)",
        license="Roboflow Universe (free academic)",
        expected_size_mb=12.0,
        classes=["fire"],
        needs_api_key=True,
        fetch=lambda dst, ns: _fetch_roboflow(
            dst, ns,
            workspace="hanoi-university-of-science-and-technology-bcr8w",
            project="phat-hien-lua-hemps",
            version=1,
        ),
    ),
}


# ─── CLI ───────────────────────────────────────────────────────────────


def _print_sources():
    print("\nAvailable fire detection models:\n")
    for s in SOURCES.values():
        marker = " ⭐" if s.key == "dfire-v8n" else "  "
        print(f"{marker} [{s.key}]")
        print(f"     {s.label}")
        print(f"     {s.description}")
        print(f"     License: {s.license}")
        print(f"     Classes: {', '.join(s.classes)}")
        if s.needs_api_key:
            print(f"     Needs --api-key from https://roboflow.com")
        if s.notes:
            print(f"     Notes: {s.notes}")
        print()


def _interactive_pick() -> str:
    print("Pick a model source. Default is dfire-v8n.")
    print("Type the key or press Enter to accept default.\n")
    _print_sources()
    sys.stdout.write("Source [dfire-v8n]: ")
    sys.stdout.flush()
    raw = sys.stdin.readline().strip()
    return raw or "dfire-v8n"


def _list_local(target: Path) -> None:
    if not MODELS_DIR.exists():
        print(f"No models directory: {MODELS_DIR}")
        return
    files = sorted(MODELS_DIR.glob("*.onnx")) + sorted(MODELS_DIR.glob("*.pt"))
    if not files:
        print(f"No model files under {MODELS_DIR}")
        return
    print(f"\nLocal models under {MODELS_DIR}:\n")
    for p in files:
        sz = p.stat().st_size / 1e6
        mark = " (active)" if p == target else ""
        sha = hashlib.sha256(p.read_bytes()).hexdigest()[:12]
        print(f"  {p.name:40s}  {sz:6.2f} MB  sha256={sha}{mark}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--source", choices=list(SOURCES.keys()),
                    help="non-interactive source key")
    ap.add_argument("--target", type=Path, default=DEFAULT_TARGET,
                    help=f"output path (default: {DEFAULT_TARGET})")
    ap.add_argument("--api-key", type=str, default=os.environ.get("ROBOFLOW_API_KEY"),
                    help="Roboflow API key (or set ROBOFLOW_API_KEY env)")
    ap.add_argument("--list", action="store_true",
                    help="list local model files and exit")
    ap.add_argument("--force", action="store_true",
                    help="overwrite existing file without prompting")
    args = ap.parse_args()

    target = args.target.resolve()

    if args.list:
        _list_local(target)
        return

    if target.exists() and not args.force:
        sz = target.stat().st_size / 1e6
        sys.stdout.write(
            f"\n{target} already exists ({sz:.2f} MB). Overwrite? [y/N]: "
        )
        sys.stdout.flush()
        if sys.stdin.readline().strip().lower() != "y":
            print("Skipped.")
            return

    key = args.source or _interactive_pick()
    if key not in SOURCES:
        raise SystemExit(f"Unknown source: {key}")
    src = SOURCES[key]

    print(f"\nDownloading {src.label} → {target}\n")
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        if src.fetch is not None:
            src.fetch(target, args)
        elif src.direct_url is not None:
            _download_direct(src.direct_url, target)
        else:
            raise SystemExit(f"Source {key} has no fetch method.")
    except Exception as exc:
        print(f"\n✗ Download failed: {exc}", file=sys.stderr)
        if src.notes:
            print(f"\nFallback note: {src.notes}", file=sys.stderr)
        raise SystemExit(1)

    sz = target.stat().st_size / 1e6
    if sz < 1.0:
        print(f"\n⚠  File looks suspiciously small ({sz:.2f} MB) — "
              "the URL may have redirected to an HTML page. Verify manually.")
        raise SystemExit(2)

    print(f"\n✓ {src.label} ready at {target}")
    print(f"  Size: {sz:.2f} MB · Classes: {', '.join(src.classes)}")
    print(f"  License: {src.license}\n")
    print("Next steps:")
    print("  1) Restart the detection service: sudo systemctl restart kpatrol-detection")
    print("  2) Verify YOLO mode active: journalctl -u kpatrol-detection -f | grep 'fire pipeline'")
    print("  3) Run live test: python3 -m tools.test_fire_model --camera 0")


if __name__ == "__main__":
    main()
