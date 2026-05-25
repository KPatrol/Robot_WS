"""Export a TwinLiteNet+ PyTorch checkpoint to ONNX.

Stand-alone CLI so the heavy torch import only happens when we actually
need to export — Pi runtime never touches this file.

Usage:
    python3 export_onnx.py \
        --repo ~/kpatrol/3rdparty/TwinLiteNetPlus \
        --checkpoint ~/kpatrol/3rdparty/TwinLiteNetPlus/pretrained/twinlitenet_plus_nano.pth \
        --variant nano \
        --input-size 192x320 \
        --output ~/kpatrol/pi-controller/models/twinlitenet_nano_192x320.onnx
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import os
import sys
from typing import Optional, Tuple


def parse_size(s: str) -> Tuple[int, int]:
    h, w = s.lower().split("x")
    return int(h), int(w)


def _import_model_module(repo_root: str):
    """Add the upstream repo to sys.path so we can import its model module.

    Current upstream layout (2024-2025):
        TwinLiteNetPlus/
        ├── model/
        │   ├── config.py     ← sc_ch_dict with nano/small/medium/large
        │   └── model.py      ← TwinLiteNetPlus class
        └── val.py            ← from model.model import TwinLiteNetPlus

    The upstream model.py drags in `matplotlib.pyplot` at module scope
    which is dead code for inference but kills the import on machines
    where matplotlib is missing or broken (Mac Python 3.14 + pyexpat
    issue). We install a stub matplotlib in sys.modules before
    importing so the line `import matplotlib.pyplot as plt` succeeds
    without any real visualisation backend.
    """
    import types
    if "matplotlib" not in sys.modules:
        stub_mpl = types.ModuleType("matplotlib")
        stub_pyplot = types.ModuleType("matplotlib.pyplot")
        stub_mpl.pyplot = stub_pyplot  # type: ignore[attr-defined]
        sys.modules["matplotlib"] = stub_mpl
        sys.modules["matplotlib.pyplot"] = stub_pyplot

    sys.path.insert(0, repo_root)
    # Force-reimport in case a previous run left a stale `model` module.
    for mod_name in list(sys.modules.keys()):
        if mod_name == "model" or mod_name.startswith("model."):
            del sys.modules[mod_name]
    candidates = [
        "model.model",          # current upstream
        "model.TwinLite",       # older naming
        "model.TwinLiteNetPlus",
        "nets.TwinLite",
    ]
    last_err: Optional[Exception] = None
    for mod_name in candidates:
        try:
            mod = importlib.import_module(mod_name)
            # Sanity: make sure the class is actually exported here. If we
            # silently picked up the package namespace (no class inside),
            # try the next candidate.
            if any(hasattr(mod, n) for n in ("TwinLiteNetPlus", "TwinLiteNet", "Net")):
                return mod
        except Exception as exc:
            last_err = exc
            continue
    raise ImportError(
        f"Could not find TwinLiteNetPlus class under {repo_root}. "
        f"Last error: {last_err!r}. Tried candidates: {candidates}."
    )


def _build_model(mod, variant: str, checkpoint_path: str):
    """Instantiate the TwinLiteNet+ class for the requested variant and
    load the pretrained weights into it.

    Upstream `TwinLiteNetPlus.__init__(self, args=None)` expects an
    argparse-style namespace with a `.config` attribute holding the
    variant key ("nano" / "small" / "medium" / "large"). We synthesize
    that namespace here so callers don't need to mirror argparse.
    """
    import argparse
    import torch  # noqa: F401  (heavy import — defer until export time)

    cls = getattr(mod, "TwinLiteNetPlus", None)
    if cls is None:
        # Older revision named the class differently — try the fallbacks.
        for attr in ("TwinLiteNet", "Net"):
            if hasattr(mod, attr):
                cls = getattr(mod, attr)
                break
    if cls is None:
        raise AttributeError(
            f"No TwinLiteNetPlus class found in {mod.__name__}. "
            f"Attrs: {[a for a in dir(mod) if not a.startswith('_')][:20]}"
        )

    args_ns = argparse.Namespace(config=variant, half=False, verbose=False)
    try:
        model = cls(args_ns)
    except TypeError:
        # Fallback for older revisions with a simpler constructor.
        try:
            model = cls(variant=variant)
        except TypeError:
            model = cls()

    ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    state = ckpt.get("state_dict", ckpt) if isinstance(ckpt, dict) else ckpt
    # Some checkpoints wrap weights in a "module." prefix from DataParallel.
    if isinstance(state, dict) and any(k.startswith("module.") for k in state):
        state = {k.replace("module.", "", 1): v for k, v in state.items()}
    missing, unexpected = model.load_state_dict(state, strict=False)
    if missing:
        print(f"[export] missing keys: {len(missing)} (first 3: {missing[:3]})")
    if unexpected:
        print(f"[export] unexpected keys: {len(unexpected)} (first 3: {unexpected[:3]})")
    model.eval()
    return model


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--repo", required=True, help="Path to cloned TwinLiteNetPlus repo")
    p.add_argument("--checkpoint", required=True, help="Path to .pth checkpoint")
    p.add_argument("--variant", default="nano", choices=["nano", "small", "medium", "large"])
    p.add_argument("--input-size", default="192x320", help="HxW, e.g. 192x320")
    p.add_argument("--output", required=True, help="ONNX file destination")
    p.add_argument("--opset", type=int, default=13)
    args = p.parse_args()

    h, w = parse_size(args.input_size)

    print(f"[export] Loading {args.variant} from {args.checkpoint}")
    mod = _import_model_module(args.repo)
    model = _build_model(mod, args.variant, args.checkpoint)

    import torch
    dummy = torch.randn(1, 3, h, w, dtype=torch.float32)
    print(f"[export] Tracing at input shape (1,3,{h},{w}) → {args.output}")
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    torch.onnx.export(
        model,
        dummy,
        args.output,
        input_names=["input"],
        # The model has two heads. Most revisions return them as a tuple
        # in this order. The Pi-side provider tolerates 1- or 2-output
        # exports, so even if upstream changes order the runtime still
        # works.
        output_names=["drivable", "lane"],
        opset_version=args.opset,
        do_constant_folding=True,
        dynamic_axes={"input": {0: "batch"}, "drivable": {0: "batch"}, "lane": {0: "batch"}},
    )

    # Quick sanity check — load it back and run a forward pass with
    # numpy zeros to make sure ORT can ingest it on this machine.
    try:
        import onnxruntime as ort
        import numpy as np
        sess = ort.InferenceSession(args.output, providers=["CPUExecutionProvider"])
        zero = np.zeros((1, 3, h, w), dtype=np.float32)
        outs = sess.run(None, {"input": zero})
        for o in outs:
            print(f"  [verify] output shape: {o.shape} dtype: {o.dtype}")
    except Exception as exc:
        print(f"[verify] WARNING — ORT round-trip failed: {exc}")

    print("[export] Done.")


if __name__ == "__main__":
    main()
