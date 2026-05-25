#!/usr/bin/env bash
# Set up TwinLiteNet+ on the K-Patrol Pi.
#
# What this script does (idempotent — re-run any time):
#   1. Clone or pull the TwinLiteNetPlus repo into ~/kpatrol/3rdparty/
#   2. Install pip deps needed for ONNX export + Pi inference
#   3. Download pretrained Nano+ checkpoint (if not already present)
#   4. Export the ONNX file the LaneSegProvider expects, at 192x320 input
#   5. Drop it into ~/kpatrol/pi-controller/models/twinlitenet_nano_192x320.onnx
#
# Run from anywhere; defaults match the Pi service layout. Override with:
#   KPATROL_HOME=/some/path  WEIGHTS_URL=...  bash setup_twinlite.sh

set -euo pipefail

KPATROL_HOME="${KPATROL_HOME:-$HOME/kpatrol}"
THIRDPARTY_DIR="${KPATROL_HOME}/3rdparty"
TWINLITE_DIR="${THIRDPARTY_DIR}/TwinLiteNetPlus"
MODEL_OUT="${KPATROL_HOME}/pi-controller/models/twinlitenet_nano_192x320.onnx"

INPUT_H="${INPUT_H:-192}"
INPUT_W="${INPUT_W:-320}"
VARIANT="${VARIANT:-nano}"   # nano | small | medium | large

bold() { printf "\033[1m%s\033[0m\n" "$*"; }
info() { printf "  • %s\n" "$*"; }

bold "─── 1. Clone TwinLiteNetPlus ─────────────────────────────────────"
mkdir -p "$THIRDPARTY_DIR"
if [ -d "$TWINLITE_DIR/.git" ]; then
  info "Repo already present — git pull"
  git -C "$TWINLITE_DIR" pull --ff-only || true
else
  git clone --depth 1 https://github.com/chequanghuy/TwinLiteNetPlus.git "$TWINLITE_DIR"
fi

bold "─── 2. Install Python deps ──────────────────────────────────────"
# torch + onnx are needed for export; on Pi 4 we install torch CPU wheel
# (no CUDA). onnxruntime is the *runtime* used by Pi inference — keep
# it small (no GPU package).
python3 -m pip install --user --upgrade \
  "torch>=2.0,<2.4" \
  "torchvision" \
  "onnx>=1.14" \
  "onnxruntime>=1.16"   # CPU build is what Pi needs

bold "─── 3. Download pretrained Nano+ checkpoint ─────────────────────"
mkdir -p "$TWINLITE_DIR/pretrained"
CKPT="$TWINLITE_DIR/pretrained/twinlitenet_plus_${VARIANT}.pth"
if [ -f "$CKPT" ]; then
  info "Checkpoint already at $CKPT (size: $(du -h "$CKPT" | cut -f1))"
else
  # The upstream repo lists release URLs on its README — check there if
  # this default breaks. We grab the smallest variant (Nano+) by default
  # because Pi 4 CPU FPS only stays usable at that scale.
  WEIGHTS_URL="${WEIGHTS_URL:-https://github.com/chequanghuy/TwinLiteNetPlus/releases/download/v1.0/twinlitenet_plus_${VARIANT}.pth}"
  info "Downloading $WEIGHTS_URL"
  curl -L --fail -o "$CKPT" "$WEIGHTS_URL" || {
    echo "[ERROR] Could not auto-download. Visit the upstream releases page and"
    echo "        place the checkpoint at: $CKPT"
    exit 1
  }
fi

bold "─── 4. Export ONNX ──────────────────────────────────────────────"
mkdir -p "$(dirname "$MODEL_OUT")"
EXPORT_SCRIPT="$(dirname "$0")/export_onnx.py"
if [ ! -f "$EXPORT_SCRIPT" ]; then
  echo "[ERROR] Expected export script at $EXPORT_SCRIPT"
  exit 1
fi
python3 "$EXPORT_SCRIPT" \
  --repo "$TWINLITE_DIR" \
  --checkpoint "$CKPT" \
  --variant "$VARIANT" \
  --input-size "${INPUT_H}x${INPUT_W}" \
  --output "$MODEL_OUT"

bold "─── 5. Verify ───────────────────────────────────────────────────"
ls -la "$MODEL_OUT"
python3 -c "
import onnxruntime as ort
sess = ort.InferenceSession('$MODEL_OUT', providers=['CPUExecutionProvider'])
print('  Input:', sess.get_inputs()[0].name, sess.get_inputs()[0].shape)
for o in sess.get_outputs():
    print('  Output:', o.name, o.shape)
"

bold "All done."
info "Model ready at: $MODEL_OUT"
info "Next: run benchmark_lane_seg.py to measure FPS on this Pi."
