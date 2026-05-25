# K-Patrol YOLO Models

Thư mục này chứa các file model ONNX/PyTorch cho 2 nhánh phát hiện AI:

```
models/
├── yolov8n.onnx            # YOLOv8n COCO — Person detection (auto-export từ ultralytics)
├── yolov8n_int8.onnx       # YOLOv8n INT8 quantized (nhanh hơn 1.5×)
├── fire_yolov8n.onnx       # YOLOv8n Fire+Smoke — V11.0 (chạy `download_fire_model`)
└── fire_yolov8n_int8.onnx  # (tùy chọn) INT8 quantize của fire model
```

---

## V11.0 Fire YOLO model — Trạng thái hiện tại

**Quyết định 2026-05-25:** `fire_pipeline` giữ default **"hsv"** vì các
model fire YOLO public hiện có trên Hugging Face đều **train trên cảnh
outdoor (forest fire / wildfire)** — không generalize được cho **indoor
demo bật lửa**:

| Model thử | Kết quả với ảnh bật lửa indoor |
|-----------|-------------------------------|
| `touati-kamel/yolov8s-forest-fire-detection` | conf ~0.02 (quá thấp) — model coi flame nhỏ là "fog" |
| `Notacodinggeek/yolov8n-fire-smoke` | Tên gây nhầm: thật ra là vodka brand recognition |
| `TommyNgx/YOLOv10-Fire-and-Smoke-Detection` | Gated repo — cần HF token |

**Tạm thời:** dùng HSV V10.4 (Stage 1b skin gate) đã tune cho indoor demo
+ verify hoạt động với bật lửa thật + reject tay người.

**Khi nào enable YOLO:**
- Khi user có model train trên indoor scenarios (recommended: HUST
  Roboflow Vietnamese model — cần đăng ký Roboflow free)
- Hoặc khi tự train từ D-Fire dataset (~30 min trên Mac MPS)

**Hệ thống tự động fallback:** khi `fire_pipeline="yolo"` mà model file
thiếu hoặc fail load → tự dùng HSV với 1 warning log. Không crash.

### Cách tải model (1 trong 3 lựa chọn)

#### Lựa chọn 1 (recommended) — D-Fire YOLOv8n

```bash
cd /home/khoavd/kpatrol/pi-controller   # hoặc thư mục pi-controller trên Mac
python3 -m tools.download_fire_model --source dfire-v8n
```

- **21,000 ảnh** training (fire + smoke)
- **mAP@0.5 ≈ 83%**
- **~6 MB ONNX**
- **CC-BY-NC 4.0** (academic OK)

#### Lựa chọn 2 — spacewalk01/yolov8-fire-and-smoke

```bash
python3 -m tools.download_fire_model --source github-spacewalk
```

- GitHub release, không cần API key
- **MIT license** (commercial OK)

#### Lựa chọn 3 — Roboflow Universe (Vietnamese HUST hoặc OMaR Tarek)

```bash
# 1. Đăng ký tài khoản free tại https://roboflow.com → Settings → API
# 2. Copy API key vào ROBOFLOW_API_KEY env hoặc --api-key

export ROBOFLOW_API_KEY=xxxxxxxxxxxxxx
python3 -m tools.download_fire_model --source roboflow-hust       # Vietnamese
# hoặc
python3 -m tools.download_fire_model --source roboflow-omar       # OMaR Tarek
```

### Verify đã cài

```bash
python3 -m tools.download_fire_model --list

# Output:
# models/fire_yolov8n.onnx      6.42 MB  sha256=abc123... (active)
# models/yolov8n.onnx           6.05 MB  sha256=def456...
```

### Test với webcam (offline test trước khi deploy Pi)

```bash
# Live test trên Mac/Pi với webcam
python3 -m tools.test_fire_model --camera 0

# Side-by-side YOLO vs HSV
python3 -m tools.test_fire_model --camera 0 --compare

# Test với ảnh tĩnh
python3 -m tools.test_fire_model --image /path/to/fire.jpg
```

Bấm phím trong cửa sổ test:
- `q` — thoát
- `s` — snapshot lưu vào /tmp/fire_test_<ts>.jpg
- `m` — toggle YOLO ↔ HSV live

### Apply trên Pi

```bash
# Rsync model từ Mac → Pi
rsync -avz robots/pi-controller/models/fire_yolov8n.onnx \
  khoavd@10.8.0.152:/home/khoavd/kpatrol/pi-controller/models/

# Restart service
ssh khoavd@10.8.0.152 'sudo systemctl restart kpatrol-detection'

# Verify YOLO mode active
ssh khoavd@10.8.0.152 'journalctl -u kpatrol-detection -f | grep "fire pipeline"'
```

Output mong đợi:
```
[detector] loading fire YOLO: models/fire_yolov8n.onnx
[detector] fire pipeline = YOLO (models/fire_yolov8n.onnx · conf=0.30 · imgsz=416)
```

---

## Switch về HSV fallback nếu cần

### Tại venue có ánh sáng lạ → YOLO bị false positive nhiều?

Option A — env override (không cần edit code):
```bash
# Pi side
sudo systemctl set-environment KPATROL_FIRE_MODE=hsv
sudo systemctl restart kpatrol-detection
```

Option B — sửa DetectionConfig default:
```python
fire_pipeline: str = "hsv"   # was "yolo"
```

---

## Model nào tốt nhất?

| Model | Pros | Cons | Best for |
|-------|------|------|----------|
| **D-Fire** ⭐ | Dataset lớn (21k), 2 class, mAP 83% | CC-BY-NC (no commercial) | Academic thesis (đề xuất) |
| spacewalk | MIT license, GitHub stable | Smaller training data | Open-source friendly |
| HUST | Vietnamese indoor | Need API key, smaller | Demo Việt Nam |
| OMaR Tarek | mAP 84%, well-trained | Need API key, larger ONNX | Demo English |

---

## Custom training (nâng cao)

Nếu các model trên không phù hợp venue cụ thể:

```bash
# Train trên Mac M-series (MPS GPU) — ~30 phút cho YOLOv8n + D-Fire
pip install ultralytics
git clone https://github.com/gaiasd/DFireDataset
cd DFireDataset

python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.train(data='data.yaml', epochs=100, imgsz=416, device='mps')
model.export(format='onnx', imgsz=416)
"

# Copy weights/best.onnx → robots/pi-controller/models/fire_yolov8n.onnx
```

---

## File ignore (git)

Models lớn (>1MB) **không nên commit** vào git. `.gitignore` cấp project
đã exclude `*.onnx`, `*.pt`. Mỗi developer chạy `download_fire_model`
local sau khi clone repo.
