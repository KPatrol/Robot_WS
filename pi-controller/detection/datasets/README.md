# K-Patrol Detection Datasets

Curated evaluation data for the person + fire detector used on the patrol robot.
The training set for `yolov8n.pt` is COCO (shipped with Ultralytics); this folder
holds **held-out indoor-patrol images** that more closely match the deployment
domain (corridor lighting, oblique camera angle on the robot chassis at ~40 cm).

## Layout

```
datasets/
├── README.md                 ← this file
├── manifest.csv              ← ground-truth labels (one row per image)
├── person/
│   ├── positive/*.jpg        ← person visible
│   └── negative/*.jpg        ← empty corridor / furniture / clutter
├── fire/
│   ├── positive/*.jpg        ← live flame / bright flame source
│   ├── negative/*.jpg        ← red clothing, warm lamps, sunset — the hard
│   │                           false-positive cases for HSV segmentation
│   └── hard_negative/*.jpg   ← optional, same as negative but manually curated
└── splits/
    ├── val.txt               ← filenames used for ROC/PR (~70%)
    └── test.txt               ← held out for final comparison (~30%)
```

## manifest.csv schema

```
relpath,label_person,label_fire,note
person/positive/0001.jpg,1,0,"full body, 2 m distance"
person/negative/0002.jpg,0,0,"empty hallway"
fire/positive/0003.jpg,0,1,"trash can fire, night shot"
fire/negative/0004.jpg,0,0,"red jacket under LED — known FP"
```

Labels are binary per class; images may have both (`label_person=1, label_fire=1`).

## Acquisition

- Person frames: recorded with `robots/camera-stream/record_route.py` during
  supervised lab walks. Frames sampled at 1 Hz to avoid near-duplicates.
- Fire frames: 40 clips from public fire datasets (BoWFire, FireNet) plus
  ~30 in-lab candle/lighter shots for realistic close-range flame.
- Negative-hard frames: collected by running the detector in dry-run mode
  through test corridors and manually saving any false-positive triggers.

## Reproducing evaluation

```bash
cd robots/pi-controller
python -m detection.eval_models --datasets detection/datasets --out reports/
```

Produces under `reports/`:
- `roc_person.png`, `pr_person.png`, `confusion_person.png`
- `roc_fire.png`,   `pr_fire.png`,   `confusion_fire.png`
- `model_comparison.csv` — mAP / precision / recall / FPS per model
- `thresholds.json` — operating points recommended for each class

## Notes for the thesis

The dataset is deliberately small (~150 images per class) — the point is
**domain validation** of an off-the-shelf model, not training a new one.
COCO already gives us person; fire is heuristic (HSV) and benefits most
from the curated negative set to tune `fire_min_area_ratio`.
