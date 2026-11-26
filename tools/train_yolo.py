"""
train_yolo.py - Train the medicine-box detector used by the server (verifier.py).

Easiest way (the script sorts everything for you):
  1. Take photos:  python tools/capture_dataset.py paracetamol 80   (and cetirizine, omeprazole)
     -> dataset/raw/<name>/*.jpg
  2. Label them at https://www.makesense.ai  (labels: paracetamol, cetirizine, omeprazole,
     in that order), then Actions -> Export Annotations -> "YOLO format" -> you get a .zip of .txt files.
  3. Unzip those .txt files into  dataset/labels_in/   (any sub-folders are fine)
  4. Run:  python tools/train_yolo.py
     It matches every photo in dataset/raw with its .txt, splits 80% train / 20% val,
     trains, and saves data/yolo/medicine_boxes.pt (the server loads it automatically).

Already have a ready YOLO dataset? Put it in dataset/images/{train,val} + dataset/labels/{train,val}
and it is used as it is.

Class ids:  0 = paracetamol, 1 = cetirizine, 2 = omeprazole  (match "yolo_class" in data/medicines.json)
Install once:   pip install ultralytics
"""
import os
import random
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DATASET = os.path.join(ROOT, "dataset")
RAW = os.path.join(DATASET, "raw")
LABELS_IN = os.path.join(DATASET, "labels_in")
OUT = os.path.join(ROOT, "data", "yolo", "medicine_boxes.pt")
CLASSES = ["paracetamol", "cetirizine", "omeprazole"]
IMG_EXT = (".jpg", ".jpeg", ".png")


def _walk(folder, exts):
    found = {}
    if not os.path.isdir(folder):
        return found
    for dirpath, _, files in os.walk(folder):
        for f in files:
            if f.lower().endswith(exts):
                found[os.path.splitext(f)[0]] = os.path.join(dirpath, f)
    return found


def _count(folder, exts):
    return len(_walk(folder, exts))


def build_split():
    """Make images/{train,val} + labels/{train,val} from dataset/raw + dataset/labels_in."""
    images = _walk(RAW, IMG_EXT)
    labels = _walk(LABELS_IN, (".txt",))
    if not images:
        print("No photos found in", RAW)
        print("Take some first:  python tools/capture_dataset.py paracetamol 80")
        print("(the server and the ESP32-CAM must be running)")
        return False
    if not labels:
        print(f"Found {len(images)} photos but no labels in {LABELS_IN}")
        print("Label the photos at https://www.makesense.ai (labels: paracetamol, cetirizine, omeprazole),")
        print("export 'YOLO format', and unzip the .txt files into dataset\\labels_in\\ , then run this again.")
        return False
    pairs = [(images[k], labels[k]) for k in images if k in labels]
    if len(pairs) < 10:
        print(f"Only {len(pairs)} photos have a matching label file (need at least 10, ideally 150+).")
        print("The .txt file names must match the photo names (makesense keeps them the same).")
        return False
    random.seed(1)
    random.shuffle(pairs)
    n_val = max(2, len(pairs) // 5)
    for split in ("train", "val"):
        for kind in ("images", "labels"):
            d = os.path.join(DATASET, kind, split)
            shutil.rmtree(d, ignore_errors=True)
            os.makedirs(d)
    for i, (img, lab) in enumerate(pairs):
        split = "val" if i < n_val else "train"
        shutil.copy(img, os.path.join(DATASET, "images", split, os.path.basename(img)))
        shutil.copy(lab, os.path.join(DATASET, "labels", split, os.path.basename(lab)))
    print(f"Dataset ready: {len(pairs) - n_val} train + {n_val} val photos "
          f"({len(images) - len(pairs)} photos had no label and were skipped).")
    return True


def main():
    os.makedirs(DATASET, exist_ok=True)
    have_split = (_count(os.path.join(DATASET, "images", "train"), IMG_EXT) > 0 and
                  _count(os.path.join(DATASET, "labels", "train"), (".txt",)) > 0)
    if os.path.isdir(LABELS_IN) or not have_split:
        if not build_split():
            sys.exit(1)

    try:
        from ultralytics import YOLO
    except ImportError:
        print("Install the trainer first:  pip install ultralytics")
        sys.exit(1)

    yaml_path = os.path.join(DATASET, "medicine_boxes.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"path: {DATASET}\ntrain: images/train\nval: images/val\nnames:\n")
        for i, c in enumerate(CLASSES):
            f.write(f"  {i}: {c}\n")

    model = YOLO("yolov8n.pt")            # small + fast, fine for 3 box types
    model.train(data=yaml_path, epochs=60, imgsz=640, batch=8, patience=15, workers=0,
                project=os.path.join(ROOT, "runs"), name="medicine_boxes", exist_ok=True)

    best = os.path.join(ROOT, "runs", "medicine_boxes", "weights", "best.pt")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    shutil.copy(best, OUT)
    print(f"\nSaved {OUT}\nRestart the server - it will print 'YOLO loaded medicine_boxes.pt'.")


if __name__ == "__main__":
    main()
