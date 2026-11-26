"""
capture_dataset.py - Save ESP32-CAM pictures for training the YOLO box detector.

1. Start the server (verifier_sim can stay true) and power the ESP32-CAM.
2. Hold / place the medicine boxes in front of the gripper camera.
3. Run:   python tools/capture_dataset.py paracetamol 60
   -> saves 60 pictures (one every 0.5 s) to dataset/raw/paracetamol/
   Move the box around, change angle, distance and lighting while it runs.
4. Do the same for cetirizine and omeprazole (and some with all 3 boxes).
5. Label the pictures (e.g. https://www.makesense.ai or Roboflow, export "YOLO"
   format) with the class names: paracetamol, cetirizine, omeprazole
6. Train:  python tools/train_yolo.py
"""
import os
import sys
import time
import urllib.request

SERVER = os.environ.get("PHARMA_SERVER", "http://localhost:8000")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    label = sys.argv[1]
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 50
    out = os.path.join(os.path.dirname(__file__), "..", "dataset", "raw", label)
    os.makedirs(out, exist_ok=True)
    saved = 0
    print(f"Saving {count} pictures to {os.path.abspath(out)} ... move the box around!")
    while saved < count:
        try:
            with urllib.request.urlopen(f"{SERVER}/api/vision/hand_snapshot", timeout=3) as r:
                data = r.read()
            name = os.path.join(out, f"{label}_{int(time.time() * 1000)}.jpg")
            with open(name, "wb") as f:
                f.write(data)
            saved += 1
            print(f"  {saved}/{count}", end="\r")
        except Exception as exc:
            print(f"  waiting for the hand camera... ({exc})")
            time.sleep(1.0)
            continue
        time.sleep(0.5)
    print(f"\nDone: {saved} pictures.")


if __name__ == "__main__":
    main()
