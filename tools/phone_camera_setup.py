"""
phone_camera_setup.py - Connect the overhead phone camera in one step.

Run from the PharmaRover folder (VS Code -> Terminal -> New Terminal):

    python tools/phone_camera_setup.py

It asks which phone you have and its IP address, then:
  1. builds the right video address (Android IP Webcam / iPhone IP Camera Lite)
  2. tests the stream for a few seconds (frames per second, picture size)
  3. looks for the ArUco tags (floor 0-3, car 5, start 6, shelf 10) and saves a
     test picture with the tags marked: markers/phone_camera_test.jpg
  4. if it works, writes the address into server/config.yaml and sets
     tracker_sim: false  - so you never have to edit the YAML by hand

You can also pass the address directly:
    python tools/phone_camera_setup.py http://192.168.43.25:8080/video
"""
import os
import re
import sys
import time

import cv2

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CONFIG = os.path.join(ROOT, "server", "config.yaml")
TAG_NAMES = {0: "floor 0", 1: "floor 1", 2: "floor 2", 3: "floor 3", 5: "car", 6: "start", 10: "shelf"}


def ask(prompt, default=""):
    try:
        val = input(f"{prompt}{f' [{default}]' if default else ''}: ").strip()
    except EOFError:
        val = ""
    return val or default


def build_url():
    print("\nWhich phone is the overhead camera?")
    print("  1) Android - IP Webcam app")
    print("  2) iPhone  - IP Camera Lite app")
    kind = ask("Type 1 or 2", "1")
    print("\nPhone IP address: shown at the bottom of the app, or")
    print("  iPhone: Settings -> Wi-Fi -> (i) next to the hotspot -> IP Address")
    ip = ask("Phone IP (e.g. 192.168.43.25)")
    ip = re.sub(r"^https?://", "", ip).split("/")[0].split(":")[0]
    if not re.match(r"^\d{1,3}(\.\d{1,3}){3}$", ip):
        sys.exit(f"'{ip}' doesn't look like an IP address (four numbers with dots).")
    if kind == "2":
        user = ask("IP Camera Lite username", "admin")
        pwd = ask("IP Camera Lite password", "admin")
        return f"http://{user}:{pwd}@{ip}:8081/video"
    return f"http://{ip}:8080/video"


def open_stream(url):
    try:
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG,
                               [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 6000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 6000])
    except Exception:
        cap = cv2.VideoCapture(url)
    return cap


def test_stream(url, seconds=5.0):
    safe = re.sub(r"//[^@/]+@", "//***:***@", url)
    print(f"\nConnecting to {safe} ...")
    cap = open_stream(url)
    if not cap.isOpened():
        print("  FAILED: could not open the stream.")
        print("  Check: the app's server is started, the phone and laptop are on the SAME hotspot,")
        print("  the IP is right, and the address opens in Chrome on the laptop.")
        if "@" in url:
            print("  iPhone: also check the username/password (IP Camera Lite default: admin / admin).")
        return None
    t0, n, frame = time.time(), 0, None
    while time.time() - t0 < seconds:
        ok, f = cap.read()
        if ok and f is not None:
            frame, n = f, n + 1
    cap.release()
    if frame is None:
        print("  FAILED: connected, but no pictures arrived (is the phone screen locked?).")
        return None
    fps = n / seconds
    h, w = frame.shape[:2]
    print(f"  OK: {w}x{h} at {fps:.1f} frames/s")
    if fps < 3:
        print("  WARNING: under 3 frames/s - tracking will be jumpy. Lower the resolution/quality in the app.")
    return frame


def check_tags(frame):
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    corners, ids, _ = det.detectMarkers(frame)
    found = sorted(set(int(i) for i in ids.flatten())) if ids is not None else []
    print("\nArUco tags in view:", ", ".join(f"{i} ({TAG_NAMES.get(i, 'box')})" for i in found) or "none")
    missing_floor = [i for i in (0, 1, 2, 3) if i not in found]
    if missing_floor:
        print(f"  Floor tags not seen: {missing_floor}. All 4 must be visible for tracking -")
        print("  move the phone higher / point it straight down. (Fine to ignore if not laid out yet.)")
    else:
        print("  All 4 floor tags visible - the 2D map will work.")
    if 5 not in found:
        print("  Car tag 5 not seen (needed when the car is in the area).")
    out = frame.copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(out, corners, ids)
    os.makedirs(os.path.join(ROOT, "markers"), exist_ok=True)
    path = os.path.join(ROOT, "markers", "phone_camera_test.jpg")
    cv2.imwrite(path, out)
    print(f"  Test picture saved: {path}")
    return found


def save_config(url):
    with open(CONFIG, "r", encoding="utf-8") as f:
        text = f.read()
    new, n1 = re.subn(r'(overhead_phone:\s*\n(?:\s*#.*\n)*\s*source:\s*)"[^"]*"', rf'\g<1>"{url}"', text, count=1)
    new, n2 = re.subn(r"(\n\s*tracker_sim:\s*)(true|false)", r"\g<1>false", new, count=1)
    if n1 != 1 or n2 != 1:
        print("  Could not find the lines to change in config.yaml - edit it by hand:")
        print(f'    overhead_phone -> source: "{url}"    and    tracker_sim: false')
        return False
    with open(CONFIG, "w", encoding="utf-8") as f:
        f.write(new)
    print("  Saved to server/config.yaml (overhead_phone source + tracker_sim: false).")
    print("  Restart the server: Ctrl+C, then  python start_all.py")
    return True


def main():
    url = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else build_url()
    frame = test_stream(url)
    if frame is None:
        sys.exit(1)
    check_tags(frame)
    if "--yes" in sys.argv or ask("\nSave this camera to the server config? (y/n)", "y").lower().startswith("y"):
        save_config(url)


if __name__ == "__main__":
    main()
