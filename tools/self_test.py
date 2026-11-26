"""
self_test.py - Run the robot self-test from the terminal (the server must be running).

    python tools/self_test.py           checks only - nothing moves
    python tools/self_test.py --move    also moves the arm and drives 10 cm forward and back

Same as the "Run self-test" button in the dashboard (Live Robot & Teach tab).
"""
import json
import sys
import time
import urllib.request

SERVER = "http://localhost:8000"
COLORS = {"PASS": "\033[92m", "WARN": "\033[93m", "FAIL": "\033[91m", "SKIP": "\033[90m", "LOOK": "\033[96m"}
RESET = "\033[0m"


def call(path, body=None):
    req = urllib.request.Request(SERVER + path, data=json.dumps(body).encode() if body is not None else None,
                                 headers={"Content-Type": "application/json"}, method="POST" if body is not None else "GET")
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


def main():
    move = "--move" in sys.argv
    if move:
        ans = input("The arm will move and the car will drive 10 cm. Is the area clear? (y/n): ").strip().lower()
        if not ans.startswith("y"):
            return
    try:
        res = call("/api/selftest/run", {"allow_motion": move})
    except OSError:
        sys.exit("The server is not running. Start it first:  python start_all.py")
    if not res.get("ok"):
        sys.exit("Could not start: " + res.get("error", "unknown error"))
    shown = 0
    while True:
        rep = call("/api/selftest")
        for r in rep["results"][shown:]:
            c = COLORS.get(r["status"], "")
            print(f"{c}{r['status']:4}{RESET}  {r['name']}: {r['detail']}")
            if r["fix"]:
                print(f"        -> {r['fix']}")
        shown = len(rep["results"])
        if not rep["running"]:
            break
        time.sleep(0.5)
    c = rep["counts"]
    print(f"\nDone: {c['PASS']} passed, {c['WARN']} warnings, {c['FAIL']} failed, {c['LOOK']} to look at, {c['SKIP']} skipped.")


if __name__ == "__main__":
    main()
