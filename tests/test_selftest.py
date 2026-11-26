"""
test_selftest.py - Runs the robot self-test against a pretend ESP32, a pretend phone camera
and a pretend TFT relay, with movement allowed.   Run: python tests/test_selftest.py
"""
import os
import socket
import subprocess
import sys
import threading
import time
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "server"))
sys.path.insert(0, HERE)

import yaml  # noqa: E402
from fake_esp32 import FakeEsp32  # noqa: E402
from car_client import CarClient  # noqa: E402
from arm_client import ArmClient  # noqa: E402
from camera import CameraGrabber  # noqa: E402
from tracker import Tracker  # noqa: E402
from verifier import BoxVerifier  # noqa: E402
from display_client import DisplayClient  # noqa: E402
from audio_in import AudioInput  # noqa: E402
from audio_out import AudioOutput  # noqa: E402
from intents import IntentParser  # noqa: E402
from orchestrator import Orchestrator  # noqa: E402
from selftest import SelfTest  # noqa: E402


def free_port():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close(); return p


class FakeRelay:
    """Stands in for the ESP32's TFT relay port: records the lines the server sends."""
    def __init__(self):
        self.srv = socket.socket(); self.srv.bind(("127.0.0.1", 0)); self.srv.listen(1)
        self.port = self.srv.getsockname()[1]; self.lines = []
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        conn, _ = self.srv.accept()
        buf = b""
        while True:
            d = conn.recv(256)
            if not d:
                return
            buf += d
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.lines.append(line.decode())


class TestSelfTest(unittest.TestCase):
    def test_everything_connected_with_motion(self):
        cfg = yaml.safe_load(open(os.path.join(HERE, "..", "server", "config.yaml")))
        esp, relay = FakeEsp32(), FakeRelay()
        esp.dist_mm = [800, 900, 400, 500]
        phone_port = free_port()
        phone = subprocess.Popen([sys.executable, os.path.join(HERE, "fake_phone.py"), str(phone_port)],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.5)
        try:
            cfg["devices"]["cameras"]["overhead_phone"]["source"] = f"http://127.0.0.1:{phone_port}/video"
            car = CarClient(host="127.0.0.1", port=esp.car_port, sim=False)
            arm = ArmClient(host="127.0.0.1", port=esp.arm_port, sim=False)
            self.assertTrue(car.connect() and arm.connect())
            display = DisplayClient(mode="relay", host="127.0.0.1", tcp_port=relay.port)
            cam = CameraGrabber(name="phone", source=cfg["devices"]["cameras"]["overhead_phone"]["source"], fps_limit=15)
            tracker = Tracker(cfg, sim=False)
            stop = threading.Event()

            def feed():                                   # what the server's tracking loop does
                last = 0.0
                while not stop.is_set():
                    f, t = cam.get_latest_frame()
                    if f is not None and t > last:
                        last = t
                        tracker.process_frame(f, t)
                    time.sleep(0.03)
            threading.Thread(target=feed, daemon=True).start()
            verifier = BoxVerifier(cfg, [], sim=True)
            audio_in = AudioInput(IntentParser(display), sim=True)
            audio_out = AudioOutput(audio_in)
            orch = Orchestrator(car, arm, tracker, verifier, display, audio_in, audio_out,
                                {"room_mm": [3000, 2400], "cell_mm": 20.0}, config=cfg)
            ctx = types.SimpleNamespace(config=cfg, car=car, arm=arm, display=display, overhead_cam=cam,
                                        hand_cam=None, tracker=tracker, verifier=verifier, orchestrator=orch, audio_out=audio_out)
            time.sleep(3)
            st = SelfTest(ctx)
            self.assertTrue(st.start(allow_motion=True)["ok"])
            t0 = time.time()
            while st.running and time.time() - t0 < 90:
                time.sleep(0.3)
            rep = st.report()
            by = {r["name"]: r for r in rep["results"]}
            for r in rep["results"]:
                print(f"   {r['status']:4} {r['name']}: {r['detail']}")
            self.assertEqual(by["Main ESP32"]["status"], "PASS")
            self.assertEqual(by["Arm servo drivers (PCA9685)"]["status"], "PASS")
            for side in ("front", "back", "left", "right"):
                self.assertEqual(by[f"Ultrasonic {side}"]["status"], "PASS")
            self.assertEqual(by["TFT display"]["status"], "LOOK")
            self.assertIn("STATE|happy", relay.lines)
            self.assertEqual(by["Phone camera"]["status"], "PASS")
            self.assertEqual(by["Floor tags 0-3"]["status"], "PASS")
            self.assertEqual(by["Car tag 5"]["status"], "PASS")
            self.assertEqual(by["Car tracking"]["status"], "PASS")
            self.assertEqual(by["Arm movement"]["status"], "PASS")
            self.assertEqual(by["Gripper"]["status"], "LOOK")
            # the pretend phone shows a car tag that never moves -> the camera check must notice
            self.assertEqual(by["Driving"]["status"], "FAIL")
            self.assertIn("only", by["Driving"]["detail"])
            # ESP32 really got the 10 cm forward + back steps
            cmds = [c for n, c, p in esp.received if n == "car"]
            self.assertIn(0x10, cmds); self.assertIn(0x11, cmds)

            # a broken sensor and a missing servo board must be reported with a fix
            esp.dist_mm[2] = 15                            # left reads 1.5 cm
            esp.pca_ok = [True, False]
            time.sleep(0.3)
            st.start(allow_motion=False)
            while st.running:
                time.sleep(0.2)
            by = {r["name"]: r for r in st.report()["results"]}
            self.assertEqual(by["Ultrasonic left"]["status"], "FAIL")
            self.assertEqual(by["Arm servo drivers (PCA9685)"]["status"], "FAIL")
            self.assertIn("A0 pad", by["Arm servo drivers (PCA9685)"]["fix"])
            stop.set()
        finally:
            phone.terminate()
            esp.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
