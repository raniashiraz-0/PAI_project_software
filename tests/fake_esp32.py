"""
fake_esp32.py - A pretend main ESP32 that speaks the same framed protocol as
firmware/src/esp32_main (car on one TCP port, arm on another). Used by the tests to
check the server's real-hardware code without any hardware.

It mirrors the firmware behaviour that matters to the server:
  * ACK / NACK for every command, CMD_DONE when a step / arm move finishes
  * telemetry layouts (base type 1 = 17 bytes, arm type 2 = 36 bytes)
  * obstacle refusal (NACK ERR_KEEPOUT) and RESP_OBSTACLE
  * arm: servo range check, busy check, PCA offline, grab-gate on gripper close
"""
import socket
import threading
import time
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "server"))
from esp_link import make_frame, FrameParser  # noqa: E402

ACK, NACK, DONE, FAILED, HEARTBEAT, TELEMETRY, OBSTACLE = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0E
JOINT_OFFSET = [90.0, 0.0, 180.0, 90.0, 90.0]
HOME_SERVO = [90.0, 110.0, 35.0, 150.0, 90.0, 90.0]
GRIP_OPEN, GRIP_CLOSED = 90.0, 170.0


class FakeEsp32:
    def __init__(self):
        self.car_srv = socket.socket(); self.car_srv.bind(("127.0.0.1", 0)); self.car_srv.listen(1)
        self.arm_srv = socket.socket(); self.arm_srv.bind(("127.0.0.1", 0)); self.arm_srv.listen(1)
        self.car_port = self.car_srv.getsockname()[1]
        self.arm_port = self.arm_srv.getsockname()[1]
        self.running = True
        # base state
        self.dist_mm = [2000, 2000, 2000, 2000]     # front, back, left, right
        self.front_stop_mm = 150
        self.moving = False
        self.step_cmd = 0
        self.step_end = 0.0
        self.received = []                          # (port, cmd, payload)
        # arm state
        self.servo = list(HOME_SERVO)
        self.target = list(HOME_SERVO)
        self.arm_moving_until = 0.0
        self.arm_cmd = 0
        self.pca_ok = [True, True]
        self.auth = None                            # (med, token, until)
        self.stall_joint = None                     # make one joint "not reach" its target
        self.grip_sensor = None                     # None = not fitted, True/False = box detected
        self._socks = {}
        for name, srv in (("car", self.car_srv), ("arm", self.arm_srv)):
            threading.Thread(target=self._serve, args=(name, srv), daemon=True).start()
        threading.Thread(target=self._tick, daemon=True).start()

    # ------------------------------------------------------------------
    def _send(self, name, cmd, payload=b""):
        s = self._socks.get(name)
        if s:
            try:
                s.sendall(make_frame(cmd, payload))
            except OSError:
                self._socks.pop(name, None)

    def _serve(self, name, srv):
        srv.settimeout(0.2)
        while self.running:
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self._socks[name] = conn
            parser = FrameParser()
            conn.settimeout(0.2)
            while self.running:
                try:
                    data = conn.recv(256)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                for cmd, p in parser.feed(data):
                    self.received.append((name, cmd, p))
                    (self._car_cmd if name == "car" else self._arm_cmd)(cmd, p)
            self._socks.pop(name, None)
            if name == "car":
                self.moving = False

    def _car_cmd(self, cmd, p):
        if cmd in (0x10, 0x11, 0x12, 0x13):
            if cmd == 0x10 and self.dist_mm[0] < self.front_stop_mm:
                self._send("car", NACK, bytes([cmd, 0x0C])); return
            self.moving, self.step_cmd, self.step_end = True, cmd, time.time() + 0.3
            self._send("car", ACK, bytes([cmd]))
        elif cmd == 0x16:
            v = int.from_bytes(p[0:2], "big", signed=True)
            self.moving = abs(v) > 5
            self.step_cmd = 0
        elif cmd == 0x14:
            self.moving, self.step_cmd = False, 0
            self._send("car", ACK, bytes([cmd]))
        elif cmd == 0x17:
            self.front_stop_mm = int.from_bytes(p[0:2], "big")
            self._send("car", ACK, bytes([cmd]))
        else:
            self._send("car", ACK, bytes([cmd]))

    def _start_arm(self, cmd, tgt):
        if not all(self.pca_ok):
            self._send("arm", NACK, bytes([cmd, 0x10])); return
        if time.time() < self.arm_moving_until:
            self._send("arm", NACK, bytes([cmd, 0x0F])); return
        for j, v in enumerate(tgt):
            lo = 90.0 if j == 5 else 0.0
            if v < lo - 0.01 or v > 180.01:
                self._send("arm", NACK, bytes([cmd, 0x07])); return
        self.target = list(tgt)
        self.arm_cmd = cmd
        self.arm_moving_until = time.time() + 0.25
        self._send("arm", ACK, bytes([cmd]))

    def _arm_cmd(self, cmd, p):
        if cmd == 0x26:
            q = [int.from_bytes(p[2 + 2 * j:4 + 2 * j], "big") / 10.0 - 180.0 for j in range(5)]
            tgt = [JOINT_OFFSET[j] + q[j] for j in range(5)] + [self.servo[5]]
            self._start_arm(cmd, tgt)
        elif cmd == 0x25:
            self._start_arm(cmd, HOME_SERVO[:5] + [self.servo[5]])
        elif cmd == 0x23:
            self._start_arm(cmd, self.servo[:5] + [GRIP_OPEN])
        elif cmd == 0x24:
            ok = self.auth and p[0] == self.auth[0] and p[1:9] == self.auth[1] and time.time() < self.auth[2]
            if not ok:
                self._send("arm", NACK, bytes([cmd, 0x09])); return
            self._start_arm(cmd, self.servo[:5] + [GRIP_CLOSED])
        elif cmd == 0x31:
            self.auth = (p[0], p[1:9], time.time() + int.from_bytes(p[9:11], "big") / 1000.0)
            self._send("arm", ACK, bytes([cmd]))
        elif cmd == 0x32:
            self.auth = None
            self._send("arm", ACK, bytes([cmd]))
        elif cmd == 0x33:
            self.arm_moving_until = 0.0
            self.target = list(self.servo)
            self._send("arm", ACK, bytes([cmd]))
        else:
            self._send("arm", ACK, bytes([cmd]))

    # ------------------------------------------------------------------
    def _tick(self):
        last_tel = 0.0
        while self.running:
            now = time.time()
            if self.step_cmd and now >= self.step_end:
                self.moving, c, self.step_cmd = False, self.step_cmd, 0
                self._send("car", DONE, bytes([c]))
            if self.moving and self.dist_mm[0] < self.front_stop_mm and self.front_stop_mm > 0:
                c, self.step_cmd, self.moving = self.step_cmd, 0, False
                self._send("car", OBSTACLE, bytes([0]) + self.dist_mm[0].to_bytes(2, "big"))
                if c:
                    self._send("car", FAILED, bytes([c, 0x0C]))
            if self.arm_cmd and now >= self.arm_moving_until:
                for j in range(6):
                    self.servo[j] = self.target[j] + (7.0 if j == self.stall_joint else 0.0)
                c, self.arm_cmd = self.arm_cmd, 0
                self._send("arm", DONE, bytes([c]))
            if now - last_tel > 0.05:
                last_tel = now
                p = bytes([1]) + b"".join(d.to_bytes(2, "big") for d in self.dist_mm)
                p += bytes([1 if self.moving else 0]) + (0).to_bytes(4, "big") + (7400).to_bytes(2, "big") + bytes([0xFF])
                self._send("car", TELEMETRY, p)
                q = [self.servo[j] - JOINT_OFFSET[j] for j in range(5)]
                moving = 1 if now < self.arm_moving_until else 0
                flags = moving | (2 if self.auth else 0) | (8 if self.pca_ok[0] else 0) | (16 if self.pca_ok[1] else 0) \
                    | (32 if self.servo[5] > (GRIP_OPEN + GRIP_CLOSED) / 2 else 0) \
                    | (64 if self.grip_sensor else 0) | (128 if self.grip_sensor is not None else 0)
                a = bytes([2]) + b"".join(int(s * 10).to_bytes(2, "big") for s in self.servo)
                a += b"".join(int(round((v + 180) * 10)).to_bytes(2, "big") for v in q)
                a += b"".join(int(t * 10).to_bytes(2, "big") for t in self.target) + bytes([flags])
                self._send("arm", TELEMETRY, a)
            time.sleep(0.01)

    def close(self):
        self.running = False
        for s in list(self._socks.values()):
            try:
                s.close()
            except OSError:
                pass
        self.car_srv.close()
        self.arm_srv.close()
