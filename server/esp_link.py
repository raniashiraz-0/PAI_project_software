"""
esp_link.py - Two-way framed TCP link to the main ESP32.

Frame: 0xAA | LEN | CMD | PAYLOAD | CHK | 0x55   (LEN = 1 + len(payload),
CHK = LEN ^ CMD ^ payload bytes). See firmware/src/esp32_main/protocol.h.

* reconnects by itself (host can be "auto" -> UDP discovery)
* background reader thread calls on_frame(cmd, payload) for every frame
* send_and_wait() waits for the ESP32's ACK / NACK of a command
* the link is declared dead if no frame (heartbeat/telemetry) arrives for 3 s
"""
import socket
import threading
import time
from typing import Callable, Dict, Optional, Tuple

START, END = 0xAA, 0x55

RESP_ACK = 0x01
RESP_NACK = 0x02
RESP_CMD_DONE = 0x03
RESP_CMD_FAILED = 0x04
RESP_HEARTBEAT = 0x05
RESP_TELEMETRY = 0x06
RESP_OBSTACLE = 0x0E

ERROR_TEXT = {
    0x00: "none",
    0x07: "joint out of range",
    0x08: "bad payload",
    0x09: "grab not authorised",
    0x0A: "servo stall",
    0x0B: "torque limit",
    0x0C: "obstacle in the way",
    0x0D: "e-stop is active",
    0x0F: "busy (still moving)",
    0x10: "PCA9685 servo driver not responding (check servo battery / I2C wires)",
    0x11: "unknown command",
}


def make_frame(cmd: int, payload: bytes = b"") -> bytes:
    length = 1 + len(payload)
    chk = length ^ cmd
    for b in payload:
        chk ^= b
    return bytes([START, length, cmd]) + bytes(payload) + bytes([chk, END])


class FrameParser:
    def __init__(self):
        self.state = 0
        self.length = 0
        self.cmd = 0
        self.chk = 0
        self.buf = bytearray()

    def feed(self, data: bytes):
        frames = []
        for b in data:
            if self.state == 0:
                if b == START:
                    self.state = 1
            elif self.state == 1:
                if b < 1 or b > 65:
                    self.state = 0
                    continue
                self.length, self.chk, self.buf = b, b, bytearray()
                self.state = 2
            elif self.state == 2:
                self.cmd = b
                self.chk ^= b
                self.state = 3 if self.length > 1 else 4
            elif self.state == 3:
                self.buf.append(b)
                self.chk ^= b
                if len(self.buf) >= self.length - 1:
                    self.state = 4
            elif self.state == 4:
                self.state = 5 if b == self.chk else 0
            elif self.state == 5:
                self.state = 0
                if b == END:
                    frames.append((self.cmd, bytes(self.buf)))
        return frames


class EspLink:
    def __init__(self, name: str, host_resolver: Callable[[], Optional[str]], port: int,
                 on_frame: Optional[Callable[[int, bytes], None]] = None,
                 on_disconnect: Optional[Callable[[str], None]] = None,
                 reconnect_s: float = 3.0, dead_after_s: float = 3.0, auto_reconnect: bool = True):
        self.name = name
        self.host_resolver = host_resolver
        self.port = port
        self.on_frame = on_frame
        self.on_disconnect = on_disconnect
        self.reconnect_s = reconnect_s
        self.dead_after_s = dead_after_s
        self.current_host: Optional[str] = None
        self.last_rx = 0.0
        self.last_error = ""

        self._sock: Optional[socket.socket] = None
        self._send_lock = threading.Lock()
        self._running = True
        self._replies: Dict[int, Tuple[threading.Event, list]] = {}
        self._reply_lock = threading.Lock()
        self._maint = None
        if auto_reconnect:
            self._maint = threading.Thread(target=self._maintain, daemon=True, name=f"EspLink-{name}")
            self._maint.start()

    # ------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> bool:
        host = self.host_resolver()
        if not host:
            self.last_error = "ESP32 address unknown (waiting for discovery)"
            return False
        try:
            sock = socket.create_connection((host, self.port), timeout=2.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(0.5)
        except OSError as exc:
            self.last_error = f"{host}:{self.port} unreachable ({exc})"
            return False
        with self._send_lock:
            old, self._sock = self._sock, sock
        if old:
            try:
                old.close()
            except OSError:
                pass
        self.current_host = host
        self.last_rx = time.time()
        threading.Thread(target=self._reader, args=(sock,), daemon=True, name=f"EspLinkRx-{self.name}").start()
        print(f"[EspLink {self.name}] connected to {host}:{self.port}")
        return True

    def send(self, cmd: int, payload: bytes = b"") -> bool:
        frame = make_frame(cmd, payload)
        with self._send_lock:
            sock = self._sock
            if sock is None:
                return False
            try:
                sock.sendall(frame)
                return True
            except OSError as exc:
                self.last_error = str(exc)
        self._drop(sock, f"send failed: {self.last_error}")
        return False

    def send_and_wait(self, cmd: int, payload: bytes = b"", timeout: float = 1.0) -> Tuple[bool, str]:
        """Send and wait for ACK (True, "") or NACK (False, reason)."""
        ev = threading.Event()
        holder: list = []
        with self._reply_lock:
            self._replies[cmd] = (ev, holder)
        if not self.send(cmd, payload):
            with self._reply_lock:
                self._replies.pop(cmd, None)
            return False, "not connected"
        ok = ev.wait(timeout)
        with self._reply_lock:
            self._replies.pop(cmd, None)
        if not ok:
            return False, "no reply from ESP32"
        kind, err = holder[0]
        if kind == RESP_ACK:
            return True, ""
        return False, ERROR_TEXT.get(err, f"error 0x{err:02X}")

    def close(self):
        self._running = False
        sock, self._sock = self._sock, None
        if sock:
            try:
                sock.close()
            except OSError:
                pass

    # ------------------------------------------------------------------
    def _maintain(self):
        while self._running:
            if self._sock is None:
                self.connect()
            elif time.time() - self.last_rx > self.dead_after_s:
                self._drop(self._sock, "no data from ESP32 for 3 s")
            time.sleep(self.reconnect_s if self._sock is None else 0.5)

    def _drop(self, sock, reason: str):
        with self._send_lock:
            if self._sock is not sock:
                return
            self._sock = None
        try:
            sock.close()
        except OSError:
            pass
        self.last_error = reason
        print(f"[EspLink {self.name}] disconnected: {reason}")
        if self.on_disconnect:
            try:
                self.on_disconnect(reason)
            except Exception as exc:  # never kill the link thread
                print(f"[EspLink {self.name}] on_disconnect error: {exc}")

    def _reader(self, sock):
        parser = FrameParser()
        while self._running and self._sock is sock:
            try:
                data = sock.recv(512)
            except socket.timeout:
                continue
            except OSError as exc:
                self._drop(sock, f"receive failed: {exc}")
                return
            if not data:
                self._drop(sock, "ESP32 closed the connection")
                return
            self.last_rx = time.time()
            for cmd, payload in parser.feed(data):
                if cmd in (RESP_ACK, RESP_NACK) and payload:
                    with self._reply_lock:
                        waiter = self._replies.get(payload[0])
                    if waiter:
                        waiter[1].append((cmd, payload[1] if len(payload) > 1 else 0))
                        waiter[0].set()
                if self.on_frame:
                    try:
                        self.on_frame(cmd, payload)
                    except Exception as exc:
                        print(f"[EspLink {self.name}] frame handler error: {exc}")
