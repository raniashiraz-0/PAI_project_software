"""
discovery.py - Finds the robot's ESP32 boards on the hotspot automatically.

Phone hotspots hand out random IP addresses, so instead of typing IPs:
  * the server broadcasts  "PRSRV <http_port>"  on UDP 4210 every 2 s
  * the main ESP32 answers "PRDEV main <ip> 5000 5001 8080"
  * the ESP32-CAM answers  "PRDEV cam <ip>"  and learns where to upload frames

Use host "auto" in config.yaml to use the discovered address.
"""
import socket
import threading
import time
from typing import Dict, Optional, Tuple


class Discovery:
    def __init__(self, udp_port: int = 4210, http_port: int = 8000, interval_s: float = 2.0):
        self.udp_port = udp_port
        self.http_port = http_port
        self.interval_s = interval_s
        self.devices: Dict[str, Tuple[str, float]] = {}   # name -> (ip, last_seen)
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None

    # ------------------------------------------------------------------
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True, name="Discovery")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    def get(self, name: str, max_age_s: float = 60.0) -> Optional[str]:
        with self._lock:
            entry = self.devices.get(name)
        if entry and time.time() - entry[1] <= max_age_s:
            return entry[0]
        return None

    def snapshot(self) -> Dict[str, Dict[str, object]]:
        now = time.time()
        with self._lock:
            return {n: {"ip": ip, "age_s": round(now - t, 1)} for n, (ip, t) in self.devices.items()}

    # ------------------------------------------------------------------
    @staticmethod
    def _broadcast_targets():
        """255.255.255.255 plus x.y.z.255 for every local IPv4 (Windows sends a
        plain broadcast out of one adapter only)."""
        targets = {"255.255.255.255"}
        try:
            for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                ip = info[4][0]
                if not ip.startswith("127."):
                    targets.add(".".join(ip.split(".")[:3] + ["255"]))
        except OSError:
            pass
        # The interface used for the default route
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe.connect(("8.8.8.8", 80))
            ip = probe.getsockname()[0]
            probe.close()
            targets.add(".".join(ip.split(".")[:3] + ["255"]))
        except OSError:
            pass
        return sorted(targets)

    def _run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind(("", 0))
            sock.settimeout(0.3)
            self._sock = sock
        except OSError as exc:
            print(f"[Discovery] could not open UDP socket: {exc}")
            self._running = False
            return

        beacon = f"PRSRV {self.http_port}".encode()
        next_beacon = 0.0
        while self._running:
            now = time.time()
            if now >= next_beacon:
                next_beacon = now + self.interval_s
                for target in self._broadcast_targets():
                    try:
                        sock.sendto(beacon, (target, self.udp_port))
                    except OSError:
                        pass
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                if not self._running:
                    break
                time.sleep(0.5)
                continue
            self._handle_reply(data, addr)

    def _handle_reply(self, data: bytes, addr):
        text = data.decode("ascii", errors="ignore").strip()
        parts = text.split()
        if len(parts) < 2 or parts[0] != "PRDEV":
            return
        name = parts[1]
        ip = addr[0]        # trust the packet source over the text
        with self._lock:
            old = self.devices.get(name)
            self.devices[name] = (ip, time.time())
        if not old or old[0] != ip:
            print(f"[Discovery] found '{name}' at {ip}")


# Shared instance used by the whole server
discovery = Discovery()


def resolve_host(host: str, fallback: str = "", device_name: str = "main") -> Optional[str]:
    """Return the IP/hostname to connect to. host == "auto" uses discovery."""
    if host and host.lower() != "auto":
        return host
    found = discovery.get(device_name)
    if found:
        return found
    return fallback or None
