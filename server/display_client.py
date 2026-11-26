"""
display_client.py - Display client for the Arduino Uno + 3.5" TFT display shield.
Supports:
  - Serial mode (USB on bench: COM port, 115200 baud)
  - TCP Relay mode (on robot: WiFi to ESP32 port 8080 -> UART -> Uno D0)
  - Sim mode (software renderer in sim_display.py)
"""
import os
import json
import time
import socket
import threading
import difflib
from typing import Optional, Dict, Any
from events import bus, Event, EventType
from sim_display import SimDisplayRenderer

# Try rapidfuzz, fallback to difflib
try:
    from rapidfuzz import fuzz, process
    def fuzzy_match_ratio(s1: str, s2: str) -> float:
        return float(fuzz.ratio(s1.lower(), s2.lower()))
except ImportError:
    def fuzzy_match_ratio(s1: str, s2: str) -> float:
        return difflib.SequenceMatcher(None, s1.lower(), s2.lower()).ratio() * 100.0

HERE = os.path.dirname(os.path.abspath(__file__))
MEDICINES_PATH = os.path.join(HERE, "..", "data", "medicines.json")

class DisplayClient:
    def __init__(self, mode: str = "sim", host="192.168.4.1", tcp_port: int = 8080, serial_port: str = "COM4", baud_rate: int = 115200):
        # host may be a string or a function returning the current ESP32 address
        self.mode = mode
        self.host = host
        self.tcp_port = tcp_port
        self.serial_port = serial_port
        self.baud_rate = baud_rate

        self.sim_renderer = SimDisplayRenderer()
        self._sock: Optional[socket.socket] = None
        self._serial = None
        self._lock = threading.Lock()
        self._queue = []
        self._running = True

        self.state = "idle"
        self.status = "IDLE"
        self.progress = 0
        self.current_med: Optional[Dict[str, Any]] = None

        # Load medicines database
        self.medicines_data = self._load_medicines()

        # Sender thread (ensures 15ms spacing for Uno UART buffer)
        self._sender_thread = threading.Thread(target=self._send_worker, daemon=True)
        self._sender_thread.start()

        # Heartbeat thread (PING every 2.0s)
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_worker, daemon=True)
        self._heartbeat_thread.start()

    def _load_medicines(self) -> Dict[str, Any]:
        if os.path.exists(MEDICINES_PATH):
            with open(MEDICINES_PATH, "r", encoding="utf-8") as f:
                return json.load(f)
        return {"medicines": [], "shelves": []}

    def set_state(self, state: str):
        """Update emotional face state: idle, listening, thinking, scanning, alert, obstacle, happy, success, error."""
        state = state.lower().strip()
        self.state = state
        self.sim_renderer.state = state
        self.sim_renderer.show_card = False
        self._enqueue(f"STATE|{state}")
        bus.publish(Event(type=EventType.DISPLAY_STATE_UPDATE, source="DisplayClient", data={"state": state}))

    def show_medicine(self, med: Dict[str, Any], status: str = "NAVIGATING", progress: int = 0):
        """Show full medicine card with status text and progress bar."""
        self.current_med = med
        self.status = status
        self.progress = progress

        # Format card fields, truncating to Uno buffer limits
        grasp_pt = med.get('grasp_point_arm_frame_mm', [250, 0, -12])
        fields = {
            "name": str(med.get("name", "Unknown"))[:24],
            "strength": str(med.get("strength", "500mg"))[:10],
            "category": str(med.get("category", "General"))[:16],
            "dose": str(med.get("dose", "1-2 tabs"))[:20],
            "freq": str(med.get("frequency", "as needed"))[:20],
            "max": str(med.get("max_per_24h", "see label"))[:20],
            "use": str(med.get("use", "relief"))[:24],
            "warn": str(med.get("warning", "Demo only"))[:32],
            "shelf": f"{med.get('shelf_id', 'S1')}/L{med.get('level', 1)}"[:10],
            "aruco": str(med.get("aruco_id", "21")),
            "weight": f"{med.get('weight_g', 45)}g",
            "ean": str(med.get("ean13", "8961000000112")),
            "grasp": f"[{grasp_pt[0]:.0f}, {grasp_pt[1]:.0f}, {grasp_pt[2]:.0f}]",
            "box_size": str(med.get("box_size_mm", "100x50x20"))
        }

        # Update simulator
        self.sim_renderer.card_data = fields
        self.sim_renderer.status_text = status[:16]
        self.sim_renderer.progress = progress
        self.sim_renderer.show_card = True

        # Enqueue CARDF protocol lines
        for k, v in fields.items():
            # Sanitize pipe and newlines
            clean_v = v.replace("|", "/").replace("\n", " ").strip()
            self._enqueue(f"CARDF|{k}|{clean_v}")

        # Enqueue CARDSHOW
        self._enqueue(f"CARDSHOW|{status[:16]}|{progress}")
        bus.publish(Event(type=EventType.DISPLAY_CARD_UPDATE, source="DisplayClient", data={"med": fields, "status": status, "progress": progress}))

    def set_status(self, status: str):
        self.status = status
        self.sim_renderer.status_text = status[:16]
        self._enqueue(f"STATUS|{status[:16]}|{self.progress}")

    def set_progress(self, progress: int):
        self.progress = max(0, min(100, int(progress)))
        self.sim_renderer.progress = self.progress
        self._enqueue(f"STATUS|{self.status[:16]}|{self.progress}")

    def celebrate(self):
        """Celebrate delivery: happy face + checkmark for 2s, then return to idle."""
        self.set_state("success")
        threading.Timer(2.0, lambda: self.set_state("idle")).start()

    def clear(self):
        self.sim_renderer.show_card = False
        self.set_state("idle")
        self._enqueue("CLEAR")

    def get_preview_frame(self):
        """Returns 480x320 BGR frame for web view."""
        return self.sim_renderer.render()

    def match_medicine(self, text: str, threshold: float = 75.0) -> Optional[Dict[str, Any]]:
        """Fuzzy match text against medicine names, categories, and aliases."""
        if not text:
            return None
        text_clean = text.lower().strip()
        best_match = None
        best_score = 0.0

        for m in self.medicines_data.get("medicines", []):
            # Candidate strings
            candidates = [m["name"], m["category"]]
            aliases = m.get("aliases", [])
            if "paracetamol" in m["name"].lower():
                candidates += ["panadol", "paracetmol", "tylenol", "painkiller", "pain", "fever"]
            elif "cetirizine" in m["name"].lower():
                candidates += ["zyrtec", "allergy", "antihistamine", "antihistamin"]
            elif "omeprazole" in m["name"].lower():
                candidates += ["prilosec", "antacid", "heartburn", "acidity", "acid"]

            candidates += aliases

            for c in candidates:
                # Substring check first
                if c.lower() in text_clean:
                    return m
                score = fuzzy_match_ratio(c, text_clean)
                if score > best_score:
                    best_score = score
                    best_match = m

        if best_score >= threshold:
            return best_match
        return None

    def match_by_barcode(self, ean: str) -> Optional[Dict[str, Any]]:
        """Exact match by EAN-13 barcode."""
        ean = str(ean).strip()
        for m in self.medicines_data.get("medicines", []):
            if str(m.get("ean13", "")).strip() == ean:
                return m
        return None

    def _enqueue(self, line: str):
        with self._lock:
            self._queue.append(line)

    def _send_worker(self):
        while self._running:
            line = None
            with self._lock:
                if self._queue:
                    line = self._queue.pop(0)

            if line:
                if self.mode == "sim":
                    pass # simulator already handled
                elif self.mode == "relay":
                    self._send_tcp(line + "\n")
                elif self.mode == "serial":
                    self._send_serial(line + "\n")
                time.sleep(0.015) # 15 ms inter-line delay for Uno buffer
            else:
                time.sleep(0.01)

    def _send_tcp(self, data: str):
        try:
            if not self._sock:
                if time.time() < getattr(self, "_retry_at", 0.0):
                    return                      # back off after a failed connect
                host = self.host() if callable(self.host) else self.host
                if not host:
                    return                      # ESP32 not discovered yet
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                try:
                    sock.connect((host, self.tcp_port))
                except OSError:
                    sock.close()
                    self._retry_at = time.time() + 3.0
                    return
                self._sock = sock
                # Re-send what the screen should currently show
                self._sock.sendall(f"STATE|{self.state}\n".encode())
            self._sock.sendall(data.encode('ascii', errors='ignore'))
        except Exception:
            try:
                if self._sock:
                    self._sock.close()
            except Exception:
                pass
            self._sock = None

    def _send_serial(self, data: str):
        """Mode "serial": the Uno is plugged into the laptop (COM4). Only ONE program can
        hold a COM port - if the Arduino IDE Serial Monitor is open on COM4, close it."""
        try:
            if not self._serial:
                if time.time() < getattr(self, "_serial_retry_at", 0.0):
                    return
                import serial
                try:
                    self._serial = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
                except Exception as exc:
                    self._serial_retry_at = time.time() + 5.0
                    if not getattr(self, "_serial_warned", False):
                        self._serial_warned = True
                        print(f"[DisplayClient] cannot open {self.serial_port} ({exc}). "
                              f"Is the Arduino IDE Serial Monitor open on it? Close it, or use mode 'relay'.")
                    return
                self._serial_warned = False
                time.sleep(2.0) # Wait for Uno DTR reset
                self._serial.write(f"STATE|{self.state}\n".encode())
            self._serial.write(data.encode('ascii', errors='ignore'))
        except Exception:
            try:
                if self._serial:
                    self._serial.close()
            except Exception:
                pass
            self._serial = None

    def _heartbeat_worker(self):
        while self._running:
            time.sleep(2.0)
            self._enqueue("PING")

    def close(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
