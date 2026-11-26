"""
camera.py - Threaded camera frame grabber for overhead phone camera and hand ESP32-CAM.
Always keeps only the LATEST frame (stale frames are worse than dropped frames).
Supports IP streams (MJPEG/RTSP), webcams, recorded frames, and HTTP POST endpoint.
"""
import time
import threading
import cv2
import numpy as np
from typing import Optional, Tuple, Dict, Any

class CameraGrabber:
    def __init__(self, name: str, source: Any = 0, fps_limit: int = 15, latency_est_s: float = 0.25):
        self.name = name
        self.source = source
        self.fps_limit = fps_limit
        self.latency_est_s = latency_est_s

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_timestamp: float = 0.0
        self._frame_count: int = 0
        self._fps: float = 0.0
        self._last_fps_calc = time.time()
        self._fps_counter = 0

        self.status = "INITIALIZING" # "CONNECTED", "RECONNECTING", "NO_FRAMES"
        self._running = True
        self._cap: Optional[cv2.VideoCapture] = None

        # Start background ingest thread if source is a physical hardware stream
        if source not in ("HTTP_POST", "sim", "none", "disabled"):
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()
        else:
            self.status = "WAITING_FOR_POST" if source == "HTTP_POST" else "SIMULATED"

    def push_frame(self, frame_bytes: bytes) -> bool:
        """Push a raw JPEG frame received via HTTP POST (e.g. from ESP32-CAM)."""
        try:
            arr = np.frombuffer(frame_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is not None:
                with self._lock:
                    self._latest_frame = frame
                    self._latest_timestamp = time.time()
                    self._frame_count += 1
                    self._fps_counter += 1
                    self.status = "CONNECTED"
                return True
        except Exception as e:
            print(f"[CameraGrabber {self.name}] Error decoding POST frame: {e}")
        return False

    def push_image(self, img: np.ndarray):
        """Push an in-memory BGR image (used by synthetic simulators)."""
        with self._lock:
            self._latest_frame = img.copy()
            self._latest_timestamp = time.time()
            self._frame_count += 1
            self._fps_counter += 1
            self.status = "CONNECTED"

    def get_latest_frame(self) -> Tuple[Optional[np.ndarray], float]:
        """Returns (frame, server_receive_timestamp)."""
        with self._lock:
            if self._latest_frame is None:
                return None, 0.0
            return self._latest_frame.copy(), self._latest_timestamp

    def get_fps(self) -> float:
        now = time.time()
        dt = now - self._last_fps_calc
        if dt >= 1.0:
            self._fps = self._fps_counter / dt
            self._fps_counter = 0
            self._last_fps_calc = now
        return round(self._fps, 1)

    def _open(self, src):
        """Open a webcam index or a network stream. Network streams get a 5 s timeout so an
        unreachable phone doesn't freeze the thread for half a minute."""
        if isinstance(src, str) and src.lower().startswith(("http://", "https://", "rtsp://")):
            try:
                cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG,
                                       [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000])
            except Exception:
                cap = cv2.VideoCapture(src)
        else:
            cap = cv2.VideoCapture(src)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # keep only the newest frame where supported
        except Exception:
            pass
        return cap

    def _capture_loop(self):
        backoff = 1.0
        fails = 0
        while self._running:
            try:
                if self._cap is None or not self._cap.isOpened():
                    self.status = "RECONNECTING"
                    src = int(self.source) if str(self.source).isdigit() else self.source
                    self._cap = self._open(src)
                    if not self._cap.isOpened():
                        time.sleep(backoff)
                        backoff = min(5.0, backoff * 1.5)
                        continue
                    backoff = 1.0
                    fails = 0
                    self.status = "CONNECTED"

                # Read as fast as the camera sends - never sleep here. Sleeping lets a
                # network stream queue up, and the picture we use gets older and older.
                ret, frame = self._cap.read()
                if ret and frame is not None:
                    fails = 0
                    with self._lock:
                        self._latest_frame = frame
                        self._latest_timestamp = time.time()
                        self._frame_count += 1
                        self._fps_counter += 1
                        self.status = "CONNECTED"
                else:
                    fails += 1
                    self.status = "NO_FRAMES"
                    if fails >= 20:                   # ~2 s without pictures: reconnect
                        self._cap.release()
                        self._cap = None
                        fails = 0
                    time.sleep(0.1)

            except Exception:
                self.status = "ERROR"
                try:
                    if self._cap is not None:
                        self._cap.release()
                except Exception:
                    pass
                self._cap = None
                time.sleep(1.0)

    def close(self):
        self._running = False
        if self._cap is not None:
            self._cap.release()
            self._cap = None
