# Pretend IP Webcam phone for tests: MJPEG at /video, ~30 fps, floor tags 0-3 + car tag 5. Usage: python fake_phone.py PORT [user:pass]
import cv2, numpy as np, time, threading, base64, sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
base = np.full((720, 1280, 3), 255, np.uint8)
for mid, (x, y) in {0: (100, 520), 1: (1060, 520), 2: (1060, 80), 3: (100, 80), 5: (600, 300)}.items():
    t = cv2.aruco.generateImageMarker(D, mid, 120); base[y:y+120, x:x+120] = cv2.cvtColor(t, cv2.COLOR_GRAY2BGR)
counter = {"n": 0}
AUTH = sys.argv[2] if len(sys.argv) > 2 else None
class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if AUTH and self.headers.get("Authorization") != "Basic " + base64.b64encode(AUTH.encode()).decode():
            self.send_response(401); self.send_header("WWW-Authenticate", 'Basic realm="cam"'); self.end_headers(); return
        if self.path != "/video": self.send_error(404); return
        self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
        try:
            while True:
                counter["n"] += 1; img = base.copy(); n = counter["n"]
                img[0:10, 0:160] = 0
                for b in range(16):
                    if (n >> b) & 1: img[0:10, b*10:(b+1)*10] = 255
                ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" + jpg.tobytes() + b"\r\n")
                time.sleep(1/30)
        except (BrokenPipeError, ConnectionResetError): pass
ThreadingHTTPServer(("127.0.0.1", int(sys.argv[1])), H).serve_forever()
