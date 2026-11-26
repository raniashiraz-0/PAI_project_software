"""
streamer.py - Real-time MJPEG visual rendering for the webserver.
Renders:
  1. Top-down overhead room map with car, shelf, start, path and obstacles
  2. Hand camera view with detection overlays and verification verdict
  3. 480x320 TFT display preview
"""
import math
import cv2
import numpy as np

def render_room_map(grid, car_pose, shelf_pose, start_pose, path=None, trace=None, width=640, height=512) -> np.ndarray:
    """
    Renders top-down 2D floor view of 3000x2400 mm room.
    Returns BGR image.
    """
    canvas = np.full((height, width, 3), (25, 28, 35), dtype=np.uint8)

    # Scale factors: 3000 mm -> width, 2400 mm -> height
    scale_x = width / 3000.0
    scale_y = height / 2400.0

    def to_px(x_mm, y_mm):
        px = int(x_mm * scale_x)
        # Flip Y so (0,0) is bottom-left
        py = int(height - (y_mm * scale_y))
        return px, py

    # 1. Draw grid lines (every 500 mm)
    for x in range(0, 3001, 500):
        p1 = to_px(x, 0)
        p2 = to_px(x, 2400)
        cv2.line(canvas, p1, p2, (40, 45, 55), 1)
    for y in range(0, 2401, 500):
        p1 = to_px(0, y)
        p2 = to_px(3000, y)
        cv2.line(canvas, p1, p2, (40, 45, 55), 1)

    # 2. Draw static shelf
    sx, sy, sth_deg = shelf_pose
    spx, spy = to_px(sx, sy)
    # Shelf box
    s_w = int(200 * scale_x)
    s_h = int(600 * scale_y)
    cv2.rectangle(canvas, (spx - s_w // 2, spy - s_h // 2), (spx + s_w // 2, spy + s_h // 2), (70, 70, 100), -1)
    cv2.rectangle(canvas, (spx - s_w // 2, spy - s_h // 2), (spx + s_w // 2, spy + s_h // 2), (180, 180, 240), 2)
    cv2.putText(canvas, "SHELF S1", (spx - 35, spy - s_h // 2 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 255), 1)

    # 3. Draw start location marker
    st_x, st_y, _ = start_pose
    st_px, st_py = to_px(st_x, st_y)
    cv2.circle(canvas, (st_px, st_py), 16, (0, 160, 220), 2)
    cv2.putText(canvas, "START", (st_px - 22, st_py + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 180, 255), 1)

    # 4. Draw planned route
    if path and len(path) > 1:
        pts = [to_px(p[0], p[1]) for p in path]
        for i in range(len(pts) - 1):
            cv2.line(canvas, pts[i], pts[i + 1], (255, 180, 0), 2, cv2.LINE_AA)
            cv2.circle(canvas, pts[i], 3, (255, 220, 100), -1)

    # 5. Draw car driven trace
    if trace and len(trace) > 1:
        t_pts = [to_px(p[0], p[1]) for p in trace[-100:]]
        for i in range(len(t_pts) - 1):
            cv2.line(canvas, t_pts[i], t_pts[i + 1], (50, 180, 80), 2)

    # 6. Draw car body and heading arrow
    if car_pose:
        cx, cy, ch_deg = car_pose
        c_px, c_py = to_px(cx, cy)
        # Car body
        car_r = int(75 * scale_x) # 150mm diameter
        cv2.circle(canvas, (c_px, c_py), car_r, (0, 220, 120), -1)
        cv2.circle(canvas, (c_px, c_py), car_r, (255, 255, 255), 2)

        # Heading vector (heading 0 deg = +x, positive = counterclockwise)
        rad = math.radians(ch_deg)
        head_len = int(120 * scale_x)
        hx = int(c_px + head_len * math.cos(rad))
        hy = int(c_py - head_len * math.sin(rad))
        cv2.arrowedLine(canvas, (c_px, c_py), (hx, hy), (0, 0, 255), 3, tipLength=0.3)
        cv2.putText(canvas, "CAR (ID 5)", (c_px - 30, c_py - car_r - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 150), 1)

    # Header stats overlay
    cv2.putText(canvas, "OVERHEAD PHONE TRACKER (3000 x 2400 mm)", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 230), 1)
    if car_pose:
        cv2.putText(canvas, f"Pose: ({car_pose[0]:.0f}, {car_pose[1]:.0f}) mm  Heading: {car_pose[2]:.1f} deg",
                    (15, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 240, 255), 1)

    return canvas

def render_hand_camera_frame(raw_frame: np.ndarray, verdict_data=None) -> np.ndarray:
    """Renders hand camera image with vision telemetry overlay."""
    if raw_frame is None:
        frame = np.full((360, 480, 3), (20, 20, 25), dtype=np.uint8)
        cv2.putText(frame, "HAND CAMERA (ESP32-CAM) WAITING", (40, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 160), 1)
        return frame

    frame = raw_frame.copy()
    h, w = frame.shape[:2]

    # Overlay banner at top
    cv2.rectangle(frame, (0, 0), (w, 40), (15, 15, 20), -1)
    cv2.putText(frame, "ESP32-CAM GRIPPER VIEW", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 1)

    # Verdict banner if available
    if verdict_data:
        res = verdict_data.get("result", "UNCERTAIN")
        color = (0, 200, 0) if res == "MATCH" else ((0, 0, 220) if res == "NO_MATCH" else (0, 180, 240))
        cv2.rectangle(frame, (w - 150, 5), (w - 10, 35), color, -1)
        cv2.putText(frame, res, (w - 140, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Display details at bottom
        reason = verdict_data.get("reason", "")
        if reason:
            cv2.rectangle(frame, (0, h - 30), (w, h), (15, 15, 20), -1)
            cv2.putText(frame, reason[:65], (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    return frame
