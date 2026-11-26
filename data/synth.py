"""synth.py - makes fake camera pictures so vision.py can be tested without any hardware."""
import cv2, numpy as np
from vision import DICT, square_object_points

def marker_image(mid, px=160, pad=40):
    m = cv2.aruco.generateImageMarker(DICT, mid, px)
    return cv2.copyMakeBorder(m, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255), px, pad

def look_at(cam_pos, target, up=(0, 0, 1.0)):
    """Camera pose (rvec, tvec) for a camera at cam_pos looking at target (OpenCV: x right, y down, z forward)."""
    z = np.array(target, float) - cam_pos; z /= np.linalg.norm(z)
    x = np.cross(z, up); x /= np.linalg.norm(x); y = np.cross(z, x)
    R = np.vstack([x, y, z]); t = -R @ cam_pos
    return cv2.Rodrigues(R)[0], t.reshape(3, 1)

def blit_marker(canvas, K, rvec, tvec, mid, centre_xyz, size_mm, heading, normal="up"):
    """Draw marker `mid` (size_mm) into the picture. normal='up': lying flat, its 'up' edge pointing along `heading` (radians)."""
    img, px, pad = marker_image(mid)
    f = (px + 2 * pad) / px; s = size_mm * f / 2.0
    local = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])                   # TL TR BR BL in marker frame (y up)
    c, sn = np.cos(heading), np.sin(heading)
    # flat marker: marker y axis (up) -> room direction (cos h, sin h); marker x axis (right) -> (sin h, -cos h)
    R = np.array([[sn, c, 0], [-c, sn, 0], [0, 0, 1.0]])
    world = (R @ local.T).T + np.array(centre_xyz)
    pts, _ = cv2.projectPoints(world, rvec, tvec, K, None); pts = pts.reshape(4, 2).astype(np.float32)
    src = np.array([[0, 0], [img.shape[1], 0], [img.shape[1], img.shape[0]], [0, img.shape[0]]], np.float32)
    M = cv2.getPerspectiveTransform(src, pts)
    warped = cv2.warpPerspective(img, M, (canvas.shape[1], canvas.shape[0]))
    mask = cv2.warpPerspective(np.full(img.shape, 255, np.uint8), M, (canvas.shape[1], canvas.shape[0]))
    canvas[mask > 0] = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)[mask > 0]

def floor_texture(room=(3000, 2400), ppm=0.5, boxes=(), floor_markers=None, marker_mm=200, seed=0):
    rng = np.random.default_rng(seed); W, H = int(room[0] * ppm), int(room[1] * ppm)
    tex = np.full((H, W, 3), 150, np.uint8) + rng.integers(-12, 12, (H, W, 3)).astype(np.uint8)
    tex = cv2.GaussianBlur(tex, (3, 3), 0)
    for x in range(0, room[0], 500): cv2.line(tex, (int(x * ppm), 0), (int(x * ppm), H), (125, 125, 125), 2)
    for y in range(0, room[1], 500): cv2.line(tex, (0, int(y * ppm)), (W, int(y * ppm)), (125, 125, 125), 2)
    for mid, (cx, cy) in (floor_markers or {}).items():
        img, px, pad = marker_image(int(mid), px=int(marker_mm * ppm), pad=int(marker_mm * ppm * 0.25))
        half = img.shape[0] / ppm / 2.0                         # half-size of the padded picture in mm
        x0, y0 = int(round((cx - half) * ppm)), int(round((room[1] - cy - half) * ppm))
        tex[y0:y0 + img.shape[0], x0:x0 + img.shape[1]] = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    colours = [(30, 40, 200), (200, 80, 30), (40, 160, 40), (20, 150, 200)]
    for i, (x0, y0, x1, y1) in enumerate(boxes):
        cv2.rectangle(tex, (int(x0 * ppm), int((room[1] - y1) * ppm)), (int(x1 * ppm), int((room[1] - y0) * ppm)), colours[i % 4], -1)
    return tex

def render_room(K, rvec, tvec, tex, room=(3000, 2400), ppm=0.5, size=(1920, 1080)):
    """What the phone sees: the floor texture seen through a pinhole camera (ray / plane intersection for every pixel)."""
    W, H = size; R = cv2.Rodrigues(rvec)[0]; C = -R.T @ tvec.reshape(3)
    u, v = np.meshgrid(np.arange(W), np.arange(H)); rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1) @ R
    with np.errstate(divide="ignore", invalid="ignore"):
        s = -C[2] / rays[..., 2]
    X = C[0] + s * rays[..., 0]; Y = C[1] + s * rays[..., 1]
    mapx = (X * ppm).astype(np.float32); mapy = ((room[1] - Y) * ppm).astype(np.float32)
    bad = (rays[..., 2] >= 0) | ~np.isfinite(mapx)
    mapx[bad] = -1; mapy[bad] = -1
    return cv2.remap(tex, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

def render_hand_marker(K, T_cam_marker, mid, size_mm=30.0, size=(640, 480), noise=3.0, seed=0):
    """What the ESP32-CAM sees: one package marker at the given pose, with sensor noise and blur."""
    rng = np.random.default_rng(seed)
    canvas = np.full((size[1], size[0], 3), 120, np.uint8)
    img, px, pad = marker_image(mid); f = (px + 2 * pad) / px
    obj = square_object_points(size_mm * f)
    R = T_cam_marker[:3, :3]; t = T_cam_marker[:3, 3]
    pts, _ = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], t.reshape(3, 1), K, None); pts = pts.reshape(4, 2).astype(np.float32)
    src = np.array([[0, 0], [img.shape[1], 0], [img.shape[1], img.shape[0]], [0, img.shape[0]]], np.float32)
    M = cv2.getPerspectiveTransform(src, pts)
    canvas = cv2.warpPerspective(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), M, size, dst=canvas, borderMode=cv2.BORDER_TRANSPARENT)
    canvas = cv2.GaussianBlur(canvas, (3, 3), 0)
    return np.clip(canvas.astype(np.float32) + rng.normal(0, noise, canvas.shape), 0, 255).astype(np.uint8)
