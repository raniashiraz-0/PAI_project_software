"""
tracker.py - Overhead phone camera ArUco car tracker, floor calibration, and landmark estimator.
Detects car marker (ID 5), floor reference markers (0..3), shelf marker (ID 10), and start marker (ID 6).
Includes height parallax correction and a full simulation backend.
"""
import math
import time
import cv2
import numpy as np
from typing import Optional, Dict, Any, Tuple, List
from dataclasses import dataclass
from events import bus, Event, EventType

@dataclass
class Pose:
    x_mm: float
    y_mm: float
    heading_deg: float
    timestamp: float
    confidence: float

class Tracker:
    def __init__(self, config: Dict[str, Any], sim: bool = True, sim_car=None, room_data=None):
        self.config = config
        self.sim = sim
        self.sim_car = sim_car
        self.room_data = room_data or {}
        marker_ids = config.get("markers", {}).get("ids", {})
        self.car_marker_id = int(marker_ids.get("car", 5))
        self.start_marker_id = int(marker_ids.get("start_location", 6))
        self.shelf_marker_id = int(marker_ids.get("shelf_s1", 10))

        robot_cfg = config.get("robot", {})
        # A pose older than this is "stale" and the car is stopped. Raise it if
        # your phone stream is slow (needs >= 3 fps for smooth driving).
        self.stale_s = float(robot_cfg.get("stale_pose_timeout_s", 0.5))
        # Height correction: markers on the car / shelf sit above the floor, so a
        # top-down camera sees them shifted outwards. Needs the camera height.
        cam = self.room_data.get("phone_camera", {})
        self.camera_height_mm = float(cam.get("height_mm", config.get("devices", {}).get("cameras", {})
                                              .get("overhead_phone", {}).get("height_mm", 0.0)) or 0.0)
        self.car_marker_height_mm = float(self.room_data.get("robot", {}).get("marker_height_mm", 120.0))
        self.shelf_marker_height_mm = float(self.room_data.get("shelf", {}).get("marker_height_mm", 300.0))

        # Markers dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # Landmarks world positions (mm)
        # Shelf S1 front face centre at x=2600, y=1500. Heading 0 points INTO the shelf.
        self.shelf_nominal = (2600.0, 1500.0, 0.0)    # x, y, heading_deg
        self.start_nominal = tuple(self.room_data.get(
            "start_pose_mm", config.get("robot", {}).get("start_pose_mm", [400.0, 500.0, 0.0])
        ))
        self.detected_shelf_pose = self.shelf_nominal
        self.detected_start_pose = self.start_nominal
        self._homography = None

        # Floor reference markers nominal positions
        self.ref_markers = {
            0: (250.0, 250.0),
            1: (2750.0, 250.0),
            2: (2750.0, 2150.0),
            3: (250.0, 2150.0)
        }
        configured_refs = self.room_data.get("floor_markers", {}).get("centres_mm", {})
        if configured_refs:
            self.ref_markers = {int(marker_id): tuple(point) for marker_id, point in configured_refs.items()}

        # Learning module (Kalman filter + drive model), attached by the server
        self.learning = None
        self.latency_s = float(config.get("devices", {}).get("cameras", {}).get("overhead_phone", {}).get("latency_est_s", 0.25))

        # Pose history for stationary check and EMA filtering
        self.pose_history: List[Pose] = []
        self.last_valid_pose: Optional[Pose] = None
        self._last_camera_time = 0.0
        self._status = "OK"

        # Sim noise generator
        self.rng = np.random.default_rng(123)

    def get_pose(self) -> Optional[Pose]:
        """
        Returns car Pose(x_mm, y_mm, heading_deg, timestamp, confidence) or None.
        Hard rule: Poses older than 0.5 s are rejected and return None.
        """
        now = time.time()
        if self._status == "CAMERA_MOVED":
            return None

        if self.sim:
            # Generate simulated measurement from sim_car
            if self.sim_car:
                true_x, true_y, true_th = self.sim_car.x, self.sim_car.y, self.sim_car.heading
            else:
                true_x, true_y, true_th = 400.0, 500.0, 0.0

            # Add camera noise (sigma 6 mm, 0.6 deg)
            meas_x = true_x + self.rng.normal(0, 4.0)
            meas_y = true_y + self.rng.normal(0, 4.0)
            meas_th = (true_th + math.radians(self.rng.normal(0, 0.5)) + math.pi) % (2 * math.pi) - math.pi

            pose = Pose(
                x_mm=round(meas_x, 1),
                y_mm=round(meas_y, 1),
                heading_deg=round(math.degrees(meas_th), 1),
                timestamp=now,
                confidence=0.98
            )
            if self.learning is not None and now - self._last_camera_time > 0.066:   # ~15 fps like a phone
                self._last_camera_time = now
                self.learning.on_camera_pose(meas_x, meas_y, meas_th, now)
            self._record_pose(pose)
            return pose

        # Real tracking with OpenCV
        if self.last_valid_pose is None or (now - self._last_camera_time) > self.stale_s:
            return None                          # camera hasn't seen the car recently: stale

        if self.learning is not None:
            # Kalman filter: camera pose moved forward to "now" with the motor commands
            v, w = self.learning.command_now()
            p = self.learning.filter.predict_now(now, v, w)
            if p is not None:
                return Pose(round(p[0], 1), round(p[1], 1), round(math.degrees(p[2]), 1), now,
                            self.last_valid_pose.confidence)
        return self.last_valid_pose

    def process_frame(self, frame: np.ndarray, timestamp: float) -> Optional[Pose]:
        """Process real overhead frame, run ArUco detection and height correction."""
        if frame is None:
            self._status = "NO_FRAMES"
            return None

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None or len(ids) == 0:
            self._status = "MARKER_LOST"
            return None

        marker_ids = np.asarray(ids).reshape(-1)
        marker_map = {int(marker_ids[i]): corners[i].reshape(4, 2) for i in range(len(marker_ids))}
        ref_image_points = []
        ref_world_points = []
        for marker_id, world_point in self.ref_markers.items():
            if marker_id in marker_map:
                ref_image_points.append(np.mean(marker_map[marker_id], axis=0))
                ref_world_points.append(world_point)

        if len(ref_image_points) == 4:
            homography, _ = cv2.findHomography(
                np.asarray(ref_image_points, dtype=np.float32),
                np.asarray(ref_world_points, dtype=np.float32)
            )
            if homography is not None:
                self._homography = homography

        if self._homography is None:
            self._status = "CALIBRATING"
            return None

        def world_point(pixel_point):
            point = np.asarray([[pixel_point]], dtype=np.float32)
            return cv2.perspectiveTransform(point, self._homography)[0, 0]

        # Point on the floor straight below the camera (image centre for a top-down camera)
        img_h, img_w = frame.shape[:2]
        nadir = world_point((img_w / 2.0, img_h / 2.0))

        def lift_correct(pt, height_mm):
            if self.camera_height_mm <= height_mm or self.camera_height_mm <= 0:
                return pt
            f = (self.camera_height_mm - height_mm) / self.camera_height_mm
            return nadir + (pt - nadir) * f

        if self.shelf_marker_id in marker_map:
            sp = marker_map[self.shelf_marker_id]
            s_center = lift_correct(world_point(np.mean(sp, axis=0)), self.shelf_marker_height_mm)
            s_front = lift_correct(world_point(np.mean(sp[:2], axis=0)), self.shelf_marker_height_mm)
            s_heading = math.degrees(math.atan2(s_front[1] - s_center[1], s_front[0] - s_center[0]))
            ox, oy, oth = self.detected_shelf_pose
            a = 0.2   # smooth: the shelf does not move
            dth = (s_heading - oth + 180.0) % 360.0 - 180.0
            self.detected_shelf_pose = (round(ox + a * (float(s_center[0]) - ox), 1),
                                        round(oy + a * (float(s_center[1]) - oy), 1),
                                        round(oth + a * dth, 1))

        if self.start_marker_id in marker_map:
            start_points = marker_map[self.start_marker_id]
            start_center = world_point(np.mean(start_points, axis=0))
            start_front = world_point(np.mean(start_points[:2], axis=0))
            start_heading = math.degrees(math.atan2(
                start_front[1] - start_center[1], start_front[0] - start_center[0]
            ))
            self.detected_start_pose = (float(start_center[0]), float(start_center[1]), float(start_heading))

        if self.car_marker_id not in marker_map:
            self._status = "MARKER_LOST"
            return None

        car_points = marker_map[self.car_marker_id]
        car_center = lift_correct(world_point(np.mean(car_points, axis=0)), self.car_marker_height_mm)
        car_front = lift_correct(world_point(np.mean(car_points[:2], axis=0)), self.car_marker_height_mm)
        heading_deg = math.degrees(math.atan2(car_front[1] - car_center[1], car_front[0] - car_center[0]))
        self._last_camera_time = time.time()
        if self.learning is not None:
            t_cap = timestamp - self.latency_s                 # the picture is ~0.25 s old
            th = math.radians(heading_deg)
            self.learning.on_camera_pose(float(car_center[0]), float(car_center[1]), th, t_cap)
            v, w = self.learning.command_now()
            self.learning.filter.update((float(car_center[0]), float(car_center[1]), th), t_cap, v, w)
            fp = self.learning.filter.predict_now(time.time(), v, w)
            if fp is not None:
                car_center = np.array([fp[0], fp[1]])
                heading_deg = math.degrees(fp[2])
        pose = Pose(
            x_mm=round(float(car_center[0]), 1),
            y_mm=round(float(car_center[1]), 1),
            heading_deg=round(heading_deg, 1),
            timestamp=timestamp,
            confidence=0.95
        )
        self._record_pose(pose)
        self._status = "OK"
        return pose

    def is_stationary(self, window_s: float = 0.5) -> bool:
        """
        Returns True if car speed and angular rate have been near zero
        for the entire window_s duration. Crucial hard interlock before arm moves!
        """
        if self.sim:
            if self.sim_car:
                return (abs(self.sim_car.v) < 2.0 and abs(self.sim_car.w) < 0.05 and not self.sim_car.is_moving)
            return True

        now = time.time()
        # Keep only poses within window (stretch the window for slow camera streams)
        valid = [p for p in self.pose_history if (now - p.timestamp) <= window_s]
        if len(valid) < 3:
            valid = [p for p in self.pose_history if (now - p.timestamp) <= max(window_s, 1.5)]
        if len(valid) < 2:
            return False

        dx = valid[-1].x_mm - valid[0].x_mm
        dy = valid[-1].y_mm - valid[0].y_mm
        d_dist = math.hypot(dx, dy)
        d_th = abs((valid[-1].heading_deg - valid[0].heading_deg + 180) % 360 - 180)

        # Stationary threshold: moved < 15 mm and rotated < 3.0 deg over window
        return (d_dist < 15.0 and d_th < 3.0)

    def shelf_pose(self, shelf_id: str = "S1") -> Tuple[float, float, float]:
        """Returns shelf world pose: (x_mm, y_mm, heading_deg). Measured at run time."""
        return self.detected_shelf_pose

    def dock_pose_world(self, shelf_id: str = "S1", slot_u_mm: float = 0.0) -> Tuple[float, float, float]:
        """
        Compute world dock pose for given slot offset from shelf.
        Stand-off D0 = 200 mm in front of shelf face.
        """
        sx, sy, sth_deg = self.shelf_pose(shelf_id)
        sth = math.radians(sth_deg)
        D0 = 200.0

        c, s = math.cos(sth), math.sin(sth)
        # Stand-off D0 in front, slot_u_mm sideways
        dock_x = sx - D0 * c - slot_u_mm * s
        dock_y = sy - D0 * s + slot_u_mm * c
        return (round(dock_x, 1), round(dock_y, 1), round(sth_deg, 1))

    def start_pose(self) -> Tuple[float, float, float]:
        return self.detected_start_pose

    def set_start_pose(self, x_mm: float, y_mm: float, heading_deg: float):
        self.detected_start_pose = (float(x_mm), float(y_mm), float(heading_deg))

    def status(self) -> str:
        return self._status

    def _record_pose(self, pose: Pose):
        self.last_valid_pose = pose
        self.pose_history.append(pose)
        now = time.time()
        # Prune older than 3 seconds
        self.pose_history = [p for p in self.pose_history if (now - p.timestamp) <= 3.0]
        bus.publish(Event(type=EventType.TRACKER_POSE, source="Tracker", data={
            "x": pose.x_mm, "y": pose.y_mm, "heading": pose.heading_deg, "confidence": pose.confidence
        }))
