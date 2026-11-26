"""
orchestrator.py - Mission Orchestrator and Arbiter for Pharma Rover.
Enforces the hard safety interlocks, manages mission tokens, coordinates car, arm,
vision, display and voice.

FETCH mission (e.g. "fetch paracetamol"):
  1. arm to HOME (checked)            2. A* route to the shelf, driven with Pure Pursuit
     using the phone camera pose; the 4 ultrasonic sensors add obstacles to the map
     and the route is re-planned (shortest path again) every time something blocks it
  3. dock in front of the shelf centre 4. arm to SEARCH pose, then turn the arm to look
     at each of the 3 slots - the ESP32-CAM + YOLO (or ArUco/barcode) says which box is
     the requested medicine          5. grab-gate authorise, IK pre-grasp -> grasp ->
     close gripper -> lift -> carry   6. GRIP CHECK: is the box really in the claws?
     (grip sensor if fitted, otherwise the hand camera) - if not, open and try again
  7. drive back to the start (same avoidance), watching that the box is not dropped
  8. check the box is still held, open gripper, arm HOME, celebrate on the TFT.
"""
import os
import json
import time
import math
import uuid
import threading
from typing import Optional, Dict, Any, List, Tuple
from events import bus, Event, EventType, MissionState, DisplayState
from intents import Intent
import numpy as np
import planner
import navigator
import kin3d as K3

HOME_Q = [0.0, 110.0, -145.0, 60.0, 0.0]
SEARCH_Q = [0.0, 55.6, -118.0, 62.3, 0.0]
CARRY_Q = [0.0, 90.0, -120.0, 30.0, 0.0]

IDLE_STATES = (MissionState.IDLE, MissionState.STOPPED, MissionState.SUCCESS, MissionState.ERROR)


class MissionToken:
    def __init__(self, mission_id: str, medicine_id: int):
        self.mission_id = mission_id
        self.medicine_id = medicine_id
        self.cancelled = False
        self.start_time = time.time()

    def cancel(self):
        self.cancelled = True


class MissionCancelled(Exception):
    pass


class Orchestrator:
    def __init__(self, car_client, arm_client, tracker, verifier, display_client, audio_in, audio_out,
                 room_config: Dict[str, Any], config: Optional[Dict[str, Any]] = None, medicines: Optional[List[Dict[str, Any]]] = None,
                 learning=None):
        self.car = car_client
        self.arm = arm_client
        self.tracker = tracker
        self.verifier = verifier
        self.display = display_client
        self.audio_in = audio_in
        self.audio_out = audio_out
        self.room_config = room_config
        self.config = config or {}
        self.medicines = medicines or []

        rc = self.config.get("robot", {})
        nav = self.config.get("navigation", {})
        vis = self.config.get("vision", {})
        self.v_max = float(rc.get("v_max_mm_s", 250.0))
        self.v_carry = float(nav.get("carry_speed_mm_s", 160.0))
        self.hard_clear = float(rc.get("hard_clearance_mm", 165.0))
        self.soft_clear = float(rc.get("soft_clearance_mm", 300.0))
        self.car_half_len = float(nav.get("car_half_length_mm", 110.0))
        self.car_half_width = float(nav.get("car_half_width_mm", 90.0))
        self.front_avoid_cm = float(nav.get("front_avoid_cm", 25.0))
        self.side_avoid_cm = float(nav.get("side_avoid_cm", 12.0))
        self.obstacle_size_mm = float(nav.get("obstacle_size_mm", 200.0))
        self.obstacle_ttl_s = float(nav.get("obstacle_memory_s", 90.0))
        self.max_replans = int(nav.get("max_replans", 12))
        self.dock_front_stop_mm = int(nav.get("dock_front_stop_mm", 50))
        self.normal_front_stop_mm = int(nav.get("front_stop_mm", 150))
        self.normal_back_stop_mm = int(nav.get("back_stop_mm", 120))
        self.pose_wait_s = float(nav.get("pose_lost_wait_s", 2.0))
        self.scan_look_x = float(vis.get("scan_look_distance_mm", 250.0))
        self.scan_retries = int(vis.get("scan_retries_per_slot", 1))
        self.hand_led = int(vis.get("hand_led_duty", 0))
        self.arm_cfg = self.config.get("arm", {})
        gcfg = self.config.get("grip_check", {})
        self.grip_retries = int(gcfg.get("retries", 1))
        self.grip_unsure_ok = str(gcfg.get("when_unsure", "continue")).lower() == "continue"
        self.medicines_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "medicines.json")
        self.medicines_data: Optional[Dict[str, Any]] = None     # set by server.py (written back by teach mode)
        self.last_grip: Dict[str, Any] = {}

        # Learning (Kalman filter, drive regression, grasp memory) - optional
        self.learning = learning
        sv = self.config.get("visual_servo", {})
        self.servo_enabled = bool(sv.get("enabled", True))
        self.servo_iters = int(sv.get("max_iterations", 3))
        self.servo_tol = float(sv.get("tolerance", 0.04))
        self.servo_gain = float(sv.get("gain", 0.8))
        self.servo_aim = tuple(sv.get("aim_point", [0.5, 0.5]))
        self.servo_dist = float(sv.get("camera_to_box_mm", 100.0))
        self.servo_hfov = math.radians(float(sv.get("camera_hfov_deg", 66.0)))
        self.servo_vfov = math.radians(float(sv.get("camera_vfov_deg", 50.0)))
        self.servo_flip_x = -1.0 if sv.get("flip_x", False) else 1.0
        self.servo_flip_y = -1.0 if sv.get("flip_y", False) else 1.0
        self.last_servo: Dict[str, Any] = {}
        self._mission_replans = 0
        self._mission_grip_attempts = 0

        self.state = MissionState.IDLE
        self.active_mission: Optional[MissionToken] = None
        self._lock = threading.Lock()
        self.carried_box: Optional[Dict[str, Any]] = None
        self.last_scan: Dict[str, Any] = {}
        self.dynamic_obstacles: List[Tuple[float, float, float]] = []   # (x, y, time)
        self._last_obstacle_speech = 0.0

        # Build room navigation grid
        self.grid = planner.GridMap(
            width_mm=room_config.get("room_mm", [3000, 2400])[0],
            height_mm=room_config.get("room_mm", [3000, 2400])[1],
            cell_mm=room_config.get("cell_mm", 20.0)
        )
        self._setup_room_grid()

        self._mission_thread: Optional[threading.Thread] = None
        bus.subscribe(EventType.STOP_TRIGGERED, lambda e: self.emergency_stop())

    # ======================================================================
    #  Map
    # ======================================================================
    def _setup_room_grid(self, keep_free=None):
        """Static shelf + remembered obstacles (from the ultrasonic sensors), then inflate."""
        sx, sy, sth = self.tracker.shelf_pose("S1")
        self.grid.occ.fill(False)
        # Shelf body behind its front face (works for any shelf heading)
        c, s = math.cos(math.radians(sth)), math.sin(math.radians(sth))
        corners = []
        for depth in (0.0, 400.0):
            for side in (-450.0, 450.0):
                corners.append((sx + depth * c - side * s, sy + depth * s + side * c))
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        self.grid.add_box(min(xs), min(ys), max(xs), max(ys))

        now = time.time()
        self.dynamic_obstacles = [o for o in self.dynamic_obstacles if now - o[2] < self.obstacle_ttl_s]
        h = self.obstacle_size_mm / 2.0
        for ox, oy, _ in self.dynamic_obstacles:
            self.grid.add_box(ox - h, oy - h, ox + h, oy + h)
        self.grid.inflate(hard_mm=self.hard_clear, soft_mm=self.soft_clear, keep_free=keep_free)

    def _add_obstacle(self, pose, sensor: str, dist_cm: float):
        """Put an obstacle on the map where the given ultrasonic sensor saw it."""
        th = math.radians(pose.heading_deg)
        if sensor == "front":
            off, ang = self.car_half_len, 0.0
        elif sensor == "back":
            off, ang = self.car_half_len, math.pi
        elif sensor == "left":
            off, ang = self.car_half_width, math.pi / 2
        else:
            off, ang = self.car_half_width, -math.pi / 2
        r = off + dist_cm * 10.0 + self.obstacle_size_mm / 2.0
        ox = pose.x_mm + r * math.cos(th + ang)
        oy = pose.y_mm + r * math.sin(th + ang)
        self.dynamic_obstacles.append((ox, oy, time.time()))
        return ox, oy

    def clear_obstacles(self):
        self.dynamic_obstacles.clear()
        self._setup_room_grid()

    # ======================================================================
    #  Hard interlocks
    # ======================================================================
    def can_move_arm(self) -> Tuple[bool, str]:
        """Arm moves ONLY when the car is stationary (camera says so, or - if the camera has
        no fresh pose - the ESP32 reports the wheels stopped for >= 0.5 s)."""
        car_stat = self.car.status()
        if car_stat.get("is_moving", False):
            return False, "Refused: Car is currently driving!"
        if self.tracker.get_pose() is not None:
            if not self.tracker.is_stationary(window_s=0.5):
                return False, "Refused: Car is not stationary (velocity > 0 within last 0.5s)!"
        elif self.car.idle_for() < 0.5:
            return False, "Refused: Car stopped less than 0.5 s ago."
        return True, "OK"

    def can_drive_car(self) -> Tuple[bool, str]:
        """Car drives ONLY when the arm is folded at HOME or in the CARRY pose."""
        arm_stat = self.arm.get_status()
        if not arm_stat.get("connected", False):
            return False, "Refused: Arm controller is disconnected; its travel pose cannot be confirmed."
        if arm_stat.get("is_moving", False):
            return False, "Refused: Arm is currently in motion!"
        if arm_stat.get("grab_authorized", False):
            return False, "Refused: Grab authorization is active!"

        q = arm_stat.get("q_cmd", [0, 0, 0, 0, 0])
        is_home = (abs(q[1] - 110.0) < 5.0 and abs(q[2] - (-145.0)) < 5.0)
        is_carry = (self.carried_box is not None and abs(q[1] - 90.0) < 15.0)
        if not (is_home or is_carry):
            return False, f"Refused: Arm is extended at non-travel pose {q[:4]}!"
        return True, "OK"

    def emergency_stop(self):
        """STOP beats everything."""
        print("[Orchestrator] EMERGENCY STOP TRIGGERED!")
        with self._lock:
            self.car.stop()
            self.arm.abort()
            if self.active_mission:
                self.active_mission.cancel()
                self.active_mission = None
            self.state = MissionState.STOPPED
            self._set_led(0)
            self.display.set_state("error")
            self.audio_out.speak("Stopping immediately.", non_blocking=True)
            if self.carried_box:
                print(f"[Orchestrator] Car was carrying {self.carried_box.get('name')}. Left in safe grasp.")

    # ======================================================================
    #  Intents
    # ======================================================================
    def dispatch_intent(self, intent: Intent) -> Dict[str, Any]:
        if intent.name == "STOP":
            self.emergency_stop()
            return {"ok": True, "message": "Stopped"}

        if self.state not in IDLE_STATES:
            msg = "I'm busy with a mission. Say stop first."
            self.audio_out.speak(msg)
            return {"ok": False, "message": msg}

        if intent.name == "FETCH":
            med = intent.params.get("medicine")
            if not med:
                return {"ok": False, "error": "No medicine specified"}
            full = next((m for m in self.medicines if m.get("id") == med.get("id")), None)
            return self.start_fetch_mission(full or med)

        if intent.name == "DRIVE_STEP":
            return self.execute_manual_drive_step(intent.params.get("dir", "fwd"), intent.params.get("dist_mm", 200.0))

        if intent.name == "TURN":
            deg = intent.params.get("deg", 45.0)
            sign = 1.0 if intent.params.get("dir", "left") == "left" else -1.0
            return self.execute_manual_turn(deg * sign)

        if intent.name == "GO_START":
            return self.start_return_to_start()

        if intent.name == "ARM_HOME":
            ok, msg = self.can_move_arm()
            if not ok:
                return {"ok": False, "error": msg}
            return {"ok": self.arm.home(), "error": getattr(self.arm, "last_error", "")}

        if intent.name == "GRIPPER_OPEN":
            ok = self.arm.open_gripper()
            return {"ok": ok, "error": "" if ok else getattr(self.arm, "last_error", "")}

        if intent.name == "GRIPPER_CLOSE":
            return self.arm.close_gripper()

        if intent.name == "JOG":
            return self._jog(intent.params)

        return {"ok": False, "error": f"Unhandled intent {intent.name}"}

    def _jog(self, params: Dict[str, Any]) -> Dict[str, Any]:
        ok, msg = self.can_move_arm()
        if not ok:
            return {"ok": False, "error": msg}
        q = list(self.arm.q_cmd)
        if params.get("axis") == "yaw":
            q[0] = max(-90.0, min(90.0, q[0] + float(params.get("step_deg", 10.0))))
            return self.arm.move_joints(q)
        tip = K3.tip_xyz(q[:4])
        tip[2] += float(params.get("step_mm", 20.0))
        q_new = K3.ik_xyz(tuple(tip), phi_deg=q[1] + q[2] + q[3])
        if q_new is None:
            return {"ok": False, "error": "Arm cannot reach that point"}
        return self.arm.move_joints(q_new + [q[4]])

    # ======================================================================
    #  Manual driving (voice "move forward" / dashboard D-pad)
    # ======================================================================
    def _manual_fail(self, default_msg: str) -> Dict[str, Any]:
        err = getattr(self.car, "last_error", "") or ""
        if "obstacle" in err:
            message = "Obstacle in the way, I can't move there."
        elif self.car.status().get("connected", False) is False or "disconnected" in err or not err:
            message = default_msg
        else:
            message = f"The base refused: {err}."
        self.audio_out.speak(message)
        return {"ok": False, "error": message}

    def execute_manual_drive_step(self, direction: str, dist_mm: float) -> Dict[str, Any]:
        """Single drive step. Works without the phone camera (open-loop on the ESP32);
        the ESP32 still stops by itself if an ultrasonic sensor sees something close."""
        ok, msg = self.can_drive_car()
        if not ok:
            self.audio_out.speak(msg)
            return {"ok": False, "error": msg}
        send_mm = self.learning.drive.step_distance(dist_mm) if self.learning is not None else dist_mm
        sent = self.car.drive_forward(send_mm) if direction == "fwd" else self.car.drive_backward(send_mm)
        if not sent:
            return self._manual_fail("Car controller is disconnected; drive command was not sent.")
        message = f"Drove {direction} {dist_mm} mm"
        self.audio_out.speak(message)
        return {"ok": True, "message": message}

    def execute_manual_turn(self, deg: float) -> Dict[str, Any]:
        ok, msg = self.can_drive_car()
        if not ok:
            self.audio_out.speak(msg)
            return {"ok": False, "error": msg}
        send_deg = self.learning.drive.step_angle(deg) if self.learning is not None else deg
        if not self.car.turn(send_deg):
            return self._manual_fail("Car controller is disconnected; turn command was not sent.")
        message = f"Turned {deg} degrees"
        self.audio_out.speak(message)
        return {"ok": True, "message": message}

    # ======================================================================
    #  Helpers used by missions
    # ======================================================================
    def _check(self, token: MissionToken):
        if token.cancelled:
            raise MissionCancelled()

    def _sleep(self, token: MissionToken, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            self._check(token)
            time.sleep(0.02)

    def _wait_pose(self, token: MissionToken, max_wait_s: float):
        pose = self.tracker.get_pose()
        t0 = time.time()
        while pose is None and time.time() - t0 < max_wait_s:
            self._check(token)
            time.sleep(0.05)
            pose = self.tracker.get_pose()
        return pose

    def _arm_move(self, token: MissionToken, q: List[float], what: str, payload_g: float = 0.0):
        self._check(token)
        ok, msg = self.can_move_arm()
        if not ok:
            raise RuntimeError(f"Arm {what}: {msg}")
        res = self.arm.move_joints(q, payload_g=payload_g)
        if not res.get("ok", False):
            raise RuntimeError(f"Arm {what} failed: {res.get('error', 'unknown error')}")
        self._check(token)

    def _drive(self, v: float, w: float):
        """Send a velocity command, corrected by the learned drive model."""
        if self.learning is not None:
            v, w = self.learning.correct_command(v, w)
        self.car.drive_velocity(v, int(w * 1000))

    def _set_led(self, duty: int):
        if hasattr(self.verifier, "led_duty"):
            self.verifier.led_duty = int(duty)

    def _speak_obstacle(self):
        if time.time() - self._last_obstacle_speech > 4.0:
            self._last_obstacle_speech = time.time()
            self.audio_out.speak("Obstacle detected. Finding a new route.", non_blocking=True)

    def _drive_route(self, token: MissionToken, goal: Tuple[float, float], v_max: float, label: str,
                     keep_free=None, stop_radius_mm: float = 50.0, ignore_near_goal_mm: float = 150.0,
                     final_lookahead_point=None, watch=None) -> None:
        """Drive to goal along the shortest A* path, avoiding new obstacles.

        Front sensor < front_avoid_cm while driving forward, or the ESP32's own
        obstacle stop -> stop, mark the obstacle on the map, re-plan.
        Side sensors < side_avoid_cm -> mark on the map; re-plan only if the
        current route now runs through it.
        """
        pose = self._wait_pose(token, self.pose_wait_s)
        if pose is None:
            raise RuntimeError("The phone camera cannot see the car (no pose)")

        def plan(from_pose):
            kf = list(keep_free or []) + [(from_pose.x_mm, from_pose.y_mm, 80.0), (goal[0], goal[1], 80.0)]
            self._setup_room_grid(keep_free=kf)
            raw = planner.astar(self.grid, (from_pose.x_mm, from_pose.y_mm), goal)
            if not raw:
                return None
            route = planner.smooth_path(self.grid, raw)
            if final_lookahead_point is not None:
                route = route + [tuple(final_lookahead_point)]
            return route

        route = plan(pose)
        if not route:
            raise RuntimeError(f"No route to {label}: path blocked")
        pp = navigator.PurePursuit(route, lookahead=200.0, v_max=v_max)
        length = planner.path_length(route)
        deadline = time.time() + max(30.0, 3.0 * length / max(50.0, v_max) + 15.0)
        dt = 0.02 if self.car.sim else 0.08
        replans = 0
        self.car.clear_obstacle_flag()

        while True:
            self._check(token)
            if watch is not None:
                problem = watch()
                if problem:
                    self.car.stop()
                    raise RuntimeError(problem)
            pose = self.tracker.get_pose()
            if pose is None:
                self.car.stop()
                pose = self._wait_pose(token, self.pose_wait_s)
                if pose is None:
                    raise RuntimeError(f"Tracking lost while driving to {label}")

            d_goal = math.hypot(pose.x_mm - goal[0], pose.y_mm - goal[1])
            if d_goal < stop_radius_mm:
                break
            if time.time() > deadline:
                raise RuntimeError(f"Timed out driving to {label}")

            ok, message = self.can_drive_car()
            if not ok:
                raise RuntimeError(message)

            # ---------------- obstacle check (4 ultrasonic sensors) ----------------
            st = self.car.status()
            dist = st.get("distances_cm", {}) or {}
            front = float(st.get("front_obstacle_cm", dist.get("front", 999.0)))
            blocked_sensor = None
            if d_goal > ignore_near_goal_mm:
                if st.get("obstacle_side") in ("FRONT", "BACK"):
                    blocked_sensor = st["obstacle_side"].lower()
                elif front < self.front_avoid_cm:
                    blocked_sensor = "front"
                else:
                    for side in ("left", "right"):
                        if float(dist.get(side, 999.0)) < self.side_avoid_cm:
                            self._add_obstacle(pose, side, float(dist[side]))
                            self._setup_room_grid(keep_free=[(pose.x_mm, pose.y_mm, 80.0), (goal[0], goal[1], 80.0)])
                            if navigator.path_blocked_ahead(self.grid, route, (pose.x_mm, pose.y_mm, math.radians(pose.heading_deg))):
                                blocked_sensor = side
                            break

            if blocked_sensor:
                self.car.stop()
                replans += 1
                self._mission_replans += 1
                if replans > self.max_replans:
                    raise RuntimeError(f"Too many obstacles on the way to {label}")
                d_cm = front if blocked_sensor == "front" else float(dist.get(blocked_sensor, 15.0))
                ox, oy = self._add_obstacle(pose, blocked_sensor, min(d_cm, 60.0))
                print(f"[Orchestrator] obstacle ({blocked_sensor}, {d_cm:.0f} cm) at ({ox:.0f},{oy:.0f}) - re-planning #{replans}")
                bus.publish(Event(type=EventType.CAR_OBSTACLE, source="Orchestrator",
                                  data={"side": blocked_sensor.upper(), "dist_cm": d_cm, "replan": replans}))
                self.display.set_state("obstacle")
                self._speak_obstacle()
                self._sleep(token, 0.5)
                pose = self._wait_pose(token, self.pose_wait_s) or pose
                new_route = plan(pose)
                if not new_route:
                    # Maybe a person walked past: wait a moment and forget the newest obstacle
                    self._sleep(token, 2.0)
                    if self.dynamic_obstacles:
                        self.dynamic_obstacles.pop()
                    new_route = plan(pose)
                    if not new_route:
                        raise RuntimeError(f"Route to {label} is blocked")
                route = new_route
                pp = navigator.PurePursuit(route, lookahead=200.0, v_max=v_max)
                self.car.clear_obstacle_flag()
                self.display.set_state("alert")
                continue

            v, w, _ = pp.command((pose.x_mm, pose.y_mm, math.radians(pose.heading_deg)))
            self._drive(v, w)
            time.sleep(dt)

        self.car.stop()

    def _dock(self, token: MissionToken, dock_world: Tuple[float, float, float]):
        """Slow final alignment in front of the shelf. The front sensor sees the shelf
        here, so its automatic stop distance is reduced while docking."""
        self.car.set_obstacle_thresholds(self.dock_front_stop_mm, self.normal_back_stop_mm)
        try:
            dock_rad = (dock_world[0], dock_world[1], math.radians(dock_world[2]))
            dt = 0.02 if self.car.sim else 0.08
            t0 = time.time()
            while time.time() - t0 < 25.0:
                self._check(token)
                p = self.tracker.get_pose()
                if not p:
                    self.car.stop()
                    p = self._wait_pose(token, self.pose_wait_s)
                    if not p:
                        raise RuntimeError("Tracking lost while docking")
                v, w, _al, _lat, _eth, done = navigator.dock_command((p.x_mm, p.y_mm, math.radians(p.heading_deg)), dock_rad)
                if done:
                    break
                self._drive(v, w)
                time.sleep(dt)
            self.car.stop()
        finally:
            self.car.set_obstacle_thresholds(self.normal_front_stop_mm, self.normal_back_stop_mm)

        t0 = time.time()
        while time.time() - t0 < 3.0:
            self._check(token)
            if self.can_move_arm()[0]:
                return
            time.sleep(0.1)
        raise RuntimeError("Car failed to become stationary after docking")

    def _slot_list(self) -> List[float]:
        slots = sorted({float(m.get("slot_u_mm", 0.0)) for m in self.medicines}) or [100.0, 0.0, -100.0]
        return slots

    def _scan_shelf(self, token: MissionToken, med: Dict[str, Any]) -> Tuple[float, Dict[str, Any]]:
        """Point the hand camera at each slot until the requested box is found.
        Returns (slot_u_mm, medicine_found)."""
        expected_slot = float(med.get("slot_u_mm", 0.0))
        if self.learning is not None:
            slots = self.learning.memory.scan_order(med["id"], expected_slot, self._slot_list())
        else:
            slots = sorted(self._slot_list(), key=lambda u: abs(u - expected_slot))
        seen: Dict[float, Any] = {}
        self._set_led(self.hand_led)
        try:
            self._arm_move(token, SEARCH_Q, "search pose")
            for i, slot in enumerate(slots):
                yaw = math.degrees(math.atan2(slot, self.scan_look_x))
                q = list(SEARCH_Q)
                q[0] = yaw
                self.display.show_medicine(med, status=f"SCAN SLOT {i + 1}/{len(slots)}", progress=55 + 3 * i)
                self._arm_move(token, q, f"look at slot {slot:+.0f} mm")
                self._sleep(token, 0.3)              # let the picture settle
                for attempt in range(1 + self.scan_retries):
                    verdict = self.verifier.verify(med["id"], min_frame_time=time.time(), sim_slot_u=slot)
                    print(f"[Orchestrator] slot {slot:+.0f}: {verdict.result} ({verdict.reason})")
                    if verdict.result == "MATCH":
                        seen[slot] = med["id"]
                        if self.learning is not None:
                            self.learning.remember_location(med["id"], slot)
                        self.last_scan = {"slots": seen, "found_slot": slot, "medicine": med["name"]}
                        bus.publish(Event(type=EventType.VERIFICATION_RESULT, source="Orchestrator",
                                          data={"slot_u_mm": slot, "result": "MATCH", "medicine": med["name"]}))
                        return slot, med
                    if verdict.result == "NO_MATCH":
                        seen[slot] = verdict.detected_medicine_id
                        if self.learning is not None and verdict.detected_medicine_id:
                            self.learning.remember_location(verdict.detected_medicine_id, slot)
                        break
            self.last_scan = {"slots": seen, "found_slot": None, "medicine": med["name"]}
            raise RuntimeError(f"Could not find {med['name']} on the shelf")
        finally:
            self._set_led(0)

    def _grasp_joint_targets(self, med: Dict[str, Any], slot_u: float, dy: float = 0.0, dz: float = 0.0) -> Dict[str, List[float]]:
        """IK for pre-grasp / grasp / lift at the slot where the box was found."""
        gx, _gy, gz = med.get("grasp_point_arm_frame_mm", [250.0, 0.0, -12.0])
        slot_u, gz = slot_u + dy, gz + dz                       # learned / visual-servo correction
        pre_d = float(self.arm_cfg.get("pre_grasp_distance_mm", 40.0))
        lift_h = float(self.arm_cfg.get("lift_height_mm", 20.0))
        phi = float(med.get("grasp_phi_deg", 0.0))          # gripper tilt saved by teach mode
        pts = {"pre_grasp": (gx - pre_d, slot_u, gz), "grasp": (gx, slot_u, gz), "lift": (gx, slot_u, gz + lift_h)}
        presets = med.get("joint_angles_deg", {})
        out = {}
        for name, p in pts.items():
            q = K3.ik_xyz(p, phi_deg=phi)
            if q is None and abs(slot_u) < 1.0 and name in presets:
                q = list(presets[name])
            if q is None:
                raise RuntimeError(f"Arm cannot reach the {name} point {tuple(round(v) for v in p)}")
            out[name] = list(q[:4]) + [0.0]
        return out

    # ======================================================================
    #  Grip check - does the gripper REALLY hold the box?
    # ======================================================================
    def _check_grip(self, med: Dict[str, Any], reference, after_time: float, stage: str) -> Dict[str, Any]:
        """Grip sensor (if fitted on the ESP32) decides; otherwise the hand camera."""
        st = self.arm.get_status()
        if st.get("grip_sensor_present"):
            held = bool(st.get("grip_sensor_box"))
            res = {"held": held, "confidence": 0.99, "method": "grip sensor",
                   "reason": "sensor feels the box" if held else "sensor: claws are empty"}
        else:
            res = self.verifier.check_grip(med["id"], reference_frame=reference, after_time=after_time,
                                           sim_held=getattr(self.arm, "sim_box_held", None) if self.arm.sim else None)
        res = dict(res, stage=stage, medicine=med.get("name"), time=round(time.time(), 1))
        self.last_grip = res
        print(f"[Orchestrator] grip check ({stage}): held={res['held']} via {res['method']} - {res['reason']}")
        bus.publish(Event(type=EventType.ARM_GRIP_DONE, source="Orchestrator", data=res))
        return res

    def _box_drop_watch(self) -> Optional[str]:
        """Called while driving home. Only the grip sensor is fast enough to watch continuously."""
        st = self.arm.get_status()
        if st.get("grip_sensor_present") and not st.get("grip_sensor_box"):
            name = self.carried_box.get("name", "box") if self.carried_box else "box"
            return f"I dropped the {name} on the way back"
        return None

    def _grasp_with_check(self, token: MissionToken, med: Dict[str, Any], found: Dict[str, Any], slot_u: float):
        """Approach, grip, lift, carry and verify the grip. Retries if the claws came up empty.
        Returns a hand-camera picture taken in the CARRY pose (reference for later checks)."""
        # Start from what worked last time at this slot (learned), then fine-tune with the camera
        dy, dz = self.learning.memory.offset_for(slot_u) if self.learning is not None else (0.0, 0.0)
        targets = self._grasp_joint_targets(found, slot_u, dy, dz)
        weight = float(med.get("weight_g", 45))
        for attempt in range(1 + self.grip_retries):
            self._mission_grip_attempts += 1
            self._check(token)
            if attempt > 0:
                self.audio_out.speak("I missed the box. Trying again.", non_blocking=True)
                self.display.set_state("thinking")
                # look again: is the box still in that slot (maybe it was pushed)?
                q = list(SEARCH_Q)
                q[0] = math.degrees(math.atan2(slot_u, self.scan_look_x))
                self._arm_move(token, q, "look again")
                self._sleep(token, 0.3)
                v = self.verifier.verify(med["id"], min_frame_time=time.time(), sim_slot_u=slot_u)
                if v.result != "MATCH":
                    raise RuntimeError(f"I could not grip the {med['name']}: it is no longer in its slot")
                self.display.set_state("happy")

            self.state = MissionState.ARM_VERIFY
            if not self.arm.authorise_grab(token.mission_id, med["id"]):
                raise RuntimeError(f"Grab authorisation failed: {getattr(self.arm, 'last_error', '')}")

            self.state = MissionState.ARM_APPROACH
            self.display.show_medicine(med, status="APPROACHING" if attempt == 0 else "RETRY GRIP", progress=75)
            if not self.arm.open_gripper():
                raise RuntimeError(f"Gripper did not open: {getattr(self.arm, 'last_error', '')}")
            self._arm_move(token, targets["pre_grasp"], "pre-grasp")
            dy, dz, aligned = self._visual_align(token, med, found, slot_u, dy, dz)
            targets = self._grasp_joint_targets(found, slot_u, dy, dz)
            self._arm_move(token, targets["grasp"], "grasp")
            self._sleep(token, 0.3)
            t_ref = time.time()
            grasp_ref = self.verifier.latest_frame(newer_than=t_ref - 0.05) if not self.verifier.sim else None

            self.state = MissionState.ARM_GRIP
            grip = self.arm.close_gripper(mission_id=token.mission_id, medicine_id=med["id"])
            if not grip.get("ok", False):
                raise RuntimeError(f"Gripper close failed: {grip.get('error')}")
            self.carried_box = med

            self.state = MissionState.ARM_LIFT
            self._arm_move(token, targets["lift"], "lift", payload_g=weight)
            self._arm_move(token, CARRY_Q, "carry pose", payload_g=weight)
            self.arm.deauthorise_grab()

            self._sleep(token, 0.4)
            check = self._check_grip(med, grasp_ref, time.time(), stage=f"after lift (try {attempt + 1})")
            if check["held"] or (check["held"] is None and self.grip_unsure_ok):
                if check["held"] is None:
                    print("[Orchestrator] grip check unsure - continuing (grip_check.when_unsure = continue)")
                if check["held"] and aligned and self.learning is not None:
                    self.learning.learn_grasp(slot_u, dy, dz)     # remember the camera correction that worked
                return self.verifier.latest_frame(newer_than=time.time() - 0.05) if not self.verifier.sim else None

            # Empty claws: open, forget the box, try again
            self.arm.open_gripper()
            self.carried_box = None
        raise RuntimeError(f"I could not grip the {med['name']}")

    def _visual_align(self, token: MissionToken, med: Dict[str, Any], found: Dict[str, Any], slot_u: float,
                      dy: float, dz: float) -> Tuple[float, float, bool]:
        """Eye-in-hand visual servoing at the pre-grasp pose: move the claws until the box sits
        at the aim point of the hand-camera picture. Returns the new (dy, dz) correction in mm."""
        if not self.servo_enabled:
            return dy, dz, False
        if self.verifier.sim and "locate_target" not in vars(self.verifier):
            self.last_servo = {"aligned": None, "reason": "skipped (simulation - no camera)", "dy": round(dy, 1), "dz": round(dz, 1)}
            return dy, dz, False
        w_mm = 2.0 * self.servo_dist * math.tan(self.servo_hfov / 2.0)   # picture width at the box
        h_mm = 2.0 * self.servo_dist * math.tan(self.servo_vfov / 2.0)
        history = []
        for i in range(self.servo_iters):
            self._check(token)
            loc = self.verifier.locate_target(med["id"], newer_than=time.time())
            if loc is None:
                self.last_servo = {"aligned": False, "reason": "box not visible to the hand camera",
                                   "dy": round(dy, 1), "dz": round(dz, 1), "steps": history}
                return dy, dz, False
            ex = (loc["ex"] - self.servo_aim[0]) * self.servo_flip_x
            ey = (loc["ey"] - self.servo_aim[1]) * self.servo_flip_y
            history.append({"ex": round(ex, 3), "ey": round(ey, 3), "source": loc.get("source")})
            if abs(ex) < self.servo_tol and abs(ey) < self.servo_tol:
                self.last_servo = {"aligned": True, "iterations": i, "dy": round(dy, 1), "dz": round(dz, 1), "steps": history}
                return dy, dz, True
            # box right of the aim point -> it is at -y (arm y points left); box low -> move down
            ddy = float(np.clip(-ex * w_mm * self.servo_gain, -15.0, 15.0))
            ddz = float(np.clip(-ey * h_mm * self.servo_gain, -15.0, 15.0))
            dy = float(np.clip(dy + ddy, -40.0, 40.0))
            dz = float(np.clip(dz + ddz, -40.0, 40.0))
            try:
                t = self._grasp_joint_targets(found, slot_u, dy, dz)
            except RuntimeError:
                break
            self._arm_move(token, t["pre_grasp"], "visual alignment")
            self._sleep(token, 0.25)
        self.last_servo = {"aligned": False, "reason": "not centred after max iterations",
                           "dy": round(dy, 1), "dz": round(dz, 1), "steps": history}
        return dy, dz, False

    # ======================================================================
    #  Teach mode - show the arm where the box is, save it
    # ======================================================================
    def arm_tip(self) -> Dict[str, Any]:
        q = (self.arm.reported_q() if hasattr(self.arm, "reported_q") else None) or list(self.arm.q_cmd)
        tip = K3.tip_xyz(q[:4])
        return {"q": [round(v, 1) for v in q], "x": round(float(tip[0]), 1), "y": round(float(tip[1]), 1),
                "z": round(float(tip[2]), 1), "phi": round(q[1] + q[2] + q[3], 1)}

    def nudge_arm(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, dyaw: float = 0.0) -> Dict[str, Any]:
        """Move the gripper tip a few mm (keeps its tilt). Used to line it up with a box."""
        ok, msg = self.can_move_arm()
        if not ok:
            return {"ok": False, "error": msg}
        q = list(self.arm.q_cmd)
        if dyaw:
            q[0] = max(-90.0, min(90.0, q[0] + dyaw))
            return self.arm.move_joints(q)
        tip = K3.tip_xyz(q[:4])
        target = (float(tip[0]) + dx, float(tip[1]) + dy, float(tip[2]) + dz)
        q_new = K3.ik_xyz(target, phi_deg=q[1] + q[2] + q[3])
        if q_new is None:
            return {"ok": False, "error": "The arm cannot reach that point"}
        return self.arm.move_joints(q_new + [q[4]])

    def teach_grasp(self, medicine_id: int, update_slot: bool = True) -> Dict[str, Any]:
        """Save the CURRENT gripper position as the grasp point of a medicine.
        Dock the car at the shelf centre first, then move the open gripper around the box."""
        med = next((m for m in self.medicines if m.get("id") == medicine_id), None)
        if med is None:
            return {"ok": False, "error": f"Unknown medicine {medicine_id}"}
        if self.arm.get_status().get("is_moving"):
            return {"ok": False, "error": "Wait until the arm stops"}
        tip = self.arm_tip()
        med["grasp_point_arm_frame_mm"] = [tip["x"], 0.0, tip["z"]]
        med["grasp_phi_deg"] = tip["phi"]
        if update_slot:
            med["slot_u_mm"] = round(tip["y"], 1)
        med["taught_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        if self.learning is not None:                      # new teach point -> old learned correction no longer applies
            self.learning.memory.offsets.pop(self.learning.memory._key(float(med.get("slot_u_mm", 0.0))), None)
            self.learning.save(force=True)
        try:
            targets = self._grasp_joint_targets(med, float(med.get("slot_u_mm", 0.0)))
        except RuntimeError as exc:
            return {"ok": False, "error": f"Saved point is not reachable for pre-grasp/lift: {exc}"}
        saved = self._save_medicines()
        return {"ok": True, "saved_to_file": saved, "medicine": med["name"], "grasp_point_mm": med["grasp_point_arm_frame_mm"],
                "slot_u_mm": med.get("slot_u_mm"), "phi_deg": med["grasp_phi_deg"],
                "targets": {k: [round(v, 1) for v in q] for k, q in targets.items()}}

    def test_grasp(self, medicine_id: int, stage: str = "pre_grasp") -> Dict[str, Any]:
        """Move the arm to the saved pre-grasp / grasp / lift pose of a medicine (gripper not touched)."""
        med = next((m for m in self.medicines if m.get("id") == medicine_id), None)
        if med is None:
            return {"ok": False, "error": f"Unknown medicine {medicine_id}"}
        ok, msg = self.can_move_arm()
        if not ok:
            return {"ok": False, "error": msg}
        try:
            targets = self._grasp_joint_targets(med, float(med.get("slot_u_mm", 0.0)))
        except RuntimeError as exc:
            return {"ok": False, "error": str(exc)}
        if stage not in targets:
            return {"ok": False, "error": f"stage must be one of {list(targets)}"}
        if stage == "grasp":                       # always come in through pre-grasp
            r = self.arm.move_joints(targets["pre_grasp"])
            if not r.get("ok"):
                return r
        return self.arm.move_joints(targets[stage])

    def _save_medicines(self) -> bool:
        if self.medicines_data is None:
            return False
        try:
            tmp = self.medicines_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self.medicines_data, f, indent=2)
            os.replace(tmp, self.medicines_path)
            return True
        except OSError as exc:
            print(f"[Orchestrator] could not save medicines.json: {exc}")
            return False

    # ======================================================================
    #  Return to start
    # ======================================================================
    def start_return_to_start(self) -> Dict[str, Any]:
        with self._lock:
            if self.state not in IDLE_STATES:
                message = "I'm busy with a mission. Say stop first."
                self.audio_out.speak(message)
                return {"ok": False, "message": message}
            pose = self.tracker.get_pose()
            if not pose:
                return {"ok": False, "error": "Tracker has no valid pose"}
            ok, message = self.can_drive_car()
            if not ok:
                self.audio_out.speak(message)
                return {"ok": False, "error": message}
            target = self.tracker.start_pose()
            if math.hypot(pose.x_mm - target[0], pose.y_mm - target[1]) < 50.0:
                self.audio_out.speak("Already at the start location.")
                return {"ok": True, "message": "Already at the start location."}
            token = MissionToken(str(uuid.uuid4())[:8], 0)
            self.active_mission = token
            self.state = MissionState.PLANNING

        self.display.set_state("thinking")
        self.audio_out.speak("Returning to the start location.")
        self._mission_thread = threading.Thread(target=self._run_return_to_start, args=(token,), daemon=True)
        self._mission_thread.start()
        return {"ok": True, "mission_id": token.mission_id, "message": "Returning to the start location."}

    def _run_return_to_start(self, token: MissionToken):
        try:
            target = self.tracker.start_pose()
            self.state = MissionState.NAVIGATING_RETURN
            self.display.set_state("alert")
            self._drive_route(token, target[:2], v_max=180.0, label="the start location", stop_radius_mm=30.0)
            self._check(token)
            self.state = MissionState.IDLE
            self.active_mission = None
            self.display.set_state("idle")
            self.audio_out.speak("Returned to the start location.")
        except MissionCancelled:
            self.car.stop()
        except Exception as exc:
            self.car.stop()
            if token.cancelled:
                return
            print(f"[Orchestrator] Return to start failed: {exc}")
            self.state = MissionState.ERROR
            self.active_mission = None
            self.display.set_state("error")
            self.audio_out.speak("Could not return to the start location.")

    # ======================================================================
    #  FETCH mission
    # ======================================================================
    def start_fetch_mission(self, medicine: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            if self.state not in IDLE_STATES:
                message = "I'm busy with a mission. Say stop first."
                self.audio_out.speak(message)
                return {"ok": False, "message": message}
            mission_id = str(uuid.uuid4())[:8]
            token = MissionToken(mission_id, medicine["id"])
            self.active_mission = token
            self.state = MissionState.PLANNING

        self.display.set_state("thinking")
        self.audio_out.speak(f"Okay, fetching {medicine['name']}.")
        self._mission_thread = threading.Thread(target=self._run_fetch_mission, args=(token, medicine), daemon=True)
        self._mission_thread.start()
        return {"ok": True, "mission_id": mission_id, "medicine": medicine["name"]}

    @staticmethod
    def _spoken_reason(reason: str) -> str:
        """Say WHY a mission stopped, in short plain words."""
        r = reason.lower()
        rules = [
            ("cannot see the car", "Mission stopped. I can't see the car. Check the phone camera, the four floor tags, and tag five on the car."),
            ("tracking lost", "Mission stopped. I lost sight of the car. Keep the car and the floor tags in the phone camera's view."),
            ("disconnected", "Mission stopped. The robot's E S P 32 is not connected. Check it is powered and on the same hotspot."),
            ("pca9685", "Mission stopped. The arm's servo driver is not answering. Check the servo battery and the I 2 C wires."),
            ("did not reach the target", "Mission stopped. An arm joint did not reach its position."),
            ("no route", "Mission stopped. I can't find a path to the shelf. Something is blocking the way."),
            ("route to", "Mission stopped. The way is blocked."),
            ("too many obstacles", "Mission stopped. Too many obstacles in the way."),
            ("timed out", "Mission stopped. It took too long to get there."),
            ("not stationary", "Mission stopped. The car did not stay still for the arm."),
        ]
        for key, text in rules:
            if key in r:
                return text
        if reason.startswith(("Could not find", "I dropped", "I could not grip")):
            return reason
        return f"Mission stopped. {reason[:110]}"

    def _log_mission(self, token: MissionToken, med: Dict[str, Any], success: bool, error: str = ""):
        if self.learning is None:
            return
        self.learning.log_mission({
            "time": time.strftime("%Y-%m-%d %H:%M:%S"), "medicine": med.get("name"), "success": success,
            "duration_s": round(time.time() - token.start_time, 1), "error": error,
            "replans": self._mission_replans, "grip_attempts": self._mission_grip_attempts,
            "found_slot": self.last_scan.get("found_slot"), "visual_servo": self.last_servo.get("aligned"),
        })

    def _run_fetch_mission(self, token: MissionToken, med: Dict[str, Any]):
        self._mission_replans = 0
        self._mission_grip_attempts = 0
        self.last_servo = {}
        bus.publish(Event(type=EventType.MISSION_STARTED, source="Orchestrator", data={"mission_id": token.mission_id, "medicine": med.get("name")}))
        try:
            # 1. Arm to safe travel pose (HOME), checked by telemetry
            self.display.set_state("alert")
            self.display.show_medicine(med, status="ARM_HOME", progress=10)
            if self.carried_box is None:
                self._arm_move(token, HOME_Q, "home")

            # 2. Where are we? (phone camera)
            p_start = self._wait_pose(token, self.pose_wait_s)
            if not p_start:
                raise RuntimeError("The phone camera cannot see the car at the start")
            start_xy = (p_start.x_mm, p_start.y_mm)

            # 3. Dock in front of the shelf CENTRE; the arm scans all 3 slots from there
            dock_world = self.tracker.dock_pose_world(med.get("shelf_id", "S1"), 0.0)
            th = math.radians(dock_world[2])
            pre_dock = (dock_world[0] - 300.0 * math.cos(th), dock_world[1] - 300.0 * math.sin(th))
            keep_free = [(dock_world[0], dock_world[1], 120.0), (pre_dock[0], pre_dock[1], 80.0)]

            # 4. Shortest route to the shelf, with obstacle avoidance + re-planning
            self.state = MissionState.NAVIGATING
            self.display.show_medicine(med, status="NAVIGATING", progress=30)
            self._drive_route(token, pre_dock, v_max=self.v_max, label="the shelf", keep_free=keep_free,
                              stop_radius_mm=60.0, final_lookahead_point=dock_world[:2])

            # 5. Fine docking
            self.state = MissionState.DOCKING
            self.display.show_medicine(med, status="DOCKING", progress=45)
            self._dock(token, dock_world)

            # 6. Find the right box (YOLO / ArUco / barcode on the ESP32-CAM picture)
            self.state = MissionState.ARM_SEARCH
            self.display.set_state("scanning")
            slot_u, found = self._scan_shelf(token, med)

            # 7-9. Grab gate, approach, grip, lift, carry - then CHECK the box is really held
            self.display.set_state("happy")
            self.display.show_medicine(med, status="MATCH_OK", progress=65)
            carry_ref = self._grasp_with_check(token, med, found, slot_u)
            self.state = MissionState.ARM_CARRY

            # 10. Back to the start (shortest route again, same avoidance), watching the box
            self.state = MissionState.NAVIGATING_RETURN
            self.display.set_state("alert")
            self.display.show_medicine(med, status="RETURNING", progress=85)
            self._sleep(token, 0.3)
            self._drive_route(token, start_xy, v_max=self.v_carry, label="the start location", stop_radius_mm=50.0,
                              watch=self._box_drop_watch)

            # 11. Make sure we still have it, then deliver
            self.state = MissionState.ARM_DELIVER
            self.display.show_medicine(med, status="DELIVERING", progress=95)
            self._sleep(token, 0.5)
            final = self._check_grip(med, carry_ref, time.time(), stage="before delivery")
            if final.get("held") is False:
                self.arm.open_gripper()
                self.carried_box = None
                self._arm_move(token, HOME_Q, "home")
                raise RuntimeError(f"I dropped the {med['name']} on the way back")
            if not self.arm.open_gripper():
                raise RuntimeError(f"Gripper did not open: {getattr(self.arm, 'last_error', '')}")
            self.carried_box = None
            self._arm_move(token, HOME_Q, "home")

            self.state = MissionState.SUCCESS
            self.display.celebrate()
            self.audio_out.speak(f"Here is your {med['name']}.")
            bus.publish(Event(type=EventType.MISSION_COMPLETED, source="Orchestrator", data={"mission_id": token.mission_id}))
            self._log_mission(token, med, True)
            time.sleep(2.5)
            if self.state == MissionState.SUCCESS:
                self.state = MissionState.IDLE
            self.active_mission = None

        except MissionCancelled:
            self.car.stop()
            self._log_mission(token, med, False, "stopped by user")
        except Exception as e:
            if token.cancelled:
                return
            print(f"[Orchestrator] Mission Failed: {e}")
            self.state = MissionState.ERROR
            self.car.stop()
            self.arm.abort()
            self._set_led(0)
            self.display.set_state("error")
            reason = str(e)
            self.audio_out.speak(self._spoken_reason(reason))
            bus.publish(Event(type=EventType.MISSION_ERROR, source="Orchestrator", data={"mission_id": token.mission_id, "error": reason}))
            self._log_mission(token, med, False, reason)
            self.active_mission = None
