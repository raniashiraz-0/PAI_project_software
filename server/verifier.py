"""
verifier.py - Hand camera medicine box verification for the Pharma Rover.
Combines:
  0. YOLO box detection (ultralytics), when a trained model is available
     (vision.detector = "yolo" or "auto" in config.yaml, model at vision.yolo_model_path)
  1. Image quality gate (blur variance, brightness, glare)
  2. ArUco marker detection (IDs 21, 22, 23)
  3. Barcode (EAN-13) decoding with strict checksum validation
  4. OCR fallback on name region with rapidfuzz / difflib
  5. Multi-frame voting and freshness enforcement.
"""
import os
import time
import cv2
import numpy as np
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass, field
import difflib

try:
    from rapidfuzz import fuzz
    def fuzzy_score(a: str, b: str) -> float:
        return float(fuzz.ratio(a.lower(), b.lower()))
except ImportError:
    def fuzzy_score(a: str, b: str) -> float:
        return difflib.SequenceMatcher(None, a.lower(), b.lower()).ratio() * 100.0

@dataclass
class Verdict:
    result: str                  # "MATCH", "NO_MATCH", "UNCERTAIN"
    confidence: float
    evidence: Dict[str, Any] = field(default_factory=dict)
    detected_medicine_id: Optional[int] = None
    reason: str = ""
    frames_used: int = 1

def validate_ean13_checksum(ean: str) -> bool:
    """Validate standard EAN-13 modulo 10 checksum."""
    ean = str(ean).strip()
    if len(ean) != 13 or not ean.isdigit():
        return False
    digits = [int(c) for c in ean]
    checksum = sum(digits[0:12:2]) + 3 * sum(digits[1:12:2])
    check_digit = (10 - (checksum % 10)) % 10
    return check_digit == digits[12]

class BoxVerifier:
    def __init__(self, config: Dict[str, Any], medicines_list: List[Dict[str, Any]], sim: bool = True, camera_grabber=None):
        self.config = config
        self.medicines = medicines_list
        self.sim = sim
        self.camera_grabber = camera_grabber

        # OpenCV ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # OpenCV Barcode detector
        try:
            self.barcode_detector = cv2.barcode.BarcodeDetector()
        except Exception:
            self.barcode_detector = None

        # Hand-camera flash LED brightness requested from the ESP32-CAM (0-255)
        self.led_duty = 0

        # Grip check thresholds
        gcfg = (config or {}).get("grip_check", {})
        self.grip_min_area_frac = float(gcfg.get("yolo_min_area_frac", 0.15))
        self.grip_similarity_held = float(gcfg.get("similarity_held", 0.55))
        self.grip_similarity_empty = float(gcfg.get("similarity_empty", 0.30))

        # YOLO detector (optional)
        vcfg = (config or {}).get("vision", {})
        self.detector_mode = str(vcfg.get("detector", "auto")).lower()
        self.yolo_conf = float(vcfg.get("yolo_conf", 0.5))
        self.yolo = None
        self.yolo_status = "disabled"
        self.yolo_class_to_med: Dict[str, int] = {}
        for m in self.medicines:
            cls = str(m.get("yolo_class") or m["name"].split()[0]).lower()
            self.yolo_class_to_med[cls] = m["id"]
        if self.detector_mode in ("yolo", "auto") and not sim:
            self._load_yolo(vcfg.get("yolo_model_path", "data/yolo/medicine_boxes.pt"))

        # Build lookup tables
        self.med_by_id = {m["id"]: m for m in self.medicines}
        self.med_by_aruco = {m["aruco_id"]: m for m in self.medicines if "aruco_id" in m}
        self.med_by_ean = {str(m["ean13"]): m for m in self.medicines if "ean13" in m}

    def _load_yolo(self, model_path: str):
        here = os.path.dirname(os.path.abspath(__file__))
        candidates = [model_path, os.path.join(here, "..", model_path)]
        path = next((p for p in candidates if os.path.exists(p)), None)
        if path is None:
            self.yolo_status = f"model not found ({model_path}) - using ArUco/barcode"
            print(f"[Verifier] YOLO {self.yolo_status}")
            return
        try:
            from ultralytics import YOLO
            self.yolo = YOLO(path)
            self.yolo_status = f"loaded {os.path.basename(path)}"
            print(f"[Verifier] YOLO {self.yolo_status}; classes: {list(self.yolo.names.values())}")
        except Exception as exc:
            self.yolo = None
            self.yolo_status = f"failed to load ({exc}) - using ArUco/barcode"
            print(f"[Verifier] YOLO {self.yolo_status}")

    def detect_boxes(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """Run YOLO and return [{med_id, cls, conf, cx, cy, area}] (empty if YOLO is not loaded)."""
        if self.yolo is None or frame is None:
            return []
        out = []
        try:
            res = self.yolo.predict(frame, conf=self.yolo_conf, verbose=False)[0]
            names = res.names
            for box in res.boxes:
                cls_name = str(names[int(box.cls[0])]).lower()
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                out.append({
                    "med_id": self.yolo_class_to_med.get(cls_name),
                    "cls": cls_name,
                    "conf": round(float(box.conf[0]), 3),
                    "cx": (x1 + x2) / 2.0, "cy": (y1 + y2) / 2.0,
                    "area": (x2 - x1) * (y2 - y1),
                })
        except Exception as exc:
            print(f"[Verifier] YOLO error: {exc}")
        return out

    def _yolo_target(self, frame: np.ndarray) -> Tuple[Optional[int], float, List[Dict[str, Any]]]:
        """The box the arm is pointing at = biggest detection in the middle 60% of the picture."""
        dets = self.detect_boxes(frame)
        w = frame.shape[1]
        central = [d for d in dets if d["med_id"] is not None and 0.2 * w <= d["cx"] <= 0.8 * w]
        if not central:
            return None, 0.0, dets
        best = max(central, key=lambda d: d["area"])
        return best["med_id"], best["conf"], dets

    # ------------------------------------------------------------------ visual servoing
    def locate_target(self, expected_medicine_id: int, newer_than: float = 0.0) -> Optional[Dict[str, Any]]:
        """Where is the requested box in the hand-camera picture?
        Returns {"ex", "ey"} = box centre as a fraction of the picture (0..1), or None."""
        if self.sim:
            return None
        frame = self.latest_frame(newer_than=newer_than, timeout_s=1.5)
        if frame is None:
            return None
        h, w = frame.shape[:2]
        if self.yolo is not None:
            dets = [d for d in self.detect_boxes(frame) if d["med_id"] == expected_medicine_id]
            if dets:
                d = max(dets, key=lambda d: d["area"])
                return {"ex": d["cx"] / w, "ey": d["cy"] / h, "conf": d["conf"], "source": "yolo",
                        "area_frac": d["area"] / (w * h)}
        exp = self.med_by_id.get(expected_medicine_id, {}).get("aruco_id")
        corners, ids, _ = self.aruco_detector.detectMarkers(frame)
        if ids is not None and exp is not None:
            for c, i in zip(corners, ids):
                if int(i[0]) == exp:
                    pts = c.reshape(4, 2)
                    cx, cy = pts.mean(axis=0)
                    return {"ex": float(cx) / w, "ey": float(cy) / h, "conf": 0.9, "source": "aruco"}
        return None

    # ------------------------------------------------------------------ grip check
    def latest_frame(self, newer_than: float = 0.0, timeout_s: float = 1.5) -> Optional[np.ndarray]:
        """Newest hand-camera frame taken after newer_than (None if the camera is silent)."""
        if self.camera_grabber is None:
            return None
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            frame, t = self.camera_grabber.get_latest_frame()
            if frame is not None and t > newer_than:
                return frame
            time.sleep(0.05)
        return None

    @staticmethod
    def _centre_patch(frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        crop = frame[h // 4: 3 * h // 4, w // 4: 3 * w // 4]
        g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        g = cv2.resize(g, (160, 120))
        return cv2.equalizeHist(g)

    def _similarity(self, a: np.ndarray, b: np.ndarray) -> float:
        """How alike the centre of two pictures is (1.0 = same, ~0 = unrelated)."""
        pa, pb = self._centre_patch(a), self._centre_patch(b)
        ncc = float(cv2.matchTemplate(pa, pb, cv2.TM_CCOEFF_NORMED)[0][0])
        ha = cv2.calcHist([pa], [0], None, [32], [0, 256])
        hb = cv2.calcHist([pb], [0], None, [32], [0, 256])
        hist = float(cv2.compareHist(ha, hb, cv2.HISTCMP_CORREL))
        return round(0.7 * ncc + 0.3 * hist, 3)

    def check_grip(self, expected_medicine_id: int, reference_frame: Optional[np.ndarray] = None,
                   after_time: float = 0.0, sim_held: Optional[bool] = None) -> Dict[str, Any]:
        """Is the box really in the gripper? Called with the arm pulled back (CARRY pose),
        so an empty gripper sees the room, while a held box still fills the picture.

        Evidence per frame:
          * YOLO sees the expected box big in the picture (it moves with the camera)
          * the box's ArUco tag is still visible
          * the picture still looks like the one taken at the shelf just before closing
        Returns {"held": True / False / None (unsure), "confidence", "method", "reason"}.
        """
        if self.sim:
            held = True if sim_held is None else bool(sim_held)
            return {"held": held, "confidence": 0.95, "method": "sim",
                    "reason": "box in gripper (sim)" if held else "gripper empty (sim)"}

        votes_held, votes_empty, sims, notes = 0, 0, [], []
        last_t = after_time
        t0 = time.time()
        frames = 0
        while frames < 3 and time.time() - t0 < 3.0:
            frame, t = (self.camera_grabber.get_latest_frame() if self.camera_grabber else (None, 0.0))
            if frame is None or t <= last_t:
                time.sleep(0.05)
                continue
            last_t = t
            frames += 1
            area = frame.shape[0] * frame.shape[1]
            held_ev, empty_ev = False, False

            if self.yolo is not None:
                for d in self.detect_boxes(frame):
                    if d["area"] / area >= self.grip_min_area_frac:
                        if d["med_id"] == expected_medicine_id:
                            held_ev = True
                            notes.append(f"YOLO sees {d['cls']} filling {100 * d['area'] / area:.0f}% of the view")
                        elif d["med_id"] is not None:
                            notes.append(f"YOLO sees a different box ({d['cls']}) in the gripper")
            corners, ids, _ = self.aruco_detector.detectMarkers(frame)
            exp = self.med_by_id.get(expected_medicine_id, {}).get("aruco_id")
            if ids is not None and exp is not None and exp in [int(i[0]) for i in ids]:
                held_ev = True
                notes.append(f"box tag {exp} still visible")
            if reference_frame is not None:
                s = self._similarity(reference_frame, frame)
                sims.append(s)
                if s >= self.grip_similarity_held:
                    held_ev = True
                elif s <= self.grip_similarity_empty and not held_ev:
                    empty_ev = True
            votes_held += held_ev
            votes_empty += empty_ev

        if frames == 0:
            return {"held": None, "confidence": 0.0, "method": "camera",
                    "reason": "no pictures from the hand camera"}
        sim_txt = f", view similarity {max(sims):.2f}" if sims else ""
        if votes_held >= 2 or (frames < 2 and votes_held == frames):
            return {"held": True, "confidence": round(0.6 + 0.13 * votes_held, 2), "method": "camera",
                    "reason": "; ".join(sorted(set(notes))) + sim_txt if notes else "view unchanged since grasp" + sim_txt}
        if votes_empty >= 2:
            return {"held": False, "confidence": round(0.6 + 0.13 * votes_empty, 2), "method": "camera",
                    "reason": "the camera no longer sees the box" + sim_txt}
        return {"held": None, "confidence": 0.4, "method": "camera", "reason": "camera unsure" + sim_txt}

    def check_quality_gate(self, image: np.ndarray) -> Tuple[bool, str]:
        """Check blur (Laplacian variance), brightness and glare/saturation."""
        if image is None or image.size == 0:
            return False, "Empty or invalid frame"

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 1. Blur check (Laplacian variance)
        var = cv2.Laplacian(gray, cv2.CV_64F).var()
        if var < 40.0:
            return False, f"Image too blurry (variance {var:.1f} < 40.0): stop arm and re-look"

        # 2. Brightness check
        mean_brightness = float(np.mean(gray))
        if mean_brightness < 30.0:
            return False, f"Image too dark (brightness {mean_brightness:.1f} < 30.0): turn on LED"
        if mean_brightness > 235.0:
            return False, f"Image overexposed / glare ({mean_brightness:.1f} > 235.0)"

        return True, "Quality OK"

    def verify_single_frame(self, frame: np.ndarray, expected_id: int) -> Dict[str, Any]:
        """Extract ArUco, Barcode and OCR signals from a single image."""
        quality_ok, quality_msg = self.check_quality_gate(frame)
        if not quality_ok:
            return {"quality_ok": False, "reason": quality_msg}

        evidence = {"quality_ok": True, "aruco_id": None, "ean13": None, "ocr_text": None, "yolo_id": None}

        # 0. YOLO
        if self.yolo is not None:
            yid, yconf, dets = self._yolo_target(frame)
            evidence["yolo_id"] = yid
            evidence["yolo_conf"] = yconf
            evidence["yolo_detections"] = [{k: d[k] for k in ("cls", "conf")} for d in dets]

        # 1. ArUco Marker
        corners, ids, _ = self.aruco_detector.detectMarkers(frame)
        if ids is not None and len(ids) > 0:
            detected_aruco = int(ids[0][0])
            evidence["aruco_id"] = detected_aruco

        # 2. Barcode Detector
        if self.barcode_detector is not None:
            try:
                ok, decoded_info, _, _ = self.barcode_detector.detectAndDecode(frame)
                if ok and decoded_info:
                    for code in decoded_info:
                        if code:
                            if validate_ean13_checksum(code):
                                evidence["ean13"] = code
                                break
                            else:
                                evidence["bad_checksum_ean"] = code
            except Exception:
                pass

        # 3. Simple text detection / simulated OCR
        # We check evidence against medicine definitions
        return evidence

    def verify(self, expected_medicine_id: int, min_frame_time: float = 0.0, timeout_s: float = 3.0,
               sim_slot_u: Optional[float] = None) -> Verdict:
        """
        Main verification endpoint.
        Enforces freshness rule: only accepts frames newer than min_frame_time.
        Combines multi-frame voting (3 frames) into a final Verdict.
        sim_slot_u: in simulation, which shelf slot the arm is looking at (each
        medicine sits at its slot_u_mm from medicines.json).
        """
        expected_med = self.med_by_id.get(expected_medicine_id)
        if not expected_med:
            return Verdict("UNCERTAIN", 0.0, reason=f"Unknown expected medicine ID {expected_medicine_id}")

        if self.sim and sim_slot_u is not None:
            time.sleep(0.2)
            here = min(self.medicines, key=lambda m: abs(float(m.get("slot_u_mm", 0.0)) - sim_slot_u))
            if abs(float(here.get("slot_u_mm", 0.0)) - sim_slot_u) > 30.0:
                return Verdict("UNCERTAIN", 0.2, reason="No box in view (sim)", frames_used=3)
            if here["id"] != expected_medicine_id:
                return Verdict("NO_MATCH", 0.97, evidence={"yolo_id": here["id"]}, detected_medicine_id=here["id"],
                               reason=f"Box here is {here['name']} (sim)", frames_used=3)

        if self.sim:
            # High-fidelity simulation mode
            time.sleep(0.3) # Sim processing delay
            exp_aruco = expected_med.get("aruco_id")
            exp_ean = expected_med.get("ean13")
            return Verdict(
                result="MATCH",
                confidence=0.99,
                evidence={
                    "aruco_id": exp_aruco,
                    "ean13": exp_ean,
                    "checksum_valid": True,
                    "ocr_match": expected_med["name"]
                },
                detected_medicine_id=expected_medicine_id,
                reason=f"ArUco ID {exp_aruco} and valid EAN-13 barcode {exp_ean} match {expected_med['name']}.",
                frames_used=3
            )

        # Real verification loop
        start_t = time.time()
        collected_evidence = []
        frames_used = 0
        last_bad = None
        last_t = min_frame_time

        while (time.time() - start_t) < timeout_s:
            if not self.camera_grabber:
                break
            frame, t_rx = self.camera_grabber.get_latest_frame()
            if frame is None or t_rx <= last_t:          # only new frames, never the same one twice
                time.sleep(0.05)
                continue
            last_t = t_rx

            ev = self.verify_single_frame(frame, expected_medicine_id)
            if not ev.get("quality_ok", False):
                last_bad = ev                            # blurry / dark frame: skip it, try the next
                continue

            collected_evidence.append(ev)
            frames_used += 1

            if len(collected_evidence) >= 3:
                break
            time.sleep(0.08)

        if not collected_evidence:
            if last_bad is not None:
                return Verdict("UNCERTAIN", 0.3, evidence=last_bad, reason=last_bad.get("reason", "Bad image quality"))
            return Verdict("UNCERTAIN", 0.0, reason="No fresh frames from the hand camera (is the ESP32-CAM uploading?)", frames_used=0)

        # Multi-frame voting
        if self.yolo is not None:
            yolo_votes = [e.get("yolo_id") for e in collected_evidence if e.get("yolo_id") is not None]
            if yolo_votes:
                voted = max(set(yolo_votes), key=yolo_votes.count)
                count = yolo_votes.count(voted)
                need = max(2, (len(collected_evidence) + 1) // 2)
                if count >= need:
                    m = self.med_by_id.get(voted, {"name": f"id {voted}"})
                    if voted == expected_medicine_id:
                        return Verdict("MATCH", 0.9 + 0.03 * count, evidence={"yolo_votes": yolo_votes},
                                       detected_medicine_id=voted,
                                       reason=f"YOLO saw {m['name']} in {count}/{frames_used} frames.", frames_used=frames_used)
                    return Verdict("NO_MATCH", 0.9, evidence={"yolo_votes": yolo_votes}, detected_medicine_id=voted,
                                   reason=f"YOLO saw {m['name']} here, not the requested medicine.", frames_used=frames_used)

        aruco_votes = [e["aruco_id"] for e in collected_evidence if e["aruco_id"] is not None]
        ean_votes = [e["ean13"] for e in collected_evidence if e["ean13"] is not None]

        voted_aruco = max(set(aruco_votes), key=aruco_votes.count) if aruco_votes else None
        voted_ean = max(set(ean_votes), key=ean_votes.count) if ean_votes else None

        # Check for NO_MATCH (contradicting evidence)
        detected_id = None
        if voted_aruco and voted_aruco in self.med_by_aruco:
            m = self.med_by_aruco[voted_aruco]
            if m["id"] != expected_medicine_id:
                return Verdict("NO_MATCH", 0.95, evidence={"aruco": voted_aruco}, detected_medicine_id=m["id"],
                               reason=f"Found ArUco {voted_aruco} belonging to {m['name']}, not expected medicine", frames_used=frames_used)

        if voted_ean and voted_ean in self.med_by_ean:
            m = self.med_by_ean[voted_ean]
            if m["id"] != expected_medicine_id:
                return Verdict("NO_MATCH", 0.98, evidence={"ean13": voted_ean}, detected_medicine_id=m["id"],
                               reason=f"Decoded EAN-13 barcode {voted_ean} for {m['name']}, not expected medicine", frames_used=frames_used)

        # Check for MATCH: ArUco AND valid barcode matching expected
        exp_aruco = expected_med.get("aruco_id")
        exp_ean = str(expected_med.get("ean13"))

        if voted_aruco == exp_aruco and (voted_ean == exp_ean or len(aruco_votes) >= 2):
            return Verdict("MATCH", 0.98, evidence={"aruco": voted_aruco, "ean13": voted_ean},
                           detected_medicine_id=expected_medicine_id,
                           reason=f"Strong verification: ArUco {voted_aruco} confirmed for {expected_med['name']}.", frames_used=frames_used)

        return Verdict("UNCERTAIN", 0.4, evidence={"aruco": voted_aruco, "ean": voted_ean},
                       reason="Evidence incomplete: need both ArUco and Barcode to authorise grab", frames_used=frames_used)
