"""
intents.py - Unified intent parser for Pharma Rover (voice and typed text).
Fuzzy matching on medicine names, aliases, driving commands, and arm jogging.
"""
import re
import difflib
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple

try:
    from rapidfuzz import fuzz
    def fuzzy_ratio(s1: str, s2: str) -> float:
        return float(fuzz.ratio(s1.lower(), s2.lower()))
except ImportError:
    def fuzzy_ratio(s1: str, s2: str) -> float:
        return difflib.SequenceMatcher(None, s1.lower(), s2.lower()).ratio() * 100.0

@dataclass
class Intent:
    name: str                           # FETCH, DRIVE_STEP, TURN, GO_START, ARM_HOME, etc.
    params: Dict[str, Any]
    raw_text: str
    confidence: float
    spoken_reply: str

WORD_NUMBERS = {"a": 1, "an": 1, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
                "seven": 7, "eight": 8, "nine": 9, "ten": 10, "twenty": 20, "thirty": 30, "forty": 40,
                "fifty": 50, "sixty": 60, "ninety": 90, "half": 0.5, "hundred": 100}
UNIT_MM = {"mm": 1.0, "millimeter": 1.0, "millimeters": 1.0, "millimetre": 1.0, "millimetres": 1.0,
           "cm": 10.0, "centimeter": 10.0, "centimeters": 10.0, "centimetre": 10.0, "centimetres": 10.0,
           "m": 1000.0, "meter": 1000.0, "meters": 1000.0, "metre": 1000.0, "metres": 1000.0}


def parse_distance_mm(t: str) -> Optional[float]:
    """'50 cm' -> 500, 'one meter' -> 1000, 'half a meter' -> 500."""
    m = re.search(r"\b(\d+(?:\.\d+)?|" + "|".join(WORD_NUMBERS) + r")\s*(?:a\s+)?(" + "|".join(sorted(UNIT_MM, key=len, reverse=True)) + r")\b", t)
    if not m:
        return None
    num = m.group(1)
    value = float(num) if num[0].isdigit() else WORD_NUMBERS[num]
    return value * UNIT_MM[m.group(2)]


def parse_degrees(t: str) -> Optional[float]:
    m = re.search(r"\b(\d+|" + "|".join(WORD_NUMBERS) + r")\s*(?:deg|degree|degrees)\b", t)
    if not m:
        return None
    num = m.group(1)
    return float(num) if num[0].isdigit() else float(WORD_NUMBERS[num])


class IntentParser:
    def __init__(self, display_client=None, step_mm: float = 200.0, turn_deg: float = 90.0, continuous_mm: float = 2000.0):
        self.display_client = display_client
        self.last_drive_step: Optional[Tuple[str, float]] = None
        self.step_mm = float(step_mm)          # "move forward"
        self.turn_deg = float(turn_deg)        # "turn right" / "move right"
        self.continuous_mm = float(continuous_mm)   # "keep moving forward" (until "stop")

    def parse(self, text: str) -> Intent:
        """Parse raw speech or typed text into a single structured intent."""
        if not text:
            return Intent("UNKNOWN", {}, "", 0.0, "I didn't catch that.")

        t = text.lower().strip()
        t_clean = re.sub(r"[^a-z0-9 ]", " ", t)

        # 1. STOP beats everything (highest priority)
        if re.search(r"\b(stop|halt|freeze|abort|cancel|hold)\b", t_clean):
            return Intent("STOP", {}, text, 1.0, "Stopping now.")

        # 2. Rescan
        if re.search(r"\b(scan again|look again|rescan|check again|look at it)\b", t_clean):
            return Intent("RESCAN", {}, text, 0.95, "Scanning again.")

        # 3. Repeat step ("a bit", "more")
        if re.search(r"\b(a bit|more|again|repeat)\b", t_clean) and self.last_drive_step:
            action, val = self.last_drive_step
            multiplier = 0.5 if "bit" in t_clean else 1.0
            new_val = val * multiplier
            if action in ("fwd", "back"):
                return Intent("DRIVE_STEP", {"dir": action, "dist_mm": new_val}, text, 0.9, f"Moving {action} {int(new_val)} mm.")
            else:
                return Intent("TURN", {"dir": action, "deg": new_val}, text, 0.9, f"Turning {action} {int(new_val)} degrees.")

        # 4. FETCH intent (medicine search)
        fetch_words = ["bring", "get", "fetch", "grab", "find", "pick", "take"]
        is_fetch = any(re.search(r"\b" + w + r"\b", t_clean) for w in fetch_words)
        if is_fetch or ("paracetamol" in t_clean or "panadol" in t_clean or "cetirizine" in t_clean or "omeprazole" in t_clean or "antacid" in t_clean or "allergy" in t_clean):
            # Try to match medicine
            med = None
            if self.display_client:
                med = self.display_client.match_medicine(t_clean, threshold=70.0)

            # Direct fallback matching
            if not med:
                if "paracetamol" in t_clean or "panadol" in t_clean or "painkiller" in t_clean or "tylenol" in t_clean:
                    med = {"id": 1, "name": "Paracetamol 500mg"}
                elif "cetirizine" in t_clean or "allergy" in t_clean or "antihistamine" in t_clean or "zyrtec" in t_clean:
                    med = {"id": 2, "name": "Cetirizine 10mg"}
                elif "omeprazole" in t_clean or "antacid" in t_clean or "heartburn" in t_clean or "acidity" in t_clean:
                    med = {"id": 3, "name": "Omeprazole 20mg"}

            if med:
                med_name = med.get("name", "medicine")
                return Intent("FETCH", {"medicine_id": med["id"], "medicine": med}, text, 0.95, f"Okay, fetching {med_name}.")

        # 5. Manual drive commands ("move forward", "move forward 50 cm", "keep going forward")
        continuous = bool(re.search(r"\b(keep (going|moving|driving)|continuous|continuously|until i say stop|non stop)\b", t_clean))
        dist = parse_distance_mm(t_clean)
        degs = parse_degrees(t_clean)
        step = dist if dist else (self.continuous_mm if continuous else self.step_mm)
        step = max(20.0, min(step, 5000.0))

        if re.search(r"\b(go forward|go front|move ahead|forward|forwards|straight|front|ahead)\b", t_clean):
            self.last_drive_step = ("fwd", step)
            return Intent("DRIVE_STEP", {"dir": "fwd", "dist_mm": step}, text, 0.95, "Moving forward.")

        if re.search(r"\b(go back|move back|reverse|backward|backwards|back up)\b", t_clean):
            self.last_drive_step = ("back", step)
            return Intent("DRIVE_STEP", {"dir": "back", "dist_mm": step}, text, 0.95, "Moving back.")

        turn = degs if degs else self.turn_deg
        short = len(t_clean.split()) <= 2          # bare "left" / "right" only as a 1-2 word command
        not_arm = not re.search(r"\barm\b", t_clean)
        if not_arm and (re.search(r"\b(turn|spin|move|go|rotate) (to the )?left\b", t_clean) or (short and re.search(r"\bleft\b", t_clean))):
            self.last_drive_step = ("left", turn)
            return Intent("TURN", {"dir": "left", "deg": turn}, text, 0.95, "Turning left.")

        if not_arm and (re.search(r"\b(turn|spin|move|go|rotate) (to the )?(right|write)\b", t_clean) or (short and re.search(r"\b(right|write)\b", t_clean))):
            self.last_drive_step = ("right", turn)
            return Intent("TURN", {"dir": "right", "deg": turn}, text, 0.95, "Turning right.")

        if re.search(r"\b(go home|come back|return home|return to start|go to start)\b", t_clean):
            return Intent("GO_START", {}, text, 0.95, "Returning to start position.")

        # 6. Arm specific commands
        if re.search(r"\b(arm home|fold arm|fold the arm|park arm)\b", t_clean):
            return Intent("ARM_HOME", {}, text, 0.95, "Folding arm home.")

        if re.search(r"\b(open gripper|open the gripper|release)\b", t_clean):
            return Intent("GRIPPER_OPEN", {}, text, 0.95, "Opening gripper.")

        if re.search(r"\b(close gripper|close the gripper|grip)\b", t_clean):
            return Intent("GRIPPER_CLOSE", {}, text, 0.95, "Closing gripper.")

        # Arm Jogging
        if re.search(r"\b(arm up|lift arm)\b", t_clean):
            return Intent("JOG", {"axis": "z", "step_mm": 20.0}, text, 0.9, "Jogging arm up.")
        if re.search(r"\b(arm down|lower arm)\b", t_clean):
            return Intent("JOG", {"axis": "z", "step_mm": -20.0}, text, 0.9, "Jogging arm down.")
        if re.search(r"\b(arm left)\b", t_clean):
            return Intent("JOG", {"axis": "yaw", "step_deg": 10.0}, text, 0.9, "Jogging arm left.")
        if re.search(r"\b(arm right)\b", t_clean):
            return Intent("JOG", {"axis": "yaw", "step_deg": -10.0}, text, 0.9, "Jogging arm right.")

        # 7. Fuzzy fallback: "moove forwad", "tern rite" ... -> closest known command
        canonical = {
            "move forward": "move forward", "go forward": "move forward", "move back": "move back",
            "go back": "move back", "turn left": "turn left", "turn right": "turn right",
            "go home": "go home", "arm home": "arm home", "open gripper": "open gripper",
            "close gripper": "close gripper",
        }
        best, best_score = None, 0.0
        for phrase in canonical:
            score = fuzzy_ratio(phrase, t_clean)
            if score > best_score:
                best, best_score = phrase, score
        if best and best_score >= 80.0 and not getattr(self, "_fuzzy_depth", 0):
            self._fuzzy_depth = 1
            try:
                guess = self.parse(canonical[best])
            finally:
                self._fuzzy_depth = 0
            guess.raw_text = text
            guess.confidence = round(best_score / 100.0 * 0.9, 2)
            return guess

        return Intent("UNKNOWN", {}, text, 0.0, "I didn't catch that.")
