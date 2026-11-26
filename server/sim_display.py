"""
sim_display.py - Visual simulation of the 480x320 Uno TFT display shield (ILI9486).
Renders both the expressive emotion face and the medicine info card using Pillow and OpenCV.
"""
import math
import time
import numpy as np
from PIL import Image, ImageDraw, ImageFont

WIDTH = 480
HEIGHT = 320

# Color palette (RGB)
BLACK   = (0, 0, 0)
WHITE   = (255, 255, 255)
CYAN    = (0, 220, 255)
BLUE    = (30, 100, 220)
NAVY    = (15, 25, 45)
GREEN   = (0, 230, 100)
RED     = (255, 50, 50)
YELLOW  = (255, 200, 0)
DARK_GRAY = (40, 40, 45)
LIGHT_GRAY = (180, 180, 185)

class SimDisplayRenderer:
    def __init__(self):
        self.state = "idle"
        self.idle_style = "SAD"
        self.show_card = False
        self.card_data = {
            "name": "Paracetamol",
            "strength": "500mg",
            "category": "Painkiller",
            "dose": "1-2 tablets",
            "freq": "every 4-6 h",
            "max": "4 g / 24 h",
            "use": "Fever, pain",
            "warn": "Do not overdose",
            "shelf": "S1/L1"
        }
        self.status_text = "IDLE"
        self.progress = 0
        self.card_page = 0
        self.last_page_time = time.time()
        self.last_anim_time = time.time()
        self.tear_y = 190

    def render(self) -> np.ndarray:
        """Returns 480x320 BGR image (OpenCV format)."""
        im = Image.new("RGB", (WIDTH, HEIGHT), NAVY if not self.show_card else (10, 15, 25))
        draw = ImageDraw.Draw(im)

        if self.show_card:
            self._draw_card(draw)
        else:
            self._draw_face(draw)

        # Convert PIL to BGR numpy array
        rgb = np.array(im)
        bgr = rgb[:, :, ::-1].copy()
        return bgr

    def _draw_face(self, draw: ImageDraw.ImageDraw):
        t = time.time()
        cx1, cy1 = 150, 130   # Left eye center
        cx2, cy2 = 330, 130   # Right eye center
        r = 50

        # State animations
        if self.state == "idle":
            # Sad droopy eyes, downcast pupils, downturned mouth
            blink = (int(t) % 5 == 0) and ((t % 1.0) < 0.25)
            if blink:
                draw.line([(cx1 - r, cy1), (cx1 + r, cy1 + 10)], fill=CYAN, width=6)
                draw.line([(cx2 - r, cy2 + 10), (cx2 + r, cy2)], fill=CYAN, width=6)
            else:
                # Droopy white/cyan eyes
                draw.ellipse([cx1 - r, cy1 - r, cx1 + r, cy1 + r], outline=CYAN, width=5, fill=(10, 30, 50))
                draw.ellipse([cx2 - r, cy2 - r, cx2 + r, cy2 + r], outline=CYAN, width=5, fill=(10, 30, 50))
                # Downcast pupils
                draw.ellipse([cx1 - 15, cy1 + 5, cx1 + 15, cy1 + 35], fill=CYAN)
                draw.ellipse([cx2 - 15, cy2 + 5, cx2 + 15, cy2 + 35], fill=CYAN)
                # Sad eyebrows
                draw.line([(cx1 - r - 5, cy1 - r + 15), (cx1 + r - 10, cy1 - r - 10)], fill=CYAN, width=5)
                draw.line([(cx2 - r + 10, cy2 - r - 10), (cx2 + r + 5, cy2 - r + 15)], fill=CYAN, width=5)

            # Downturned sad mouth
            draw.arc([190, 230, 290, 280], start=200, end=340, fill=CYAN, width=6)
            # Occasional tear dropping
            tear_pos = 180 + int((t * 60) % 70)
            draw.ellipse([cx1 + 25, tear_pos, cx1 + 33, tear_pos + 12], fill=(100, 200, 255))

        elif self.state == "listening":
            # Wide attentive eyes, centered bright pupils
            draw.ellipse([cx1 - r - 5, cy1 - r - 5, cx1 + r + 5, cy1 + r + 5], outline=WHITE, width=6, fill=CYAN)
            draw.ellipse([cx2 - r - 5, cy2 - r - 5, cx2 + r + 5, cy2 + r + 5], outline=WHITE, width=6, fill=CYAN)
            draw.ellipse([cx1 - 18, cy1 - 18, cx1 + 18, cy1 + 18], fill=NAVY)
            draw.ellipse([cx2 - 18, cy2 - 18, cx2 + 18, cy2 + 18], fill=NAVY)
            # Small O mouth
            draw.ellipse([225, 230, 255, 260], outline=WHITE, width=5)
            # Sound waves on sides
            for off in [15, 30]:
                draw.arc([cx1 - r - 30 - off, cy1 - 30, cx1 - r + off, cy1 + 30], start=120, end=240, fill=CYAN, width=3)
                draw.arc([cx2 + r - off, cy2 - 30, cx2 + r + 30 + off, cy2 + 30], start=300, end=60, fill=CYAN, width=3)

        elif self.state == "thinking":
            # Eyes looking upward/sideways, animated "..." dots
            draw.ellipse([cx1 - r, cy1 - r, cx1 + r, cy1 + r], outline=YELLOW, width=5, fill=(40, 40, 20))
            draw.ellipse([cx2 - r, cy2 - r, cx2 + r, cy2 + r], outline=YELLOW, width=5, fill=(40, 40, 20))
            draw.ellipse([cx1 - 10, cy1 - 35, cx1 + 20, cy1 - 5], fill=YELLOW)
            draw.ellipse([cx2 - 10, cy2 - 35, cx2 + 20, cy2 - 5], fill=YELLOW)
            # Wavy mouth
            draw.arc([200, 230, 280, 260], start=0, end=180, fill=YELLOW, width=5)
            # Thought dots
            step = int(t * 3) % 4
            for i in range(step):
                draw.ellipse([340 + i * 20, 50 - i * 5, 350 + i * 20, 60 - i * 5], fill=YELLOW)

        elif self.state == "scanning":
            # Narrow focused eyes, sweeping horizontal laser scan line
            draw.line([(cx1 - r, cy1), (cx1 + r, cy1)], fill=GREEN, width=8)
            draw.line([(cx2 - r, cy2), (cx2 + r, cy2)], fill=GREEN, width=8)
            # Scanning beam
            scan_y = 50 + int((t * 120) % 220)
            draw.line([(40, scan_y), (440, scan_y)], fill=GREEN, width=3)
            draw.line([(210, 240), (270, 240)], fill=GREEN, width=5)

        elif self.state == "happy":
            # Happy upward arcs ^ ^, big smile
            draw.arc([cx1 - r, cy1 - 30, cx1 + r, cy1 + 30], start=180, end=360, fill=GREEN, width=7)
            draw.arc([cx2 - r, cy2 - 30, cx2 + r, cy2 + 30], start=180, end=360, fill=GREEN, width=7)
            draw.arc([180, 200, 300, 270], start=0, end=180, fill=GREEN, width=7)

        elif self.state == "alert":
            # Alert wide eyes, looking in direction
            draw.ellipse([cx1 - r, cy1 - r, cx1 + r, cy1 + r], outline=WHITE, width=5, fill=(20, 60, 100))
            draw.ellipse([cx2 - r, cy2 - r, cx2 + r, cy2 + r], outline=WHITE, width=5, fill=(20, 60, 100))
            draw.ellipse([cx1 + 5, cy1 - 15, cx1 + 35, cy1 + 15], fill=WHITE)
            draw.ellipse([cx2 + 5, cy2 - 15, cx2 + 35, cy2 + 15], fill=WHITE)
            draw.line([200, 240, 280, 240], fill=WHITE, width=5)

        elif self.state == "obstacle":
            # Startled round eyes with "!" icon
            draw.ellipse([cx1 - r - 10, cy1 - r - 10, cx1 + r + 10, cy1 + r + 10], outline=RED, width=6, fill=RED)
            draw.ellipse([cx2 - r - 10, cy2 - r - 10, cx2 + r + 10, cy2 + r + 10], outline=RED, width=6, fill=RED)
            draw.ellipse([cx1 - 10, cy1 - 10, cx1 + 10, cy1 + 10], fill=WHITE)
            draw.ellipse([cx2 - 10, cy2 - 10, cx2 + 10, cy2 + 10], fill=WHITE)
            # Big "!" in center
            draw.line([(240, 40), (240, 85)], fill=RED, width=8)
            draw.ellipse([(236, 95), (244, 103)], fill=RED)
            draw.ellipse([215, 230, 265, 275], outline=RED, width=5)

        elif self.state == "success":
            # Big happy face + green checkmark
            draw.arc([cx1 - r, cy1 - 30, cx1 + r, cy1 + 30], start=180, end=360, fill=GREEN, width=8)
            draw.arc([cx2 - r, cy2 - 30, cx2 + r, cy2 + 30], start=180, end=360, fill=GREEN, width=8)
            draw.arc([170, 190, 310, 270], start=0, end=180, fill=GREEN, width=8)
            # Checkmark in top corner
            draw.line([(400, 50), (415, 65)], fill=GREEN, width=7)
            draw.line([(415, 65), (445, 30)], fill=GREEN, width=7)

        elif self.state == "error":
            # "X" eyes and zigzag mouth
            draw.line([(cx1 - 35, cy1 - 35), (cx1 + 35, cy1 + 35)], fill=RED, width=7)
            draw.line([(cx1 + 35, cy1 - 35), (cx1 - 35, cy1 + 35)], fill=RED, width=7)
            draw.line([(cx2 - 35, cy2 - 35), (cx2 + 35, cy2 + 35)], fill=RED, width=7)
            draw.line([(cx2 + 35, cy2 - 35), (cx2 - 35, cy2 + 35)], fill=RED, width=7)
            # Zigzag mouth
            pts = [(190, 250), (210, 235), (230, 255), (250, 235), (270, 255), (290, 240)]
            draw.line(pts, fill=RED, width=5)

        # Status text at bottom
        draw.text((20, 290), f"STATE: {self.state.upper()}", fill=LIGHT_GRAY)
        draw.text((360, 290), "UNO 3.5\" TFT", fill=DARK_GRAY)

    def _draw_card(self, draw: ImageDraw.ImageDraw):
        # Auto-cycle card pages every 3 seconds
        t = time.time()
        if t - self.last_page_time >= 3.0:
            self.card_page = (self.card_page + 1) % 3
            self.last_page_time = t

        # Common Header banner (Name + Strength)
        draw.rectangle([0, 0, WIDTH, 50], fill=BLUE)
        name_str = f"{self.card_data.get('name', 'Unknown')}  {self.card_data.get('strength', '')}"
        draw.text((20, 12), name_str[:32], fill=WHITE)

        # Page 0: Overview & Logistics
        if self.card_page == 0:
            cat_str = f"Type: {self.card_data.get('category', 'Analgesic')}"
            draw.text((20, 65), cat_str, fill=CYAN)
            shelf_str = f"Shelf & Slot: {self.card_data.get('shelf', 'S1, slot +100mm')}"
            draw.text((20, 95), shelf_str, fill=WHITE)
            aruco_str = f"ArUco ID: {self.card_data.get('aruco', '21')}    |    Weight: {self.card_data.get('weight', '45g')}"
            draw.text((20, 130), aruco_str, fill=YELLOW)
            draw.text((20, 175), f"Mission: {self.status_text}", fill=GREEN)
            draw.text((20, 298), "Page 1/3 - Overview | Pharma Rover Mission", fill=LIGHT_GRAY)

        # Page 1: Clinical Dosage & Indications
        elif self.card_page == 1:
            draw.text((20, 65), f"Dose: {self.card_data.get('dose', '1-2 tablets')}", fill=WHITE)
            draw.text((20, 95), f"Frequency: {self.card_data.get('freq', 'every 4-6 h')}", fill=WHITE)
            draw.text((20, 125), f"Max 24h: {self.card_data.get('max', '4000mg/24h')}", fill=WHITE)
            draw.text((20, 155), f"Use: {self.card_data.get('use', 'Fever, pain, headache')}", fill=LIGHT_GRAY)
            # Warning box (highlighted red/yellow)
            draw.rectangle([15, 195, WIDTH - 15, 245], fill=(60, 20, 20), outline=RED, width=2)
            warn_str = f"NOTE: {self.card_data.get('warn', 'Liver care: do not overdose')}"
            draw.text((25, 212), warn_str[:54], fill=YELLOW)
            draw.text((20, 298), "Page 2/3 - Clinical Info | Safety Validated", fill=LIGHT_GRAY)

        # Page 2: Robot Kinematics & Grasp
        elif self.card_page == 2:
            grasp_pt = self.card_data.get('grasp', '[250, 0, -12]')
            draw.text((20, 65), f"Arm Grasp: {grasp_pt} mm", fill=CYAN)
            ean_str = f"EAN-13: {self.card_data.get('ean', '8961000000112')}"
            draw.text((20, 95), ean_str, fill=WHITE)
            box_str = f"Box Size: {self.card_data.get('box_size', '100x50x20')} mm"
            draw.text((20, 125), box_str, fill=WHITE)

            # Status and Progress Bar
            draw.text((20, 230), f"STATUS: {self.status_text} ({self.progress}%)", fill=GREEN)
            draw.rectangle([20, 255, WIDTH - 20, 275], fill=(30, 30, 40), outline=LIGHT_GRAY, width=1)
            pct = max(0, min(100, self.progress))
            fill_w = int((WIDTH - 40) * (pct / 100.0))
            if fill_w > 0:
                draw.rectangle([21, 256, 20 + fill_w, 274], fill=GREEN)
            draw.text((20, 298), "Page 3/3 - Robot Grasp & Verification Status", fill=LIGHT_GRAY)
