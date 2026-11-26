"""
make_markers.py - Printable ArUco tags for Pharma Rover (A4 PDF, exact size, vector = sharp solid black).

    python tools/make_markers.py        -> markers/PharmaRover_markers.pdf

Every tag already has its white border (quiet zone) printed around it, with nothing
inside it except a small light-grey arrow and number near the outer edge (the camera ignores those):
  Page 1-4 : floor tags 0, 1, 2, 3  (170 mm, 20 mm white border = the whole page, NO cutting)
  Page 5   : start tag 6 (120 mm, 20 mm border, cut on the dashed line)
  Page 6   : shelf tag 10 (100 mm, 20 mm border, cut on the dashed line)
  Page 7   : car tag 5 (80 mm, 20 mm border, cut on the dashed line)
  Page 8   : box tags 21, 22, 23 (30 mm, 8 mm border so they fit on the box), cut on the dashed lines

Print at 100% / "Actual size" (NOT "Fit to page"), quality "Best", toner save OFF.
Check with a ruler that the black square is the size written on the page.
"""
import os
import cv2
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.colors import black, Color
from reportlab.pdfgen import canvas

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PAGE_W, PAGE_H = A4
LIGHT = Color(0.72, 0.72, 0.72)      # light grey: invisible to the tag detector
TEXT = Color(0.35, 0.35, 0.35)
BLEED = 0.06 * mm                    # tiny overlap so black cells never show hairline gaps


def draw_tag(c, marker_id, size_mm, cx, cy):
    """Solid black ArUco tag, centre (cx, cy) in points. Returns the tag's half size in points."""
    bits = cv2.aruco.generateImageMarker(DICT, marker_id, 6)   # 6x6 cells incl. the black frame
    s = size_mm * mm
    cell = s / 6.0
    x0, y_top = cx - s / 2, cy + s / 2
    c.setFillColor(black)
    c.setStrokeColor(black)
    for r in range(6):
        col = 0
        while col < 6:
            if bits[r, col] == 0:                      # black run
                start = col
                while col < 6 and bits[r, col] == 0:
                    col += 1
                x = x0 + start * cell
                y = y_top - (r + 1) * cell
                c.rect(x - BLEED, y - BLEED, (col - start) * cell + 2 * BLEED, cell + 2 * BLEED,
                       stroke=0, fill=1)
            else:
                col += 1
    return s / 2


def direction_arrow(c, cx, y, label):
    """Small light-grey arrow (pointing up the page) + label, placed in the outer part of the border."""
    c.setFillColor(LIGHT)
    c.setStrokeColor(LIGHT)
    p = c.beginPath()
    p.moveTo(cx, y + 3.5 * mm)
    p.lineTo(cx - 2.5 * mm, y)
    p.lineTo(cx + 2.5 * mm, y)
    p.close()
    c.drawPath(p, stroke=0, fill=1)
    c.setFont("Helvetica", 7)
    c.drawString(cx + 4 * mm, y + 0.5 * mm, label)


def cut_box(c, cx, cy, half):
    c.setStrokeColor(LIGHT)
    c.setLineWidth(0.4)
    c.setDash(3, 3)
    c.rect(cx - half, cy - half, 2 * half, 2 * half, stroke=1, fill=0)
    c.setDash()


def caption(c, x, y, lines, size=9):
    c.setFillColor(TEXT)
    for i, (txt, sz) in enumerate(lines):
        c.setFont("Helvetica-Bold" if i == 0 else "Helvetica", sz)
        c.drawString(x, y - i * (sz + 3), txt)


def cut_tag(c, marker_id, size_mm, border_mm, cx, cy, name, note, arrow=True):
    """Tag + white border + dashed cut line; light-grey arrow and ID inside the border's outer edge."""
    half = draw_tag(c, marker_id, size_mm, cx, cy)
    outer = half + border_mm * mm
    cut_box(c, cx, cy, outer)
    if arrow and border_mm >= 15:
        direction_arrow(c, cx, cy + outer - 6 * mm, f"UP / front   ID {marker_id}")
    caption(c, cx - outer, cy - outer - 6 * mm,
            [(f"{name}  (ID {marker_id}, black square {size_mm:.0f} mm)", 10), (note, 8),
             ("Cut on the dashed line. Keep the white border.", 8)])


def main():
    out_dir = os.path.join(ROOT, "markers")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "PharmaRover_markers.pdf")
    c = canvas.Canvas(out, pagesize=A4)
    c.setTitle("Pharma Rover ArUco markers")

    # ---- pages 1-4: floor tags, 170 mm + 20 mm border = the page itself, no cutting ----
    corners = {0: "bottom-left", 1: "bottom-right", 2: "top-right", 3: "top-left"}
    for mid in range(4):
        cx, cy = PAGE_W / 2, PAGE_H / 2
        half = draw_tag(c, mid, 170, cx, cy)
        direction_arrow(c, cx, cy + half + 32 * mm, f"UP   (floor tag {mid})")
        caption(c, 20 * mm, 22 * mm,
                [(f"FLOOR TAG {mid} - {corners[mid]} corner  (ID {mid}, black square 170 mm)", 10),
                 ("No cutting needed: the page margin is the white border. Tape it flat, all 4 the same way up.", 7.5)])
        c.showPage()

    # ---- page 5: start tag ----
    cut_tag(c, 6, 120, 20, PAGE_W / 2, PAGE_H / 2 + 20 * mm, "START TAG",
            "On the floor where the car starts and returns. Arrow = the way the car faces when parked.")
    c.showPage()

    # ---- page 6: shelf ----
    cut_tag(c, 10, 100, 20, PAGE_W / 2, PAGE_H / 2 + 20 * mm, "SHELF TAG",
            "Flat on the shelf top, above the middle of the front edge. Arrow points INTO the shelf.")
    c.showPage()

    # ---- page 7: car ----
    cut_tag(c, 5, 80, 20, PAGE_W / 2, PAGE_H / 2 + 20 * mm, "CAR TAG",
            "Flat on top of the car, centred over the arm base. Arrow = the car's FRONT.")
    c.showPage()

    # ---- page 8: box tags (2 of each), 8 mm border so they fit on the box face ----
    names = {21: "PARACETAMOL", 22: "CETIRIZINE", 23: "OMEPRAZOLE"}
    for row, mid in enumerate((21, 22, 23)):
        for col in range(2):
            cx = (55 + col * 100) * mm
            cy = PAGE_H - (60 + row * 85) * mm
            half = draw_tag(c, mid, 30, cx, cy)
            cut_box(c, cx, cy, half + 8 * mm)
            caption(c, cx - half - 8 * mm, cy - half - 14 * mm,
                    [(f"{names[mid]}  (ID {mid}, 30 mm)", 9), ("Front face of the box. Cut on the dashed line.", 7)])
    c.showPage()
    c.save()
    print(f"Saved {out} (8 pages). Print at 100% / Actual size, Best quality, toner save OFF.")


if __name__ == "__main__":
    main()
