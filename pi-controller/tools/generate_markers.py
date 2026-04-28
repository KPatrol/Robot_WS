#!/usr/bin/env python3
"""
generate_markers.py — ArUco marker PDF generator for K-Patrol pillars

Generates a print-ready PDF with 20 markers:
  - 5 IDs (0=HOME, 1-4=CORNER)
  - 4 copies per ID (one for each face of a pillar)
  - Each marker 10x10 cm with 1cm white border
  - 4 markers per A4 landscape page

Usage:
    python3 generate_markers.py
    # Outputs: kpatrol_markers.pdf in current directory

Print settings:
    - Paper: A4
    - Orientation: Landscape
    - Scale: 100% (DO NOT "fit to page")
    - Verify marker size with a ruler after printing (should be exactly 10 cm)
"""

import os
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# ── Marker configuration ──────────────────────────────────────
ARUCO_DICT = cv2.aruco.DICT_4X4_50  # 50 unique IDs, fast detection
MARKER_SIZE_CM = 10.0                # Physical size when printed
BORDER_CM = 1.0                      # White border around marker
LABEL_HEIGHT_CM = 1.5                # Text label below marker

# A4 landscape in cm: 29.7 x 21.0
PAGE_W_CM = 29.7
PAGE_H_CM = 21.0
DPI = 300  # Print quality

# Marker IDs and labels
MARKERS = [
    (0, "HOME"),
    (1, "CORNER 1"),
    (2, "CORNER 2"),
    (3, "CORNER 3"),
    (4, "CORNER 4"),
]
COPIES_PER_ID = 4  # 4 faces per pillar


def cm_to_px(cm):
    return int(round(cm * DPI / 2.54))


def generate_marker_image(marker_id, size_px):
    """Generate a single ArUco marker as PIL Image."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_px)
    # Convert to RGB PIL image
    return Image.fromarray(marker_img).convert("RGB")


def compose_marker_with_label(marker_id, label):
    """Compose one marker tile: white border + marker + label text."""
    marker_px = cm_to_px(MARKER_SIZE_CM)
    border_px = cm_to_px(BORDER_CM)
    label_px = cm_to_px(LABEL_HEIGHT_CM)

    tile_w = marker_px + 2 * border_px
    tile_h = marker_px + 2 * border_px + label_px

    tile = Image.new("RGB", (tile_w, tile_h), color=(255, 255, 255))
    marker = generate_marker_image(marker_id, marker_px)
    tile.paste(marker, (border_px, border_px))

    # Draw label
    draw = ImageDraw.Draw(tile)
    text = f"ID {marker_id}  -  {label}"
    try:
        font_size = cm_to_px(0.6)
        font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", font_size)
    except Exception:
        font = ImageFont.load_default()

    bbox = draw.textbbox((0, 0), text, font=font)
    text_w = bbox[2] - bbox[0]
    text_y = marker_px + 2 * border_px + (label_px - (bbox[3] - bbox[1])) // 2
    draw.text(((tile_w - text_w) // 2, text_y), text, fill=(0, 0, 0), font=font)

    # Cut marks (thin corners)
    cut_len = cm_to_px(0.3)
    color = (180, 180, 180)
    for x, y in [(0, 0), (tile_w - 1, 0), (0, tile_h - 1), (tile_w - 1, tile_h - 1)]:
        dx = cut_len if x == 0 else -cut_len
        dy = cut_len if y == 0 else -cut_len
        draw.line([(x, y), (x + dx, y)], fill=color, width=2)
        draw.line([(x, y), (x, y + dy)], fill=color, width=2)

    return tile


def create_page(marker_tiles):
    """Create an A4 landscape page with up to 4 marker tiles (2x2 grid)."""
    page_w = cm_to_px(PAGE_W_CM)
    page_h = cm_to_px(PAGE_H_CM)
    page = Image.new("RGB", (page_w, page_h), color=(255, 255, 255))

    if not marker_tiles:
        return page

    tile_w, tile_h = marker_tiles[0].size

    # 2x2 grid layout with even spacing
    cols, rows = 2, 2
    total_tile_w = cols * tile_w
    total_tile_h = rows * tile_h
    h_margin = (page_w - total_tile_w) // (cols + 1)
    v_margin = (page_h - total_tile_h) // (rows + 1)

    for idx, tile in enumerate(marker_tiles):
        col = idx % cols
        row = idx // cols
        x = h_margin + col * (tile_w + h_margin)
        y = v_margin + row * (tile_h + v_margin)
        page.paste(tile, (x, y))

    return page


def main():
    print("=" * 60)
    print("K-Patrol ArUco Marker Generator")
    print("=" * 60)
    print(f"Dictionary: DICT_4X4_50")
    print(f"Marker size: {MARKER_SIZE_CM} cm (when printed at 100% scale)")
    print(f"Markers: {len(MARKERS)} IDs x {COPIES_PER_ID} copies = {len(MARKERS) * COPIES_PER_ID} total")
    print(f"Page: A4 landscape ({PAGE_W_CM}x{PAGE_H_CM} cm) @ {DPI} DPI")
    print()

    # Build all marker tiles
    all_tiles = []
    for marker_id, label in MARKERS:
        print(f"  Generating ID {marker_id} ({label})...")
        for copy_idx in range(COPIES_PER_ID):
            tile = compose_marker_with_label(marker_id, f"{label} #{copy_idx + 1}")
            all_tiles.append(tile)

    # Pack into pages (4 per page = 1 page per ID)
    pages = []
    for i in range(0, len(all_tiles), 4):
        page_tiles = all_tiles[i:i + 4]
        page = create_page(page_tiles)
        pages.append(page)
        print(f"  Page {len(pages)} composed ({len(page_tiles)} markers)")

    # Save as PDF
    output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "kpatrol_markers.pdf")
    pages[0].save(
        output_path,
        save_all=True,
        append_images=pages[1:],
        format="PDF",
        resolution=DPI,
    )

    print()
    print(f"PDF saved: {output_path}")
    print(f"Total pages: {len(pages)}")
    print()
    print("PRINTING INSTRUCTIONS:")
    print("  1. Open the PDF in Preview / Adobe Reader")
    print("  2. Print to A4, LANDSCAPE orientation")
    print("  3. Scale: 100% (NOT 'fit to page' or 'shrink to fit')")
    print("  4. Verify marker edge with a ruler = 10.0 cm exactly")
    print()
    print("PILLAR ASSEMBLY:")
    print("  - Pillar body: square tube ~12x12x35 cm (cardboard or foam)")
    print("  - Paste 4 copies of SAME ID on the 4 faces")
    print("  - Marker center should be ~25-30 cm from floor")
    print("  - Heavy base (rocks/weight) to prevent toppling")
    print("  - 5 pillars total: HOME + 4 corners")
    print()


if __name__ == "__main__":
    main()
