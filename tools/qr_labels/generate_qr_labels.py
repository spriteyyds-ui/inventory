#!/usr/bin/env python3
"""Generate cabinet QR labels as QR PNGs and A4 printable PDFs."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

try:
    import qrcode
    from qrcode.constants import ERROR_CORRECT_H
except ImportError:  # pragma: no cover - optional dependency
    qrcode = None
    ERROR_CORRECT_H = None

try:
    import cv2
except ImportError:  # pragma: no cover - optional fallback
    cv2 = None


MM_PER_INCH = 25.4


def mm_to_px(mm: float, dpi: int) -> int:
    return int(round(mm / MM_PER_INCH * dpi))


def load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        "arial.ttf",
        "Arial.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    ]
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size=size)
        except OSError:
            continue
    return ImageFont.load_default()


def make_qr_image(payload: str, size_px: int) -> Image.Image:
    if qrcode is not None:
        qr = qrcode.QRCode(
            version=None,
            error_correction=ERROR_CORRECT_H,
            box_size=20,
            border=4,
        )
        qr.add_data(payload)
        qr.make(fit=True)
        image = qr.make_image(fill_color="black", back_color="white").convert("RGB")
    elif cv2 is not None:
        params = cv2.QRCodeEncoder_Params()
        params.correction_level = cv2.QRCODE_ENCODER_CORRECT_LEVEL_H
        params.mode = cv2.QRCODE_ENCODER_MODE_NUMERIC
        encoder = cv2.QRCodeEncoder_create(params)
        encoded = encoder.encode(payload)
        image = Image.fromarray(encoded).convert("RGB")
        bordered = Image.new("RGB", (image.size[0] + 8, image.size[1] + 8), "white")
        bordered.paste(image, (4, 4))
        image = bordered
    else:
        raise SystemExit(
            "Missing QR generator. Install qrcode with: python -m pip install qrcode[pil]"
        )
    base_size = image.size[0]
    scale = max(1, size_px // max(1, base_size))
    scaled_size = base_size * scale
    image = image.resize((scaled_size, scaled_size), Image.Resampling.NEAREST)
    if scaled_size == size_px:
        return image

    canvas = Image.new("RGB", (size_px, size_px), "white")
    offset = ((size_px - scaled_size) // 2, (size_px - scaled_size) // 2)
    canvas.paste(image, offset)
    return canvas


def make_a4_pdf_page(qr_image: Image.Image, label: str, dpi: int, qr_size_mm: float) -> Image.Image:
    page_w = mm_to_px(210.0, dpi)
    page_h = mm_to_px(297.0, dpi)
    qr_px = mm_to_px(qr_size_mm, dpi)
    page = Image.new("RGB", (page_w, page_h), "white")

    qr = qr_image.resize((qr_px, qr_px), Image.Resampling.NEAREST)
    x = (page_w - qr_px) // 2
    y = mm_to_px(35.0, dpi)
    page.paste(qr, (x, y))

    draw = ImageDraw.Draw(page)
    font = load_font(mm_to_px(24.0, dpi))
    text = str(label)
    bbox = draw.textbbox((0, 0), text, font=font)
    text_w = bbox[2] - bbox[0]
    text_h = bbox[3] - bbox[1]
    text_x = (page_w - text_w) // 2
    text_y = y + qr_px + mm_to_px(12.0, dpi)
    draw.text((text_x, text_y), text, fill="black", font=font)

    footer_font = load_font(mm_to_px(6.0, dpi))
    footer = f"Cabinet {label} - QR payload: {label}"
    footer_bbox = draw.textbbox((0, 0), footer, font=footer_font)
    footer_x = (page_w - (footer_bbox[2] - footer_bbox[0])) // 2
    footer_y = page_h - mm_to_px(12.0, dpi) - (footer_bbox[3] - footer_bbox[1])
    draw.text((footer_x, footer_y), footer, fill="black", font=footer_font)

    return page


def generate_labels(
    output_dir: Path,
    start: int,
    end: int,
    dpi: int,
    qr_size_mm: float,
    png_size_px: int,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    qr_px = mm_to_px(qr_size_mm, dpi)
    for cabinet_id in range(start, end + 1):
        payload = str(cabinet_id)
        stem = f"cabinet_{cabinet_id:02d}"
        preview_qr = make_qr_image(payload, png_size_px)
        preview_qr.save(output_dir / f"{stem}.png")

        print_qr = make_qr_image(payload, qr_px)
        page = make_a4_pdf_page(print_qr, payload, dpi, qr_size_mm)
        page.save(output_dir / f"{stem}.pdf", "PDF", resolution=float(dpi))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent / "output")
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=36)
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--qr-size-mm", type=float, default=190.0)
    parser.add_argument("--png-size-px", type=int, default=800)
    args = parser.parse_args()

    if args.start <= 0 or args.end < args.start:
        raise SystemExit("--start must be positive and --end must be >= --start")
    if args.dpi <= 0 or args.qr_size_mm <= 0 or args.png_size_px <= 0:
        raise SystemExit("--dpi, --qr-size-mm, and --png-size-px must be positive")

    generate_labels(
        args.output_dir,
        args.start,
        args.end,
        args.dpi,
        args.qr_size_mm,
        args.png_size_px,
    )
    print(f"Generated labels {args.start}..{args.end} in {args.output_dir}")


if __name__ == "__main__":
    main()
