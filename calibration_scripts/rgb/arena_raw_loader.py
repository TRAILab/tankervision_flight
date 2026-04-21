#!/usr/bin/env python3
"""
arena_raw_loader.py
-------------------
Loads .raw images saved by arena_camera_node and returns numpy arrays
suitable for OpenCV camera calibration.

Filename format produced by the node:
    frame_{id}_{YYYYMMDD}_{HHMMSS}_{us}_{width}x{height}_{bpp}bpp.raw

Usage
-----
As a module:
    from arena_raw_loader import load_raw_images
    images = load_raw_images("/path/to/raw/files")

As a script:
    python arena_raw_loader.py --loc /path/to/raw/files [--rgb]

Arguments
---------
--loc   Directory containing .raw files  (overrides RAW_DIR below)
--rgb   Save debayered RGB PNGs into <loc>/rgb/ and return RGB arrays.
        Without this flag the function returns grayscale uint8 arrays,
        which is what most OpenCV calibration routines expect.
"""

import argparse
import os
import re
import sys
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

# -- Default location (used when running as a module without arguments) --------
RAW_DIR: str = "/home/atlas/tankervision_flight/home/atlas/captures/arena_raw/"

# -- Filename pattern ----------------------------------------------------------
# frame_{id}_{YYYYMMDD}_{HHMMSS}_{us}_{W}x{H}_{bpp}bpp.raw
_FNAME_RE = re.compile(
    r"frame_(?P<id>\d+)_"
    r"(?P<date>\d{8})_(?P<time>\d{6})_(?P<us>\d{6})_"
    r"(?P<width>\d+)x(?P<height>\d+)_"
    r"(?P<bpp>\d+)bpp\.raw$"
)

# -- Bayer pattern lookup ------------------------------------------------------
DEFAULT_PIXEL_FORMAT: str = "BayerRG8"

_BAYER_CODE = {
    "bayerrg8":    cv2.COLOR_BayerBG2BGR,
    "bayergb8":    cv2.COLOR_BayerGB2BGR,
    "bayerbg8":    cv2.COLOR_BayerRG2BGR,
    "bayergr8":    cv2.COLOR_BayerGR2BGR,
    "bayer_rggb8": cv2.COLOR_BayerBG2BGR,
    "bayer_gbrg8": cv2.COLOR_BayerGB2BGR,
    "bayer_bggr8": cv2.COLOR_BayerRG2BGR,
    "bayer_grbg8": cv2.COLOR_BayerGR2BGR,
    # 16-bit aliases (treated same -- we normalise to 8-bit before demosaic)
    "bayerrg16":   cv2.COLOR_BayerBG2BGR,
    "bayergb16":   cv2.COLOR_BayerGB2BGR,
    "bayerbg16":   cv2.COLOR_BayerRG2BGR,
    "bayergr16":   cv2.COLOR_BayerGR2BGR,
}

_BAYER_CODE_GRAY = {
    "bayerrg8":    cv2.COLOR_BayerBG2GRAY,
    "bayergb8":    cv2.COLOR_BayerGB2GRAY,
    "bayerbg8":    cv2.COLOR_BayerRG2GRAY,
    "bayergr8":    cv2.COLOR_BayerGR2GRAY,
    "bayer_rggb8": cv2.COLOR_BayerBG2GRAY,
    "bayer_gbrg8": cv2.COLOR_BayerGB2GRAY,
    "bayer_bggr8": cv2.COLOR_BayerRG2GRAY,
    "bayer_grbg8": cv2.COLOR_BayerGR2GRAY,
    "bayerrg16":   cv2.COLOR_BayerBG2GRAY,
    "bayergb16":   cv2.COLOR_BayerGB2GRAY,
    "bayerbg16":   cv2.COLOR_BayerRG2GRAY,
    "bayergr16":   cv2.COLOR_BayerGR2GRAY,
}

# -----------------------------------------------------------------------------

def _parse_filename(path: Path) -> Tuple[int, int, int, int]:
    """Extract (width, height, bpp, frame_id) from an arena .raw filename."""
    m = _FNAME_RE.match(path.name)
    if not m:
        raise ValueError(
            f"Filename does not match arena_camera_node pattern: {path.name}"
        )
    return (
        int(m.group("width")),
        int(m.group("height")),
        int(m.group("bpp")),
        int(m.group("id")),
    )


def _read_pixel_format(raw_path: Path) -> str:
    """
    Read pixel format from a sidecar <stem>.fmt file if present,
    otherwise return DEFAULT_PIXEL_FORMAT.
    """
    fmt_path = raw_path.with_suffix(".fmt")
    if fmt_path.exists():
        return fmt_path.read_text().strip()
    return DEFAULT_PIXEL_FORMAT


def _bayer_to_uint8(bayer: np.ndarray, bpp: int) -> np.ndarray:
    """
    Convert a raw Bayer plane (any bit depth) to a normalised uint8 plane
    ready for cv2 demosaicing, with CLAHE applied for contrast.
    """
    if bayer.dtype == np.uint8:
        b8 = bayer
    else:
        # Determine effective bit depth from actual data range
        actual_max = int(bayer.max())
        if actual_max <= 255:
            b8 = bayer.astype(np.uint8)
        else:
            effective_bits = int(np.ceil(np.log2(max(actual_max + 1, 2))))
            effective_bits = max(effective_bits, bpp)
            shift = max(effective_bits - 8, 0)
            b8 = (bayer >> shift).astype(np.uint8)

    # CLAHE improves tag corner detection on high-res industrial cameras
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return clahe.apply(b8)


def _decode_image(
    raw_bytes: bytes,
    width: int,
    height: int,
    bpp: int,
    pixel_format: str,
    as_rgb: bool,
) -> np.ndarray:
    """
    Decode raw bytes into a numpy array.

    Returns
    -------
    np.ndarray  shape (H, W)      if as_rgb=False  -> grayscale uint8
                shape (H, W, 3)   if as_rgb=True   -> RGB uint8
    """
    dtype = np.uint8 if bpp <= 8 else np.uint16

    # Truncate to exactly what we expect so frombuffer never raises
    expected_bytes = width * height * (2 if dtype == np.uint16 else 1)
    raw_bytes = raw_bytes[:expected_bytes]

    flat = np.frombuffer(raw_bytes, dtype=dtype)
    fmt_key = pixel_format.lower().replace(" ", "")

    # -- Mono formats ---------------------------------------------------------
    if fmt_key in ("mono8", "mono", "8"):
        img = flat.reshape((height, width))
        if bpp < 8:
            img = (img << (8 - bpp)).astype(np.uint8)
        if as_rgb:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        return img

    if fmt_key in ("mono16", "mono12", "mono10"):
        img = flat.reshape((height, width))
        img = (img >> (bpp - 8)).astype(np.uint8)
        if as_rgb:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        return img

    # -- Packed colour --------------------------------------------------------
    if fmt_key in ("rgb8",):
        img = flat.reshape((height, width, 3))
        return img if as_rgb else cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    if fmt_key in ("bgr8",):
        bgr = flat.reshape((height, width, 3))
        if as_rgb:
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # -- Bayer formats --------------------------------------------------------
    if fmt_key in _BAYER_CODE:
        bayer = flat.reshape((height, width))
        bayer8 = _bayer_to_uint8(bayer, bpp)
        if as_rgb:
            bgr = cv2.cvtColor(bayer8, _BAYER_CODE[fmt_key])
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return cv2.cvtColor(bayer8, _BAYER_CODE_GRAY[fmt_key])

    # -- Unknown: best-effort -------------------------------------------------
    print(
        f"[WARN] Unknown pixel format '{pixel_format}'. "
        "Returning raw single-channel uint8 image.",
        file=sys.stderr,
    )
    channels = max((bpp * width * height) // (8 * width * height), 1)
    return flat.astype(np.uint8).reshape((height, width * channels))


def load_raw_images(
    location: str = RAW_DIR,
    as_rgb: bool = False,
    save_rgb: bool = False,
) -> List[np.ndarray]:
    """
    Load all .raw files in *location* saved by arena_camera_node.

    Parameters
    ----------
    location : str
        Directory containing .raw files.
    as_rgb : bool
        If True, debayer/convert to RGB (uint8, HxWx3).
        If False (default), return grayscale uint8 (HxW).
    save_rgb : bool
        If True (and as_rgb=True), save PNG copies into <location>/rgb/.

    Returns
    -------
    List[np.ndarray]  sorted by frame ID.
    """
    loc = Path(location).resolve()
    if not loc.is_dir():
        raise FileNotFoundError(f"Directory not found: {loc}")

    raw_files = sorted(loc.glob("*.raw"))
    if not raw_files:
        print(f"[WARN] No .raw files found in {loc}", file=sys.stderr)
        return []

    def _sort_key(p: Path) -> Tuple[int, str]:
        try:
            _, _, _, fid = _parse_filename(p)
            return (fid, p.name)
        except ValueError:
            return (999999999, p.name)

    raw_files.sort(key=_sort_key)

    rgb_dir = loc / "rgb"
    if save_rgb and as_rgb:
        rgb_dir.mkdir(exist_ok=True)

    images: List[np.ndarray] = []

    for raw_path in raw_files:
        try:
            width, height, bpp, fid = _parse_filename(raw_path)
        except ValueError as e:
            print(f"[SKIP] {e}", file=sys.stderr)
            continue

        pixel_format = _read_pixel_format(raw_path)

        expected_bytes = width * height * ((bpp + 7) // 8)
        actual_bytes = raw_path.stat().st_size
        if actual_bytes != expected_bytes:
            print(
                f"[WARN] {raw_path.name}: expected {expected_bytes} bytes "
                f"but file is {actual_bytes} bytes -- attempting anyway.",
                file=sys.stderr,
            )

        raw_bytes = raw_path.read_bytes()
        img = _decode_image(raw_bytes, width, height, bpp, pixel_format, as_rgb)
        images.append(img)

        if save_rgb and as_rgb:
            out_path = rgb_dir / raw_path.with_suffix(".png").name
            cv2.imwrite(str(out_path), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        mode = "RGB" if as_rgb else "GRAY"
        print(
            f"[OK] {raw_path.name}  ->  {img.shape}  {img.dtype}  "
            f"fmt={pixel_format}  mode={mode}"
        )

    print(f"\nLoaded {len(images)} image(s) from {loc}")
    if save_rgb and as_rgb and images:
        print(f"RGB PNGs saved to {rgb_dir}")

    return images


# -- CLI ----------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Load arena_camera_node .raw images as numpy arrays.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--loc",
        default=RAW_DIR,
        help=f"Directory containing .raw files (default: {RAW_DIR})",
    )
    p.add_argument(
        "--rgb",
        action="store_true",
        help="Debayer to RGB uint8 (H x W x 3) and save PNGs into <loc>/rgb/.",
    )
    return p


if __name__ == "__main__":
    args = _build_parser().parse_args()
    imgs = load_raw_images(
        location=args.loc,
        as_rgb=args.rgb,
        save_rgb=args.rgb,
    )
    if imgs:
        shapes = {img.shape for img in imgs}
        print(f"\nUnique shapes: {shapes}")
        print("Ready for cv2.calibrateCamera / cv2.findChessboardCorners")
    else:
        print("No images loaded.")
        sys.exit(1)