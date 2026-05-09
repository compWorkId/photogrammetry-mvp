"""
utils.py — Shared utilities for the photogrammetry pipeline.

Responsibilities:
  - Configure a consistent logger used across all modules
  - Verify COLMAP is available on PATH
  - Discover image files inside a directory
"""

import logging
import shutil
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

def get_logger(name: str = "photogrammetry") -> logging.Logger:
    """Return a module-level logger with a human-readable format."""
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(
            logging.Formatter(
                fmt="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
                datefmt="%H:%M:%S",
            )
        )
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


# ---------------------------------------------------------------------------
# COLMAP availability check
# ---------------------------------------------------------------------------

def check_colmap() -> str:
    """
    Verify that COLMAP is installed and on the system PATH.

    Returns
    -------
    str
        Absolute path to the colmap executable.

    Raises
    ------
    SystemExit
        If colmap cannot be found, a helpful error message is printed and
        the process exits with code 1.
    """
    colmap_path = shutil.which("colmap")
    if colmap_path is None:
        logger = get_logger()
        logger.error(
            "COLMAP not found on PATH.\n"
            "Install it with:\n"
            "  brew install colmap          # macOS via Homebrew\n"
            "  sudo apt install colmap      # Ubuntu / Debian\n"
            "Then re-run this script."
        )
        sys.exit(1)
    return colmap_path


# ---------------------------------------------------------------------------
# Image discovery
# ---------------------------------------------------------------------------

SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif"}


def discover_images(image_dir: Path) -> list[Path]:
    """
    Return a sorted list of image paths found under *image_dir*.

    Parameters
    ----------
    image_dir : Path
        Directory to search (non-recursive).

    Returns
    -------
    list[Path]
        Sorted list of image file paths.

    Raises
    ------
    SystemExit
        If the directory does not exist or contains fewer than 2 images.
    """
    logger = get_logger()

    if not image_dir.exists():
        logger.error("Image directory not found: %s", image_dir)
        sys.exit(1)

    images = sorted(
        p for p in image_dir.iterdir()
        if p.is_file() and p.suffix.lower() in SUPPORTED_EXTENSIONS
    )

    logger.info("Found %d image(s) in %s", len(images), image_dir)

    if len(images) < 2:
        logger.error(
            "At least 2 overlapping images are required for reconstruction. "
            "Found %d.",
            len(images),
        )
        sys.exit(1)

    return images
