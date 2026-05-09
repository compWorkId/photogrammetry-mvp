"""
main.py — Entry point for the Photogrammetry Pipeline.

Usage
-----
    python main.py [--image-dir PATH] [--output-dir PATH]
                   [--skip-reconstruction] [--skip-visualization]

Defaults
--------
    image_dir  = <project_root>/data/images
    output_dir = <project_root>/data/output
"""

import argparse
import logging
from pathlib import Path

from src.pipeline import run_pipeline
from src.utils import get_logger

# Project root is the directory this file lives in.
PROJECT_ROOT = Path(__file__).parent.resolve()

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Photogrammetry Pipeline — Structure-from-Motion via COLMAP + Open3D",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--image-dir",
        type=Path,
        default=PROJECT_ROOT / "data" / "images",
        help="Directory containing input images.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "data" / "output",
        help="Directory where reconstruction outputs are stored.",
    )
    parser.add_argument(
        "--skip-reconstruction",
        action="store_true",
        help="Skip COLMAP and go straight to visualization (sparse.ply must exist).",
    )
    parser.add_argument(
        "--skip-visualization",
        action="store_true",
        help="Stop after generating sparse.ply without opening Open3D.",
    )
    parser.add_argument(
        "--generate-mesh",
        action="store_true",
        help="Run Poisson Surface Reconstruction on the sparse point cloud.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable DEBUG-level logging.",
    )
    return parser


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = _build_parser().parse_args()

    logger = get_logger()
    if args.verbose:
        logger.setLevel(logging.DEBUG)

    logger.info("=================================================")
    logger.info("  Photogrammetry Pipeline")
    logger.info("=================================================")
    logger.info("  image_dir  : %s", args.image_dir)
    logger.info("  output_dir : %s", args.output_dir)
    logger.info("-------------------------------------------------")

    run_pipeline(
        image_dir=args.image_dir,
        output_dir=args.output_dir,
        skip_reconstruction=args.skip_reconstruction,
        skip_visualization=args.skip_visualization,
        generate_mesh=args.generate_mesh,
    )


if __name__ == "__main__":
    main()
