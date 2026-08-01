"""
reconstruction.py — Orchestrates COLMAP for Structure-from-Motion.

Pipeline steps
--------------
1. Feature extraction   (colmap feature_extractor)
2. Feature matching     (colmap exhaustive_matcher, or sequential_matcher
                          above EXHAUSTIVE_MATCH_LIMIT images)
3. Sparse mapping       (colmap mapper)
4. PLY export           (colmap model_converter)

All intermediate data lives in data/output/colmap_workspace/.
The final sparse point cloud is exported to data/output/sparse.ply.
"""

import subprocess
import sys
from pathlib import Path

from src.utils import check_colmap, discover_images, get_logger

logger = get_logger(__name__)

# Exhaustive matching is O(n^2) in image count. Past this many images it
# stops being worth it — switch to sequential matching, which only compares
# each image against its nearby neighbors in capture order. This assumes
# filenames sort in roughly the order the images were captured (true for
# camera-numbered sequences like a walkaround or drone orbit).
EXHAUSTIVE_MATCH_LIMIT = 150


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _run(cmd: list[str], step_name: str) -> None:
    """
    Execute a shell command, streaming stdout/stderr to the terminal.
    Raises SystemExit on non-zero return code.
    """
    logger.info(">>> %s", " ".join(cmd))
    result = subprocess.run(cmd, text=True)
    if result.returncode != 0:
        logger.error("COLMAP step '%s' failed (exit code %d).", step_name, result.returncode)
        sys.exit(result.returncode)


def _pick_best_model(components: list[Path]) -> Path:
    """
    Select the reconstruction component with the most registered images.
    Uses the size of images.bin/images.txt as a proxy for registered cameras.
    """
    def _score(p: Path) -> int:
        for fname in ("images.bin", "images.txt"):
            candidate = p / fname
            if candidate.exists():
                return candidate.stat().st_size
        return 0

    return max(components, key=_score)


def _log_reconstruction_stats(sparse_dir: Path) -> None:
    """
    Parse COLMAP text outputs to print a human-readable quality summary
    (cameras registered, 3-D points, points-per-camera ratio).

    Note: only works after model_converter has exported text format,
    OR if the mapper wrote text format directly.  We handle both bin and txt.
    """
    components = [p for p in sparse_dir.iterdir() if p.is_dir()]
    if not components:
        return

    total_images = 0
    total_points = 0

    for comp in sorted(components):
        images_txt = comp / "images.txt"
        if images_txt.exists():
            lines = [l for l in images_txt.read_text().splitlines()
                     if l.strip() and not l.startswith("#")]
            total_images += len(lines) // 2   # 2 lines per image in COLMAP format

        pts_txt = comp / "points3D.txt"
        if pts_txt.exists():
            lines = [l for l in pts_txt.read_text().splitlines()
                     if l.strip() and not l.startswith("#")]
            total_points += len(lines)

    if total_images == 0 and total_points == 0:
        # Binary format — we can't easily count without pycolmap; skip.
        logger.info("Quality stats unavailable (binary model format).")
        return

    ratio = total_points / total_images if total_images else 0
    quality = (
        "Excellent" if ratio > 1000
        else "Good"    if ratio > 300
        else "Sparse — try more overlapping images or a less reflective subject"
    )

    logger.info("─" * 52)
    logger.info("Reconstruction quality summary:")
    logger.info("  Components   : %d", len(components))
    logger.info("  Cameras reg. : %d", total_images)
    logger.info("  3-D points   : %d", total_points)
    logger.info("  Pts/camera   : %.0f  →  %s", ratio, quality)
    logger.info("─" * 52)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def run_reconstruction(image_dir: Path, output_dir: Path) -> Path:
    """
    Run the full COLMAP automatic reconstruction pipeline.

    Parameters
    ----------
    image_dir : Path
        Directory containing the input images.
    output_dir : Path
        Root output directory.  Intermediate files go into a subdirectory;
        the final PLY is written directly into *output_dir*.

    Returns
    -------
    Path
        Path to the exported sparse.ply point-cloud file.
    """
    colmap = check_colmap()

    # ------------------------------------------------------------------
    # Directory layout
    # ------------------------------------------------------------------
    workspace  = output_dir / "colmap_workspace"
    database   = workspace / "database.db"
    sparse_dir = workspace / "sparse"
    ply_path   = output_dir / "sparse.ply"

    workspace.mkdir(parents=True, exist_ok=True)
    sparse_dir.mkdir(parents=True, exist_ok=True)

    logger.info("=== Step 1/4 — Feature extraction ===")
    _run(
        [
            colmap, "feature_extractor",
            "--database_path", str(database),
            "--image_path",    str(image_dir),
            "--ImageReader.single_camera", "1",
            # Default 8k features for balanced speed/detail
            "--SiftExtraction.max_num_features", "8192",
        ],
        "feature_extractor",
    )

    image_count = len(discover_images(image_dir))
    if image_count > EXHAUSTIVE_MATCH_LIMIT:
        logger.info("=== Step 2/4 — Sequential feature matching (%d images > %d, exhaustive is O(n^2)) ===",
                    image_count, EXHAUSTIVE_MATCH_LIMIT)
        _run(
            [
                colmap, "sequential_matcher",
                "--database_path", str(database),
            ],
            "sequential_matcher",
        )
    else:
        logger.info("=== Step 2/4 — Exhaustive feature matching ===")
        _run(
            [
                colmap, "exhaustive_matcher",
                "--database_path", str(database),
            ],
            "exhaustive_matcher",
        )

    logger.info("=== Step 3/4 — Sparse mapping (mapper) ===")
    _run(
        [
            colmap, "mapper",
            "--database_path", str(database),
            "--image_path",    str(image_dir),
            "--output_path",   str(sparse_dir),
            # Back to standard defaults
        ],
        "mapper",
    )


    # ------------------------------------------------------------------
    # Quality summary (works when mapper writes text format)
    # ------------------------------------------------------------------
    _log_reconstruction_stats(sparse_dir)

    # COLMAP writes one sub-folder per reconstructed component (0/, 1/, …).
    # We use the largest one (most registered images).
    components = sorted(sparse_dir.iterdir(), key=lambda p: p.name)
    if not components:
        logger.error(
            "Mapper produced no reconstruction components. "
            "Check that your images overlap sufficiently."
        )
        sys.exit(1)

    best_model = _pick_best_model(components)
    logger.info("Using reconstruction component: %s", best_model)

    logger.info("=== Step 4/4 — Exporting results (PLY + TXT) ===")
    # Export PLY for the point cloud
    _run(
        [
            colmap, "model_converter",
            "--input_path",  str(best_model),
            "--output_path", str(ply_path),
            "--output_type", "PLY",
        ],
        "model_converter_ply",
    )
    
    # Export TXT for the camera positions (easier to parse)
    _run(
        [
            colmap, "model_converter",
            "--input_path",  str(best_model),
            "--output_path", str(best_model), # Output text files back into the model dir
            "--output_type", "TXT",
        ],
        "model_converter_txt",
    )


    logger.info("Sparse point cloud saved to: %s", ply_path)
    return ply_path, best_model
