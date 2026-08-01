"""
pipeline.py — Ties reconstruction and visualization into one callable unit.

This module is intentionally thin. All heavy lifting is delegated to
reconstruction.py and visualization.py.
"""

from pathlib import Path

from src.reconstruction import run_reconstruction
from src.utils import discover_images, get_logger
from src.visualization import visualize_point_cloud

logger = get_logger(__name__)


def run_pipeline(
    image_dir: Path,
    output_dir: Path,
    *,
    skip_reconstruction: bool = False,
    skip_visualization: bool = False,
    generate_bim: bool = False,
) -> None:
    """
    Execute the full photogrammetry pipeline.

    Parameters
    ----------
    image_dir : Path
        Folder that contains the input images.
    output_dir : Path
        Folder where all output artefacts are written.
    skip_reconstruction : bool
        If True, skip COLMAP and go straight to visualisation
        (useful when a sparse.ply already exists).
    skip_visualization : bool
        If True, exit after generating the PLY without opening Open3D.
    generate_bim : bool
        If True, export a typed IFC model (model.ifc) from the sparse point cloud.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    ply_path = output_dir / "sparse.ply"
    model_dir = output_dir / "colmap_workspace" / "sparse" / "0"

    if not skip_reconstruction:
        # Sanity-check the image directory before calling COLMAP
        discover_images(image_dir)
        ply_path, model_dir = run_reconstruction(image_dir, output_dir)
    else:
        logger.info("Skipping reconstruction — using existing PLY: %s", ply_path)
        if not ply_path.exists():
            logger.error("No existing sparse.ply found. Run without --skip-reconstruction first.")
            import sys; sys.exit(1)
        
        # INSTANT FIX: If camera metadata is missing, generate it now (takes 2 seconds)
        if not (model_dir / "images.txt").exists():
            logger.info("Generating camera metadata for Studio Viewer...")
            from src.reconstruction import _run, check_colmap
            _run([
                check_colmap(), "model_converter",
                "--input_path", str(model_dir),
                "--output_path", str(model_dir),
                "--output_type", "TXT"
            ], "instant_metadata_export")

    if generate_bim:
        from src.bim_export import export_ifc
        export_ifc(ply_path, output_dir, image_dir=image_dir, model_dir=model_dir)

    if not skip_visualization:
        visualize_point_cloud(ply_path, model_dir)
    else:
        logger.info("Skipping visualization (--skip-visualization flag set).")
        logger.info("Pipeline complete. Output: %s", ply_path)
