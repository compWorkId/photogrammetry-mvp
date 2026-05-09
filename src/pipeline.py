"""
pipeline.py — Ties reconstruction and visualization into one callable unit.

This module is intentionally thin.  All heavy lifting is delegated to
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
    generate_mesh: bool = False,
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

    # Optional: Poisson Surface Reconstruction
    mesh_path = output_dir / "mesh.ply"
    if generate_mesh:
        from src.mesh import generate_mesh as _gen_mesh
        mesh_path = _gen_mesh(ply_path, output_dir)

    if not skip_visualization:
        visualize_point_cloud(ply_path, model_dir, mesh_path if mesh_path.exists() else None)
    else:
        logger.info("Skipping visualization (--skip-visualization flag set).")
        logger.info("Pipeline complete.  Output: %s", ply_path)
