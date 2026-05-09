"""
mesh.py — Poisson Surface Reconstruction from a sparse point cloud.

Steps:
  1. Load sparse PLY
  2. Estimate normals (required for Poisson)
  3. Run Poisson reconstruction
  4. Trim low-density vertices (cleans up holes at the edges)
  5. Save mesh as mesh.ply
"""

import sys
from pathlib import Path

from src.utils import get_logger

logger = get_logger(__name__)


def generate_mesh(ply_path: Path, output_dir: Path) -> Path:
    """
    Run Poisson Surface Reconstruction on the sparse point cloud.

    Parameters
    ----------
    ply_path   : Path to the input sparse.ply
    output_dir : Directory to write mesh.ply into

    Returns
    -------
    Path to the generated mesh.ply
    """
    try:
        import open3d as o3d
        import numpy as np
    except ImportError:
        logger.error("open3d is not installed. Run: pip install open3d")
        sys.exit(1)

    if not ply_path.exists():
        logger.error("PLY not found: %s", ply_path)
        sys.exit(1)

    mesh_path = output_dir / "mesh.ply"

    logger.info("=== Mesh Generation ===")
    logger.info("Loading point cloud: %s", ply_path)
    pcd = o3d.io.read_point_cloud(str(ply_path))

    n_points = len(pcd.points)
    logger.info("Loaded %d points.", n_points)

    if n_points < 100:
        logger.error("Point cloud too sparse for meshing (only %d points).", n_points)
        sys.exit(1)

    # ----------------------------------------------------------------
    # Step 1: Estimate Normals
    # Poisson REQUIRES normals to know which way surfaces face.
    # ----------------------------------------------------------------
    logger.info("Estimating surface normals...")
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    # Orient normals consistently (towards camera origin)
    pcd.orient_normals_consistent_tangent_plane(k=15)
    logger.info("Normals estimated.")

    # ----------------------------------------------------------------
    # Step 2: Poisson Surface Reconstruction
    # depth=9 is a good balance: detail vs noise for sparse clouds.
    # Lower depth (7-8) = faster, blockier. Higher (10-11) = slower, finer.
    # ----------------------------------------------------------------
    logger.info("Running Poisson reconstruction (depth=9)... this may take ~30 seconds.")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9, width=0, scale=1.1, linear_fit=False
    )
    logger.info(
        "Mesh created: %d vertices, %d triangles.",
        len(mesh.vertices),
        len(mesh.triangles),
    )

    # ----------------------------------------------------------------
    # Step 3: Trim low-density vertices
    # Poisson creates a "watertight" surface that extends past the
    # actual data into empty space. Trimming removes that outer shell.
    # ----------------------------------------------------------------
    import numpy as np
    logger.info("Trimming low-density artifacts...")
    densities = np.asarray(densities)
    # Remove vertices below the 5th percentile density
    vertices_to_remove = densities < np.quantile(densities, 0.05)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    logger.info(
        "After trim: %d vertices, %d triangles.",
        len(mesh.vertices),
        len(mesh.triangles),
    )

    # ----------------------------------------------------------------
    # Step 4: Compute per-vertex normals for rendering
    # ----------------------------------------------------------------
    mesh.compute_vertex_normals()

    # ----------------------------------------------------------------
    # Step 5: Save
    # ----------------------------------------------------------------
    o3d.io.write_triangle_mesh(str(mesh_path), mesh)
    logger.info("Mesh saved to: %s", mesh_path)

    return mesh_path
