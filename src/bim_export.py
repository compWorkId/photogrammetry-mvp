"""
bim_export.py — Converts a COLMAP sparse point cloud into a typed IFC model.

Approach
--------
1. If GPS EXIF is available on the source photos, calibrate real-world
   scale (estimate_scale_from_gps) and the true vertical axis (
   estimate_up_from_gps) directly from GPS — far more reliable than
   inferring either from plane geometry. Without GPS, both fall back to
   geometric estimates (plane-normal covariance for up; coordinates stay
   in COLMAP's arbitrary nominal units, unscaled).
2. Iteratively RANSAC-fit the dominant flat planes in the cloud (walls,
   ground/pavement, roof faces).
3. Each plane is classified by its angle to the vertical axis (wall / slab
   / roof) and exported as a properly typed, correctly placed IFC element
   (IfcWall / IfcSlab / IfcRoof) inside a real spatial hierarchy
   (Project -> Site -> Building -> Storey). A wall plane's occupied
   footprint is split into 8-connected components so real, physically
   separate facade regions (e.g. two bands split by a floor line, or two
   unrelated walls RANSAC coincidentally merged) each become their own
   element instead of one being silently merged into or dropped in favor
   of the other.
4. The lowest slab-classified plane is labeled the ground (BASESLAB); any
   other flat plane (e.g. an elevated deck) is labeled FLOOR, not ground.
5. If available, GPS anchors IfcSite's RefLatitude/RefLongitude/
   RefElevation plus a minimal IfcMapConversion (see _add_georeferencing).
6. Leftover non-planar points (vegetation, clutter, background) are
   clustered and exported as generic IfcBuildingElementProxy convex-hull
   meshes, so nothing is silently dropped — but nothing here claims
   semantic meaning beyond "unclassified context." The full cleaned cloud
   is also exported as a color-matched point-marker reference envelope
   (_build_reference_envelope), since plane classification is deliberately
   conservative and can under-detect real architecture.
7. A per-run HTML summary (report.html) is written alongside model.ifc —
   see _write_summary_report.

Known limitations (see README)
-------------------------------
- This is a single-sided exterior scan: real wall/slab thickness cannot be
  measured, only assumed (WALL_THICKNESS / SLAB_THICKNESS below).
- The sparse SfM cloud (COLMAP's SfM keypoints, not dense MVS/LiDAR) is too
  sparse to resolve windows, doors, or trim as separate elements.
- Scale and up-axis calibration both require GPS EXIF on the source photos
  (typical of drone shoots). Without it, coordinates stay in COLMAP's
  arbitrary nominal units and up-axis falls back to plane geometry, which
  is less reliable for structures with unusual surfaces.
- Georeferencing anchors the model's local origin to the mean GPS position;
  it is not a survey-grade projected-CRS transform (no true-north bearing).
"""

import sys
from pathlib import Path

import numpy as np

import ifcopenshell
import ifcopenshell.api
import ifcopenshell.api.aggregate
import ifcopenshell.api.context
import ifcopenshell.api.geometry
import ifcopenshell.api.root
import ifcopenshell.api.spatial
import ifcopenshell.api.style
import ifcopenshell.api.unit

try:
    import open3d as o3d
except ImportError:
    o3d = None

from src.utils import get_logger

logger = get_logger(__name__)

# ---------------------------------------------------------------------------
# Tunable parameters
# ---------------------------------------------------------------------------
RANSAC_SEED = 42                   # fixed seed — deterministic runs, no flip-flopping classification
RANSAC_DISTANCE_THRESHOLD = 0.05   # meters (nominal) — plane-fit inlier tolerance
RANSAC_N = 3
RANSAC_ITERATIONS = 2000
MIN_PLANE_FRACTION = 0.006         # stop once the best remaining plane is < 0.6% of total points
MIN_PLANE_ABSOLUTE = 200           # ...or fewer than this many points, whichever is larger
MAX_PLANES = 18                    # raised so secondary roof faces (e.g. a second gable) aren't cut off

# A raw Poisson-reconstructed "reference envelope" is exported alongside the
# typed elements below, so the model's outer shape always matches the scan
# even where plane classification under-detects real architecture.
ENVELOPE_VOXEL_SIZE = 0.09         # downsample target for the envelope's point markers
ENVELOPE_MARKER_SIZE = 0.14        # cube edge > voxel spacing, so markers overlap into a solid-looking skin
ENVELOPE_COLOR_BUCKETS = 64        # k-means color groups — 16 was too coarse, flattened the window/brick contrast
                                    # that's what actually makes the shape read as a building, not just geometry


# Real scan-to-BIM tools (Revit+ReCap, etc.) never alpha-blend the point
# cloud and the solid model into one composite view — the point cloud is a
# reference backdrop you view separately, and typed elements are fully
# opaque. Trying to render both through each other fights the renderer
# (Eevee has no reliable order-independent transparency) for no real gain.
WALL_TRANSPARENCY = 0.0
SLAB_TRANSPARENCY = 0.0

SLAB_ANGLE_THRESHOLD = 25.0        # deg from vertical axis -> horizontal => slab
WALL_ANGLE_THRESHOLD = 65.0        # deg from vertical axis -> vertical => wall
                                    # in between => sloped roof face

MIN_ELEMENT_DIM = 0.3              # meters — degenerate planes below this are dropped
# Points per m^2 of footprint, in real meters (calibrated against GPS-scaled
# drone datasets, our actual demo target) — below this, a plane is
# thinly-supported RANSAC noise stretched across a wide area, not a real
# coherent surface. Fixed, not spacing-derived: transferring a density
# calibration between datasets with different absolute scale (e.g. South
# Building's unverified nominal units vs. a GPS-scaled real-meters cloud)
# doesn't hold up — points/m^2 scales with the square of any scale error.
MIN_POINT_DENSITY = 8.0

WALL_THICKNESS = 0.30              # nominal, unmeasured (single-sided facade scan)
WALL_CELL_SIZE = 0.4               # meters — occupancy-grid cell size for masked wall construction
MIN_WALL_COVERAGE = 0.15           # below this fraction of occupied cells, there's not enough real structure
SLAB_THICKNESS = 0.15
ROOF_THICKNESS = 0.25

DBSCAN_EPS = 0.15
DBSCAN_MIN_POINTS = 30
MIN_CLUSTER_POINTS_FOR_PROXY = 80

# Statistical outlier removal only catches points that are locally sparse
# relative to their own neighborhood — a tight little cluster of bad points
# (e.g. SfM mis-triangulating sky/cloud texture as geometry) passes right
# through it, since each point in the bad cluster has plenty of nearby
# neighbors. This second pass clusters the whole cloud and drops any
# cluster too small, relative to the largest one, to plausibly be part of
# the real structure.
MAIN_CLUSTER_DBSCAN_EPS = 0.2
MAIN_CLUSTER_DBSCAN_MIN_POINTS = 8
MAIN_CLUSTER_MIN_FRACTION = 0.01

COPLANAR_MERGE_ANGLE = 10.0                        # deg — normals within this angle are "the same plane"
COPLANAR_MERGE_OFFSET = RANSAC_DISTANCE_THRESHOLD * 4  # meters — perpendicular gap within this is "the same surface"

# All the geometry constants above were originally hand-tuned against one
# dataset's point spacing. Rather than re-guess them for every new dataset,
# they're re-derived at runtime as multiples of the point cloud's own
# median nearest-neighbor spacing — a density measure that scales
# naturally with the data. The multipliers below are calibrated so that a
# cloud with the same spacing as the dataset the constants above were
# originally tuned on reproduces those exact values (a consistency check,
# not a coincidence).
SPACING_TO_RANSAC_THRESHOLD = 8.0
SPACING_TO_WALL_CELL_SIZE = 65.0
SPACING_TO_ENVELOPE_VOXEL_SIZE = 14.5
SPACING_TO_ENVELOPE_MARKER_SIZE = 22.5
# Density thresholds (MIN_POINT_DENSITY, GABLE_MIN_DENSITY) are deliberately
# NOT derived here — see their definitions below for why.


def _characteristic_spacing(pcd) -> float:
    """Median nearest-neighbor point spacing, in the point cloud's own units."""
    dists = np.asarray(pcd.compute_nearest_neighbor_distance())
    return float(np.median(dists))


def _apply_adaptive_thresholds(spacing: float) -> None:
    """
    Re-derive the density/size-dependent geometry constants from this
    dataset's own point spacing, so they scale with any dataset's density
    instead of needing hand-tuning every time (see the module-level
    SPACING_TO_* constants above for the calibration).

    Rebinds the module globals for the current process — safe here because
    export_ifc() is the single entry point and there's no concurrent/
    multi-dataset use within one run.
    """
    global RANSAC_DISTANCE_THRESHOLD, COPLANAR_MERGE_OFFSET
    global WALL_CELL_SIZE, ENVELOPE_VOXEL_SIZE, ENVELOPE_MARKER_SIZE, _CUBE_LOCAL_OFFSETS

    RANSAC_DISTANCE_THRESHOLD = spacing * SPACING_TO_RANSAC_THRESHOLD
    COPLANAR_MERGE_OFFSET = RANSAC_DISTANCE_THRESHOLD * 4
    WALL_CELL_SIZE = spacing * SPACING_TO_WALL_CELL_SIZE
    ENVELOPE_VOXEL_SIZE = spacing * SPACING_TO_ENVELOPE_VOXEL_SIZE
    ENVELOPE_MARKER_SIZE = spacing * SPACING_TO_ENVELOPE_MARKER_SIZE
    _CUBE_LOCAL_OFFSETS = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1,  1], [1, -1,  1], [1, 1,  1], [-1, 1,  1],
    ], dtype=float) * (ENVELOPE_MARKER_SIZE / 2.0)
    # MIN_POINT_DENSITY / GABLE_MIN_DENSITY intentionally not touched here —
    # fixed, real-meter constants calibrated against the tower dataset (see
    # their definitions), not derived from spacing.


# ---------------------------------------------------------------------------
# Point cloud -> plane segmentation
# ---------------------------------------------------------------------------

def _isolate_main_structure(pcd):
    """Drop DBSCAN clusters too small, relative to the largest, to be real structure."""
    o3d.utility.random.seed(RANSAC_SEED)
    labels = np.array(pcd.cluster_dbscan(eps=MAIN_CLUSTER_DBSCAN_EPS, min_points=MAIN_CLUSTER_DBSCAN_MIN_POINTS))
    if labels.max(initial=-1) < 0:
        return pcd  # nothing formed a cluster; leave as-is rather than dropping everything

    unique, counts = np.unique(labels[labels >= 0], return_counts=True)
    largest = counts.max()
    keep_labels = set(unique[counts >= MAIN_CLUSTER_MIN_FRACTION * largest].tolist())
    keep_mask = np.fromiter((label in keep_labels for label in labels), dtype=bool, count=len(labels))
    return pcd.select_by_index(np.where(keep_mask)[0])


def _load_cleaned_cloud(ply_path: Path):
    pcd = o3d.io.read_point_cloud(str(ply_path))
    cleaned, _ = pcd.remove_statistical_outlier(nb_neighbors=35, std_ratio=1.5)
    cleaned = _isolate_main_structure(cleaned)
    return cleaned


def segment_planes(pcd):
    """
    Iteratively RANSAC-fit dominant planes, largest first.

    Returns
    -------
    (planes, leftover)
        planes   : list of dicts with normal/centroid/points/count
        leftover : the remaining non-planar point cloud
    """
    o3d.utility.random.seed(RANSAC_SEED)

    total = len(pcd.points)
    min_inliers = max(MIN_PLANE_ABSOLUTE, int(MIN_PLANE_FRACTION * total))

    working = pcd
    planes = []
    for _ in range(MAX_PLANES):
        if len(working.points) < min_inliers:
            break
        plane_model, inlier_idx = working.segment_plane(
            distance_threshold=RANSAC_DISTANCE_THRESHOLD,
            ransac_n=RANSAC_N,
            num_iterations=RANSAC_ITERATIONS,
        )
        if len(inlier_idx) < min_inliers:
            break

        a, b, c, _d = plane_model
        normal = np.array([a, b, c])
        normal = normal / np.linalg.norm(normal)
        inlier_points = np.asarray(working.points)[inlier_idx]

        planes.append({
            "normal": normal,
            "centroid": inlier_points.mean(axis=0),
            "points": inlier_points,
            "count": len(inlier_idx),
        })
        working = working.select_by_index(inlier_idx, invert=True)

    return planes, working


def _plane_radius(points: np.ndarray, centroid: np.ndarray) -> float:
    """Farthest distance from centroid to any of the plane's own points — a rough spatial reach."""
    return float(np.max(np.linalg.norm(points - centroid, axis=1))) if len(points) else 0.0


def merge_coplanar_planes(planes: list) -> list:
    """
    Merge planes that are really the same physical surface found twice.

    RANSAC segments greedily: a large, slightly noisy/curved surface (e.g. a
    brick plaza) can get "onion-peeled" into several near-parallel planes at
    slightly different offsets instead of one. Planes whose normals agree
    within COPLANAR_MERGE_ANGLE and whose perpendicular offset is within
    COPLANAR_MERGE_OFFSET are combined into a single plane before any IFC
    element is built.

    Coplanar isn't the same as "the same surface", though: two genuinely
    separate, distant walls can share both an orientation and an offset by
    coincidence (e.g. opposite sides of a symmetric structure). Merging
    those produces one nonsensical oversized plane spanning empty space
    between them. So a plane is only merged if its footprint is actually
    close to the other's — within their combined spatial reach — not just
    coplanar in the abstract.
    """
    ordered = sorted(planes, key=lambda p: -p["count"])
    radii = [_plane_radius(p["points"], p["centroid"]) for p in ordered]
    used = [False] * len(ordered)
    merged = []

    for i, p in enumerate(ordered):
        if used[i]:
            continue
        used[i] = True
        group = [p["points"]]
        for j in range(i + 1, len(ordered)):
            if used[j]:
                continue
            q = ordered[j]
            if _angle_from_axis(p["normal"], q["normal"]) > COPLANAR_MERGE_ANGLE:
                continue
            delta = q["centroid"] - p["centroid"]
            offset = abs(np.dot(delta, p["normal"]))
            if offset > COPLANAR_MERGE_OFFSET:
                continue
            inplane_dist = float(np.linalg.norm(delta - np.dot(delta, p["normal"]) * p["normal"]))
            if inplane_dist > radii[i] + radii[j] + COPLANAR_MERGE_OFFSET:
                continue
            group.append(q["points"])
            used[j] = True

        pts = np.vstack(group)
        merged.append({
            "normal": p["normal"],
            "centroid": pts.mean(axis=0),
            "points": pts,
            "count": len(pts),
        })

    return merged


# ---------------------------------------------------------------------------
# Vertical-axis estimation (COLMAP has no gravity reference)
# ---------------------------------------------------------------------------

def _angle_from_axis(normal: np.ndarray, axis: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip(abs(np.dot(normal, axis)), 0.0, 1.0))))


def estimate_up_axis(planes: list) -> np.ndarray:
    """
    Estimate the true vertical axis from the set of dominant plane normals.

    Wall normals are horizontal and (for a building with more than one
    facade) span a 2-D horizontal subspace. The direction orthogonal to
    that subspace — the eigenvector of the smallest eigenvalue of the
    inlier-weighted normal covariance — is the vertical axis.
    """
    normals = np.array([p["normal"] for p in planes])
    weights = np.array([p["count"] for p in planes], dtype=float)
    moment = (normals * weights[:, None]).T @ normals
    eigvals, eigvecs = np.linalg.eigh(moment)
    up = eigvecs[:, 0]  # smallest eigenvalue -> axis least represented among plane normals
    return up / np.linalg.norm(up)


def resolve_up_sign(up: np.ndarray, all_points: np.ndarray, planes: list) -> np.ndarray:
    """
    Flip 'up' so it points from ground toward sky.

    Heuristic: the largest near-horizontal plane is assumed to be ground /
    pavement. If its height along the current 'up' guess is above the
    median height of the whole cloud, the axis is inverted.
    """
    horizontal = [p for p in planes if _angle_from_axis(p["normal"], up) < SLAB_ANGLE_THRESHOLD]
    if not horizontal:
        return up
    ground = max(horizontal, key=lambda p: p["count"])
    ground_height = float(np.dot(ground["centroid"], up))
    median_height = float(np.median(all_points @ up))
    return -up if ground_height > median_height else up


GPS_UP_MIN_IMAGES = 20      # need at least this many matched GPS+pose pairs to trust the fit
GPS_UP_MIN_R2 = 0.8         # below this fit quality, the correlation isn't trustworthy — fall back
EARTH_RADIUS_M = 6371000.0


def _parse_camera_positions(images_txt: Path) -> dict:
    """Camera centers (world coords) from a COLMAP images.txt, keyed by image filename."""
    if not images_txt.exists():
        return {}
    lines = [l for l in images_txt.read_text().splitlines() if l.strip() and not l.startswith("#")]
    cameras = {}
    for i in range(0, len(lines), 2):
        parts = lines[i].split()
        qw, qx, qy, qz = map(float, parts[1:5])
        tx, ty, tz = map(float, parts[5:8])
        rot = np.array([
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ])
        cameras[parts[9]] = -rot.T @ np.array([tx, ty, tz])
    return cameras


def _read_gps(path: Path):
    """(lat, lon, alt) in decimal degrees / meters from a photo's EXIF, or None."""
    try:
        from PIL import Image
        from PIL.ExifTags import TAGS
    except ImportError:
        return None
    try:
        exif = Image.open(path)._getexif()
    except Exception:
        return None
    if not exif:
        return None
    gps_info = next((v for k, v in exif.items() if TAGS.get(k) == "GPSInfo"), None)
    if not gps_info:
        return None

    def _dms_to_dd(dms, ref):
        dd = float(dms[0]) + float(dms[1]) / 60 + float(dms[2]) / 3600
        return -dd if ref in ("S", "W") else dd

    lat_dms, lat_ref = gps_info.get(2), gps_info.get(1)
    lon_dms, lon_ref = gps_info.get(4), gps_info.get(3)
    alt = gps_info.get(6)
    if lat_dms is None or lon_dms is None or alt is None:
        return None
    lat = _dms_to_dd(lat_dms, lat_ref)
    lon = _dms_to_dd(lon_dms, lon_ref)
    return lat, lon, float(alt)


def _haversine_distance(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance in meters between two lat/lon points."""
    phi1, phi2 = np.radians(lat1), np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    a = np.sin(dphi / 2) ** 2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2) ** 2
    return float(2 * EARTH_RADIUS_M * np.arcsin(np.sqrt(a)))


def estimate_up_from_gps(image_dir: Path, model_dir: Path):
    """
    Determine the true vertical axis from drone GPS altitude (EXIF), not
    plane geometry — for datasets with real GPS data, this is far more
    reliable than inferring "up" from wall normals, which breaks down when
    the dominant RANSAC plane isn't a clean single surface (e.g. a scaffold
    or lattice that only loosely satisfies the plane tolerance).

    Fits camera_altitude ~= a*X + b*Y + c*Z (least squares) using each
    image's EXIF GPS altitude against its reconstructed COLMAP camera
    position. The fitted direction (a, b, c) is the axis altitude increases
    along fastest — i.e. up.

    Returns
    -------
    (up_unit_vector, r_squared) or None if there isn't enough GPS data or
    the fit is too weak to trust (camera altitude should correlate almost
    perfectly with position for a rigid SfM reconstruction; a poor fit
    means something else is wrong and this method shouldn't be trusted).
    """
    cameras = _parse_camera_positions(model_dir / "images.txt")
    if not cameras:
        return None

    matched = []
    for name, cam_pos in cameras.items():
        gps = _read_gps(image_dir / name)
        if gps is not None:
            matched.append((gps[2], cam_pos))

    if len(matched) < GPS_UP_MIN_IMAGES:
        return None

    alt = np.array([m[0] for m in matched])
    pos = np.array([m[1] for m in matched])
    design = np.column_stack([pos, np.ones(len(pos))])
    coef, *_ = np.linalg.lstsq(design, alt, rcond=None)
    pred = design @ coef
    ss_res = float(np.sum((alt - pred) ** 2))
    ss_tot = float(np.sum((alt - alt.mean()) ** 2))
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
    if r2 < GPS_UP_MIN_R2:
        return None

    direction = coef[:3]
    norm = np.linalg.norm(direction)
    if norm < 1e-9:
        return None
    return direction / norm, r2


GPS_SCALE_MIN_SEPARATION = 5.0   # meters — ignore close-together pairs where GPS noise dominates
GPS_SCALE_MIN_PAIRS = 20


def estimate_scale_from_gps(image_dir: Path, model_dir: Path):
    """
    Determine the real-world scale factor (COLMAP units -> meters) from GPS.

    Monocular SfM has no inherent sense of absolute scale — COLMAP's units
    are whatever its arbitrary initialization happened to produce. GPS
    gives real, physical distances: for every pair of images far enough
    apart that consumer-GPS noise (~a few meters) doesn't dominate, this
    compares real-world distance (haversine + altitude) to the distance
    between the same two cameras in COLMAP's reconstruction, and takes the
    median ratio across all such pairs for robustness.

    Returns
    -------
    (scale_factor, coefficient_of_variation, num_pairs) or None if there
    isn't enough GPS data to trust a result. Multiply COLMAP coordinates by
    scale_factor to get real meters.
    """
    cameras = _parse_camera_positions(model_dir / "images.txt")
    if not cameras:
        return None

    gps = {}
    for name in cameras:
        g = _read_gps(image_dir / name)
        if g is not None:
            gps[name] = g

    names = list(gps.keys())
    if len(names) < GPS_UP_MIN_IMAGES:
        return None

    ratios = []
    for i in range(len(names)):
        lat1, lon1, alt1 = gps[names[i]]
        for j in range(i + 1, len(names)):
            lat2, lon2, alt2 = gps[names[j]]
            horiz = _haversine_distance(lat1, lon1, lat2, lon2)
            real_dist = float(np.hypot(horiz, alt2 - alt1))
            if real_dist < GPS_SCALE_MIN_SEPARATION:
                continue
            colmap_dist = float(np.linalg.norm(cameras[names[i]] - cameras[names[j]]))
            if colmap_dist < 1e-9:
                continue
            ratios.append(real_dist / colmap_dist)

    if len(ratios) < GPS_SCALE_MIN_PAIRS:
        return None

    ratios = np.array(ratios)
    scale = float(np.median(ratios))
    cv = float(np.std(ratios) / scale) if scale > 0 else float("inf")
    return scale, cv, len(ratios)


def _outward_normal(normal: np.ndarray, centroid: np.ndarray, cloud_center: np.ndarray) -> np.ndarray:
    """Orient a plane normal to point away from the overall point-cloud mass."""
    return normal if np.dot(normal, centroid - cloud_center) >= 0 else -normal


def _rotation_aligning(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rotation matrix mapping unit vector a onto unit vector b (Rodrigues' formula)."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = np.linalg.norm(v)
    if s < 1e-8:
        if c > 0:
            return np.eye(3)
        # a and b are anti-parallel: rotate 180 degrees about any axis perpendicular to a
        perp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, perp)
        axis /= np.linalg.norm(axis)
        vx = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return np.eye(3) + 2 * (vx @ vx)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s ** 2))


def _planar_axes(z_dir: np.ndarray, points: np.ndarray, centroid: np.ndarray):
    """Two axes spanning a plane with normal z_dir, aligned to the point cluster's principal directions."""
    arbitrary = np.array([1.0, 0.0, 0.0]) if abs(z_dir[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    ref_x = np.cross(z_dir, arbitrary)
    ref_x /= np.linalg.norm(ref_x)
    ref_y = np.cross(z_dir, ref_x)

    rel = points - centroid
    proj = np.stack([rel @ ref_x, rel @ ref_y], axis=1)
    _eigvals, eigvecs = np.linalg.eigh(np.cov(proj.T))
    a, b = eigvecs[:, 1], eigvecs[:, 0]

    x_dir = a[0] * ref_x + a[1] * ref_y
    y_dir = b[0] * ref_x + b[1] * ref_y
    x_dir /= np.linalg.norm(x_dir)
    y_dir /= np.linalg.norm(y_dir)
    if np.dot(np.cross(x_dir, y_dir), z_dir) < 0:
        y_dir = -y_dir
    return x_dir, y_dir


def _placement_matrix(origin, x_dir, y_dir, z_dir) -> np.ndarray:
    m = np.eye(4)
    m[:3, 0] = x_dir
    m[:3, 1] = y_dir
    m[:3, 2] = z_dir
    m[:3, 3] = origin
    return m


def _convex_hull_2d(points_2d: np.ndarray) -> np.ndarray:
    """Andrew's monotone chain convex hull — no extra dependency beyond numpy."""
    pts = np.unique(points_2d, axis=0)
    if len(pts) < 3:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def _polygon_area(hull_2d: np.ndarray) -> float:
    x, y = hull_2d[:, 0], hull_2d[:, 1]
    return 0.5 * abs(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


# ---------------------------------------------------------------------------
# IFC element builders
# ---------------------------------------------------------------------------

def _apply_style(file, rep, name, rgb, transparency=0.0):
    style = ifcopenshell.api.run("style.add_style", file, name=name)
    attrs = {"SurfaceColour": {"Name": None, "Red": rgb[0], "Green": rgb[1], "Blue": rgb[2]}}
    if transparency:
        attrs["Transparency"] = transparency
    ifcopenshell.api.run("style.add_surface_style", file, style=style, ifc_class="IfcSurfaceStyleShading", attributes=attrs)
    ifcopenshell.api.run("style.assign_representation_styles", file, shape_representation=rep, styles=[style])


WALL_COLOR = (0.72, 0.45, 0.28)
SLAB_COLOR = (0.55, 0.55, 0.58)
ROOF_COLOR = (0.45, 0.45, 0.5)


def _is_well_supported(point_count: int, dim_a: float, dim_b: float) -> bool:
    """Reject planes whose footprint is large relative to how many points actually support it."""
    area = dim_a * dim_b
    return area <= 0 or (point_count / area) >= MIN_POINT_DENSITY


def _connected_components(cells: set) -> list:
    """8-connected flood fill over a set of (i, j) grid cells; returns all components, largest first."""
    remaining = set(cells)
    components = []
    while remaining:
        seed = next(iter(remaining))
        remaining.discard(seed)
        stack = [seed]
        component = {seed}
        while stack:
            ci, cj = stack.pop()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    neighbor = (ci + di, cj + dj)
                    if neighbor in remaining:
                        remaining.discard(neighbor)
                        component.add(neighbor)
                        stack.append(neighbor)
        components.append(component)
    components.sort(key=len, reverse=True)
    return components


WALL_MIN_COMPONENT_CELLS = 6   # below this, a disconnected fragment is noise, not a real second wall band


def _add_wall(file, context, storey, plane, up, cloud_center):
    """
    Build a wall-classified plane as an occupancy-masked grid of small
    blocks, one per occupied cell, rather than a single solid rectangle.

    A plain rectangle (or even a convex hull) assumes the whole footprint
    is solid material. That's wrong for anything that isn't a real flat
    wall but still roughly satisfies RANSAC's plane tolerance — e.g. an
    open scaffold/lattice attached to a facade, which is planar-ish but
    has real gaps (like a big hole where the lattice doesn't cover). A
    solid rectangle would silently paper over that hole with fake wall
    material. Masking to occupied cells only builds material where the
    scan actually has points, so a real solid wall renders as an
    essentially-continuous block (high occupancy) while an open lattice
    renders as a sparse, gap-toothed grid — an honest difference.

    RANSAC only checks distance-to-plane, not spatial contiguity, so its
    inlier set can silently bridge two physically separate surfaces that
    happen to share an orientation and offset (e.g. two different walls on
    a site that coincidentally line up, or the lower and upper facade of a
    tower separated by an intervening floor band). Splitting occupied cells
    into connected components and building each one that's large enough as
    its own IfcWall handles both cases honestly: a real single wall stays
    one component and becomes one element; a facade with a real vertical
    gap (or a coincidental cross-site merge) becomes multiple separate
    wall elements instead of one connected (wrong) rectangle or a single
    kept fragment that silently drops the rest.

    Returns a list of created IfcWall entities (possibly empty).
    """
    outward = _outward_normal(plane["normal"], plane["centroid"], cloud_center)
    inward = -outward

    z_dir = up
    x_dir = np.cross(z_dir, inward)
    x_norm = np.linalg.norm(x_dir)
    if x_norm < 1e-6:
        return []
    x_dir /= x_norm
    y_dir = inward  # thickness extrudes into the building, not into empty air

    pts, centroid = plane["points"], plane["centroid"]
    u = (pts - centroid) @ x_dir
    v = (pts - centroid) @ z_dir

    u0, v0 = float(u.min()), float(v.min())
    ui = np.floor((u - u0) / WALL_CELL_SIZE).astype(int)
    vi = np.floor((v - v0) / WALL_CELL_SIZE).astype(int)
    cell_of_point = list(zip(ui.tolist(), vi.tolist()))
    occupied_all = set(cell_of_point)

    box_faces = [
        (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0),
    ]

    walls = []
    for occupied in _connected_components(occupied_all):
        if len(occupied) < WALL_MIN_COMPONENT_CELLS:
            continue

        kept_cu = [c[0] for c in occupied]
        kept_cv = [c[1] for c in occupied]
        length = float((max(kept_cu) - min(kept_cu) + 1) * WALL_CELL_SIZE)
        height = float((max(kept_cv) - min(kept_cv) + 1) * WALL_CELL_SIZE)
        if length < MIN_ELEMENT_DIM or height < MIN_ELEMENT_DIM:
            continue

        n_u_trim = max(kept_cu) - min(kept_cu) + 1
        n_v_trim = max(kept_cv) - min(kept_cv) + 1
        coverage = len(occupied) / (n_u_trim * n_v_trim)
        if coverage < MIN_WALL_COVERAGE:
            continue

        kept_point_count = sum(1 for cell in cell_of_point if cell in occupied)
        if not _is_well_supported(kept_point_count, length, height):
            continue

        vertices, faces = [], []
        for cu, cv in occupied:
            u_a, u_b = u0 + cu * WALL_CELL_SIZE, u0 + (cu + 1) * WALL_CELL_SIZE
            v_a, v_b = v0 + cv * WALL_CELL_SIZE, v0 + (cv + 1) * WALL_CELL_SIZE
            outer = [
                centroid + u_a * x_dir + v_a * z_dir, centroid + u_b * x_dir + v_a * z_dir,
                centroid + u_b * x_dir + v_b * z_dir, centroid + u_a * x_dir + v_b * z_dir,
            ]
            inner = [p + y_dir * WALL_THICKNESS for p in outer]
            base = len(vertices)
            vertices.extend(tuple(p) for p in outer + inner)
            faces.extend([base + a, base + b, base + c] for a, b, c in box_faces)

        wall = ifcopenshell.api.run("root.create_entity", file, ifc_class="IfcWall",
                                     name="Wall (auto, occupancy-masked facade plane)")
        rep = ifcopenshell.api.run(
            "geometry.add_mesh_representation", file, context=context,
            vertices=[vertices], faces=[faces],
        )
        ifcopenshell.api.run("geometry.assign_representation", file, product=wall, representation=rep)
        ifcopenshell.api.run("spatial.assign_container", file, products=[wall], relating_structure=storey)
        _apply_style(file, rep, "Wall", WALL_COLOR)
        walls.append(wall)

    return walls


def _add_slab(file, context, storey, plane, up, is_lowest=False):
    """is_lowest: True only for the lowest slab-classified plane in the whole
    model — that one gets called BASESLAB/"ground slab". Any other flat
    surface (e.g. an elevated deck or platform) is still a real slab, just
    not the ground, so it's named/typed accordingly rather than mislabeled."""
    z_dir = up  # near-horizontal by classification; force exactly flat
    x_dir, y_dir = _planar_axes(z_dir, plane["points"], plane["centroid"])

    pts, centroid = plane["points"], plane["centroid"]
    u = (pts - centroid) @ x_dir
    v = (pts - centroid) @ y_dir
    length, width = float(u.max() - u.min()), float(v.max() - v.min())
    if length < MIN_ELEMENT_DIM or width < MIN_ELEMENT_DIM:
        return None
    if not _is_well_supported(plane["count"], length, width):
        return None
    origin = centroid + u.min() * x_dir + v.min() * y_dir
    polyline = [(0.0, 0.0), (length, 0.0), (length, width), (0.0, width), (0.0, 0.0)]

    if is_lowest:
        predefined_type, name = "BASESLAB", "Slab (auto, ground slab)"
    else:
        predefined_type, name = "FLOOR", "Slab (auto, elevated flat surface — not the ground)"
    slab = ifcopenshell.api.run("root.create_entity", file, ifc_class="IfcSlab",
                                 predefined_type=predefined_type, name=name)
    rep = ifcopenshell.api.run(
        "geometry.add_slab_representation", file, context=context,
        depth=SLAB_THICKNESS, direction_sense="NEGATIVE", polyline=polyline,
    )
    ifcopenshell.api.run("geometry.assign_representation", file, product=slab, representation=rep)
    ifcopenshell.api.run("geometry.edit_object_placement", file, product=slab,
                          matrix=_placement_matrix(origin, x_dir, y_dir, z_dir))
    ifcopenshell.api.run("spatial.assign_container", file, products=[slab], relating_structure=storey)
    _apply_style(file, rep, "Slab", SLAB_COLOR, SLAB_TRANSPARENCY)
    return slab


GABLE_MIN_DENSITY = 3.5    # points per m^2 of *hull* area, real meters — lower than MIN_POINT_DENSITY
                           # because a tight convex hull, unlike a bounding rectangle, doesn't waste
                           # area on empty corners, so the same point count supports a smaller true threshold


def _add_roof(file, context, storey, plane, up, cloud_center):
    """
    Build a roof-classified plane as a solid clipped to its actual convex-hull
    footprint, not a rectangle. A steep gable is triangular, not rectangular —
    forcing a bounding box around it either invents empty corner area (making
    a real feature look unsupported) or gets rejected outright. Clipping to
    the hull uses only the area the points actually justify.
    """
    outward = _outward_normal(plane["normal"], plane["centroid"], cloud_center)
    z_dir = -outward  # extrude inward, into the roof/gable structure
    x_dir, y_dir = _planar_axes(z_dir, plane["points"], plane["centroid"])

    pts, centroid = plane["points"], plane["centroid"]
    u = (pts - centroid) @ x_dir
    v = (pts - centroid) @ y_dir
    length, width = float(u.max() - u.min()), float(v.max() - v.min())
    if length < MIN_ELEMENT_DIM or width < MIN_ELEMENT_DIM:
        return None

    hull = _convex_hull_2d(np.column_stack([u, v]))
    if len(hull) < 3:
        return None
    area = _polygon_area(hull)
    if area <= 0 or (plane["count"] / area) < GABLE_MIN_DENSITY:
        return None

    top = [centroid + hu * x_dir + hv * y_dir for hu, hv in hull]
    bottom = [p + z_dir * ROOF_THICKNESS for p in top]
    n = len(hull)
    vertices = [tuple(p) for p in top] + [tuple(p) for p in bottom]
    faces = []
    for i in range(1, n - 1):
        faces.append([0, i, i + 1])
    for i in range(1, n - 1):
        faces.append([n, n + i + 1, n + i])
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j])
        faces.append([i, n + j, n + i])

    roof = ifcopenshell.api.run("root.create_entity", file, ifc_class="IfcRoof",
                                 name="Roof/gable face (auto, clipped to real footprint)")
    rep = ifcopenshell.api.run(
        "geometry.add_mesh_representation", file, context=context,
        vertices=[vertices], faces=[faces],
    )
    ifcopenshell.api.run("geometry.assign_representation", file, product=roof, representation=rep)
    ifcopenshell.api.run("spatial.assign_container", file, products=[roof], relating_structure=storey)
    _apply_style(file, rep, "Roof", ROOF_COLOR)
    return roof


GABLE_MARGIN = 0.15                # meters above wall-top counted as "roof candidate" points
GABLE_MIN_POINTS = 30              # below this, there's nothing to fit — fall back to a flat cap
GABLE_RIDGE_CANDIDATES = 25        # grid search resolution for the ridge (changepoint) position


def _fit_gable_roof(file, context, storey, wall_planes, all_points_rotated, up):
    """
    Least-squares fit a ridge + two roof slopes ("tent" shape) to the points
    sitting above the walls, instead of forcing a flat cap.

    RANSAC needs many points within a tight distance tolerance to accept a
    plane — that's exactly what roof coverage from ground-level walkaround
    photos doesn't have (steeper viewing angle, more distance, less texture
    contrast than the walls get). A least-squares fit uses every point at
    once without requiring any single one to be a tight inlier, so it can
    recover an honest ridge shape from data too sparse for RANSAC to trust.

    Returns None (caller should fall back to a flat cap) if there isn't
    enough data, or if the fit doesn't look like a real ridge (e.g. it
    degenerates to a single slope with no peak).
    """
    all_wall_pts = np.vstack([p["points"] for p in wall_planes])
    centroid = all_wall_pts.mean(axis=0)
    top_height = float(np.max(all_wall_pts @ up))

    # Ridge runs parallel to the longest (most confidently fit) wall.
    longest = max(wall_planes, key=lambda p: p["count"])
    outward = _outward_normal(longest["normal"], longest["centroid"], centroid)
    ridge_dir = np.cross(up, -outward)
    ridge_norm = np.linalg.norm(ridge_dir)
    if ridge_norm < 1e-6:
        return None
    ridge_dir /= ridge_norm
    across_dir = np.cross(up, ridge_dir)  # horizontal, perpendicular to the ridge

    height_all = all_points_rotated @ up
    roof_pts = all_points_rotated[height_all > (top_height - GABLE_MARGIN)]
    if len(roof_pts) < GABLE_MIN_POINTS:
        return None

    s = (roof_pts - centroid) @ across_dir
    z = (roof_pts - centroid) @ up
    r = (roof_pts - centroid) @ ridge_dir

    # Grid search for the ridge (changepoint) position that minimizes a
    # continuous two-segment least-squares fit — a "hinge" regression.
    candidates = np.linspace(np.percentile(s, 10), np.percentile(s, 90), GABLE_RIDGE_CANDIDATES)
    best = None
    for ridge_s in candidates:
        hinge = np.clip(s - ridge_s, 0, None)
        design = np.column_stack([np.ones_like(s), s, hinge])
        coef, *_ = np.linalg.lstsq(design, z, rcond=None)
        rss = float(np.sum((z - design @ coef) ** 2))
        if best is None or rss < best[0]:
            best = (rss, ridge_s, coef)
    _rss, ridge_s, (b0, b1, b2) = best

    def height_at(s_val):
        return b0 + b1 * s_val + b2 * max(0.0, s_val - ridge_s)

    s_min, s_max = float(s.min()), float(s.max())
    r_min, r_max = float(r.min()), float(r.max())
    ridge_h, eave_l_h, eave_r_h = height_at(ridge_s), height_at(s_min), height_at(s_max)

    if not (ridge_h > eave_l_h and ridge_h > eave_r_h):
        return None  # fit degenerated to a plain slope, not a real ridge

    def world(s_val, r_val, z_val):
        return centroid + s_val * across_dir + r_val * ridge_dir + z_val * up

    ridge_a, ridge_b = world(ridge_s, r_min, ridge_h), world(ridge_s, r_max, ridge_h)
    eave_l_a, eave_l_b = world(s_min, r_min, eave_l_h), world(s_min, r_max, eave_l_h)
    eave_r_a, eave_r_b = world(s_max, r_min, eave_r_h), world(s_max, r_max, eave_r_h)

    roof = ifcopenshell.api.run(
        "root.create_entity", file, ifc_class="IfcRoof", predefined_type="GABLE_ROOF",
        name="Roof (auto, least-squares gable fit — ridge + two slopes)",
    )
    vertices, faces = [], []
    for quad in [(eave_l_a, eave_l_b, ridge_b, ridge_a), (ridge_a, ridge_b, eave_r_b, eave_r_a)]:
        p0, p1, p2, p3 = quad
        normal = np.cross(p1 - p0, p3 - p0)
        norm_len = np.linalg.norm(normal)
        normal = normal / norm_len if norm_len > 1e-9 else up
        top = [p0, p1, p2, p3]
        bottom = [p - normal * ROOF_THICKNESS for p in top]
        base = len(vertices)
        vertices.extend(tuple(v) for v in top + bottom)
        faces.append([base + 0, base + 1, base + 2])
        faces.append([base + 0, base + 2, base + 3])
        faces.append([base + 4, base + 6, base + 5])
        faces.append([base + 4, base + 7, base + 6])
        for i in range(4):
            j = (i + 1) % 4
            faces.append([base + i, base + j, base + 4 + j])
            faces.append([base + i, base + 4 + j, base + 4 + i])

    rep = ifcopenshell.api.run(
        "geometry.add_mesh_representation", file, context=context,
        vertices=[vertices], faces=[faces],
    )
    ifcopenshell.api.run("geometry.assign_representation", file, product=roof, representation=rep)
    ifcopenshell.api.run("spatial.assign_container", file, products=[roof], relating_structure=storey)
    _apply_style(file, rep, "Roof", ROOF_COLOR)
    return roof


def _add_roof_cap(file, context, storey, wall_planes, up):
    """
    A simple flat slab spanning the combined footprint of the successfully
    built walls, sitting at the height of the tallest one.

    This building's real roof has multiple gables plus a cupola added in
    1861 — no single flat plane fit can honestly represent that, and every
    attempt at RANSAC-detecting one either failed the density check (good —
    it really was spurious) or produced an unstable, barely-supported
    guess. Rather than force a fake "roof shape," this closes the
    silhouette with an explicit, geometrically-constructed approximation
    that makes no claim about the real roof's form.
    """
    all_points = np.vstack([p["points"] for p in wall_planes])
    centroid = all_points.mean(axis=0)
    x_dir, y_dir = _planar_axes(up, all_points, centroid)
    top_height = float(np.max(all_points @ up))

    u = (all_points - centroid) @ x_dir
    v = (all_points - centroid) @ y_dir
    length, width = float(u.max() - u.min()), float(v.max() - v.min())
    origin = centroid + u.min() * x_dir + v.min() * y_dir + (top_height - float(centroid @ up)) * up
    polyline = [(0.0, 0.0), (length, 0.0), (length, width), (0.0, width), (0.0, 0.0)]

    cap = ifcopenshell.api.run(
        "root.create_entity", file, ifc_class="IfcRoof", predefined_type="FLAT_ROOF",
        name="Roof cap (auto, simplified flat approximation — not a modeled roof shape)",
    )
    rep = ifcopenshell.api.run(
        "geometry.add_slab_representation", file, context=context,
        depth=ROOF_THICKNESS, direction_sense="POSITIVE", polyline=polyline,
    )
    ifcopenshell.api.run("geometry.assign_representation", file, product=cap, representation=rep)
    ifcopenshell.api.run("geometry.edit_object_placement", file, product=cap,
                          matrix=_placement_matrix(origin, x_dir, y_dir, up))
    ifcopenshell.api.run("spatial.assign_container", file, products=[cap], relating_structure=storey)
    _apply_style(file, rep, "RoofCap", (0.3, 0.27, 0.27), 0.35)
    return cap


def _build_proxy_elements(file, context, storey, leftover_pcd):
    """Cluster non-planar leftover points and export each cluster as a generic proxy hull."""
    if len(leftover_pcd.points) < MIN_CLUSTER_POINTS_FOR_PROXY:
        return []

    labels = np.array(leftover_pcd.cluster_dbscan(eps=DBSCAN_EPS, min_points=DBSCAN_MIN_POINTS))
    created = []
    for label in sorted(set(labels)):
        if label == -1:
            continue  # DBSCAN noise
        idx = np.where(labels == label)[0]
        if len(idx) < MIN_CLUSTER_POINTS_FOR_PROXY:
            continue
        cluster = leftover_pcd.select_by_index(idx.tolist())
        try:
            hull, _ = cluster.compute_convex_hull()
        except RuntimeError:
            continue

        vertices = [tuple(v) for v in np.asarray(hull.vertices)]
        faces = [[list(map(int, tri)) for tri in np.asarray(hull.triangles)]]

        proxy = ifcopenshell.api.run("root.create_entity", file, ifc_class="IfcBuildingElementProxy",
                                      name="Context clutter (auto, unclassified)")
        rep = ifcopenshell.api.run(
            "geometry.add_mesh_representation", file, context=context,
            vertices=[vertices], faces=faces,
        )
        ifcopenshell.api.run("geometry.assign_representation", file, product=proxy, representation=rep)
        ifcopenshell.api.run("spatial.assign_container", file, products=[proxy], relating_structure=storey)
        created.append(proxy)
    return created


def _kmeans_colors(colors: np.ndarray, k: int, iters: int = 15, seed: int = 0):
    """Minimal numpy-only k-means (avoids adding scipy/sklearn as a hard dependency)."""
    rng = np.random.default_rng(seed)
    k = min(k, len(colors))
    centers = colors[rng.choice(len(colors), size=k, replace=False)].copy()
    labels = np.zeros(len(colors), dtype=int)
    for _ in range(iters):
        dist = ((colors[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2)
        labels = dist.argmin(axis=1)
        for j in range(k):
            mask = labels == j
            if mask.any():
                centers[j] = colors[mask].mean(axis=0)
    return labels, centers


_CUBE_LOCAL_OFFSETS = np.array([
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1,  1], [1, -1,  1], [1, 1,  1], [-1, 1,  1],
], dtype=float) * (ENVELOPE_MARKER_SIZE / 2.0)
_CUBE_FACES = [
    (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
    (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
    (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0),
]


def _cubes_mesh(centers: np.ndarray):
    vertices = []
    faces_flat = []
    for center in centers:
        base = len(vertices)
        vertices.extend(tuple(center + off) for off in _CUBE_LOCAL_OFFSETS)
        faces_flat.extend([base + a, base + b, base + c] for a, b, c in _CUBE_FACES)
    return vertices, faces_flat


def _build_reference_envelope(file, context, storey, pcd, rotation):
    """
    Render the *whole* cleaned point cloud (not just leftover points) as
    small, overlapping, color-matched cube markers — one per (downsampled)
    point — grouped into a handful of IfcBuildingElementProxy elements by
    color cluster and exported as explicitly unclassified reference geometry.

    This exists because plane classification is necessarily conservative —
    it only types elements it's confident about, so it can under-detect real
    architecture (e.g. a second, smaller roof gable). Without this envelope,
    the model's outer shape can end up looking far plainer than the actual
    scan.

    Two earlier approaches were tried and rejected:
      - Surface reconstruction (Poisson, alpha-shape): both try to
        interpolate a smooth watertight surface, and both produced
        crumpled, unrecognizable geometry on this sparse, unevenly covered
        SfM cloud (dense multi-view/LiDAR data would fare better).
      - Plain uncolored cube markers: spatially correct, but a flat grey
        blob without the point cloud's actual colors reads as a rock, not
        a building — the window/brick/foliage color pattern is what
        actually makes the shape legible, not the geometry alone.
    """
    down = pcd.voxel_down_sample(ENVELOPE_VOXEL_SIZE)
    centers = (rotation @ np.asarray(down.points).T).T
    colors = np.asarray(down.colors) if down.has_colors() else np.full((len(centers), 3), 0.6)

    labels, cluster_colors = _kmeans_colors(colors, ENVELOPE_COLOR_BUCKETS)

    envelope_elements = []
    total_faces = 0
    for cluster_id in range(len(cluster_colors)):
        mask = labels == cluster_id
        if not mask.any():
            continue
        vertices, faces_flat = _cubes_mesh(centers[mask])
        total_faces += len(faces_flat)

        proxy = ifcopenshell.api.run(
            "root.create_entity", file, ifc_class="IfcBuildingElementProxy",
            name="Reference scan envelope (unclassified — color-matched point markers, not a typed element)",
        )
        rep = ifcopenshell.api.run(
            "geometry.add_mesh_representation", file, context=context,
            vertices=[vertices], faces=[faces_flat],
        )
        ifcopenshell.api.run("geometry.assign_representation", file, product=proxy, representation=rep)
        ifcopenshell.api.run("spatial.assign_container", file, products=[proxy], relating_structure=storey)

        r, g, b = cluster_colors[cluster_id]
        style = ifcopenshell.api.run("style.add_style", file, name=f"Envelope color {cluster_id}")
        ifcopenshell.api.run(
            "style.add_surface_style", file, style=style, ifc_class="IfcSurfaceStyleShading",
            attributes={"SurfaceColour": {"Name": None, "Red": float(r), "Green": float(g), "Blue": float(b)}},
        )
        ifcopenshell.api.run("style.assign_representation_styles", file, shape_representation=rep, styles=[style])

        envelope_elements.append(proxy)

    return envelope_elements, len(centers), total_faces


def _add_georeferencing(file, site, image_dir, model_dir):
    """
    Anchor IfcSite to a real-world GPS location, if available.

    Sets RefLatitude/RefLongitude/RefElevation to the mean GPS position of
    all cameras, and adds a minimal IfcMapConversion treating the model's
    local origin as coincident with that anchor (Eastings=Northings=0, no
    rotation to true north). This records *where* the building is, which is
    the practically useful part; it is not a survey-grade projected-CRS
    transform (that would need a known true-north bearing and a real
    projected CRS, e.g. a UTM zone, which this dataset doesn't give us).

    Returns (lat, lon, alt) of the anchor, or None if there's no GPS data.
    """
    if image_dir is None or model_dir is None:
        return None
    cameras = _parse_camera_positions(model_dir / "images.txt")
    if not cameras:
        return None
    gps_points = [gps for name in cameras if (gps := _read_gps(image_dir / name)) is not None]
    if not gps_points:
        return None

    lat0 = float(np.mean([g[0] for g in gps_points]))
    lon0 = float(np.mean([g[1] for g in gps_points]))
    alt0 = float(np.mean([g[2] for g in gps_points]))

    import ifcopenshell.util.geolocation as geolocation
    site.RefLatitude = geolocation.dd2dms(lat0, use_us=True)
    site.RefLongitude = geolocation.dd2dms(lon0, use_us=True)
    site.RefElevation = alt0

    ifcopenshell.api.run("georeference.add_georeferencing", file, ifc_class="IfcMapConversion", name="LOCAL")
    ifcopenshell.api.run(
        "georeference.edit_georeferencing", file,
        projected_crs={
            "Name": "LOCAL",
            "Description": "Local origin treated as coincident with IfcSite's RefLatitude/RefLongitude — "
                            "not a surveyed projected CRS or true-north-aligned transform.",
        },
        coordinate_operation={"Eastings": 0.0, "Northings": 0.0, "OrthogonalHeight": 0.0},
    )
    return lat0, lon0, alt0


def _write_summary_report(output_dir: Path, stats: dict) -> Path:
    """Write a plain, self-contained HTML summary of one export_ifc() run."""
    import datetime
    import html as html_module

    def esc(v):
        return html_module.escape(str(v))

    plane_rows = "\n".join(
        f"<tr><td>{esc(p['kind'])}</td><td>{p['count']:,}</td>"
        f"<td>{100 * p['count'] / stats['total_points']:.1f}%</td>"
        f"<td>{p['angle']:.1f}&deg;</td></tr>"
        for p in stats["plane_summary"]
    )

    scale_row = (
        f"1 COLMAP unit = {stats['scale_factor']:.4f} m "
        f"({stats['scale_pairs']} camera pairs, {stats['scale_cv'] * 100:.1f}% spread)"
        if stats.get("scale_factor") else "Not available — coordinates are COLMAP's nominal (unscaled) units"
    )
    up_row = (
        f"GPS altitude correlation (R&sup2;={stats['up_r2']:.4f})"
        if stats.get("up_r2") is not None else "Wall-plane geometry (no usable GPS data)"
    )
    geo_row = (
        f"lat={stats['geo_lat']:.6f}, lon={stats['geo_lon']:.6f}, alt={stats['geo_alt']:.1f} m"
        if stats.get("geo_lat") is not None else "Not available — no GPS data"
    )

    html_doc = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>BIM Export Report — {esc(stats['building_name'])}</title>
<style>
  body {{ font-family: -apple-system, Helvetica, Arial, sans-serif; max-width: 760px;
         margin: 2.5rem auto; padding: 0 1.5rem; color: #222; line-height: 1.5; }}
  h1 {{ font-size: 1.4rem; margin-bottom: 0.1rem; }}
  .meta {{ color: #777; font-size: 0.85rem; margin-bottom: 1.8rem; }}
  h2 {{ font-size: 1rem; text-transform: uppercase; letter-spacing: 0.04em; color: #555;
        border-top: 1px solid #ddd; padding-top: 1.1rem; margin-top: 1.6rem; }}
  table {{ border-collapse: collapse; width: 100%; font-size: 0.9rem; }}
  td, th {{ text-align: left; padding: 0.3rem 0.6rem 0.3rem 0; }}
  th {{ color: #777; font-weight: 600; border-bottom: 1px solid #ddd; }}
  tr:nth-child(even) {{ background: #fafafa; }}
  .stat {{ display: flex; justify-content: space-between; padding: 0.25rem 0; font-size: 0.92rem; }}
  .stat span:first-child {{ color: #555; }}
  code {{ background: #f2f2f2; padding: 0.1rem 0.35rem; border-radius: 3px; font-size: 0.85em; }}
</style></head>
<body>
  <h1>BIM Export Report</h1>
  <div class="meta">{esc(stats['building_name'])} &middot; generated {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}</div>

  <h2>Input</h2>
  <div class="stat"><span>Point cloud</span><span>{stats['total_points']:,} points after noise filtering</span></div>
  <div class="stat"><span>Point spacing</span><span>{stats['spacing']:.5f} {'m' if stats.get('scale_factor') else '(nominal units)'}</span></div>
  <div class="stat"><span>Scale calibration</span><span>{scale_row}</span></div>
  <div class="stat"><span>Vertical axis source</span><span>{up_row}</span></div>
  <div class="stat"><span>Georeference anchor</span><span>{geo_row}</span></div>

  <h2>Plane classification</h2>
  <table>
    <tr><th>Kind</th><th>Points</th><th>% of cloud</th><th>Angle from vertical</th></tr>
    {plane_rows}
  </table>

  <h2>Typed IFC elements</h2>
  <div class="stat"><span>Walls</span><span>{stats['n_walls']}</span></div>
  <div class="stat"><span>Slabs</span><span>{stats['n_slabs']}</span></div>
  <div class="stat"><span>Roofs</span><span>{stats['n_roofs']} <em>({esc(stats['roof_method'])})</em></span></div>
  <div class="stat"><span>Context proxies (clutter)</span><span>{stats['n_proxies']} (from {stats['n_leftover']:,} leftover points)</span></div>
  <div class="stat"><span>Reference envelope</span><span>{stats['env_points']:,} markers, {stats['env_faces']:,} faces, {stats['env_colors']} color groups</span></div>

  <h2>Output</h2>
  <div class="stat"><span>File</span><span><code>{esc(stats['output_path'])}</code></span></div>
</body></html>"""

    report_path = output_dir / "report.html"
    report_path.write_text(html_doc)
    return report_path


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def export_ifc(ply_path: Path, output_dir: Path, building_name: str = "Reconstructed Building",
                image_dir: Path = None, model_dir: Path = None) -> Path:
    """
    Convert a COLMAP sparse point cloud into a typed IFC4 model.

    Parameters
    ----------
    ply_path      : Path to the input sparse.ply
    output_dir    : Directory to write model.ifc into
    building_name : Name assigned to the IfcProject/IfcBuilding
    image_dir     : Optional — source photos, for GPS-based up-axis estimation
    model_dir     : Optional — COLMAP model dir (images.txt), for GPS-based up-axis estimation

    Returns
    -------
    Path to the generated model.ifc
    """
    if o3d is None:
        logger.error("open3d is not installed. Run: pip install open3d")
        sys.exit(1)
    if not ply_path.exists():
        logger.error("PLY not found: %s", ply_path)
        sys.exit(1)

    logger.info("=== BIM Export ===")
    logger.info("Loading point cloud: %s", ply_path)
    pcd = _load_cleaned_cloud(ply_path)
    total = len(pcd.points)
    logger.info("Working with %d points after noise filtering.", total)

    stats = {"building_name": building_name, "total_points": total}

    # COLMAP's monocular SfM has no inherent absolute scale. If GPS EXIF is
    # available, calibrate one from real-world distances between camera
    # pairs — everything downstream (thresholds, thicknesses, dimensions in
    # the IFC) then means real meters instead of an arbitrary unit.
    scale_factor = None
    if image_dir is not None and model_dir is not None:
        scale_result = estimate_scale_from_gps(image_dir, model_dir)
        if scale_result is not None:
            scale_factor, scale_cv, scale_pairs = scale_result
            pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) * scale_factor)
            logger.info("Scale from GPS: 1 COLMAP unit = %.4f m (from %d camera pairs, %.1f%% spread)",
                        scale_factor, scale_pairs, scale_cv * 100)
            stats.update(scale_factor=scale_factor, scale_cv=scale_cv, scale_pairs=scale_pairs)
        else:
            logger.info("No usable GPS data for scale calibration — coordinates stay in COLMAP's nominal units.")

    spacing = _characteristic_spacing(pcd)
    _apply_adaptive_thresholds(spacing)
    logger.info("Point spacing: %.5f %s -> geometry thresholds auto-scaled to match.",
                spacing, "m" if scale_factor else "(nominal units)")
    stats["spacing"] = spacing

    all_points = np.asarray(pcd.points)
    cloud_center = all_points.mean(axis=0)

    planes, leftover = segment_planes(pcd)
    if not planes:
        logger.error("No significant planes found — point cloud too sparse or noisy for BIM export.")
        sys.exit(1)

    raw_plane_count = len(planes)
    planes = merge_coplanar_planes(planes)
    if len(planes) < raw_plane_count:
        logger.info("Merged %d near-duplicate coplanar plane(s) (%d -> %d).",
                    raw_plane_count - len(planes), raw_plane_count, len(planes))

    plane_point_frac = 100 * sum(p["count"] for p in planes) / total
    logger.info("Found %d significant planes covering %.1f%% of points.", len(planes), plane_point_frac)

    up_raw = None
    if image_dir is not None and model_dir is not None:
        gps_result = estimate_up_from_gps(image_dir, model_dir)
        if gps_result is not None:
            up_raw, r2 = gps_result
            logger.info("Vertical (up) axis from GPS altitude (COLMAP frame): %s (R^2=%.4f)",
                        np.round(up_raw, 3), r2)
            stats["up_r2"] = r2
    if up_raw is None:
        up_raw = estimate_up_axis(planes)
        up_raw = resolve_up_sign(up_raw, all_points, planes)
        logger.info("Vertical (up) axis from wall-plane geometry (COLMAP frame): %s "
                    "(no usable GPS data — falling back to plane-based estimate)", np.round(up_raw, 3))

    # COLMAP's frame is arbitrarily oriented; rotate everything so the
    # estimated up axis becomes world +Z, matching IFC/BIM-viewer convention.
    rotation = _rotation_aligning(up_raw, np.array([0.0, 0.0, 1.0]))
    for p in planes:
        p["normal"] = rotation @ p["normal"]
        p["centroid"] = rotation @ p["centroid"]
        p["points"] = (rotation @ p["points"].T).T
    leftover.rotate(rotation, center=(0.0, 0.0, 0.0))
    cloud_center = rotation @ cloud_center
    all_points_rotated = (rotation @ all_points.T).T
    up = np.array([0.0, 0.0, 1.0])

    stats["plane_summary"] = []
    for p in planes:
        angle = _angle_from_axis(p["normal"], up)
        if angle < SLAB_ANGLE_THRESHOLD:
            p["kind"] = "slab"
        elif angle > WALL_ANGLE_THRESHOLD:
            p["kind"] = "wall"
        else:
            p["kind"] = "roof"
        logger.info("  plane: kind=%-4s points=%-6d (%.1f%%) angle_from_up=%.1f",
                    p["kind"], p["count"], 100 * p["count"] / total, angle)
        stats["plane_summary"].append({"kind": p["kind"], "count": p["count"], "angle": angle})

    # --- Build IFC structure ---
    file = ifcopenshell.file(schema="IFC4")
    project = ifcopenshell.api.run("root.create_entity", file, ifc_class="IfcProject", name=building_name)
    ifcopenshell.api.run("unit.assign_unit", file, length={"is_metric": True, "raw": "METERS"})
    model_ctx = ifcopenshell.api.run("context.add_context", file, context_type="Model")
    body_ctx = ifcopenshell.api.run(
        "context.add_context", file, context_type="Model",
        context_identifier="Body", target_view="MODEL_VIEW", parent=model_ctx,
    )
    site = ifcopenshell.api.run("root.create_entity", file, ifc_class="IfcSite", name="Site")
    geo_anchor = _add_georeferencing(file, site, image_dir, model_dir)
    if geo_anchor is not None:
        logger.info("Georeferenced to lat=%.6f lon=%.6f alt=%.1fm (local-origin approximation)", *geo_anchor)
        stats["geo_lat"], stats["geo_lon"], stats["geo_alt"] = geo_anchor
    building = ifcopenshell.api.run("root.create_entity", file, ifc_class="IfcBuilding", name=building_name)
    storey = ifcopenshell.api.run("root.create_entity", file, ifc_class="IfcBuildingStorey", name="Ground Storey")
    ifcopenshell.api.run("aggregate.assign_object", file, relating_object=project, products=[site])
    ifcopenshell.api.run("aggregate.assign_object", file, relating_object=site, products=[building])
    ifcopenshell.api.run("aggregate.assign_object", file, relating_object=building, products=[storey])

    slab_planes = [p for p in planes if p["kind"] == "slab"]
    lowest_slab_id = id(min(slab_planes, key=lambda p: p["centroid"] @ up)) if slab_planes else None

    counts = {"wall": 0, "slab": 0, "roof": 0}
    built_wall_planes = []
    for p in planes:
        if p["kind"] == "wall":
            walls = _add_wall(file, body_ctx, storey, p, up, cloud_center)
            counts["wall"] += len(walls)
            if walls:
                built_wall_planes.append(p)
            else:
                logger.debug("Skipped degenerate wall plane (too small after fitting).")
            continue
        elif p["kind"] == "slab":
            elem = _add_slab(file, body_ctx, storey, p, up, is_lowest=(id(p) == lowest_slab_id))
        else:
            elem = _add_roof(file, body_ctx, storey, p, up, cloud_center)
        if elem is not None:
            counts[p["kind"]] += 1
        else:
            logger.debug("Skipped degenerate %s plane (too small after fitting).", p["kind"])

    logger.info("Typed elements — walls: %d, slabs: %d, roofs: %d", counts["wall"], counts["slab"], counts["roof"])

    # Roof/gable cascade, most-authentic first: (1) a real RANSAC-detected
    # plane clipped to its actual hull shape [already attempted above, in
    # the per-plane loop] (2) a least-squares ridge fit using all the sparse
    # above-wall points at once (3) a flat cap, purely to close the
    # silhouette, only if neither of the above found anything real.
    roof_method = "RANSAC-detected plane" if counts["roof"] > 0 else "none"
    if counts["roof"] == 0 and built_wall_planes:
        gable = _fit_gable_roof(file, body_ctx, storey, built_wall_planes, all_points_rotated, up)
        if gable is not None:
            logger.info("No RANSAC-detected roof plane — fit a gabled roof via least-squares instead.")
            counts["roof"] = 1
            roof_method = "least-squares gable fit"
        else:
            _add_roof_cap(file, body_ctx, storey, built_wall_planes, up)
            logger.info("No RANSAC roof plane and gable fit found no real ridge — fell back to a flat roof cap.")
            counts["roof"] = 1
            roof_method = "flat cap (fallback)"

    proxies = _build_proxy_elements(file, body_ctx, storey, leftover)
    logger.info("Context proxy elements (clustered clutter): %d (from %d leftover points)",
                len(proxies), len(leftover.points))

    envelope_elements, env_points, env_faces = _build_reference_envelope(file, body_ctx, storey, pcd, rotation)
    logger.info("Reference scan envelope: %d point markers, %d faces across %d color groups (unclassified, matches full scan shape)",
                env_points, env_faces, len(envelope_elements))

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "model.ifc"
    file.write(str(output_path))
    logger.info("IFC model saved to: %s", output_path)

    stats.update(
        n_walls=counts["wall"], n_slabs=counts["slab"], n_roofs=counts["roof"], roof_method=roof_method,
        n_proxies=len(proxies), n_leftover=len(leftover.points),
        env_points=env_points, env_faces=env_faces, env_colors=len(envelope_elements),
        output_path=str(output_path),
    )
    report_path = _write_summary_report(output_dir, stats)
    logger.info("Summary report saved to: %s", report_path)

    return output_path
