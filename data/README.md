# Adding a dataset

This folder is intentionally empty in version control — datasets are large
binary files that don't belong in git. Add your own images here before
running the pipeline.

## Structure

Put each dataset in its own subfolder, with images directly inside an
`images/` directory:

```
data/
└── <dataset-name>/
    └── images/
        ├── photo001.jpg
        ├── photo002.jpg
        └── ...
```

Then run:

```bash
python main.py --image-dir data/<dataset-name>/images --output-dir data/<dataset-name>/output --generate-bim
```

Reconstruction outputs (`sparse.ply`, `model.ifc`, the COLMAP workspace)
will be written into `--output-dir` and are also git-ignored.

## What makes a good dataset for BIM export

`--generate-bim` works best with **drone photos that carry GPS altitude in
their EXIF data**. The pipeline uses GPS altitude to determine the true
vertical axis directly (correlated against COLMAP's reconstructed camera
positions) — this is far more reliable than inferring "up" from wall
geometry, especially for structures with unusual surfaces (scaffolding,
lattices, non-rectilinear roofs) that don't fit a simple plane model. GPS
data also usually means the shoot included elevated/overhead angles, which
ground-level walkarounds structurally can't provide — without them, roof
and upper-facade coverage is thin no matter how good the algorithm is.

Good candidates:
- Drone orbits of a single, mostly-boxy building or structure, ideally with
  passes at a few different altitudes (not just one constant height)
- Reasonably isolated subject — a lone building reconstructs more cleanly
  than a dense multi-building block

## Quick test dataset (no drone required)

The classic COLMAP "South Building" dataset (128 ground-level photos, no
GPS) works for a quick end-to-end test of reconstruction and the viewer,
but it's a weaker case for `--generate-bim` specifically — no GPS means the
up-axis falls back to plane-based estimation, and ground-level-only
coverage means the roof is barely visible. Treat it as a smoke test, not a
demo of best-case BIM output.

Download: https://github.com/colmap/colmap/releases/download/3.11.1/south-building.zip
