# Photogrammetry Pipeline

A Structure-from-Motion (SfM) pipeline that converts overlapping photos into an interactive 3D point cloud. Built with COLMAP for reconstruction and Open3D for visualization.

---

## What It Does

1. Reads overlapping photos from `data/images/`
2. Runs **COLMAP** (feature extraction → matching → sparse mapping)
3. Exports a colored 3D point cloud as `data/output/sparse.ply`
4. Opens an interactive **Studio Viewer** built with Open3D
5. Optionally runs **Poisson Surface Reconstruction** to generate a triangle mesh

---

## Project Structure

```
photogrammetry/
├── data/
│   ├── images/             ← input photos go here
│   └── output/             ← COLMAP workspace + outputs land here
├── src/
│   ├── __init__.py
│   ├── reconstruction.py   # COLMAP pipeline orchestration
│   ├── visualization.py    # Open3D Studio Viewer (GUI)
│   ├── mesh.py             # Poisson Surface Reconstruction
│   ├── pipeline.py         # wires everything together
│   └── utils.py            # logger, image discovery, COLMAP path check
├── requirements.txt
├── main.py                 # CLI entry point
└── README.md
```

---

## 1 — Install COLMAP

COLMAP must be installed separately as a system dependency.

### macOS
```bash
brew install colmap
```

### Ubuntu / Debian
```bash
sudo apt update && sudo apt install -y colmap
```

### Windows
Download the pre-built binary from the official releases page:
> https://github.com/colmap/colmap/releases

Extract the zip and add the folder containing `colmap.exe` to your system `PATH`:
1. Search for **"Environment Variables"** in the Start menu
2. Under **System Variables**, select `Path` → Edit
3. Add the full path to the folder containing `colmap.exe`

Verify on all platforms:
```bash
colmap --version
# Expected: COLMAP 3.x.x
```

---

## 2 — Set Up the Python Environment

Requires **Python 3.10+**.

### macOS / Linux
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### Windows (Command Prompt)
```cmd
python -m venv .venv
.venv\Scripts\activate
pip install --upgrade pip
pip install -r requirements.txt
```

### Windows (PowerShell)
```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install -r requirements.txt
```

---

## 3 — Add Your Images

Drop **overlapping JPG or PNG photos** into:
```
data/images/
```

**Shooting guidelines for good reconstruction:**
- Aim for ~60–80% overlap between consecutive shots
- Walk in a circle around a static subject
- Use consistent exposure and focus — avoid auto-exposure drift
- Shoot in diffuse natural light (overcast is ideal)
- Avoid reflective, transparent, or textureless surfaces (glass, mirrors, plain walls)
- Minimum ~10 images; 30–150 is typical

**Sample datasets if you don't have your own:**

| Source | Link |
|--------|------|
| COLMAP example data | https://colmap.github.io/tutorial.html |
| ETH3D multi-view | https://www.eth3d.net/datasets |
| Tanks and Temples | https://www.tanksandtemples.org |

---

## 4 — Run the Pipeline

### Full run (reconstruction + viewer)
```bash
python main.py
```

### Re-open the viewer without re-running COLMAP
```bash
python main.py --skip-reconstruction
```

### Run COLMAP only, no viewer (useful for servers / headless)
```bash
python main.py --skip-visualization
```

### Generate a Poisson mesh from the existing point cloud
```bash
python main.py --skip-reconstruction --generate-mesh
```

### Custom paths
```bash
python main.py --image-dir /path/to/photos --output-dir /path/to/results
```

### Debug / verbose logging
```bash
python main.py --verbose
```

---

## 5 — Studio Viewer Controls

The viewer opens automatically after reconstruction. It runs inside a native window and does not require a browser.

| Control | Action |
|---------|--------|
| Left-click + drag | Rotate |
| Right-click + drag | Pan |
| Scroll wheel | Zoom in / out |
| Pinch (trackpad) | Zoom in / out |

**Sidebar panels:**

| Panel | What it does |
|-------|--------------|
| **MESH VIEW** | Toggle between point cloud and Poisson mesh (only shown when `--generate-mesh` was used) |
| **APPEARANCE** — Point Size | Increase to make the cloud look denser |
| **APPEARANCE** — Voxel Density | Reduce point count for performance (0 = off) |
| **NOISE FILTERING** — Filter Strength | Remove floating artifact points. Lower value = more aggressive removal |
| **NOISE FILTERING** — Enable Filtering | Toggle between cleaned and raw cloud |
| **ACTIONS** — Save Screenshot | Saves current view to `data/output/view_HHMMSS.png` |
| **ACTIONS** — Reset Camera | Re-centers the camera on the model |

---

## 6 — Output Files

After a successful run, `data/output/` contains:

```
data/output/
├── colmap_workspace/
│   ├── database.db               # COLMAP feature database
│   └── sparse/
│       └── 0/                    # reconstruction component
│           ├── cameras.bin / cameras.txt
│           ├── images.bin  / images.txt
│           └── points3D.bin / points3D.txt
├── sparse.ply                    # colored point cloud
└── mesh.ply                      # Poisson mesh (if --generate-mesh was used)
```

The `.ply` files can be opened in **Blender**, **MeshLab**, **CloudCompare**, or any other 3D software.

---

## 7 — Troubleshooting

### `COLMAP not found on PATH`
COLMAP is not installed or not on your system PATH.
- macOS: `brew install colmap`
- Linux: `sudo apt install colmap`
- Windows: Add the folder containing `colmap.exe` to your `PATH` (see Section 1)

### `Mapper produced no reconstruction components`
COLMAP couldn't register any cameras. Common causes:

| Cause | Fix |
|-------|-----|
| Fewer than 10 images | Add more overlapping photos |
| Less than 60% overlap | Re-shoot with more overlap |
| Blurry or dark images | Use sharp, well-lit photos |
| Reflective / textureless surfaces | Add textured foreground elements |
| All images from same viewpoint | Use images with distinct angles |

### Point cloud has too many floating artifacts
Use the **Noise Filter** slider in the viewer sidebar. Drag it left (lower value) to aggressively remove outlier points without re-running reconstruction.

### `open3d` ImportError
```bash
pip install open3d
```

On Apple Silicon (M1/M2/M3/M4):
```bash
pip install open3d
# If that fails:
pip install open3d --find-links https://storage.googleapis.com/open3d-releases/open3d-stubs/
```

On headless Linux (no display):
```bash
sudo apt install libgl1-mesa-glx libglib2.0-0
```

On Windows, if you get a `vcruntime` or DLL error:
> Install the **Microsoft Visual C++ Redistributable** from https://aka.ms/vs/17/release/vc_redist.x64.exe

### Viewer crashes / segmentation fault (macOS)
This is a known issue with Open3D's Filament graphics engine on certain macOS + GPU combinations. If the viewer crashes on launch, try:
```bash
# Force software rendering
LIBGL_ALWAYS_SOFTWARE=1 python main.py --skip-reconstruction
```

---

## 8 — Performance Notes

| Dataset Size | Feature Extraction | Matching | Mapping | Dense (not included) |
|---|---|---|---|---|
| 10–30 images | < 1 min | < 1 min | < 2 min | 5–15 min |
| 50–100 images | 2–5 min | 5–20 min | 5–10 min | 30–90 min |
| 100–200 images | 5–15 min | 20–60 min | 10–20 min | 2–6 hours |

Matching is the bottleneck — it scales as O(n²) in exhaustive mode. For datasets larger than ~150 images, switch to sequential or vocabulary-tree matching.

**Hardware:**
- 8 GB RAM minimum; 16 GB+ recommended for large datasets
- NVIDIA GPU (CUDA) dramatically speeds up matching and dense reconstruction
- Apple Silicon (M-series) works well for sparse reconstruction but lacks CUDA for dense
- A cooling solution matters for long runs — sustained CPU load will throttle fanless laptops

---

## 9 — Known Limitations

- **Reflective / transparent surfaces** — SIFT struggles with glass, metal, and mirrors
- **Textureless surfaces** — plain walls and smooth objects have no keypoints to match
- **Dense reconstruction not included** — the mesh output is Poisson from a sparse cloud, not a true MVS dense result. It shows the approximate shape but lacks fine surface detail
- **Single-camera assumption** — if your dataset mixes multiple cameras, remove `--ImageReader.single_camera 1` from `reconstruction.py`
