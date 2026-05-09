# Photogrammetry Project Summary

This project is a professional tool that turns a collection of standard 2D photographs into a high-fidelity 3D model. It is designed to be fast, portable, and easy to use for presenting 3D reconstructions of real-world objects.

---

## 1. How the Pipeline Works

The process follows three main technical stages to turn photos into 3D data:

### Phase A: Feature Extraction
The software "looks" at every photo and identifies thousands of unique points (like the corner of a window or a pattern on a brick). It memorizes these visual fingerprints so it can find them again in other photos.

### Phase B: Image Matching
The system compares all the photos to see which ones are looking at the same parts of the object. By finding the same "fingerprints" in different images, it calculates exactly where the camera was standing when each photo was taken.

### Phase C: Sparse Mapping (3D Reconstruction)
Using the camera positions and the matched points, the software uses geometry (triangulation) to place thousands of dots in 3D space. This creates a "Sparse Point Cloud" — a 3D skeleton that perfectly represents the shape and color of the original object.

---

## 2. The Interactive Studio Viewer

Once the 3D model is created, it opens in a custom-built "Studio Viewer" designed for presentation. Key features include:

*   **Interactive Navigation:** Smoothly rotate, pan, and zoom around the object to inspect it from any angle.
*   **Automatic Orientation:** The software automatically detects the ground and fixes the model's position so it starts upright and centered, rather than upside down or tilted.
*   **Live Noise Filtering:** Every 3D scan has "noise" (stray floating dots). We implemented a live slider that lets you clean up the model in real-time, making it look professional and polished.
*   **Density Control:** You can adjust the size and density of the 3D points to make the model look more solid or more detailed depending on the screen you are presenting on.
*   **High-Resolution Screenshots:** Built-in tools to capture professional images of the 3D reconstruction for reports or slides.

---

## 3. Technical Achievements

*   **Optimized for Portability:** The entire system is tuned to run on modern portable hardware (like the MacBook Air) without overheating, while still processing large datasets of 100+ images.
*   **Professional Grade Outputs:** The models are exported in standard industry formats (.PLY), meaning they can be used in professional 3D software like Blender, Unity, or Unreal Engine.
*   **Stability & Precision:** We solved complex coordinate system issues to ensure that what you see on the screen is a mathematically accurate representation of the real-world object.

---


## 4. The Technology Stack

This project leverages industry-standard tools to achieve professional-grade results:

*   **COLMAP (Structure-from-Motion Engine):** A world-class computer vision library that handles the heavy lifting of calculating camera positions and triangulating 3D points from 2D images.
*   **Open3D (3D Visualization & Interaction):** Used to build the Studio Viewer. It provides the high-performance graphics engine needed to render thousands of points smoothly and the user interface for the interactive controls.
*   **Python (Orchestration):** The "glue" that connects the reconstruction engine to the viewer, ensuring data flows correctly and errors are handled gracefully.
*   **NumPy & OpenCV (Data Processing):** Powerful mathematical and image-processing libraries used to prepare images for processing and handle the complex 3D coordinate transformations.

