# MEMS-Laser-3D-Reconstruction
A software framework for laser-based **3D reconstruction** using a **MEMS scanning mirror** and an industrial camera.

## Features

- Multiple MEMS scanning trajectories
- Synchronized control of the MEMS scanner and Basler camera
- Laser line detection
- Pixel-to-MEMS angle mapping
- Laser plane calibration
- 3D point cloud reconstruction using triangulation
- Interactive point cloud visualization

---

# Reconstruction Pipeline

The complete 3D reconstruction workflow consists of the following stages:

```text
Raster Scan Calibration
        │
        ▼
Pixel-to-Angle Mapping
        │
        ▼
Laser Plane Calibration
        │
        ▼
Object Scan (Lissajous / Raster)
        │
        ▼
Laser Pixel Detection
        │
        ▼
Pixel-to-Angle Mapping
        │
        ▼
3D Triangulation
        │
        ▼
Point Cloud (.ply)
        │
        ▼
Visualization
```

## 1. Raster Scan Calibration

A raster scan is first performed on a reference target.

During this process, the software:

- scans the entire **Field of View (FoV)**;
- detects the laser line in every captured frame;
- associates each detected laser pixel with the corresponding MEMS mirror angle;
- generates a pixel-to-angle lookup table:

```text
pixel_angle_mapping.csv
```

This lookup table is later used to recover the MEMS mirror angle for any detected laser pixel.

---

## 2. Laser Plane Calibration

Using the calibration measurements, the laser plane parameters are estimated.

The calibrated laser plane defines the geometric relationship between the camera and the laser projector and serves as the basis for subsequent 3D triangulation.

---

## 3. Object Scanning

After calibration, the object is scanned using one of the available trajectories:

- Raster Scan
- Triangular Lissajous
- Sinusoidal Lissajous

The MEMS controller and the Basler camera operate in **Hardware Trigger** mode to ensure precise synchronization between mirror position and image acquisition.

Depending on the selected mode, the software captures either:

- a complete trajectory image; or
- a sequence of synchronized frames.

---

## 4. Pixel-to-Angle Mapping

The captured laser image is processed to detect all illuminated laser pixels.

Each detected pixel is matched with its corresponding MEMS mirror angle using the previously generated lookup table.

---

## 5. 3D Triangulation

For every detected laser point, the software performs triangulation using:

- calibrated camera model;
- calibrated laser plane;
- corresponding MEMS mirror angle.

The reconstructed 3D points are exported as a point cloud:

```text
cloud_raster.ply
```

---

## 6. Point Cloud Visualization

A separate Python utility visualizes the reconstructed point cloud using **Open3D**.

The visualization tool:

- loads the generated PLY file;
- computes depth statistics;
- applies depth-based color mapping;
- displays a color bar representing the Z-distance;
- opens an interactive 3D viewer for inspection of the reconstructed object.

### Depth Color Coding

- 🔴 **Red** – closest points
- 🟡 **Yellow / Green** – intermediate distances
- 🔵 **Blue** – farthest points

---

# Software Architecture

The reconstruction pipeline is implemented through the following software modules.

## System Initialization

The application initializes:

- Basler industrial camera;
- MTI MEMS controller;
- MEMS trajectory generator.

At startup, the following calibration data are loaded:

- Camera calibration;
- Field of View (FoV);
- Calibrated Field of Regard (CFOR);
- ILUT (Interpolation Lookup Table);
- Triangulation parameters.

---

## Trajectory Generation

The software supports multiple scanning trajectories:

- Raster Scan
- Triangular Lissajous
- Sinusoidal Lissajous
- Sinusoidal Frame Capture
- Vertical Raster Scan

Each trajectory is converted into MEMS control commands using the ILUT calibration table to compensate for mirror nonlinearities.

---

## Camera Synchronization

The Basler camera operates in **Hardware Trigger** mode.

Each captured frame is synchronized with the current MEMS mirror position, ensuring accurate correspondence between image pixels and scanner angles.

---

## Image Processing

Depending on the selected trajectory, the software performs:

- laser line detection;
- grayscale conversion (if required);
- thresholding;
- laser center extraction;
- pixel-to-angle mapping;
- frame merging (for frame-based acquisition modes).

---

## Output Files

The software generates:

```text
pixel_angle_mapping.csv    # Pixel-to-angle lookup table
cloud_raster.ply           # Reconstructed point cloud
*.png                      # Captured trajectory images
```

---

## Point Cloud Visualization Module

The visualization utility provides:

- interactive Open3D viewer;
- depth-based color mapping;
- Z-distance statistics;
- color scale for depth interpretation.

---

# Dependencies

### C++

- OpenCV
- Eigen
- MTI SDK
- Basler Pylon SDK
- CMake

### Python

- Open3D
- NumPy
- Matplotlib

Install Python dependencies with:

```bash
pip install open3d numpy matplotlib
```

---

# Final Output

The final output of the reconstruction pipeline is a colored **3D point cloud** that can be inspected interactively or exported for further processing.

```text
pixel_angle_mapping.csv
cloud_raster.ply
trajectory.png
```
