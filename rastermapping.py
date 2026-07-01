# 1) This is the code that visualizes the 3D point cloud from the illuminated pixels in the image using the pixel-to-angle mapping.

# 2) In order to use this code, you need to have the "pixel_angle_mapping.csv" file generated from the MTICamera-3DScan-Demo.cpp program. Then, use the "triangular_10.png" image 
# as input to extract the illuminated pixels and generate the 3D point cloud. 

# 3) If needed, you can adjust the threshold value in the cv2.threshold() function to better capture the illuminated pixels in your specific image.

# 4) Change the constants at the top of the script if your camera parameters differ from the ones used in this example.

# 5) The calibration of the point cloud is done using RANSAC to find the wall plane and then rotating the point cloud to align with the Z-axis. 
# The heights of the objects are calculated relative to the wall plane.

import numpy as np
import pandas as pd
import cv2
import open3d as o3d
import os
import matplotlib.pyplot as plt

# --- CONSTANTS ---
B = 156.1
HFOV, VFOV = 34.5, 26.2
CAM_W, CAM_H = 720, 540
MTI_DEGTORAD = np.pi / 180.0
CX, CY = 358.0, 268.0
PHI_DEG = 21.318  

FX = (CAM_W * 0.5) / np.tan((HFOV * MTI_DEGTORAD) * 0.5)
FY = (CAM_H * 0.5) / np.tan((VFOV * MTI_DEGTORAD) * 0.5)

def pixel_to_point_xyz(u_px, v_px, theta_m_deg):
    u, v = float(u_px) - CX, float(v_px) - CY
    camX, camY = np.arctan2(u, FX), np.arctan2(-v, FY)
    theta, phi = theta_m_deg * MTI_DEGTORAD, PHI_DEG * MTI_DEGTORAD
    denom = np.tan(phi + camX) - np.tan(theta)
    if abs(denom) < 1e-4: return None
    Z = B / denom # Removed negative sign to match standard calibration logic
    return [Z * np.tan(camX), Z * np.tan(camY), Z]

def get_rotation_matrix(vec1, vec2):
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b); c = np.dot(a, b); s = np.linalg.norm(v)
    if s < 1e-6: return np.eye(3)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s**2))

# 1) Load Mapping and Image
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_file = os.path.join(script_dir, "pixel_angle_mapping.csv")
df_map = pd.read_csv(csv_file)

# Build a lookup table for angles
theta_grid = np.full((CAM_H, CAM_W), np.nan, dtype=np.float32)
for _, row in df_map.iterrows():
    u, v = int(row["pixel_x"]), int(row["pixel_y"])
    if 0 <= u < CAM_W and 0 <= v < CAM_H:
        theta_grid[v, u] = row["mems_x"]

image_filename = "triangular_50.png" 
img_path = os.path.join(script_dir, image_filename)
img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
if img is None: raise FileNotFoundError(f"Image not found at {img_path}")

# 2) Extract 3D points from Illuminated Pixels
_, mask = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)
ys, xs = np.where(mask > 0)

points = []
for u, v in zip(xs, ys):
    theta_m_deg = theta_grid[v, u]
    if np.isnan(theta_m_deg): continue
    p = pixel_to_point_xyz(u, v, theta_m_deg)
    if p: points.append(p)

if not points: raise SystemExit("No valid 3D points detected.")
xyz_raw = np.array(points)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz_raw)

# 3) CALIBRATION STEP: Remove Tilt using RANSAC
plane_model, _ = pcd.segment_plane(distance_threshold=2.0, ransac_n=3, num_iterations=1000)
normal = np.array(plane_model[:3])
R = get_rotation_matrix(normal, [0, 0, 1])
pcd.rotate(R, center=pcd.get_center())

# 4) CALIBRATION STEP: Calculate Heights
xyz_calibrated = np.asarray(pcd.points)
z_wall = np.median(xyz_calibrated[:, 2])
z_height = z_wall - xyz_calibrated[:, 2] # Distance from wall to object

# Update coordinates for visualization (Z = Height)
xyz_final = xyz_calibrated.copy()
xyz_final[:, 2] = z_height
pcd.points = o3d.utility.Vector3dVector(xyz_final)

# 5) Save Calibrated Results to CSV
df_out = pd.DataFrame({
    'pixel_x': xs[:len(xyz_calibrated)], # ensure lengths match
    'pixel_y': ys[:len(xyz_calibrated)],
    'calibrated_z_abs': xyz_calibrated[:, 2],
    'object_height': z_height
})
df_out.to_csv(os.path.join(script_dir, "calibrated_illuminated_output.csv"), index=False)

# 6) Visualization with Jet Color Map
z_min, z_max = np.min(z_height), np.max(z_height)
cmap = plt.get_cmap('jet')
z_norm = (z_height - z_min) / (z_max - z_min) if (z_max - z_min) > 0 else z_height * 0
pcd.colors = o3d.utility.Vector3dVector(cmap(z_norm)[:, :3])

print(f"Calibration Complete. Max Height: {z_max:.2f}mm. Wall at: {z_wall:.2f}mm")

o3d.visualization.draw_geometries([pcd], window_name="Calibrated Illuminated Scan", width=1280, height=720)