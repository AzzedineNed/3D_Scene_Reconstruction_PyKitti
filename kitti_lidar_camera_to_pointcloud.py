# Import necessary libraries
import pykitti
import matplotlib.pyplot as plt
import numpy as np
import plyfile as ply

basedir = 'KITTI_SAMPLE/RAW'  # Base directory of the KITTI dataset
date = '2011_09_26'  # Date of the data
drive = '0009'  # Drive identifier

# Load the KITTI RAW dataset
dataset = pykitti.raw(basedir, date, drive)

# Access the OXTS data
oxts_data = dataset.oxts

# Initialize lists to store the X, Y, and Z coordinates
x, y, z = [], [], []

# Extract the transformation matrices "T_w_imu" from OXTS data
T_w_imu = [packet.T_w_imu for packet in oxts_data]

# Extract X, Y, and Z coordinates from the transformation matrices
for transformation_matrix in T_w_imu:
    # Extract the translation part (X, Y, Z)
    translation = transformation_matrix[:3, 3]
    x.append(translation[0])
    y.append(translation[1])
    z.append(translation[2])

# Plot the trajectory in 2D (X-Y)
plt.figure()
plt.plot(x, y)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectory X-Y')
plt.grid(True)

# Plot the trajectory in 2D (Y-Z)
plt.figure()
plt.plot(y, z)
plt.xlabel('Y')
plt.ylabel('Z')
plt.title('Trajectory Y-Z')
plt.grid(True)

# Plot the trajectory in 2D (X-Z)
plt.figure()
plt.plot(x, z)
plt.xlabel('X')
plt.ylabel('Z')
plt.title('Trajectory X-Z')
plt.grid(True)

plt.show()

# Initialize lists to store LiDAR and camera data
pxt, pyt, pzt, R, G, B = [], [], [], [], [], []

# Process data for 50 frames
for i in range(50):
    # Get camera and LiDAR data for the current frame
    img_cam2 = np.array(dataset.get_cam2(i))
    velo = dataset.get_velo(i)

    # Filter out LiDAR points with X values less than 5
    velo = velo[velo[:, 0] > 5]
    velo[:, 3] = 1

    # Transform LiDAR data from LiDAR frame to IMU frame
    test4 = np.linalg.inv(dataset.calib.T_velo_imu) @ velo.T
    testz = dataset.oxts[i].T_w_imu @ test4

    # Define camera calibration and projection matrices
    Tp = dataset.calib.T_cam2_velo
    K = dataset.calib.K_cam2
    mat = np.eye(4)
    mat[0:3, 0:3] = K
    T = mat @ Tp

    # Project LiDAR points into camera frame
    prod = T @ velo.transpose()
    proj = prod[:2] / prod[2]

    # Mask points that are outside the image frame
    mask = ((proj[0] < 0) | (proj[0] > img_cam2.shape[1] - 1) | (proj[1] < 0) | (proj[1] > img_cam2.shape[0] - 1))

    # Collect valid pixel coordinates
    px, py = proj[0][~mask], proj[1][~mask]
    x, y = py.astype(int), px.astype(int)

    # Retrieve RGB color information from the camera
    cam3D = img_cam2[x, y]
    R = np.append(R, cam3D[:, 0])
    G = np.append(G, cam3D[:, 1])
    B = np.append(B, cam3D[:, 2])

    # Store transformed LiDAR data
    pxt = np.append(pxt, testz[0][~mask])
    pyt = np.append(pyt, testz[1][~mask])
    pzt = np.append(pzt, testz[2][~mask])

# Combine 3D point cloud data with color information
X, Y, Z = pxt.ravel(), pyt.ravel(), pzt.ravel()
Red, Green, Blue = R.ravel(), G.ravel(), B.ravel()

# Create a point cloud structure
pct = list(zip(X, Y, Z, Red, Green, Blue))
vertex = np.array(pct, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])

# Write the point cloud data to a PLY file
el = ply.PlyElement.describe(vertex, 'vertex')
ply.PlyData([el]).write('output.ply')
