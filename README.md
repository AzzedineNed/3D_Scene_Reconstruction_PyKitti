# 3D Scene Reconstruction with PyKitti

## Overview
This project demonstrates the process of reconstructing a 3D scene using the KITTI dataset, which includes data from both LiDAR and camera sensors. By combining the sensor data, a 3D point cloud is generated that can be visualized in color. The point cloud is saved in the PLY format, commonly used for 3D visualizations.

## Dataset
The project uses the **KITTI dataset**, which is a popular dataset for autonomous driving research. It includes a variety of sensor data, such as LiDAR point clouds and camera images. The dataset can be downloaded from the official [KITTI website](https://www.cvlibs.net/datasets/kitti/).

## Project Workflow
- **Data Loading**: The KITTI dataset is loaded using the PyKitti library. The relevant OXTS (Onboard Sensor) data is extracted, which contains transformation matrices for each frame, representing the vehicle’s position and orientation.

- **Trajectory Plotting**: The transformation matrices are used to extract the X, Y, and Z coordinates representing the vehicle’s trajectory. These coordinates are plotted in 2D for three planes: X-Y, Y-Z, and X-Z.

- **LiDAR to Camera Projection**: For each frame, the LiDAR data is filtered, and the points are transformed from the LiDAR frame to the IMU (Inertial Measurement Unit) frame. The points are then projected into the camera frame, and the corresponding RGB values are retrieved from the camera images.

- **Point Cloud Generation**: The LiDAR points, now in the camera frame, are combined with the RGB color values to create a 3D point cloud. This point cloud is saved in the PLY format, which can be opened and visualized using various 3D visualization tools.

## Output
- **Trajectory Plots**: The project generates 2D plots showing the vehicle's trajectory in three planes: X-Y, Y-Z, and X-Z.
- **3D Point Cloud**: A 3D point cloud is created from the LiDAR and camera data. This point cloud is saved in the PLY format as `output.ply` and can be visualized using MeshLab.

## Visuals
Here are some visual representations related to the project:

### KITTI Car Blueprint  
This image shows the top-down view of the car used in the KITTI dataset, providing an idea of its orientation during data collection.  
![KITTI Car Blueprint](https://github.com/AzzedineNed/3D_Scene_Reconstruction_PyKitti/blob/main/setup_top_view.png)

### Car Moving Forward (GIF)  
This GIF shows the forward motion of the car in the KITTI dataset, illustrating its movement over time.  
![Car Moving Forward](https://github.com/AzzedineNed/3D_Scene_Reconstruction_PyKitti/blob/main/KITTI_POV_camera2.gif)

### 3D Scene Reconstruction (GIF)  
This GIF demonstrates the 3D scene reconstruction from the LiDAR and camera data, showing how the 3D point cloud evolves and can be visualized using MeshLab.  
![3D Scene Reconstruction](https://github.com/AzzedineNed/3D_Scene_Reconstruction_PyKitti/blob/main/results.gif)
