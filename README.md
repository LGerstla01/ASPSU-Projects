# Autonomous Systems: Perception and Situation Understanding

This repository contains the project work for the course **Autonomous Systems: Perception and Situation Understanding** (ASPSU), completed as part of the Master's program at HHN.

## Project Structure

The project is divided into two main subprojects:

---

## 1. Perception

This subproject focuses on the perception pipeline for autonomous systems, including:

- **Stereo Vision and 3D Reconstruction:**  
  - Projection of 2D image points to 3D world coordinates using stereo images and camera calibration.
  - Validation of 3D reconstruction using objects of known size.

- **Stereo Disparity Map Estimation:**  
  - Implementation of a block-matching algorithm using Normalized Cross-Correlation (NCC) for disparity map computation.
  - Comparison with OpenCV's StereoBM algorithm.

- **LiDAR Data Registration:**  
  - Registration of LiDAR point clouds using odometry-based initialization and Iterative Closest Point (ICP) refinement.
  - Visualization and analysis of registration results.

All perception tasks are documented in Jupyter notebooks located in the `Perception/Projekt` directory.

---

## 2. Situation Understanding

This subproject addresses the higher-level understanding and estimation tasks, including:

- **Multi-Robot Tracking:**  
  - Implementation of an Extended Kalman Filter (EKF) for tracking multiple robots in a simulated environment.
  - Data association, state estimation, and visualization using ROS2 and RViz.

- **Sensor Fusion:**  
  - Integration of camera and lidar detections for robust state estimation.

- **Simulation and Visualization:**  
  - Launch files and scripts for running the full simulation, including robot controllers, sensor nodes, and visualization tools.

The situation understanding code is located in the `Situation Understanding` directory, with ROS2 packages and launch files under `src/`.

---

## Getting Started

### Prerequisites

- Python 3.x
- ROS2 Foxy or newer (for Situation Understanding)
- OpenCV, NumPy, Matplotlib, Open3D (for Perception notebooks)
- Jupyter Notebook

### Running Perception Notebooks

1. Navigate to the `Perception/Projekt` directory.
2. Open the desired notebook (`1 Point projection and coordinate transformation.ipynb`, etc.) in Jupyter.
3. Ensure the required data files are present in the `Additional_files` subdirectory.

### Running Situation Understanding (ROS2)

1. Build the ROS2 workspace:
    ```bash
    colcon build
    source install/setup.bash
    ```
2. Launch the simulation:
    ```bash
    ros2 launch robot_estimator simulation.launch.py
    ```
3. Visualize in RViz using the provided configuration.

---

## Authors

- Lukas Gerstlauer
- Jakob Kurz

---

## License

This project is licensed under the Apache License 2.0. See the `LICENSE` files in each subproject for details.
