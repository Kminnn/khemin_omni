# Khemin Omni Robot Gazebo Simulation

<img width="1018" height="632" alt="Screenshot from 2025-08-22 02-04-45-Photoroom" src="https://github.com/user-attachments/assets/353fae6d-f4e3-403b-a09c-e4dad657b0de" />

## Introduction

This repository contains the source code, models, and configurations for a custom omni-directional mobile robot.

The project includes:

* **Robot Models**: The URDF and mesh files (`.stl`) for the robot's visual and collision properties, suitable for simulation.
* **ROS Packages**: Packages for the robot's control (`omni_wheel_drive_controller`), hardware interface, and sensor integration.
* **Simulation Environments**: Files for simulating the robot in a Gazebo environment, useful for testing navigation, control, and sensor data.

### Acknowledgements

The map files used in this repository are sourced from a pre-existing project: `ros2_omni_robot_sim` by `YePeOn7`.

### Tested on

- Ubuntu 24.04
- ROS Jazzy
- Gazebo Harmonic

## How to Use This Repository

Please clone this repository to your home directory.

<pre lang="markdown"> git clone https://github.com/Kminnn/khemin_omni.git  </pre>

Install dependency by running the following command

<pre lang="markdown">
sudo apt install -y ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-controller-manager \
  ros-jazzy-twist-mux \
  ros-jazzy-robot-localization \
  ros-jazzy-spatio-temporal-voxel-layer \
  ros-jazzy-topic-tools \
  ros-jazzy-slam-toolbox </pre>

Don't forget to build and source the package. 

```bash
cd ~/khemin_omni/
colcon build
source install/setup.bash
```

### 1. Manual Teleop

<pre lang="markdown"> ros2 launch khemin_omni gazebo.launch.py </pre>

Open another terminal then run teleop command.

<pre lang="markdown"> ros2 run teleop_twist_keyboard teleop_twist_keyboard </pre>

![gazebo](https://github.com/user-attachments/assets/e83167b0-990b-432a-a17e-313b701a35e1)

### 2. SLAM with slam_toolbox

Start the SLAM by using the following command

<pre lang="markdown"> ros2 launch khemin_omni slam.launch.py </pre>

Open another terminal then run teleop command.

<pre lang="markdown"> ros2 run teleop_twist_keyboard teleop_twist_keyboard </pre>

<img width="1853" height="1013" alt="Screenshot from 2025-08-23 17-05-50" src="https://github.com/user-attachments/assets/58e9af39-0caf-4d76-94cd-20febebb88af" />

### 3. Navigation

<pre lang="markdown"> ros2 launch khemin_omni navigation.launch.py  </pre>

The `Nav2 Goal` tool in `RViz` lets you send a navigation target to your robot. Simply click on the map for the desired position, drag the mouse to set the orientation, and the robot will begin moving to the selected goal.

![Screencast from 2025-08-23 17-17-31 (online-video-cutter com)](https://github.com/user-attachments/assets/62e4a1c7-73e2-4406-86fa-cfd546db16e1)

To view the camera stream from the robot, you can use the `rqt_image_view` tool.

![Screencast from 2025-08-25 13-51-56(1) (online-video-cutter com)](https://github.com/user-attachments/assets/b30c388f-da14-4a2a-8407-ee7ee485982d)





