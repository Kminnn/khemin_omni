# Khemin Omni Robot Gazebo Simulation

<img width="1018" height="632" alt="Screenshot from 2025-08-22 02-04-45" src="https://github.com/user-attachments/assets/53473e78-9651-49ce-91b0-ae6205ca57bf" />

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

<pre lang="markdown"> ros2 launch khemin_omni gazebo.launch.py </pre>

![Screencast from 2025-08-23 17-17-31 (online-video-cutter com)](https://github.com/user-attachments/assets/62e4a1c7-73e2-4406-86fa-cfd546db16e1)








