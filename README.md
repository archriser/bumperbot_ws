# BumperBot - ROS2 Simulation

## Overview
BumperBot is a mobile robot project designed to run in **ROS2**. This project includes a **URDF model** of the robot with **wheels and caster joints**. The robot can be simulated and visualized in **Gazebo** and **RViz**.

## Features
- **URDF Model:** Defines the robot structure.
- **ROS2 Support:** Works with ROS2 Foxy and later.
- **Simulation:** Can be visualized in **RViz** and **Gazebo**.

## Installation
### 1. Install ROS2 (if not installed)
Follow the official ROS2 installation guide:  
[https://docs.ros.org/en/foxy/Installation.html](https://docs.ros.org/en/foxy/Installation.html)

### 2. Create a ROS2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Clone the Repository
```bash
git clone https://github.com/your-repo/bumperbot.git
cd ..
colcon build
source install/setup.bash
```

## Usage
### 1. Convert Xacro to URDF
```bash
ros2 run xacro xacro bumperbot.urdf.xacro > bumperbot.urdf
```

### 2. Check for Errors in URDF
```bash
check_urdf bumperbot.urdf
```

### 3. Visualize in RViz
```bash
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix bumperbot_description`/urdf/bumperbot.urdf
```

### 4. Run in Gazebo
```bash
ros2 launch gazebo_ros gazebo.launch.py
ros2 spawn_entity.py -file `ros2 pkg prefix bumperbot_description`/urdf/bumperbot.urdf -entity bumperbot
```

## Troubleshooting
### 1. URDF Not Showing in RViz
- Make sure the **mesh file paths** are correct.
- Run:
  ```bash
  roscd bumperbot_description/meshes/
  ls
  ```
  Ensure all `.STL` files exist.

### 2. Gazebo Crashes
- Check if `gazebo_ros` is installed:
  ```bash
  ros2 pkg list | grep gazebo_ros
  ```
- Run with verbose logging:
  ```bash
  gazebo --verbose
  ```

### 3. Xacro File Not Processing
- Ensure **xacro** is installed:
  ```bash
  sudo apt install ros-foxy-xacro
  ```
- Run:
  ```bash
  ros2 run xacro xacro --inorder bumperbot.urdf.xacro
  ```

## License
This project follows the **MIT License**. Feel free to modify and distribute.

## Credits
This project is based on the **BumperBot Course on Udemy**.

