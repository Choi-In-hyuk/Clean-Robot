# ROS2 Humble + Franka Panda Setup Guide

## System Requirements
- Ubuntu 22.04 LTS
- Sufficient disk space (ROS2 + Gazebo + MoveIt2 requires ~5GB)

## 1. ROS2 Humble Installation

### Set locale
```bash
locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

### Setup sources
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### Install ROS2 packages
```bash
sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

### Environment setup
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify installation
```bash
ros2 run demo_nodes_cpp talker
```

## 2. Gazebo, MoveIt2, and Franka Packages
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
sudo apt install ros-humble-moveit
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-joint-state-publisher-gui ros-humble-xacro
sudo apt install ros-humble-moveit-resources-panda-moveit-config
```

## 3. Franka ROS2 Workspace Build

### Create workspace and clone repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/frankaemika/franka_ros2.git -b humble
```

### Install dependencies
```bash
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Build workspace
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_ws/install/setup.bash
```

## 4. Running Simulations

### MoveIt2 Panda Demo (RViz only)
```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

### Franka Gazebo Simulation with Gripper
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py load_gripper:=true
```

## Available Launch Files

### Gazebo Examples
```bash
ros2 launch franka_gazebo_bringup gazebo_joint_impedance_controller_example.launch.py load_gripper:=true
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py load_gripper:=true
ros2 launch franka_gazebo_bringup gazebo_joint_velocity_controller_example.launch.py load_gripper:=true
```

### Check Launch Arguments
```bash
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py --show-args
```

## Troubleshooting

### Source workspace in every new terminal
```bash
source ~/ros2_ws/install/setup.bash
```

### Or add to bashrc
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```
