# Franka FR3 + MoveIt Servo 조이스틱 텔레오퍼레이션 가이드

## 환경
- Ubuntu 22.04
- ROS2 Humble
- Franka ROS2 설치 완료
- MoveIt2 설치 완료
- Logitech Extreme 3D Pro 조이스틱

---

## 개요

기존 Joint Velocity 방식의 문제점:
- 조이스틱 입력이 각 관절에 직접 매핑됨
- 직관적인 Cartesian 제어 불가능

MoveIt Servo 방식의 장점:
- 조이스틱 입력이 End-Effector의 Cartesian 좌표로 변환됨
- 실시간 Inverse Kinematics 계산
- 직관적인 제어 가능

**구조:**
```
joystick_device_servo.py → /servo_node/delta_twist_cmds → MoveIt Servo → IK 계산 → /joint_velocity_controller/commands → 로봇
```

---

## 1. 사전 요구사항 확인

### MoveIt2 패키지 확인
```bash
ros2 pkg list | grep moveit
```

필수 패키지:
- `moveit_servo`
- `franka_fr3_moveit_config`

### Franka 패키지 확인
```bash
ros2 pkg list | grep franka
```

---

## 2. Servo 설정 파일 생성

### servo_config.yaml 생성
```bash
nano ~/ros2_ws/src/franka_teleop_collection/config/servo_config.yaml
```

```yaml
# MoveIt Servo configuration for Franka FR3

###############################################
# Shared parameters for Servo node
###############################################

use_gazebo: false  # Whether using Gazebo simulation

## Properties of incoming commands
command_in_type: "speed_units"  # "unitless" or "speed_units"
scale:
  # Scale parameters for speed_units mode
  linear: 0.1   # m/s (max linear velocity)
  rotational: 0.2  # rad/s (max rotational velocity)
  joint: 0.5  # rad/s (max joint velocity for joint commands)

# What type of topic does your robot driver expect?
command_out_type: "std_msgs/Float64MultiArray"  # For velocity controller

# What to publish? Can save some bandwidth if not needed
publish_joint_positions: false
publish_joint_velocities: true
publish_joint_accelerations: false

## Plugins for smoothing outgoing commands
use_smoothing: true
smoothing_filter_plugin_name: "online_signal_smoothing::ButterworthFilterPlugin"

## MoveIt properties
move_group_name: "fr3_arm"  # Planning group name
planning_frame: "fr3_link0"  # Base frame of the robot
ee_frame_name: "fr3_link8"  # End-effector frame
robot_link_command_frame: "fr3_link8"  # Frame for interpreting commands

## Incoming Joint State properties
joint_topic: "/joint_states"
override_velocity_scaling_factor: 0.0  # 0 = use MoveIt's default

## Configure handling of singularities and joint limits
lower_singularity_threshold: 17.0  # Start slowing down at this condition number
hard_stop_singularity_threshold: 30.0  # Stop at this condition number
leaving_singularity_threshold_multiplier: 2.0
joint_limit_margins: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # radians

## Topic names
status_topic: "~/status"  # Publish servo status
command_out_topic: "/joint_velocity_controller/commands"  # Velocity commands output

## Collision checking
check_collisions: true
collision_check_rate: 10.0  # Hz
self_collision_proximity_threshold: 0.01  # meters
scene_collision_proximity_threshold: 0.02  # meters

## Input command timeout
incoming_command_timeout: 0.1  # seconds

## Control loop rate
servo_loop_rate: 100.0  # Hz
```

---

## 3. Joystick Device 노드 생성 (Servo용)

### joystick_device_servo.py 생성
```bash
nano ~/ros2_ws/src/franka_teleop_collection/franka_teleop_collection/devices/joystick_device_servo.py
```

```python
"""
ROS2 Joystick Device Node for Logitech Extreme 3D Pro
Modified for MoveIt Servo integration
"""

import numpy as np
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Int32


class JoystickDevice(Node):
    """
    ROS2 node for Logitech Extreme 3D Pro joystick.
    
    Button mapping:
        Trigger (button 0): Toggle gripper
        Button 2: Reset simulation
    
    Axis mapping:
        X axis (left/right): Control y-axis movement
        Y axis (forward/backward): Control x-axis movement  
        Throttle: Control z-axis movement
        Z axis (twist): Control yaw rotation
        HAT: Control pitch/roll rotation
    """

    def __init__(
        self,
        pos_sensitivity=1.0,
        rot_sensitivity=1.0,
    ):
        super().__init__('joystick_device')
        
        self._pos_sensitivity = pos_sensitivity
        self._rot_sensitivity = rot_sensitivity
        
        # Publishers
        # Changed topic name for MoveIt Servo
        self.twist_pub = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )
        self.gripper_pub = self.create_publisher(
            Bool, 
            '/joystick/gripper', 
            10
        )
        self.reset_pub = self.create_publisher(
            Bool, 
            '/joystick/reset', 
            10
        )
        
        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected!")
            raise RuntimeError("No joystick detected!")
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.get_logger().info(f"Connected to: {self.joystick.get_name()}")
        self.get_logger().info(f"Number of axes: {self.joystick.get_numaxes()}")
        self.get_logger().info(f"Number of buttons: {self.joystick.get_numbuttons()}")
        self.get_logger().info(f"Number of hats: {self.joystick.get_numhats()}")
        
        # Button states for toggle detection
        self._button_states = {}
        self._gripper_state = False  # False = open, True = closed
        
        # Deadzone for axes
        self._deadzone = 0.1
        
        # Timer for publishing (50Hz)
        self.timer = self.create_timer(0.02, self.publish_state)
        
    def _apply_deadzone(self, value):
        """Apply deadzone to axis value."""
        if abs(value) < self._deadzone:
            return 0.0
        return value
    
    def publish_state(self):
        """Read joystick and publish state"""
        pygame.event.pump()
        
        # Read axes (same mapping as robosuite code)
        x_axis = self._apply_deadzone(self.joystick.get_axis(0))
        y_axis = self._apply_deadzone(self.joystick.get_axis(1))
        twist = self._apply_deadzone(self.joystick.get_axis(2))
        throttle = self.joystick.get_axis(3)
        
        # Convert throttle from [-1, 1] to [0, 1] range and invert
        z_axis = -throttle
        
        # Position control (x, y, z) - Keep normalized -1 to 1 range
        pos_x = y_axis * self._pos_sensitivity
        pos_y = x_axis * self._pos_sensitivity
        pos_z = (z_axis - 0.5) * self._pos_sensitivity
        
        # Rotation control (roll, pitch, yaw)
        if self.joystick.get_numhats() > 0:
            hat = self.joystick.get_hat(0)
            rot_x = hat[0] * self._rot_sensitivity   # Roll
            rot_y = -hat[1] * self._rot_sensitivity  # Pitch
        else:
            rot_x = 0.0
            rot_y = 0.0
        
        rot_z = twist * self._rot_sensitivity  # Yaw
        
        # Publish twist
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'fr3_link0'  # Base frame for world-fixed control
        twist_msg.twist.linear.x = pos_x
        twist_msg.twist.linear.y = pos_y
        twist_msg.twist.linear.z = pos_z
        twist_msg.twist.angular.x = rot_x
        twist_msg.twist.angular.y = rot_y
        twist_msg.twist.angular.z = rot_z
        self.twist_pub.publish(twist_msg)
        
        # Read buttons
        trigger_pressed = self.joystick.get_button(0)
        reset_button = self.joystick.get_button(2) if self.joystick.get_numbuttons() > 2 else False
        
        # Toggle gripper on trigger press (detect rising edge)
        if trigger_pressed and not self._button_states.get('trigger', False):
            self._gripper_state = not self._gripper_state
            gripper_msg = Bool()
            gripper_msg.data = self._gripper_state
            self.gripper_pub.publish(gripper_msg)
            self.get_logger().info(f"Gripper: {'CLOSED' if self._gripper_state else 'OPEN'}")
        
        self._button_states['trigger'] = trigger_pressed
        
        # Publish reset
        if reset_button:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_pub.publish(reset_msg)
            self.get_logger().info("Reset button pressed")
    
    def destroy_node(self):
        """Clean up on shutdown"""
        if hasattr(self, 'joystick'):
            self.joystick.quit()
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = JoystickDevice(
        pos_sensitivity=1.0,
        rot_sensitivity=1.0
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 4. Launch 파일 생성

### servo_gazebo.launch.py 생성
```bash
nano ~/ros2_ws/src/franka_teleop_collection/launch/servo_gazebo.launch.py
```

```python
"""
Launch file for Gazebo + MoveIt Servo joystick teleoperation.
This launches Servo node to work with already running Gazebo simulation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_name, file_path):
    """Load a yaml file from a package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Declare arguments
    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='true',
        description='Load gripper or not'
    )
    
    load_gripper = LaunchConfiguration('load_gripper')
    
    # Get package paths
    franka_description_path = get_package_share_directory('franka_description')
    teleop_collection_path = get_package_share_directory('franka_teleop_collection')
    
    # Robot description (URDF)
    franka_xacro_file = os.path.join(
        franka_description_path,
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ',
        franka_xacro_file,
        ' hand:=', load_gripper,
        ' robot_ip:=dont-care',
        ' use_fake_hardware:=true',
        ' fake_sensor_commands:=true',
        ' ros2_control:=true'
    ])
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_config, value_type=str)
    }
    
    # Semantic description (SRDF)
    franka_semantic_xacro_file = os.path.join(
        franka_description_path,
        'robots', 'fr3', 'fr3.srdf.xacro'
    )
    
    robot_description_semantic_config = Command([
        FindExecutable(name='xacro'), ' ',
        franka_semantic_xacro_file,
        ' hand:=', load_gripper
    ])
    
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            robot_description_semantic_config, value_type=str
        )
    }
    
    # Kinematics configuration
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    
    # Servo configuration
    servo_yaml_path = os.path.join(teleop_collection_path, 'config', 'servo_config.yaml')
    with open(servo_yaml_path, 'r') as f:
        servo_params = yaml.safe_load(f)
    
    servo_params_with_ns = {'moveit_servo': servo_params}
    
    # MoveIt Servo node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            servo_params_with_ns,
        ],
    )
    
    # Auto start servo service
    start_servo = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger'],
        output='screen',
    )
    
    delayed_start_servo = TimerAction(
        period=3.0,
        actions=[start_servo],
    )
    
    return LaunchDescription([
        load_gripper_arg,
        servo_node,
        delayed_start_servo,
    ])
```

---

## 5. setup.py 수정

```bash
nano ~/ros2_ws/src/franka_teleop_collection/setup.py
```

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'franka_teleop_collection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choi',
    maintainer_email='cpsc.inhyuk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_node = franka_teleop_collection.devices.joystick_device:main',
            'joystick_bridge = franka_teleop_collection.joystick_to_joint_bridge:main',
            'joystick_servo = franka_teleop_collection.devices.joystick_device_servo:main',
        ],
    },
)
```

---

## 6. package.xml 수정

```bash
nano ~/ros2_ws/src/franka_teleop_collection/package.xml
```

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>franka_teleop_collection</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="cpsc.inhyuk@gmail.com">choi</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>moveit_servo</depend>
  <depend>moveit_ros_planning_interface</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## 7. 패키지 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select franka_teleop_collection
source ~/ros2_ws/install/setup.bash
```

---

## 8. 실행 방법 (Gazebo 시뮬레이션)

### 터미널 1: Gazebo 실행
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch franka_gazebo_bringup gazebo_joint_velocity_controller_example.launch.py load_gripper:=true
```

### 터미널 2: Velocity Controller 설정 (Gazebo 완전히 로드된 후)
```bash
source ~/ros2_ws/install/setup.bash
ros2 control set_controller_state joint_velocity_example_controller inactive
ros2 run controller_manager spawner joint_velocity_controller --controller-type velocity_controllers/JointGroupVelocityController --param-file ~/ros2_ws/src/franka_teleop_collection/config/franka_velocity_controller.yaml
```

### 터미널 3: MoveIt Servo 실행
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch franka_teleop_collection servo_gazebo.launch.py
```

### 터미널 4: Joystick 노드 실행
```bash
source ~/ros2_ws/install/setup.bash
ros2 run franka_teleop_collection joystick_servo
```

---

## 9. 조이스틱 매핑 (MoveIt Servo 버전)

### 축 (Axes) - End-Effector Cartesian 제어
| 조이스틱 입력 | End-Effector 동작 |
|-------------|------------------|
| Y축 (앞/뒤) | X 방향 이동 (앞/뒤) |
| X축 (좌/우) | Y 방향 이동 (좌/우) |
| Throttle | Z 방향 이동 (상/하) |
| Twist (비틀기) | Yaw 회전 |
| HAT 좌/우 | Roll 회전 |
| HAT 상/하 | Pitch 회전 |

### 버튼 (Buttons)
| 버튼 | 기능 |
|-----|------|
| Trigger (0번) | 그리퍼 열기/닫기 토글 |
| 버튼 2 | 리셋 |

### 좌표계
- **Base Frame (fr3_link0)**: 월드 고정 좌표계 기준으로 제어
- 조이스틱을 앞으로 밀면 로봇이 항상 같은 방향(앞)으로 이동

---

## 10. Published Topics

| 토픽 | 타입 | 설명 |
|-----|------|------|
| `/servo_node/delta_twist_cmds` | geometry_msgs/TwistStamped | Cartesian 속도 명령 |
| `/joystick/gripper` | std_msgs/Bool | 그리퍼 상태 |
| `/joystick/reset` | std_msgs/Bool | 리셋 버튼 |
| `/joint_velocity_controller/commands` | std_msgs/Float64MultiArray | Joint velocity 명령 (Servo 출력) |

---

## 11. 속도 조절

### servo_config.yaml에서 조절
```yaml
scale:
  linear: 0.1      # m/s - 값을 줄이면 느려짐
  rotational: 0.2  # rad/s - 값을 줄이면 느려짐
```

수정 후 빌드 및 Servo 재시작 필요.

---

## 12. 디버깅 명령어

### 조이스틱 입력 확인
```bash
ros2 topic echo /servo_node/delta_twist_cmds
```

### Servo 출력 확인
```bash
ros2 topic echo /joint_velocity_controller/commands
```

### Servo 상태 확인
```bash
ros2 topic echo /servo_node/status
```

### 컨트롤러 상태 확인
```bash
ros2 control list_controllers
```

### Servo 서비스 목록
```bash
ros2 service list | grep servo
```

### Servo 수동 시작 (자동 시작 안 될 경우)
```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
```

---

## 13. 트러블슈팅

### 로봇이 움직이지 않음
1. Servo가 시작되었는지 확인:
   ```bash
   ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
   ```

2. `/joint_states` 토픽이 발행되고 있는지 확인:
   ```bash
   ros2 topic echo /joint_states
   ```

3. Velocity controller가 active인지 확인:
   ```bash
   ros2 control list_controllers
   ```

### 방향이 이상함
- `frame_id` 확인: `fr3_link0` (base) vs `fr3_link8` (end-effector)
- joystick_device_servo.py에서 축 매핑 수정

### 속도가 너무 빠르거나 느림
- servo_config.yaml의 `scale.linear`와 `scale.rotational` 값 조절

### Servo 노드 에러
- `publish_joint_positions`와 `publish_joint_velocities` 중 하나만 true로 설정

---

## 14. Joint Velocity vs MoveIt Servo 비교

| 항목 | Joint Velocity 방식 | MoveIt Servo 방식 |
|-----|-------------------|------------------|
| 제어 방식 | 관절 직접 제어 | Cartesian 제어 (IK 자동 계산) |
| 직관성 | 낮음 | 높음 |
| 필요 노드 | joystick_node + joystick_bridge | joystick_servo + servo_node |
| 좌표계 | 각 관절 기준 | Base 또는 EE 프레임 기준 |
| 복잡도 | 단순 | MoveIt 설정 필요 |

---

## 15. 실제 로봇 적용 가이드

### 1. 속도 제한 (필수)
servo_config.yaml에서 안전한 값으로 설정:
```yaml
scale:
  linear: 0.05    # 시작은 매우 느리게
  rotational: 0.1
```

### 2. 충돌 체크 활성화
```yaml
check_collisions: true
self_collision_proximity_threshold: 0.02
scene_collision_proximity_threshold: 0.03
```

### 3. 실행 순서 (실제 로봇)

터미널 1 - 로봇 연결:
```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2
```

터미널 2 - MoveIt Servo:
```bash
ros2 launch franka_teleop_collection servo_gazebo.launch.py
```

터미널 3 - Joystick:
```bash
ros2 run franka_teleop_collection joystick_servo
```

---

## 참고 자료

- [MoveIt Servo Documentation](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [Franka ROS2 Documentation](https://github.com/frankaemika/franka_ros2)
- [ROS2 Control Documentation](https://control.ros.org/)
