# Franka + Logitech Extreme 3D Pro 조이스틱 설정 가이드

## 환경
- Ubuntu 22.04
- ROS2 Humble
- Franka ROS2 설치 완료
- Logitech Extreme 3D Pro 조이스틱

## 1. 패키지 확인

### Franka 패키지 확인
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep franka
```

## 2. RealSense 설치
```bash
sudo apt update
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
```

### 설치 확인
```bash
ros2 pkg list | grep realsense
```

## 3. 조이스틱 패키지 설치
```bash
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy jstest-gtk
```

## 4. 조이스틱 연결 및 테스트

### 장치 인식 확인
```bash
ls /dev/input/js*
```

출력 예시: `/dev/input/js0`

### 조이스틱 테스트
```bash
jstest /dev/input/js0
```

조이스틱을 움직이고 버튼을 눌러서 값이 변하는지 확인. 종료는 `Ctrl+C`.

**Extreme 3D Pro 스펙:**
- Axes: 4개
- Buttons: 12개  
- Hats: 1개

## 5. ROS2 데이터 수집 패키지 생성

### 패키지 생성
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python franka_teleop_collection
```

### 디렉토리 구조 생성
```bash
cd ~/ros2_ws/src/franka_teleop_collection
mkdir -p franka_teleop_collection/devices
mkdir -p franka_teleop_collection/collectors
mkdir -p launch
mkdir -p config
```

### 구조 확인
```bash
tree -L 2
```

## 6. Python 모듈 초기화 파일 생성
```bash
touch ~/ros2_ws/src/franka_teleop_collection/franka_teleop_collection/devices/__init__.py
touch ~/ros2_ws/src/franka_teleop_collection/franka_teleop_collection/collectors/__init__.py
```

## 7. Joystick 디바이스 노드 작성

### 파일 생성
```bash
nano ~/ros2_ws/src/franka_teleop_collection/franka_teleop_collection/devices/joystick_device.py
```

### 코드 작성
```python
"""
ROS2 Joystick Device Node for Logitech Extreme 3D Pro
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
        self.twist_pub = self.create_publisher(
            TwistStamped, 
            '/joystick/twist', 
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
        
        # Position control (x, y, z) - Counter-clockwise 90 degree rotation
        pos_x = y_axis * self._pos_sensitivity * 0.001
        pos_y = x_axis * self._pos_sensitivity * 0.001
        pos_z = (z_axis - 0.5) * self._pos_sensitivity * 0.001
        
        # Rotation control (roll, pitch, yaw)
        if self.joystick.get_numhats() > 0:
            hat = self.joystick.get_hat(0)
            rot_x = -hat[1] * self._rot_sensitivity * 0.002  # Roll
            rot_y = hat[0] * self._rot_sensitivity * 0.002   # Pitch
        else:
            rot_x = 0.0
            rot_y = 0.0
        
        rot_z = twist * self._rot_sensitivity * 0.002  # Yaw
        
        # Publish twist
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'joystick'
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
        pos_sensitivity=1.5,
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

## 8. setup.py 수정
```bash
nano ~/ros2_ws/src/franka_teleop_collection/setup.py
```

`entry_points`의 `console_scripts` 부분을 다음과 같이 수정:
```python
    entry_points={
        'console_scripts': [
            'joystick_node = franka_teleop_collection.devices.joystick_device:main',
        ],
    },
```

## 9. package.xml 수정
```bash
nano ~/ros2_ws/src/franka_teleop_collection/package.xml
```

`<license>` 태그 다음에 의존성 추가:
```xml
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
```

## 10. pygame 설치
```bash
pip install pygame
```

## 11. 패키지 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select franka_teleop_collection
```

## 12. 조이스틱 노드 실행 및 테스트

### 노드 실행
```bash
source ~/ros2_ws/install/setup.bash
ros2 run franka_teleop_collection joystick_node
```

예상 출력:
```
[INFO] [joystick_device]: Connected to: Logitech Extreme 3D
[INFO] [joystick_device]: Number of axes: 4
[INFO] [joystick_device]: Number of buttons: 12
[INFO] [joystick_device]: Number of hats: 1
```

### 토픽 확인 (새 터미널)
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /joystick/twist
```

조이스틱을 움직이면 값이 변하는 것을 확인할 수 있음.

## Published Topics

- `/joystick/twist` (geometry_msgs/TwistStamped): 위치 및 회전 속도 명령
- `/joystick/gripper` (std_msgs/Bool): 그리퍼 상태 (toggle)
- `/joystick/reset` (std_msgs/Bool): 리셋 버튼

## 조이스틱 매핑

### 축 (Axes)
- X축 (좌/우): Y 방향 이동
- Y축 (앞/뒤): X 방향 이동
- Throttle: Z 방향 이동
- Twist: Yaw 회전
- HAT: Roll/Pitch 회전

### 버튼 (Buttons)
- Trigger (0번): 그리퍼 열기/닫기 토글
- 버튼 2: 리셋

## 다음 단계

- [ ] Gazebo에서 로봇 제어 테스트
- [ ] RealSense 카메라 통합
- [ ] 데이터 수집 파이프라인 구축

## 13. 조이스틱-로봇 브릿지 노드 작성

조이스틱 입력을 joint velocity 명령으로 변환하는 브릿지 노드를 만듭니다.
```bash
nano ~/ros2_ws/src/franka_teleop_collection/franka_teleop_collection/joystick_to_joint_bridge.py
```
```python
"""
Bridge node: Joystick twist -> Joint velocity commands
Simple mapping without IK for quick testing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class JoystickToJointBridge(Node):
    """
    Subscribes to /joystick/twist
    Publishes to /joint_velocity_controller/commands
    """
    
    def __init__(self):
        super().__init__('joystick_to_joint_bridge')
        
        # Subscribe to joystick twist
        self.twist_sub = self.create_subscription(
            TwistStamped,
            '/joystick/twist',
            self.twist_callback,
            10
        )
        
        # Publish joint velocities
        self.joint_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_velocity_controller/commands',
            10
        )
        
        self.get_logger().info("Joystick to Joint Bridge started")
        
    def twist_callback(self, msg):
        """
        Convert twist to joint velocities
        Simple mapping: use first 6 DOF
        """
        joint_vel = Float64MultiArray()
        
        # Map twist to joints with higher gain
        joint_vel.data = [
            msg.twist.linear.x * 200.0,   # Joint 1
            msg.twist.linear.y * 200.0,   # Joint 2
            msg.twist.linear.z * 200.0,   # Joint 3
            msg.twist.angular.x * 100.0,  # Joint 4
            msg.twist.angular.y * 100.0,  # Joint 5
            msg.twist.angular.z * 100.0,  # Joint 6
            0.0                            # Joint 7
        ]
        
        self.joint_vel_pub.publish(joint_vel)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickToJointBridge()
    
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

## 14. setup.py에 브릿지 노드 추가
```bash
nano ~/ros2_ws/src/franka_teleop_collection/setup.py
```

`entry_points`에 브릿지 노드 추가:
```python
    entry_points={
        'console_scripts': [
            'joystick_node = franka_teleop_collection.devices.joystick_device:main',
            'joystick_bridge = franka_teleop_collection.joystick_to_joint_bridge:main',
        ],
    },
```

## 15. 패키지 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select franka_teleop_collection
source ~/ros2_ws/install/setup.bash
```

## 16. JointGroupVelocityController 설정 파일 작성

표준 velocity controller를 사용하기 위한 설정 파일을 만듭니다.
```bash
nano ~/ros2_ws/src/franka_teleop_collection/config/franka_velocity_controller.yaml
```
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    
    joint_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

joint_velocity_controller:
  ros__parameters:
    joints:
      - fr3_joint1
      - fr3_joint2
      - fr3_joint3
      - fr3_joint4
      - fr3_joint5
      - fr3_joint6
      - fr3_joint7
```

## 17. Gazebo에서 조이스틱으로 로봇 제어 테스트

### 터미널 1: Gazebo 실행
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch franka_gazebo_bringup gazebo_joint_velocity_controller_example.launch.py load_gripper:=true
```

### 터미널 2: 표준 Velocity Controller 로드

Gazebo가 완전히 로드된 후:
```bash
source ~/ros2_ws/install/setup.bash
ros2 control set_controller_state joint_velocity_example_controller inactive
ros2 run controller_manager spawner joint_velocity_controller --controller-type velocity_controllers/JointGroupVelocityController --param-file ~/ros2_ws/src/franka_teleop_collection/config/franka_velocity_controller.yaml
```

예상 출력:
```
[INFO] [spawner_joint_velocity_controller]: Loaded joint_velocity_controller
[INFO] [spawner_joint_velocity_controller]: Configured and activated joint_velocity_controller
```

### 터미널 3: 조이스틱 노드 실행
```bash
source ~/ros2_ws/install/setup.bash
ros2 run franka_teleop_collection joystick_node
```

### 터미널 4: 브릿지 노드 실행
```bash
source ~/ros2_ws/install/setup.bash
ros2 run franka_teleop_collection joystick_bridge
```

조이스틱을 움직이면 Gazebo에서 로봇이 움직여야 합니다!

## 18. 디버깅 및 확인 명령어

### 컨트롤러 상태 확인
```bash
ros2 control list_controllers
```

### 토픽 확인
```bash
ros2 topic list | grep command
ros2 topic echo /joint_velocity_controller/commands
```

### 조이스틱 입력 확인
```bash
ros2 topic echo /joystick/twist
```

## 주의사항

- **gain 값**: 브릿지 노드의 gain 값(200.0, 100.0)은 조이스틱 민감도에 따라 조정 필요
- **안전성**: 실제 로봇에서는 속도 제한과 안전 체크 추가 필요
- **매핑**: 현재는 간단한 1:1 매핑이며, 실제로는 IK(Inverse Kinematics) 필요

## 다음 단계

- [ ] RealSense 카메라 통합
- [ ] 데이터 수집 파이프라인 구축
- [ ] 실제 로봇 테스트
- [ ] Inverse Kinematics 구현으로 Cartesian control 개선
