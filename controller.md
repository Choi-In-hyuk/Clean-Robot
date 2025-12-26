# Franka FR3 Gazebo Controller Setup Guide

## 시스템 구성

### 하드웨어/소프트웨어 환경
- Ubuntu 22.04 LTS
- ROS2 Humble
- Franka ROS2 (FR3 model)
- Gazebo Simulation

### 로봇 구성
- **로봇 모델**: Franka Research 3 (FR3)
- **Joint 개수**: 7 DOF
- **Gripper**: fr3_finger_joint1
- **Joint 이름**:
  - fr3_joint1
  - fr3_joint2
  - fr3_joint3
  - fr3_joint4
  - fr3_joint5
  - fr3_joint6
  - fr3_joint7

## 컨트롤러 설정 과정

### 1. 기존 Controller 확인

**사용 가능한 Controller 타입 확인:**
```bash
ros2 control list_controller_types
```

**주요 컨트롤러:**
- `franka_example_controllers/JointPositionExampleController` - Pre-programmed 동작만 수행
- `franka_example_controllers/CartesianPoseExampleController` - Cartesian pose 예제
- `franka_example_controllers/CartesianVelocityExampleController` - Cartesian velocity 예제
- `position_controllers/JointGroupPositionController` - External command 수신 가능 ✓
- `joint_trajectory_controller/JointTrajectoryController` - Trajectory 기반 제어

### 2. YAML 설정 파일 수정

**파일 위치:** 
```
~/ros2_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml
```

**백업:**
```bash
cp ~/ros2_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml ~/ros2_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml.backup
```

**수정된 전체 YAML 내용:**
```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    joint_position_example_controller:
      type: franka_example_controllers/JointPositionExampleController

    joint_velocity_example_controller:
      type: franka_example_controllers/JointVelocityExampleController

    joint_impedance_example_controller:
      type: franka_example_controllers/JointImpedanceExampleController

    # OpenVLA control - Joint position controller
    joint_group_position_controller:
      type: position_controllers/JointGroupPositionController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_position_example_controller:
  ros__parameters:
    gazebo: true

joint_velocity_example_controller:
  ros__parameters:
    gazebo: true
  
joint_impedance_example_controller:
  ros__parameters:
    k_gains:
      - 24.0
      - 24.0
      - 24.0
      - 24.0
      - 10.0
      - 6.0
      - 2.0
    d_gains:
      - 2.0
      - 2.0
      - 2.0
      - 1.0
      - 1.0
      - 1.0
      - 0.5

# OpenVLA joint group position controller configuration
joint_group_position_controller:
  ros__parameters:
    joints:
      - fr3_joint1
      - fr3_joint2
      - fr3_joint3
      - fr3_joint4
      - fr3_joint5
      - fr3_joint6
      - fr3_joint7
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### 3. Workspace 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select franka_gazebo_bringup
source ~/ros2_ws/install/setup.bash
```

### 4. Gazebo 실행 및 Controller 전환

**Terminal 1: Gazebo 실행**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py load_gripper:=true
```

**Terminal 2: Controller 전환 (Gazebo 완전히 로드된 후)**
```bash
source ~/ros2_ws/install/setup.bash

# 현재 컨트롤러 확인
ros2 control list_controllers

# Example controller 정지
ros2 control set_controller_state joint_position_example_controller inactive

# 새 컨트롤러 로드
ros2 control load_controller joint_group_position_controller

# Configure
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: 'joint_group_position_controller'}"

# Activate (Switch)
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['joint_group_position_controller'], deactivate_controllers: ['joint_position_example_controller'], strictness: 1, activate_asap: true, timeout: {sec: 0, nanosec: 0}}"

# 최종 확인
ros2 control list_controllers
ros2 topic list | grep joint_group
```

## 테스트 결과

### Controller 상태 확인
```bash
ros2 control list_controllers
```

**출력 결과:**
```
joint_state_broadcaster           joint_state_broadcaster/JointStateBroadcaster              active  
joint_position_example_controller franka_example_controllers/JointPositionExampleController  inactive
joint_group_position_controller   position_controllers/JointGroupPositionController          active  
```

### Command Topic 확인
```bash
ros2 topic list | grep joint_group
```

**출력 결과:**
```
/joint_group_position_controller/commands
/joint_group_position_controller/transition_event
```

### 메시지 타입 확인
```bash
ros2 topic info /joint_group_position_controller/commands
```

**출력 결과:**
```
Type: std_msgs/msg/Float64MultiArray
Publisher count: 0
Subscription count: 1
```

### Joint States 확인
```bash
ros2 topic echo /joint_states --once
```

**출력 예시:**
```
header:
  stamp:
    sec: 179
    nanosec: 13000000
  frame_id: base_link
name:
- fr3_joint1
- fr3_joint3
- fr3_joint2
- fr3_joint4
- fr3_joint5
- fr3_joint6
- fr3_joint7
- fr3_finger_joint1
position:
- 0.077
- 0.077
- -0.708
- -2.279
- -0.077
- 1.647
- 0.862
- 0.000
velocity: [...]
effort: [...]
```

### 제어 명령 테스트
```bash
# 간단한 joint position 명령
ros2 topic pub --once /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, -0.3, 0.0, -2.0, 0.0, 1.5, 0.78]"
```

**결과:** 로봇이 지정된 joint 위치로 이동 ✓

## 다음 단계

### OpenVLA 연동을 위한 Python 패키지 개발

**디렉토리 구조:**
```
~/choi_ws/openvla-oft/experiments/robot/franka/
├── constants.py              # FR3 로봇 상수
├── requirements_franka.txt   # 의존성
├── franka_utils.py           # FK/IK, 좌표 변환
├── gazebo_env.py             # ROS2 Gazebo 환경 wrapper
└── run_franka_gazebo_eval.py # OpenVLA 연동 메인
```

**개발 항목:**
1. **constants.py** - Joint limits, gripper range, 좌표계 정의
2. **franka_utils.py** - FK/IK solver (PyKDL), delta pose 적용
3. **gazebo_env.py** - ROS2 node, joint state subscriber, command publisher
4. **run_franka_gazebo_eval.py** - OpenVLA server 연동, 평가 루프

**제어 흐름:**
```
OpenVLA Model (delta poses)
    ↓ [delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz, gripper]
Gazebo Env (gazebo_env.py)
    ↓ FK: 현재 end-effector pose 계산
    ↓ Delta 적용: 목표 pose = 현재 pose + delta
    ↓ IK: 목표 joint angles 계산
    ↓ Publish to /joint_group_position_controller/commands
Franka FR3 Robot in Gazebo
```

## 참고사항

### Controller Manager 주요 서비스
- `/controller_manager/configure_controller` - Controller 설정
- `/controller_manager/switch_controller` - Controller 전환
- `/controller_manager/list_controllers` - Controller 목록
- `/controller_manager/load_controller` - Controller 로드
- `/controller_manager/unload_controller` - Controller 언로드

### 중요 Topic
- `/joint_states` - 현재 joint 상태 (sensor_msgs/msg/JointState)
- `/joint_group_position_controller/commands` - Joint position 명령 (std_msgs/msg/Float64MultiArray)
- `/tf` - Transform tree

### OpenVLA 액션 포맷
- **LIBERO 모델 출력**: `[delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz, gripper]`
- **주파수**: 71-110 Hz
- **좌표계**: Base frame 기준 delta
- **Gripper**: -1 (close), 1 (open)
