'''
python A_collect_demonstration_with_images.py \
  --bddl-file /home/choi/LIBERO/libero/libero/bddl_files/libero_object/pick_up_the_alphabet_soup_and_place_it_in_the_basket.bddl \
  --device keyboard \
  --robots Panda \
  --num-demonstration 1

python A_collect_demonstration_with_images.py \
  --bddl-file /home/choi/LIBERO/libero/libero/bddl_files/libero_90/KITCHEN_SCENE6_put_the_yellow_and_white_mug_in_the_microwave.bddl \
  --device joystick \
  --robots Panda \
  --num-demonstration 1
'''

import argparse
import cv2
import datetime
import h5py
import init_path
import json
import numpy as np
import os
import robosuite as suite
import time
from glob import glob
from robosuite import load_controller_config
from robosuite.wrappers import VisualizationWrapper
from robosuite.utils.input_utils import input2action

import libero.libero.envs.bddl_utils as BDDLUtils
from libero.libero.envs import *


class ImageDataCollectionWrapper:
    """
    Custom wrapper for collecting demonstration data with images.
    Stores images, actions, robot states, and joint states.
    """
    def __init__(self, env, directory, camera_names, camera_height=256, camera_width=256):
        self.env = env
        self.directory = directory
        self.camera_names = camera_names
        self.camera_height = camera_height
        self.camera_width = camera_width
        
        # Create directory
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"Created directory: {directory}")
        
        # Episode data storage
        self.reset_episode_data()
        
        # Episode counter
        self.ep_count = 0
        self.ep_directory = None
        
    def reset_episode_data(self):
        """Reset episode data storage"""
        self.episode_images = {cam: [] for cam in self.camera_names}
        self.episode_actions = []
        self.episode_eef_pos = []
        self.episode_eef_quat = []
        self.episode_gripper_qpos = []
        self.episode_joint_pos = []
        self.episode_rewards = []
        
    def reset(self):
        """Reset environment and start new episode"""
        self.reset_episode_data()
        obs = self.env.reset()
        self._store_observation(obs)
        return obs
    
    def _store_observation(self, obs):
        """Store observation data"""
        # Store images from each camera
        for cam in self.camera_names:
            img_key = f"{cam}_image"
            if img_key in obs:
                # Convert BGR to RGB if needed, and ensure uint8
                img = obs[img_key]
                if img.dtype != np.uint8:
                    img = (img * 255).astype(np.uint8)
                self.episode_images[cam].append(img)
        
        # Store robot state
        # EEF position (3D)
        if "robot0_eef_pos" in obs:
            self.episode_eef_pos.append(obs["robot0_eef_pos"].copy())
        
        # EEF quaternion (4D)
        if "robot0_eef_quat" in obs:
            self.episode_eef_quat.append(obs["robot0_eef_quat"].copy())
        
        # Gripper state (2D for Panda)
        if "robot0_gripper_qpos" in obs:
            self.episode_gripper_qpos.append(obs["robot0_gripper_qpos"].copy())
        
        # Joint positions (7D for Panda)
        if "robot0_joint_pos" in obs:
            self.episode_joint_pos.append(obs["robot0_joint_pos"].copy())
    
    def step(self, action):
        """Take a step and store data"""
        obs, reward, done, info = self.env.step(action)
        
        # Store action (before storing next observation)
        self.episode_actions.append(action.copy())
        self.episode_rewards.append(reward)
        
        # Store observation
        self._store_observation(obs)
        
        return obs, reward, done, info
    
    def save_episode(self, language_instruction, success=True):
        """Save collected episode data to HDF5"""
        if len(self.episode_actions) == 0:
            print("No data to save")
            return None
        
        self.ep_count += 1
        t1, t2 = str(time.time()).split(".")
        self.ep_directory = os.path.join(self.directory, f"ep_{t1}_{t2}")
        os.makedirs(self.ep_directory, exist_ok=True)
        
        hdf5_path = os.path.join(self.ep_directory, "demo.hdf5")
        
        with h5py.File(hdf5_path, "w") as f:
            # Create data group
            grp = f.create_group("data")
            demo_grp = grp.create_group("demo_1")
            obs_grp = demo_grp.create_group("obs")
            
            # Save images for each camera
            for cam in self.camera_names:
                if len(self.episode_images[cam]) > 0:
                    # Remove last observation (we want obs[t] -> action[t] -> obs[t+1])
                    # So we have N actions and N+1 observations, keep first N observations
                    images = np.array(self.episode_images[cam][:-1])
                    obs_grp.create_dataset(
                        f"{cam}_image", 
                        data=images,
                        dtype=np.uint8,
                        compression="gzip"
                    )
                    print(f"  {cam}_image: {images.shape}")
            
            # Save robot state (remove last observation)
            if len(self.episode_eef_pos) > 0:
                eef_pos = np.array(self.episode_eef_pos[:-1], dtype=np.float32)
                obs_grp.create_dataset("robot0_eef_pos", data=eef_pos)
                print(f"  robot0_eef_pos: {eef_pos.shape}")
            
            if len(self.episode_eef_quat) > 0:
                eef_quat = np.array(self.episode_eef_quat[:-1], dtype=np.float32)
                obs_grp.create_dataset("robot0_eef_quat", data=eef_quat)
                print(f"  robot0_eef_quat: {eef_quat.shape}")
            
            if len(self.episode_gripper_qpos) > 0:
                gripper = np.array(self.episode_gripper_qpos[:-1], dtype=np.float32)
                obs_grp.create_dataset("robot0_gripper_qpos", data=gripper)
                print(f"  robot0_gripper_qpos: {gripper.shape}")
            
            if len(self.episode_joint_pos) > 0:
                joints = np.array(self.episode_joint_pos[:-1], dtype=np.float32)
                obs_grp.create_dataset("robot0_joint_pos", data=joints)
                print(f"  robot0_joint_pos: {joints.shape}")
            
            # Save actions
            actions = np.array(self.episode_actions, dtype=np.float32)
            demo_grp.create_dataset("actions", data=actions)
            print(f"  actions: {actions.shape}")
            
            # Save rewards
            rewards = np.array(self.episode_rewards, dtype=np.float32)
            demo_grp.create_dataset("rewards", data=rewards)
            
            # Save metadata
            grp.attrs["language_instruction"] = language_instruction
            grp.attrs["success"] = success
            grp.attrs["num_samples"] = len(self.episode_actions)
            
            now = datetime.datetime.now()
            grp.attrs["date"] = f"{now.year}-{now.month:02d}-{now.day:02d}"
            grp.attrs["time"] = f"{now.hour:02d}:{now.minute:02d}:{now.second:02d}"
        
        print(f"Saved episode to: {hdf5_path}")
        print(f"  Total timesteps: {len(self.episode_actions)}")
        
        return hdf5_path
    
    def render(self):
        self.env.render()
    
    def _check_success(self):
        return self.env._check_success()
    
    def close(self):
        self.env.close()
    
    @property
    def robots(self):
        return self.env.robots
    
    @property
    def viewer(self):
        return self.env.viewer


def collect_human_trajectory(env, device, arm, env_configuration, language_instruction):
    """
    Collect a single demonstration trajectory.
    
    Returns:
        tuple: (success, saved_path)
    """
    # Reset environment
    reset_success = False
    while not reset_success:
        try:
            env.reset()
            reset_success = True
        except:
            continue
    
    env.render()
    
    # Task completion counter
    task_completion_hold_count = -1
    device.start_control()
    
    saving = True
    step_count = 0
    
    print("\n" + "="*50)
    print("Controls:")
    print("  - Use device to control robot")
    print("  - Press 'q' or reset to discard and retry")
    print("  - Complete task to save demonstration")
    print("="*50 + "\n")
    
    while True:
        step_count += 1
        
        # Get active robot
        active_robot = (
            env.robots[0]
            if env_configuration == "bimanual"
            else env.robots[arm == "left"]
        )
        
        # Get action from device
        action, grasp = input2action(
            device=device,
            robot=active_robot,
            active_arm=arm,
            env_configuration=env_configuration,
        )
        
        # Check for reset/quit
        if action is None:
            print("Episode discarded by user")
            saving = False
            break
        
        # Step environment
        env.step(action)
        env.render()
        
        # Check for task completion
        if task_completion_hold_count == 0:
            print("Task completed successfully!")
            break
        
        # State machine for success detection
        if env._check_success():
            if task_completion_hold_count > 0:
                task_completion_hold_count -= 1
            else:
                task_completion_hold_count = 10
        else:
            task_completion_hold_count = -1
    
    print(f"Total steps: {step_count}")
    
    # Save if successful
    saved_path = None
    if saving:
        saved_path = env.save_episode(language_instruction, success=True)
    
    return saving, saved_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", type=str, default="demonstration_data_with_images")
    parser.add_argument("--robots", nargs="+", type=str, default="Panda")
    parser.add_argument("--config", type=str, default="single-arm-opposed")
    parser.add_argument("--arm", type=str, default="right")
    parser.add_argument("--controller", type=str, default="OSC_POSE")
    parser.add_argument("--device", type=str, default="keyboard")
    parser.add_argument("--pos-sensitivity", type=float, default=1.5)
    parser.add_argument("--rot-sensitivity", type=float, default=1.0)
    parser.add_argument("--num-demonstration", type=int, default=1)
    parser.add_argument("--bddl-file", type=str, required=True)
    parser.add_argument("--vendor-id", type=int, default=9583)
    parser.add_argument("--product-id", type=int, default=50734)
    parser.add_argument("--camera-height", type=int, default=256)
    parser.add_argument("--camera-width", type=int, default=256)
    
    args = parser.parse_args()
    
    # Controller config
    controller_config = load_controller_config(default_controller=args.controller)
    
    # Camera names for observation
    camera_names = ["agentview", "robot0_eye_in_hand"]
    
    # Get problem info from BDDL file
    assert os.path.exists(args.bddl_file), f"BDDL file not found: {args.bddl_file}"
    problem_info = BDDLUtils.get_problem_info(args.bddl_file)
    
    problem_name = problem_info["problem_name"]
    domain_name = problem_info["domain_name"]
    language_instruction = problem_info["language_instruction"]
    
    print(f"\nTask: {language_instruction}")
    print(f"Problem: {problem_name}")
    print(f"Domain: {domain_name}\n")
    
    # Environment config
    env_config = {
        "robots": args.robots,
        "controller_configs": controller_config,
    }
    
    if "TwoArm" in problem_name:
        env_config["env_configuration"] = args.config
    
    # Create environment WITH camera observations
    env = TASK_MAPPING[problem_name](
        bddl_file_name=args.bddl_file,
        **env_config,
        has_renderer=True,
        has_offscreen_renderer=True,  # Required for camera obs
        render_camera="agentview",
        ignore_done=True,
        use_camera_obs=True,  # Enable camera observations
        camera_names=camera_names,
        camera_heights=args.camera_height,
        camera_widths=args.camera_width,
        reward_shaping=True,
        control_freq=50,
    )
    
    # Wrap with visualization
    env = VisualizationWrapper(env)
    
    # Create output directory with task info
    t1, t2 = str(time.time()).split(".")
    task_dir = os.path.join(
        args.directory,
        f"{domain_name}_{problem_name}_{t1}_{t2}_" + 
        language_instruction.replace(" ", "_").strip('""')
    )
    
    # Wrap with custom data collection wrapper
    env = ImageDataCollectionWrapper(
        env, 
        task_dir,
        camera_names=camera_names,
        camera_height=args.camera_height,
        camera_width=args.camera_width
    )
    
    # Initialize input device
    if args.device == "keyboard":
        from robosuite.devices import Keyboard
        device = Keyboard(
            pos_sensitivity=args.pos_sensitivity, 
            rot_sensitivity=args.rot_sensitivity
        )
        env.viewer.add_keypress_callback(device.on_press)
    elif args.device == "spacemouse":
        from robosuite.devices import SpaceMouse
        device = SpaceMouse(
            args.vendor_id,
            args.product_id,
            pos_sensitivity=args.pos_sensitivity,
            rot_sensitivity=args.rot_sensitivity,
        )
    elif args.device == "joystick":
        from joystick import Joystick
        device = Joystick(
            pos_sensitivity=args.pos_sensitivity,
            rot_sensitivity=args.rot_sensitivity,
        )
    else:
        raise ValueError(f"Invalid device: {args.device}. Choose 'keyboard', 'spacemouse', or 'joystick'.")
    
    # Collect demonstrations
    collected = 0
    saved_paths = []
    
    while collected < args.num_demonstration:
        print(f"\n{'='*50}")
        print(f"Collecting demonstration {collected + 1}/{args.num_demonstration}")
        print(f"{'='*50}")
        
        success, path = collect_human_trajectory(
            env, device, args.arm, args.config, language_instruction
        )
        
        if success and path:
            collected += 1
            saved_paths.append(path)
            print(f"Progress: {collected}/{args.num_demonstration} demonstrations collected")
    
    env.close()
    
    print(f"\n{'='*50}")
    print("Data collection complete!")
    print(f"Saved {len(saved_paths)} demonstrations to:")
    for p in saved_paths:
        print(f"  - {p}")
    print(f"{'='*50}\n")
