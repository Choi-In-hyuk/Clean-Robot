# LIBERO-10 Dataset Information

## Overview
- **Dataset Name**: liber_o10
- **Version**: 1.0.0
- **Total Size**: 3.5GB (3,656,799,026 bytes)
- **Total Episodes**: 379
- **File Format**: TFRecord
- **Number of Shards**: 32

## Dataset Statistics
- **Episodes per shard**: 11-12 episodes
- **Average episode length**: ~214 steps (variable length)
- **Split**: train only

## Directory Structure
```
libero_10_no_noops_backup/1.0.0/
├── dataset_info.json           # Dataset metadata
├── features.json               # Feature schema definition
└── liber_o10-train.tfrecord-*  # 32 TFRecord files (00000-00031)
```

## Data Structure

### Episode Level
Each episode contains:
- `episode_metadata`: Metadata about the episode
  - `file_path` (Text): Path to the original data file

- `steps`: Sequence of timesteps with variable length

### Step Level (per timestep)
Each step contains:

#### Action
- `action` (float32[7]): Robot end-effector action

#### Observation
- `observation/image` (uint8[256, 256, 3]): Main camera RGB image (JPEG encoded)
- `observation/wrist_image` (uint8[256, 256, 3]): Wrist camera RGB image (JPEG encoded)
- `observation/state` (float32[8]): Robot end-effector state (6D pose + 2D gripper)
- `observation/joint_state` (float32[7]): Robot joint angles

#### Task Information
- `language_instruction` (Text): Natural language instruction for the task

#### Episode Flags
- `is_first` (bool): True on the first step of the episode
- `is_last` (bool): True on the last step of the episode
- `is_terminal` (bool): True on the terminal step (always True for demonstrations)

#### Reward/Discount
- `reward` (float32): Reward value (1.0 on final step for demonstrations, 0.0 otherwise)
- `discount` (float32): Discount factor (default: 1.0)

## Example Episode
First episode from `liber_o10-train.tfrecord-00000-of-00032`:
- **Steps**: 214
- **Language instruction length**: 87 bytes
- **Main camera image size**: ~17.9KB (JPEG compressed)
- **Wrist camera image size**: ~17.0KB (JPEG compressed)

## Sample Data Values
```
action: [0.016, 0.0, -0.0, 0.0, 0.0, -0.0, -1.0, ...]
state: [-0.053, 0.007, 0.678, 3.141, 0.002, -0.090, 0.039, -0.039]
joint_state: [0.004, -0.141, 0.011, -2.431, 0.004, 2.233, 0.797]
discount: [1.0, 1.0, 1.0, ...]
reward: [0.0, 0.0, 0.0, ..., 1.0]  # 1.0 on final step
```

## Notes
- All images are JPEG encoded to reduce storage size
- Episode length is variable (not fixed)
- This is a demonstration dataset (all episodes are successful demonstrations)
- No-ops have been removed from the original LIBERO-10 dataset
