# LIBERO-10 Dataset Information

## Dataset Overview

- **Name**: `liber_o10`
- **Version**: 1.0.0
- **Format**: TFRecord
- **Total Size**: 3.5 GB
- **Total Episodes**: 379
- **Shards**: 32 files (11-12 episodes each)

## Episode Hierarchy

```
Episode
├── steps (variable length sequence)
│   ├── action: float32[7]
│   ├── observation
│   │   ├── image: uint8[256,256,3] (JPEG)
│   │   ├── wrist_image: uint8[256,256,3] (JPEG)
│   │   ├── state: float32[8]
│   │   └── joint_state: float32[7]
│   ├── language_instruction: string
│   ├── reward: float32
│   ├── discount: float32
│   ├── is_first: bool
│   ├── is_last: bool
│   └── is_terminal: bool
└── episode_metadata
    └── file_path: string
```

## Feature Descriptions

| Feature | Type | Shape | Description |
|---------|------|-------|-------------|
| **action** | float32 | [7] | Robot EEF action (6D pose + gripper) |
| **observation/image** | uint8 | [256,256,3] | Main camera RGB (JPEG) |
| **observation/wrist_image** | uint8 | [256,256,3] | Wrist camera RGB (JPEG) |
| **observation/state** | float32 | [8] | Robot EEF state (6D pose + 2D gripper) |
| **observation/joint_state** | float32 | [7] | Robot joint angles |
| **language_instruction** | string | - | Natural language task description |
| **reward** | float32 | - | 1.0 on final step, 0.0 otherwise |
| **discount** | float32 | - | Default 1.0 |
| **is_first** | bool | - | True on first step |
| **is_last** | bool | - | True on last step |
| **is_terminal** | bool | - | True on terminal step |

## TFRecord Storage Format

Each episode is stored as a flattened `tf.train.Example`:

```
steps/action: float_list[num_steps × 7]
steps/observation/image: bytes_list[num_steps]
steps/observation/wrist_image: bytes_list[num_steps]
steps/observation/state: float_list[num_steps × 8]
steps/observation/joint_state: float_list[num_steps × 7]
steps/language_instruction: bytes_list[num_steps]
steps/reward: float_list[num_steps]
steps/discount: float_list[num_steps]
steps/is_first: int64_list[num_steps]
steps/is_last: int64_list[num_steps]
steps/is_terminal: int64_list[num_steps]
episode_metadata/file_path: bytes_list[1]
```

## Sample Episode Statistics

- Episode length: ~214 timesteps (average)
- Main camera image: ~17.9 KB/frame (JPEG)
- Wrist camera image: ~17.0 KB/frame (JPEG)
- Language instruction: ~87 bytes
