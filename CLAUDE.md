# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Common Development Tasks

### Installation
The project uses a conda environment with Python 3.10:
```bash
# Create environment
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv

# Install dependencies
pip install -r requirements.txt

# Install submodules
cd teleop/televuer && pip install -e .
cd ../robot_control/dex-retargeting && pip install -e .
```

### Running the System
Main entry point is `teleop/teleop_hand_and_arm.py`:
```bash
# Basic simulation with G1_29 + Dex3 hand
python teleop_hand_and_arm.py --ee=dex3 --sim --record

# Physical robot deployment
python teleop_hand_and_arm.py --xr-mode=hand --arm=G1_29 --ee=dex3
```

### Testing Individual Components
Key modules have standalone test functionality:
- `teleop/robot_control/robot_arm_ik.py` - Inverse kinematics testing
- `teleop/robot_control/robot_arm.py` - Robot arm control testing
- `teleop/image_server/image_client.py` - Image transmission testing
- `teleop/utils/weighted_moving_filter.py` - Filter testing

## Code Architecture

### Core Components
- **TeleVuer Integration**: Located in `teleop/televuer/` submodule for XR device communication
- **Robot Control**: `teleop/robot_control/` handles all robot hardware interfaces
- **Image Processing**: `teleop/image_server/` manages camera feeds and transmission
- **Data Recording**: `teleop/utils/` contains recording and visualization utilities

### Robot Hardware Support
The system supports multiple Unitree robot configurations:
- **G1 (29 DoF)**: Full humanoid with 29 degrees of freedom
- **G1 (23 DoF)**: Simplified G1 configuration
- **H1_2 (7-DoF arm)**: H1 with 7-degree-of-freedom arms
- **H1 (4-DoF arm)**: Original H1 with 4-degree-of-freedom arms

### Hand Controllers
- **Dex3-1**: Unitree dexterous hand via `robot_hand_unitree.py`
- **Inspire**: Inspire dexterous hand via `robot_hand_inspire.py`
- **Dex1-1**: Gripper control via `robot_hand_unitree.py`

### Inverse Kinematics System
Each robot type has dedicated IK solver in `robot_arm_ik.py`:
- Uses Pinocchio robotics library with CasADi optimization
- Meshcat visualization support for debugging
- Weighted moving filter for smooth motion (`utils/weighted_moving_filter.py`)

### Communication Protocol
- **unitree_sdk2_python**: Primary robot communication library
- **DDS (Data Distribution Service)**: Real-time message passing
- **WebSocket**: XR device communication via televuer
- **ZMQ**: Image transmission protocol

### Hand Retargeting
- **DexPilot Algorithm**: Default retargeting method for precise fingertip control
- **Vector Method**: Alternative retargeting algorithm
- **Configuration**: YAML-based setup in `assets/*/` directories

### Data Recording
- **Episode Writer**: `utils/episode_writer.py` for imitation learning data
- **Rerun Visualizer**: `utils/rerun_visualizer.py` for 3D visualization
- **Image Logging**: Synchronized camera and robot state recording

## Key File Locations

### Robot Assets
- `assets/g1/`: G1 robot URDF files and meshes
- `assets/h1/`, `assets/h1_2/`: H1 robot configurations
- `assets/inspire_hand/`, `assets/unitree_hand/`: Hand controller configs

### Hardware Integration
- `hardware/`: 3D-printed mounting hardware for cameras
- Camera mounts for head stereo vision and wrist-mounted depth sensors

### Submodules
- `teleop/televuer/`: XR device interface (git submodule)
- `teleop/robot_control/dex-retargeting/`: Hand retargeting algorithms (git submodule)

## Development Notes

### Debugging
- Set `Unit_Test = True` in IK classes for standalone testing
- Use `Visualization = True` for Meshcat 3D visualization
- Enable `--headless` flag for deployment without GUI

### Safety Features
- Emergency stop via keyboard ('q' key)
- Smooth motion transitions with weighted filters
- Joint limit enforcement in IK solvers
- Debug mode vs motion mode for robot control

### Multi-Modal Support
- Hand tracking mode (`--xr-mode=hand`)
- Controller tracking mode (`--xr-mode=controller`)
- Simulation mode (`--sim`) for testing without hardware
- Recording mode (`--record`) for data collection