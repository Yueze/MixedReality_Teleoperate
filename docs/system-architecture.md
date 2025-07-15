# System Architecture

## Overview
The XR Teleoperate system enables real-time control of Unitree humanoid robots using Extended Reality (XR) devices including Apple Vision Pro, Meta Quest 3, and PICO 4 Ultra Enterprise. The system provides precise hand and arm teleoperation capabilities with visual feedback through stereo cameras.

## Core Architecture

### 1. Main Entry Point
- **File**: `teleop/teleop_hand_and_arm.py`
- **Function**: Orchestrates the entire teleoperation pipeline
- **Key Features**:
  - Multi-threaded execution for real-time performance
  - Keyboard controls (r=start, q=quit, s=toggle recording)
  - Support for simulation and physical robot modes

### 2. XR Device Interface
- **Module**: `teleop/televuer/` (Git submodule)
- **Wrapper**: `TeleVuerWrapper` class
- **Capabilities**:
  - Hand tracking from XR devices
  - Controller tracking mode
  - WebSocket communication protocol
  - Support for multiple XR platforms

### 3. Robot Control Layer

#### Arm Controllers
Located in `teleop/robot_control/robot_arm.py`:
- `G1_29_ArmController` - G1 robot with 29 degrees of freedom
- `G1_23_ArmController` - G1 robot with 23 degrees of freedom  
- `H1_2_ArmController` - H1_2 robot with 7-DoF arms
- `H1_ArmController` - H1 robot with 4-DoF arms

#### Inverse Kinematics
Located in `teleop/robot_control/robot_arm_ik.py`:
- Uses Pinocchio robotics library with CasADi optimization
- Individual IK solvers for each robot type
- Meshcat visualization support for debugging
- Weighted moving filter for smooth motion trajectories

#### Hand Controllers
- **Unitree Hands**: `robot_hand_unitree.py`
  - `Dex3_1_Controller` - Dexterous hand with 6 DOF per hand
  - `Gripper_Controller` - Simple gripper control
- **Inspire Hands**: `robot_hand_inspire.py`
  - `Inspire_Controller` - Third-party dexterous hand integration

### 4. Hand Retargeting System
- **Module**: `teleop/robot_control/dex-retargeting/` (Git submodule)
- **Default Algorithm**: DexPilot (improved precision over Vector method)
- **Configuration**: YAML-based setup in `assets/*/` directories
- **Features**:
  - Real-time fingertip position mapping
  - Joint limit enforcement
  - Collision avoidance

### 5. Image Processing Pipeline
Located in `teleop/image_server/`:
- **Server**: `image_server.py` - Captures from robot-mounted cameras
- **Client**: `image_client.py` - Transmits images to XR device
- **Protocol**: ZMQ (ZeroMQ) for low-latency image transmission
- **Camera Types**:
  - Head-mounted stereo cameras
  - Wrist-mounted depth sensors (Intel RealSense D405)

### 6. Data Recording & Visualization
Located in `teleop/utils/`:
- **Episode Writer**: `episode_writer.py` - Records imitation learning data
- **Rerun Visualizer**: `rerun_visualizer.py` - 3D visualization and debugging
- **Weighted Filter**: `weighted_moving_filter.py` - Motion smoothing

## Communication Protocols

### Robot Communication
- **Library**: `unitree_sdk2_python`
- **Protocol**: DDS (Data Distribution Service)
- **Features**: Real-time control with low latency

### XR Device Communication  
- **Protocol**: WebSocket over HTTPS
- **Certificates**: SSL certificates for secure communication
- **Data**: Hand/controller poses, button states

### Image Transmission
- **Protocol**: ZMQ (ZeroMQ)
- **Compression**: Configurable image compression
- **Synchronization**: Frame-synchronized with robot state

## Robot Hardware Support

### Supported Robots
| Robot | DoF | Status | Features |
|-------|-----|--------|----------|
| G1 (29 DoF) | 29 | ✅ Complete | Full humanoid with dexterous hands |
| G1 (23 DoF) | 23 | ✅ Complete | Simplified G1 configuration |
| H1_2 (7-DoF arm) | 7 per arm | ✅ Complete | Enhanced arm dexterity |
| H1 (4-DoF arm) | 4 per arm | ✅ Complete | Basic arm functionality |

### Supported End Effectors
| Device | Type | DoF | Integration |
|--------|------|-----|-------------|
| Dex3-1 | Dexterous Hand | 6 per hand | `robot_hand_unitree.py` |
| Dex1-1 | Gripper | 1 per hand | `robot_hand_unitree.py` |
| Inspire | Dexterous Hand | Variable | `robot_hand_inspire.py` |

## Operational Modes

### Simulation Mode (`--sim`)
- No physical robot required
- Environment validation
- Hardware failure diagnostics
- Scene reset capabilities

### Motion Mode
- Full robot control
- Real-time teleoperation
- Safety features enabled

### Headless Mode (`--headless`)
- No GUI visualization
- Suitable for deployment
- Reduced computational overhead

### Recording Mode (`--record`)
- Data collection for imitation learning
- Synchronized robot state and camera feeds
- Episode-based data organization

## Safety Features

### Emergency Controls
- Keyboard emergency stop ('q' key)
- Graceful shutdown procedures
- Connection monitoring

### Motion Safety
- Joint limit enforcement in IK solvers
- Smooth motion transitions with weighted filters
- Collision avoidance in hand retargeting
- Velocity and acceleration limits

### System Monitoring
- Real-time connection status
- Error logging and recovery
- Hardware health monitoring

## File Structure Summary

```
teleop/
├── teleop_hand_and_arm.py          # Main entry point
├── televuer/                       # XR device interface (submodule)
├── robot_control/
│   ├── robot_arm.py               # Arm controllers
│   ├── robot_arm_ik.py            # Inverse kinematics
│   ├── robot_hand_unitree.py      # Unitree hand controllers
│   ├── robot_hand_inspire.py      # Inspire hand controller
│   └── dex-retargeting/           # Hand retargeting (submodule)
├── image_server/
│   ├── image_server.py            # Camera capture
│   └── image_client.py            # Image transmission
└── utils/
    ├── episode_writer.py          # Data recording
    ├── rerun_visualizer.py        # 3D visualization
    └── weighted_moving_filter.py  # Motion filtering
```

## Dependencies

### Core Libraries
- `pinocchio` - Robotics kinematics and dynamics
- `casadi` - Optimization for inverse kinematics  
- `numpy` - Numerical computations
- `unitree_sdk2_python` - Robot communication

### Visualization & UI
- `meshcat` - 3D visualization
- `rerun-sdk` - Data visualization and debugging
- `matplotlib` - Plotting and analysis

### Communication
- `websockets` - XR device communication
- `zmq` - Image transmission
- `cv2` - Computer vision and image processing

This architecture provides a robust, modular system for XR-based robot teleoperation with emphasis on real-time performance, safety, and extensibility.