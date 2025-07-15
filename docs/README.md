# XR Teleoperate Documentation

This documentation provides comprehensive coverage of the XR teleoperation system for Unitree humanoid robots.

## Documentation Structure

### 📋 [System Architecture](system-architecture.md)
Complete overview of the teleoperation system architecture including:
- Core components and their interactions
- Communication protocols (DDS, WebSocket, ZMQ)
- XR device integration via TeleVuer
- Robot control pipeline
- Image processing and transmission
- Data recording and visualization

### 🤖 [Robot Hardware Configurations](robot-hardware-configurations.md)
Detailed specifications for supported robot platforms:
- **Unitree G1**: 29-DoF and 23-DoF configurations
- **Unitree H1**: 7-DoF and 4-DoF arm variants
- **End Effectors**: Dex3-1, Dex1-1, and Inspire hands
- Hand retargeting algorithms (DexPilot, Vector)
- Camera mounting hardware and installation

### 🛠️ [Development Workflow](development-workflow.md)
Complete development guide covering:
- Environment setup with Conda and dependencies
- Testing methodologies for individual components
- Simulation and visualization modes
- Debugging tools and performance monitoring
- Code quality standards and best practices

## Quick Start Guide

### Environment Setup
```bash
# Create conda environment
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv

# Install dependencies
pip install -r requirements.txt

# Install submodules
cd teleop/televuer && pip install -e .
cd ../robot_control/dex-retargeting && pip install -e .
```

### Basic Usage
```bash
# Simulation mode (no hardware required)
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --sim --record

# Physical robot deployment
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --xr-mode=hand
```

## System Overview

The XR Teleoperate system enables real-time control of Unitree humanoid robots using Extended Reality devices including:
- **Apple Vision Pro**: Hand tracking and immersive control
- **Meta Quest 3**: Controller-based teleoperation
- **PICO 4 Ultra Enterprise**: Professional XR deployment

### Key Features
- **Real-time Teleoperation**: Low-latency robot control
- **Dexterous Hand Control**: Precise fingertip manipulation
- **Stereo Vision**: First-person and wrist-mounted cameras
- **Data Recording**: Imitation learning dataset collection
- **Safety Systems**: Emergency stops and joint limits
- **Modular Design**: Support for multiple robot platforms

### Supported Hardware
| Robot | DoF | End Effector | Status |
|-------|-----|--------------|--------|
| G1 (29-DoF) | 29 | Dex3-1 Hand | ✅ Complete |
| G1 (23-DoF) | 23 | Gripper | ✅ Complete |
| H1_2 (7-DoF arm) | 7 per arm | Dex3-1/Inspire | ✅ Complete |
| H1 (4-DoF arm) | 4 per arm | Gripper | ✅ Complete |

## Architecture Highlights

### Communication Stack
- **Robot Control**: DDS via unitree_sdk2_python
- **XR Interface**: WebSocket with SSL certificates
- **Image Transmission**: ZMQ for low-latency video
- **Hand Retargeting**: DexPilot algorithm for precise control

### Core Components
- **Main Controller**: `teleop_hand_and_arm.py`
- **XR Integration**: `televuer/` submodule
- **Robot Control**: `robot_control/` package
- **Image Pipeline**: `image_server/` package
- **Utilities**: Motion filters and data recording

### Safety Features
- Emergency keyboard controls (q=quit, r=start, s=record)
- Joint limit enforcement in IK solvers
- Smooth motion transitions with weighted filters
- Real-time performance monitoring

## Getting Help

For additional support:
1. Check the individual documentation files for detailed information
2. Review the main README.md for setup instructions
3. Examine the CLAUDE.md file for development guidelines
4. Test individual components using unit test modes

## Contributing

When contributing to this codebase:
1. Follow the modular design principles
2. Add unit tests for new components
3. Update documentation for new features
4. Maintain safety and performance standards

---

*This documentation was generated to provide comprehensive coverage of the XR teleoperation system architecture, configuration, and development practices.*