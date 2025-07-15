# Robot Hardware Configurations

## Supported Robot Platforms

### Unitree G1 Humanoid Robot

The G1 robot comes in multiple configurations optimized for different use cases:

#### G1 (29 DoF) Configuration
- **Total DoF**: 29 degrees of freedom
- **File**: `assets/g1/g1_body29_hand14.urdf`
- **Breakdown**:
  - Legs: 6 DoF × 2 = 12 DoF
  - Waist: 3 DoF
  - Arms: 7 DoF × 2 = 14 DoF
  - Hands: Optional (with Dex3-1: 7 DoF × 2 = 14 DoF)
- **Mode Machine**: 2 (with 14.5 hip roll reduction ratio)
- **Use Case**: Full humanoid teleoperation with maximum dexterity

#### G1 (23 DoF) Configuration  
- **Total DoF**: 23 degrees of freedom
- **File**: `assets/g1/g1_body23.urdf`
- **Breakdown**:
  - Legs: 6 DoF × 2 = 12 DoF
  - Waist: 1 DoF (simplified)
  - Arms: 5 DoF × 2 = 10 DoF
  - Hands: 0 DoF (gripper only)
- **Mode Machine**: 1
- **Use Case**: Simplified control for basic manipulation tasks

### Unitree H1 Robot Variants

#### H1_2 (7-DoF Arms)
- **Total Arm DoF**: 7 per arm (14 total for both arms)
- **File**: `assets/h1_2/h1_2.urdf`
- **Kinematic Chain**: 51 total DoF including hands
  ```
  Pelvis → Torso → Shoulder(3) → Elbow(2) → Wrist(2) → Hand
  ```
- **Hand Integration**: Supports dexterous hands (12 DoF per hand)
- **Features**: Enhanced manipulation capabilities
- **Joint Configuration**:
  - Shoulder: pitch, roll, yaw
  - Elbow: pitch, roll  
  - Wrist: pitch, yaw

#### H1 (4-DoF Arms)
- **Total Arm DoF**: 4 per arm (8 total for both arms)
- **File**: `assets/h1/h1_with_hand.urdf`
- **Features**: Basic manipulation with simplified arm kinematics
- **Use Case**: Lower complexity teleoperation tasks

## End Effector Configurations

### Unitree Dex3-1 Dexterous Hand
- **DoF**: 6 per hand (12 total for bilateral)
- **Controller**: `robot_hand_unitree.py:Dex3_1_Controller`
- **Configuration**: `assets/unitree_hand/unitree_dex3.yml`
- **Features**:
  - Individual finger control
  - Precise fingertip positioning
  - Force feedback capabilities
- **Joint Mapping**:
  - Thumb: 2 joints (proximal, distal)
  - Index: 2 joints (proximal, distal)  
  - Middle: 2 joints (proximal, distal)

### Unitree Dex1-1 Gripper
- **DoF**: 1 per hand (2 total for bilateral)
- **Controller**: `robot_hand_unitree.py:Gripper_Controller`
- **Features**: Simple open/close gripper functionality
- **Use Case**: Basic grasping tasks

### Inspire Dexterous Hand
- **DoF**: 6 per hand
- **Controller**: `robot_hand_inspire.py:Inspire_Controller`
- **Configuration**: `assets/inspire_hand/inspire_hand.yml`
- **Features**:
  - Third-party hand integration
  - DexPilot retargeting algorithm
  - Configurable scaling factor (1.20)
  - Low-pass filtering (alpha: 0.2)

## Hand Retargeting Configuration

### DexPilot Algorithm (Default)
```yaml
type: DexPilot
wrist_link_name: "L_hand_base_link"
finger_tip_link_names: 
  - "L_thumb_tip"
  - "L_index_tip" 
  - "L_middle_tip"
  - "L_ring_tip"
  - "L_pinky_tip"
```

### Vector Algorithm (Alternative)
```yaml
type: vector
target_origin_link_names: ["L_hand_base_link", ...]
target_task_link_names: ["L_thumb_tip", ...]
scaling_factor: 1.20
low_pass_alpha: 0.2
```

## Camera Mount Hardware

### Head-Mounted Stereo Cameras
- **File**: `hardware/head_stereo_camera_mount.STEP`
- **Purpose**: First-person view for operator
- **Installation**: Mounted on robot head/torso

### Wrist-Mounted Depth Sensors
- **Files**: 
  - `hardware/left_wrist_D405_camera_mount.STEP`
  - `hardware/right_wrist_D405_camera_mount.STEP`
- **Sensor**: Intel RealSense D405
- **Purpose**: Close-up manipulation view
- **Features**: Depth perception for precision grasping

### Wrist Ring Mount
- **File**: `hardware/wrist_ring_mount.STEP`
- **Purpose**: Secure camera mounting system

## Robot Selection Commands

### G1 Robot Deployment
```bash
# G1 29-DoF with Dex3-1 hands (simulation)
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --sim --record

# G1 29-DoF physical robot
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --xr-mode=hand

# G1 23-DoF with gripper
python teleop_hand_and_arm.py --arm=G1_23 --ee=gripper
```

### H1 Robot Deployment  
```bash
# H1_2 with 7-DoF arms and Dex3-1 hands
python teleop_hand_and_arm.py --arm=H1_2 --ee=dex3

# H1 with 4-DoF arms and Inspire hands
python teleop_hand_and_arm.py --arm=H1 --ee=inspire
```

## Hardware Specifications

### Unitree G1 (29 DoF)
- **Height**: ~1.3m
- **Weight**: ~35kg
- **Battery Life**: ~2 hours continuous operation
- **Payload**: ~5kg per arm
- **Communication**: DDS over WiFi/Ethernet

### Unitree H1_2
- **Height**: ~1.8m  
- **Weight**: ~47kg
- **Battery Life**: ~1 hour continuous operation
- **Payload**: ~10kg per arm
- **Communication**: DDS over WiFi/Ethernet

## Physical Installation

### Camera Mounting Process
1. Install head stereo camera mount on robot torso
2. Mount Intel RealSense D405 cameras on wrist mounts
3. Connect camera cables to onboard computing system
4. Configure camera calibration parameters

### Network Configuration
- Robot WiFi network setup
- SSL certificate installation for secure communication
- XR device pairing and calibration

### Safety Considerations
- Emergency stop systems
- Joint limit configuration
- Collision detection parameters
- Workspace boundaries

## Troubleshooting Common Hardware Issues

### Robot Communication
- Check DDS network connectivity
- Verify SDK version compatibility
- Monitor real-time performance metrics

### Camera Issues
- USB bandwidth limitations with multiple cameras
- Image compression settings
- Synchronization timing

### Hand Retargeting Problems
- Calibration of hand tracking system
- Joint limit violations
- Retargeting algorithm convergence

This configuration system provides flexible hardware support for various teleoperation scenarios while maintaining safety and performance standards.