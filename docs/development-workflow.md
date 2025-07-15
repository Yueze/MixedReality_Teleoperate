# Development Workflow and Testing

## Environment Setup

### Conda Environment Creation
```bash
# Create environment with core dependencies
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv

# Install Python dependencies
pip install -r requirements.txt
```

### Git Submodule Installation
```bash
# Install TeleVuer XR interface
cd teleop/televuer && pip install -e .

# Install dex-retargeting algorithms  
cd ../robot_control/dex-retargeting && pip install -e .
```

## Core Dependencies

### Required Libraries
```txt
matplotlib==3.7.5        # Plotting and visualization
rerun-sdk==0.20.1        # 3D data visualization
meshcat==0.3.2           # Web-based 3D visualization
sshkeyboard==2.3.1       # Keyboard input handling
```

### System Libraries
- **pinocchio**: Robotics kinematics and dynamics
- **casadi**: Optimization for inverse kinematics
- **unitree_sdk2_python**: Robot communication
- **opencv-python**: Computer vision and image processing

## Development Modes

### Unit Testing Mode
Individual components can be tested in isolation using the `Unit_Test = True` flag:

```python
# Example: Testing IK solver standalone
class G1_29_ArmIK:
    def __init__(self, Unit_Test = False, Visualization = False):
        self.Unit_Test = Unit_Test
        if Unit_Test:
            # Load URDF with relative path for testing
            self.robot = pin.RobotWrapper.BuildFromURDF('../../assets/g1/g1_body29_hand14.urdf', '../../assets/g1/')
```

**Files with unit test support**:
- `teleop/robot_control/robot_arm_ik.py`
- `teleop/robot_control/robot_arm.py` 
- `teleop/robot_control/robot_hand_unitree.py`
- `teleop/image_server/image_client.py`

### Visualization Mode
Enable 3D visualization for debugging kinematics and motion planning:

```python
# Enable Meshcat visualization
ik_solver = G1_29_ArmIK(Unit_Test=False, Visualization=True)
```

**Features**:
- Real-time robot model visualization
- Joint angle monitoring
- End-effector trajectory display
- Collision detection visualization

### Simulation Mode
Test the complete system without physical hardware:

```bash
# Run in simulation mode
python teleop_hand_and_arm.py --sim --arm=G1_29 --ee=dex3 --record
```

**Benefits**:
- Environment validation
- Hardware failure diagnostics
- Algorithm testing
- Scene reset capabilities

## Testing Individual Components

### 1. Inverse Kinematics Testing
```bash
cd teleop/robot_control
python robot_arm_ik.py
```
**Test Features**:
- Joint limit validation
- Convergence testing
- Performance benchmarking
- Visualization of IK solutions

### 2. Robot Arm Control Testing
```bash
cd teleop/robot_control  
python robot_arm.py
```
**Test Features**:
- Communication with robot hardware
- Joint position control
- Safety limit enforcement
- Real-time performance monitoring

### 3. Image Transmission Testing
```bash
cd teleop/image_server
python image_client.py
```
**Test Features**:
- Camera connectivity
- Image compression performance
- Network latency measurement
- Frame synchronization

### 4. Motion Filter Testing
```bash
cd teleop/utils
python weighted_moving_filter.py
```
**Test Features**:
- Filter response visualization
- Noise reduction effectiveness
- Latency analysis
- Parameter tuning

## Development Best Practices

### Code Structure Guidelines
1. **Modular Design**: Each component has clear interfaces and responsibilities
2. **Unit Testing**: All major components support standalone testing
3. **Configuration Management**: YAML-based configuration for robot parameters
4. **Error Handling**: Comprehensive error checking and recovery mechanisms

### Debug and Development Flags
```python
# Common debug flags used throughout codebase
Unit_Test = True          # Enable unit testing mode
Visualization = True      # Enable 3D visualization  
Debug = True             # Enable debug logging
Headless = False         # Disable GUI for deployment
```

### Logging Configuration
```python
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)
```

## Running the Complete System

### Development Configuration
```bash
# Full system with visualization
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --sim --record
```

### Production Configuration  
```bash
# Deployed system without GUI
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --headless --xr-mode=hand
```

### Supported Command Line Arguments
- `--arm`: Robot type (G1_29, G1_23, H1_2, H1)
- `--ee`: End effector (dex3, dex1, inspire, gripper)
- `--sim`: Enable simulation mode
- `--record`: Enable data recording
- `--headless`: Disable visualization
- `--xr-mode`: XR tracking mode (hand, controller)

## Testing Workflow

### 1. Component-Level Testing
```bash
# Test each component individually
python teleop/robot_control/robot_arm_ik.py      # IK solver
python teleop/robot_control/robot_arm.py         # Robot control
python teleop/image_server/image_client.py       # Image transmission
python teleop/utils/weighted_moving_filter.py    # Motion filtering
```

### 2. Integration Testing
```bash
# Test with simulation (no hardware required)
python teleop_hand_and_arm.py --sim --arm=G1_29 --ee=dex3
```

### 3. Hardware Testing
```bash
# Test with physical robot (requires hardware)
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --xr-mode=hand
```

## Performance Monitoring

### Real-Time Metrics
- **IK Solver Performance**: Convergence time and success rate
- **Communication Latency**: Robot command and image transmission delays
- **Frame Rate**: XR tracking and image processing frequency
- **Joint Position Accuracy**: Error between commanded and actual positions

### Debugging Tools
1. **Meshcat Visualizer**: Web-based 3D visualization
2. **Rerun Visualizer**: Advanced data visualization and logging
3. **Performance Profiling**: Built-in timing measurements
4. **Log Analysis**: Structured logging with configurable levels

## Common Development Issues

### 1. URDF Path Resolution
- **Problem**: Different relative paths for testing vs production
- **Solution**: Use `Unit_Test` flag to switch path resolution

### 2. Network Configuration
- **Problem**: XR device connection failures
- **Solution**: Verify SSL certificates and network connectivity

### 3. IK Convergence
- **Problem**: Inverse kinematics fails to find solutions
- **Solution**: Check joint limits and initial conditions

### 4. Image Transmission Latency
- **Problem**: High latency in video feed
- **Solution**: Adjust compression settings and network bandwidth

## Code Quality Standards

### Testing Requirements
- All major components must support unit testing
- Integration tests for complete workflows
- Performance benchmarks for real-time components

### Documentation Standards
- Comprehensive docstrings for public APIs
- Configuration file documentation
- Hardware setup guides

### Safety Considerations
- Emergency stop mechanisms
- Joint limit enforcement
- Communication timeout handling
- Error recovery procedures

This development workflow ensures reliable, maintainable code while supporting both research and production deployment scenarios.