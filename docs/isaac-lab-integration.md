# Isaac Lab Integration Guide

## Installation Summary

Isaac Lab has been successfully installed and integrated with the existing teleoperation system. This document provides guidance on leveraging Isaac Lab for advanced simulation and learning capabilities.

## Installation Details

### Environment Setup
- **Python Version**: 3.10 (required for Isaac Sim compatibility)
- **Environment Name**: `isaaclab`
- **Isaac Sim Version**: 4.5.0.0
- **Isaac Lab Version**: 0.40.21

### Installed Components
```bash
# Core packages installed:
- isaacsim[all,extscache]==4.5.0
- isaaclab==0.40.21
- PyTorch 2.5.1 with CUDA 12.1 support
- Various robotics libraries (pinocchio, dex-retargeting)
```

### Directory Structure
```
/home/razer/avp_teleoperate/
├── IsaacLab/                    # Isaac Lab repository
│   ├── source/                  # Core Isaac Lab packages
│   ├── scripts/                 # Examples and utilities
│   └── apps/                    # Application configurations
├── teleop/                      # Original teleoperation system
└── docs/                        # Documentation
```

## Integration Capabilities

### 1. Advanced Physics Simulation
Isaac Lab provides GPU-accelerated physics simulation with:
- Real-time robot dynamics
- Contact simulation
- Deformable objects
- Fluid simulation capabilities

### 2. Reinforcement Learning Integration
```python
# Example: Create RL environment for G1 robot
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.sim import SimulationConfig

# Configure G1 robot environment
env_cfg = SimulationConfig(
    robot_model="g1_29dof",
    physics_dt=1/240.0,
    rendering_dt=1/60.0
)
```

### 3. Sensor Simulation
- **Camera Arrays**: Multi-camera setups for stereo vision
- **IMU Sensors**: Inertial measurement simulation
- **Force/Torque Sensors**: Contact force measurement
- **Ray Casting**: LIDAR and proximity sensors

### 4. Asset Integration
Isaac Lab can directly load and simulate:
- **G1 Robot**: Both 29-DoF and 23-DoF configurations
- **H1 Variants**: H1 and H1_2 with different arm configurations
- **Dexterous Hands**: Unitree Dex3-1, Inspire hands
- **Custom Assets**: Via URDF/USD conversion

## Usage Examples

### Basic Simulation Setup
```python
import isaaclab
from isaaclab.app import AppLauncher

# Launch headless simulation
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

# Load G1 robot
from isaaclab.assets import ArticulationConfig
from isaaclab.sim import SimulationContext

robot_cfg = ArticulationConfig(
    prim_path="/World/Robot",
    spawn=UsdFileCfg(
        usd_path="../assets/g1/g1_body29_hand14.usd",
    ),
)
```

### Teleoperation Data Collection
```python
# Record teleoperation sessions for imitation learning
from isaaclab.envs import ManagerBasedEnv
from isaaclab.utils.io import write_hdf5

# Configure data recording
env = ManagerBasedEnv(cfg=env_cfg)
episode_data = []

for step in range(1000):
    # Collect XR input from existing teleop system
    xr_poses = get_xr_poses()  # From existing teleop/televuer
    
    # Apply to Isaac Lab simulation
    env.step(xr_poses)
    
    # Record state for imitation learning
    episode_data.append({
        "obs": env.obs,
        "actions": xr_poses,
        "rewards": env.rewards
    })

# Save dataset
write_hdf5("teleop_dataset.h5", episode_data)
```

### Training Pipeline Integration
```python
# Train policies on teleoperation data
from isaaclab_mimic import MimicConfig, train_mimic

# Configure imitation learning
mimic_cfg = MimicConfig(
    dataset_path="teleop_dataset.h5",
    policy_type="transformer",
    observation_space=env.observation_space,
    action_space=env.action_space
)

# Train policy
policy = train_mimic(mimic_cfg)
```

## Integration with Existing Teleoperation System

### 1. Unified Asset Pipeline
```python
# Convert existing URDF assets to USD for Isaac Lab
from isaaclab.assets.usd import convert_urdf_to_usd

# Convert G1 robot
convert_urdf_to_usd(
    urdf_path="../assets/g1/g1_body29_hand14.urdf",
    usd_path="../assets/g1/g1_body29_hand14.usd"
)
```

### 2. Shared Hand Retargeting
```python
# Use existing dex-retargeting with Isaac Lab
from dex_retargeting import RetargetingConfig
from isaaclab.controllers import DexterousHandController

# Initialize retargeting with existing config
retargeting_cfg = RetargetingConfig.from_yaml("../assets/unitree_hand/unitree_dex3.yml")
hand_controller = DexterousHandController(retargeting_cfg)
```

### 3. Camera Integration
```python
# Synchronize Isaac Lab cameras with existing image pipeline
from isaaclab.sensors import CameraCfg
from teleop.image_server import ImageClient

# Configure cameras to match physical setup
camera_cfg = CameraCfg(
    height=720,
    width=1280,
    data_types=["rgb", "depth"],
    spawn=RigidPrimCfg(
        rigid_props=RigidBodyPropertiesCfg(),
        mass_props=MassPropertiesCfg(mass=0.1),
    )
)
```

## Advantages for XR Teleoperation

### 1. Simulation-to-Reality Transfer
- **Physics Validation**: Test control algorithms in simulation
- **Safety Testing**: Validate motions before physical execution
- **Failure Recovery**: Train robust behaviors in simulation

### 2. Data Augmentation
- **Diverse Scenarios**: Generate varied training conditions
- **Domain Randomization**: Improve policy generalization
- **Synthetic Data**: Supplement real teleoperation data

### 3. Multi-Agent Training
- **Parallel Environments**: Train multiple robots simultaneously
- **Curriculum Learning**: Progressive difficulty increase
- **Policy Distillation**: Transfer from teacher to student policies

### 4. Advanced Rendering
- **Ray-traced Visuals**: High-fidelity rendering for VR/AR
- **Real-time Lighting**: Dynamic lighting conditions
- **Material Simulation**: Realistic surface properties

## Integration Workflow

### Phase 1: Asset Migration
1. Convert existing URDF models to USD format
2. Validate physics properties and joint limits
3. Test basic simulation functionality

### Phase 2: Controller Integration
1. Integrate existing IK solvers with Isaac Lab
2. Port hand retargeting algorithms
3. Validate motion quality and performance

### Phase 3: Learning Pipeline
1. Set up data collection from teleoperation sessions
2. Implement imitation learning workflows
3. Train and evaluate policies

### Phase 4: Deployment
1. Deploy trained policies back to physical robots
2. Implement sim-to-real transfer techniques
3. Continuous learning from new demonstrations

## Environment Activation

To use Isaac Lab in the teleoperation project:

```bash
# Activate Isaac Lab environment
source /home/razer/miniforge3/bin/activate isaaclab

# Navigate to Isaac Lab directory
cd /home/razer/avp_teleoperate/IsaacLab

# Run examples (requires EULA acceptance on first run)
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py --headless

# Or use Python directly
python scripts/demos/quadrupeds.py --headless
```

## Next Steps

1. **Accept EULA**: On first run, accept the NVIDIA Omniverse EULA
2. **Asset Conversion**: Convert G1/H1 models to USD format for optimal performance
3. **Data Collection**: Set up recording pipeline for teleoperation sessions
4. **Policy Training**: Implement imitation learning on collected data
5. **Sim-to-Real**: Deploy trained policies to physical robots

## Performance Considerations

- **GPU Memory**: Isaac Lab requires significant GPU memory (~8GB+)
- **Compute Requirements**: RTX 3070 is suitable for development
- **Rendering Performance**: Use headless mode for training, rendering for visualization
- **Multi-Environment**: Leverage parallel simulation for faster training

## Support Resources

- **Isaac Lab Documentation**: https://isaac-sim.github.io/IsaacLab/
- **Community Forums**: NVIDIA Developer Forums
- **Example Scripts**: Located in `IsaacLab/scripts/`
- **Asset Library**: Pre-built robots and environments

This integration enables the teleoperation system to leverage advanced simulation capabilities for improved training, validation, and deployment of robotic control policies.