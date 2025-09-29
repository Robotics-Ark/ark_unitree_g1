# G1 MuJoCo Simulation

This folder contains a MuJoCo-based simulation environment for testing the Unitree G1 robot without requiring access to the physical hardware. The simulation provides a realistic physics environment with elastic band support for lifting and controlling the robot.

## Overview

The simulation creates a virtual G1 robot with 29 degrees of freedom (29 DOF) including both arms and hands, allowing you to test control algorithms, motion planning, and robot behavior in a safe, repeatable environment.

## Files

- **`unitree_mujoco.py`** - Main simulation script that launches the MuJoCo viewer and simulation
- **`unitree_sdk2py_bridge.py`** - Bridge between Unitree SDK2 protocol and MuJoCo simulation
- **`config.py`** - Configuration settings for the simulation
- **`motor_indices.py`** - Joint index mappings for the G1 robot's 29 degrees of freedom

## Features

### Robot Model
- **29 DOF G1 Robot**: Complete simulation with legs, waist, arms, and hands
- **Realistic Physics**: MuJoCo physics engine for accurate dynamics
- **Sensor Simulation**: IMU, motor states, and position sensors

### Elastic Band Control
The simulation includes an elastic band feature that acts like a virtual spring to help lift and position the robot. This is particularly useful for testing locomotion and manipulation tasks.

#### Keyboard Controls for Elastic Band:
- **Key 7**: Decrease elastic band length (pull robot up)
- **Key 8**: Increase elastic band length (lower robot)
- **Key 9**: Toggle elastic band on/off

### Communication Bridge
- **DDS Protocol**: Full Unitree SDK2 protocol support
- **Real-time Control**: 250Hz arm control, 250Hz hand control
- **Message Publishing**: Motor states, IMU data, wireless controller input

### Gamepad Support (Optional)
- **Xbox Controller**: Full support for Xbox gamepad layout
- **Nintendo Switch Pro**: Support for Switch Pro controller
- **Wireless Controller Simulation**: Emulates Unitree wireless controller

## Configuration

Edit `config.py` to customize the simulation:

```python
ROBOT = "g1"  # Robot type
ENABLE_ELASTIC_BAND = True  # Enable/disable elastic band
USE_JOYSTICK = 0  # Enable gamepad control (0=disabled, 1=enabled)
JOYSTICK_TYPE = "xbox"  # "xbox" or "switch"
PRINT_SCENE_INFORMATION = True  # Print robot model details
```

### Control Frequencies
- **Arm Control**: 250Hz
- **Hand Control**: 250Hz
- **Gripper Control**: 200Hz

## Usage

### Basic Simulation
```bash
cd tests/g1_mujoco_sim
python unitree_mujoco.py
```

### With Gamepad Control
1. Connect your gamepad (Xbox or Switch Pro)
2. Set `USE_JOYSTICK = 1` in `config.py`
3. Run the simulation

### Elastic Band Controls
When the simulation is running:
- Press **7** to shorten the elastic band (lift the robot)
- Press **8** to lengthen the elastic band (lower the robot)
- Press **9** to toggle the elastic band on/off

## Robot Model Structure

The G1 robot in simulation includes:

### Legs (12 DOF)
- Left/Right Hip: Pitch, Roll, Yaw (6 DOF)
- Left/Right Knee (2 DOF)
- Left/Right Ankle: Pitch, Roll (4 DOF)

### Waist (3 DOF)
- Waist: Yaw, Roll, Pitch

### Arms (14 DOF total, 7 per arm)
- Left/Right Shoulder: Pitch, Roll, Yaw (6 DOF)
- Left/Right Elbow (2 DOF)
- Left/Right Wrist: Roll, Pitch, Yaw (6 DOF)

## Dependencies

- **MuJoCo**: Physics simulation engine
- **Unitree SDK2**: Robot communication protocol
- **Pygame**: Gamepad input handling
- **NumPy**: Numerical computations

## Troubleshooting

### Common Issues
1. **No gamepad detected**: Ensure your controller is connected and recognized by the system
2. **Simulation runs slowly**: Check your system specifications and reduce control frequencies if needed
3. **Elastic band not working**: Ensure `ENABLE_ELASTIC_BAND = True` in config.py

### Performance Tips
- The simulation runs at 250Hz for control and 50Hz for visualization
- For better performance, you can reduce control frequencies in `config.py`
- Ensure your system meets MuJoCo's requirements for smooth simulation

## Integration

This simulation can be used with:
- Unitree SDK2 applications
- Custom control algorithms
- Motion planning systems
- Reinforcement learning environments

The simulation provides the same DDS message interface as the real robot, making it easy to test code that will later run on hardware.
