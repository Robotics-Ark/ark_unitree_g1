# Unitree G1 Integration

This repository provides drivers, simulation bridges, and demos to control the Unitree G1 robot (arms, hands, legs/waist) with multiple running modalities: real robot, PyBullet simulation, and MuJoCo simulation.

## Install
```
pip install -r requirements.txt
```

## Prerequirements (external)
- https://github.com/unitreerobotics/unitree_sdk2_python.git
- conda install conda-forge::pinocchio


## Repository Structure (high-level)
- `unitree_g1/`
  - `unitree_g1.py`: Node wrapper integrating robot driver with Ark comms and joint groups.
  - `unitree_g1_driver.py`: Real robot driver that talks to Unitree SDK (arms, hands, legs/waist).
  - `robot_arm.py`: Low-level dual-arm controller using Unitree SDK channels (G1_29 arm indices).
  - `robot_hand_unitree.py`: Low-level hand controller (Dex3-1) using Unitree SDK channels.
  - `unitree_g1.yaml`: Configuration for real robot.
  - `unitree_g1_sim.yaml`: Configuration for simulation.
  - `urdf/urdf/g1_description.urdf`: Robot description used by IK and simulation tooling.
- `tests/`
  - `g1_pybullet_sim/`: PyBullet-based simulation utilities and example nodes.
    - `sim_node.py`: Example simulation node wiring Ark topics in PyBullet.
    - `g1_controller_interface.py`: Minimal example command publisher for all joints.
    - `simple_hand_test.py`: Quick hand open/close test script.
  - `g1_mujoco_sim/`: MuJoCo simulation bridge and configuration.
    - `unitree_mujoco.py`: MuJoCo scene launcher.
    - `unitree_sdk2py_bridge.py`: Bridge that publishes/consumes Unitree SDK-like topics from MuJoCo.
  - `sim_real/`: Real-to-sim bridge utilities.
    - `real_to_sim.py`: Subscribes real `joint_states` and republishes to sim as `joint_group_command`.
  - `test_demo/`: Higher-level demos and IK utilities.
    - `demo_pick_place.py`: Right-arm pick-and-place demo using IK and streaming commands.
    - `robot_arm_ik.py`: Pinocchio-based IK solvers for G1/H1 variants.


## Running Modalities

### 1) Real Robot (arms, hands, legs/waist)
- Ensure Unitree SDK2 Python is installed and reachable (topics, DDS domains, NIC).
- Configure `unitree_g1/unitree_g1.yaml` as needed (network interface, domain id, enabled sensors).
- In `unitree_g1/unitree_g1.py`, SIM determines topic names; for real robot set `SIM = False`.

Run:
```
cd unitree_g1
python unitree_g1.py
```

This launches the Ark node, creates publishers/subscribers, and manages joint groups. The driver (`unitree_g1_driver.py`) internally spins threads to track state and send commands to arms and hands.


### 2) PyBullet Simulation
Example minimal controller that drives all joints with tiny position targets (useful for plumbing checks):
```
python tests/g1_pybullet_sim/g1_controller_interface.py
```

Simple hand test sequence (open/close):
```
python tests/g1_pybullet_sim/simple_hand_test.py
```

Simulation configuration is under `tests/g1_pybullet_sim/config/`.


### 3) MuJoCo Simulation
The MuJoCo bridge emulates Unitree SDK topics for body and hands and consumes low-level commands.

Key entry points in `tests/g1_mujoco_sim/`:
- `unitree_mujoco.py`: Loads the scene and model.
- `unitree_sdk2py_bridge.py`: Publishes simulated `LowState`, `HandState` and consumes `LowCmd`, `HandCmd`.

Typical flow is to start the MuJoCo scene, then the bridge, then send commands from client scripts.


### 4) Real-to-Sim Topic Bridge
Bridge the real robot state into a simulation instance:
```
python tests/sim_real/real_to_sim.py
```
This subscribes to `unitree_g1/joint_states` and republishes the values to `unitree_g1_sim/joint_group_command` with group `all`.


### 5) Demos (IK-driven)
Pick-and-Place using IK for the right arm:
```
python tests/test_demo/demo_pick_place.py
```

Pinocchio-based IK utilities and manual demos are in `tests/test_demo/robot_arm_ik.py`.

Can be used both for controlling the robot in Mujoco or the real one.


## Joint Groups and Control
- Joint order: the system uses a 43-DoF ordering (legs/waist, arms, hands). See `unitree_g1/unitree_g1.py` for the full list and indices.
- Available groups include:
  - `all` (position control)
  - `left_hand` (position control; 7 joints)
  - `right_hand` (position control; 7 joints)
  - special: `arms_q_tau` expects 28 numbers `[q14, tau14]` (7 left + 7 right for `q` and `tau`). Useful for smooth motion with feed forward feedback.

Publish commands via Ark `joint_group_command_t` with the appropriate `name` and `cmd` payload.


## Troubleshooting
- Ensure the correct network interface and DDS domain are set in your config for real robot.
- For MuJoCo or PyBullet, verify the system dependencies (graphics, drivers) and the versions from `requirements.txt`.
- Ensure your real G1 is in Debug Mode before starting.

## Acknowledgments
Large parts of this repository are adapted from `unitreerobotics/xr_teleoperate`:
- https://github.com/unitreerobotics/xr_teleoperate


