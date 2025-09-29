ROBOT = "g1" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1" 
ROBOT_SCENE = f"../../unitree_{ROBOT}/mjcf/scene_29dof_with_hand.xml" # Robot scene
DOMAIN_ID = 1 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = 0 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = True # Virtual spring band, used for lifting h1

# Control frequencies
ARM_CONTROL_FREQ = 250.0  # 250Hz for arm
HAND_CONTROL_FREQ = 250.0  # 100Hz for hands
GRIPPER_CONTROL_FREQ = 200.0  # 200Hz for gripper

# Simulation timesteps
SIMULATE_DT = 0.002  # 250Hz for arm control (1/ARM_CONTROL_FREQ)
VIEWER_DT = 0.02  # 50 fps for viewer
