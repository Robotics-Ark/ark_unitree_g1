import numpy as np
from arktypes import joint_group_command_t, task_space_command_t, joint_state_t
from arktypes.utils import unpack, pack
from ark.client.comm_infrastructure.instance_node import InstanceNode
import pickle
from time import sleep

SIM = True  # Using real robot

class G1ControllerNode(InstanceNode):

    def __init__(self):
        '''
        Initialize the G1.
        '''
        super().__init__("G1Controller")

        if SIM == True:
            self.joint_group_command = self.create_publisher("unitree_g1_sim/joint_group_command", joint_group_command_t)

            self.state = self.create_listener("unitree_g1_sim/joint_states", joint_state_t)
        else:
            self.joint_group_command = self.create_publisher("unitree_g1/joint_group_command", joint_group_command_t)
            self.state = self.create_listener("unitree_g1/joint_states", joint_state_t)

controller = G1ControllerNode()
sleep(5)
# Create command for ALL joints with small values
# G1 robot has 29 body joints + 14 hand joints = 43 total joints
# Based on config: 6 (left leg) + 6 (right leg) + 3 (waist) + 7 (left arm) + 7 (right arm) + 14 (hands) = 43
all_joints_command = np.full(43, 0.001)  # Small value for all 43 joints

print(f"Controller: Sending small values (0.001) to ALL {len(all_joints_command)} joints")
print("Joint structure: 29 body joints + 14 hand joints = 43 total")

# Position Control - All Joints
while True:
    controller.joint_group_command.publish(pack.joint_group_command(all_joints_command, "all"))
    sleep(0.1)

