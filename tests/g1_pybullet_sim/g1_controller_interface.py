import numpy as np
from arktypes import joint_group_command_t, task_space_command_t, joint_state_t
from arktypes.utils import unpack, pack
from ark.client.comm_infrastructure.instance_node import InstanceNode
import pickle
from time import sleep

SIM = True

class G1ControllerNode(InstanceNode):

    def __init__(self):
        '''
        Initialize the G1.
        This class is responsible for controlling the Husky robot's joints.
        '''
        super().__init__("G1Controller")

        if SIM == True:
            self.joint_group_command = self.create_publisher("unitree_g1/joint_group_command/sim", joint_group_command_t)

            self.state = self.create_listener("unitree_g1/joint_states", joint_state_t)

controller = G1ControllerNode()
sleep(5)
# FORWARDS
joint_command = np.array([0.5]*42)
# Velocity Control
while True:
    controller.joint_group_command.publish(pack.joint_group_command(joint_command, "all"))
    print(unpack.joint_state(controller.state.get()))

