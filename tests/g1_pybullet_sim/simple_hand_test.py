import numpy as np
from arktypes import joint_group_command_t, task_space_command_t, joint_state_t
from arktypes.utils import unpack, pack
from ark.client.comm_infrastructure.instance_node import InstanceNode
import pickle
from time import sleep

SIM = False  # Using real robot

class G1SimpleHandTest(InstanceNode):
    """
    Simple hand test script for quick testing
    """

    def __init__(self):
        '''
        Initialize the G1 Simple Hand Test Controller.
        '''
        super().__init__("G1SimpleHandTest")

        if SIM == True:
            self.joint_group_command = self.create_publisher("unitree_g1_sim/joint_group_command", joint_group_command_t)
            self.state = self.create_listener("unitree_g1_sim/joint_states", joint_state_t)
        else:
            self.joint_group_command = self.create_publisher("unitree_g1/joint_group_command", joint_group_command_t)
            self.state = self.create_listener("unitree_g1/joint_states", joint_state_t)

    def send_hand_command(self, hand_group, positions, duration=2.0):
        """Send hand command and wait"""
        self.joint_group_command.publish(pack.joint_group_command(positions, hand_group))
        sleep(duration)

    def run_simple_test(self):
        """Run simple hand test sequence"""
        print("Simple Hand Test")
        print("=" * 30)
        
        # Define positions
        open_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        closed_pos = {
            'left': np.array([0.0, 0.3, 1.0, -0.7, -1.0, -0.7, -1.0]),
            'right': np.array([0.0, -0.3, -1.0, 0.8, 1.0, 0.8, 1.0])
        }
        
        try:
            # 1. Open all hands
            print("Opening all hands...")
            self.send_hand_command("left_hand", open_pos, 1.0)
            self.send_hand_command("right_hand", open_pos, 1.0)
            sleep(1.0)
            
            # 2. Close left hand
            print("Closing left hand...")
            self.send_hand_command("left_hand", closed_pos['left'], 2.0)
            sleep(1.0)
            
            # 3. Close right hand
            print("Closing right hand...")
            self.send_hand_command("right_hand", closed_pos['right'], 2.0)
            sleep(2.0)
            
            # 4. Open left hand
            print("Opening left hand...")
            self.send_hand_command("left_hand", open_pos, 2.0)
            sleep(1.0)
            
            # 5. Open right hand
            print("Opening right hand...")
            self.send_hand_command("right_hand", open_pos, 2.0)
            sleep(1.0)
            
            print("Test completed")
            
        except KeyboardInterrupt:
            print("\nTest interrupted")
            self.send_hand_command("left_hand", open_pos, 0.5)
            self.send_hand_command("right_hand", open_pos, 0.5)

# Run the test
if __name__ == "__main__":
    try:
        test = G1SimpleHandTest()
        print("Simple Hand Test initialized")
        sleep(3)  # Wait for robot
        test.run_simple_test()
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        print("Test finished")
