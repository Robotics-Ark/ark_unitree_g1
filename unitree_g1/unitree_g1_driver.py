import sys
import time
import copy
import numpy as np
import threading
from pathlib import Path
from typing import Dict, Any, List
import cv2
from multiprocessing import Array, Lock

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
    ChannelPublisher,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_._MotorState_ import MotorState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_._IMUState_ import IMUState_
from unitree_sdk2py.idl.nav_msgs.msg.dds_._Odometry_ import Odometry_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from ark.tools.log import log
from ark.system.driver.robot_driver import RobotDriver

# Import the arm and hand controllers
from robot_arm import G1_29_ArmController, G1_29_JointIndex, G1_29_JointArmIndex
from robot_hand_unitree import Dex3_1_Controller, Dex3_1_Left_JointIndex, Dex3_1_Right_JointIndex


class UnitreeG1Driver(RobotDriver):
    def __init__(
        self, component_name: str, component_config: Dict[str, Any] = None
    ) -> None:
        """!
        Initialize the Unitree G1 driver using robot_arm.py and robot_hand_unitree.py packages

        @param component_name Name of the robot.
        @param component_config Configuration dictionary for the robot.
        """
        super().__init__(component_name, component_config, False)
        self.network_interface = self.config.get("network_interface", "lo")
        self.domain_id = self.config.get("domain_id", 1)
        self.name = "unitree_g1"
        
        # G1 Joint names matching the Ark system order (43 DOF total)
        self.joint_names = [
            # Left leg (6 DOF)
            "left_hip_pitch_joint",
            "left_hip_roll_joint", 
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            # Right leg (6 DOF)
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint", 
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            # Waist (3 DOF)
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            # Left arm (7 DOF)
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            # Right arm (7 DOF)
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
            # Left hand (7 DOF)
            "left_hand_thumb_0_joint",
            "left_hand_thumb_1_joint",
            "left_hand_thumb_2_joint",
            "left_hand_middle_0_joint",
            "left_hand_middle_1_joint",
            "left_hand_index_0_joint",
            "left_hand_index_1_joint",
            # Right hand (7 DOF)
            "right_hand_thumb_0_joint",
            "right_hand_thumb_1_joint",
            "right_hand_thumb_2_joint",
            "right_hand_middle_0_joint",
            "right_hand_middle_1_joint",
            "right_hand_index_0_joint",
            "right_hand_index_1_joint",
        ]

        # Total joints: 43 (6+6+3+7+7+7+7)
        self.num_joints = len(self.joint_names)
        self.num_leg_waist_joints = 15  # 6+6+3
        self.num_arm_joints = 14  # 7+7
        self.num_hand_joints = 14  # 7+7

        # Initialize joint state arrays
        self.joint_positions = [0.0] * self.num_joints
        self.joint_velocities = [0.0] * self.num_joints
        self.joint_accelerations = [0.0] * self.num_joints
        print("Network interface:", self.network_interface)
        print("Domain ID:", self.domain_id)
        # Initialize the arm controller
        print("Initializing G1_29_ArmController...")
        self.arm_controller = G1_29_ArmController(
            network_interface=self.network_interface,
            domain_id=self.domain_id
        )
        time.sleep(5)

        # Initialize hand control arrays for the hand controller
        self.left_hand_array = Array('d', 7, lock=True)
        self.right_hand_array = Array('d', 7, lock=True)
        self.dual_hand_data_lock = Lock()
        self.dual_hand_state_array = Array('d', 14, lock=True)  # 7+7
        self.dual_hand_action_array = Array('d', 14, lock=True)  # 7+7

        # Initialize the hand controller
        print("Initializing Dex3_1_Controller...")
        self.hand_controller = Dex3_1_Controller(
            left_hand_array=self.left_hand_array,
            right_hand_array=self.right_hand_array,
            dual_hand_data_lock=self.dual_hand_data_lock,
            dual_hand_state_array=self.dual_hand_state_array,
            dual_hand_action_array=self.dual_hand_action_array,
            fps=100.0,
            Unit_Test=False,
            network_interface=self.network_interface,
            domain_id=self.domain_id
        )

        # Create joint mapping indices
        self._create_joint_mappings()

        # Initialize low-level communication for legs and waist
        self._init_low_level_communication()

        # Start state update thread
        self._running = True
        self.state_update_thread = threading.Thread(target=self._update_joint_states)
        self.state_update_thread.daemon = True
        self.state_update_thread.start()

        print("UnitreeG1Driver initialization complete")

    def _create_joint_mappings(self):
        """Create mappings between Ark joint names and controller indices"""
        
        # Map Ark joint names to G1_29_JointIndex for legs, waist, and arms
        self.ark_to_g1_mapping = {}
        self.g1_to_ark_mapping = {}
        
        # Legs and waist mapping (0-14)
        leg_waist_mapping = {
            "left_hip_pitch_joint": G1_29_JointIndex.kLeftHipPitch,
            "left_hip_roll_joint": G1_29_JointIndex.kLeftHipRoll,
            "left_hip_yaw_joint": G1_29_JointIndex.kLeftHipYaw,
            "left_knee_joint": G1_29_JointIndex.kLeftKnee,
            "left_ankle_pitch_joint": G1_29_JointIndex.kLeftAnklePitch,
            "left_ankle_roll_joint": G1_29_JointIndex.kLeftAnkleRoll,
            "right_hip_pitch_joint": G1_29_JointIndex.kRightHipPitch,
            "right_hip_roll_joint": G1_29_JointIndex.kRightHipRoll,
            "right_hip_yaw_joint": G1_29_JointIndex.kRightHipYaw,
            "right_knee_joint": G1_29_JointIndex.kRightKnee,
            "right_ankle_pitch_joint": G1_29_JointIndex.kRightAnklePitch,
            "right_ankle_roll_joint": G1_29_JointIndex.kRightAnkleRoll,
            "waist_yaw_joint": G1_29_JointIndex.kWaistYaw,
            "waist_roll_joint": G1_29_JointIndex.kWaistRoll,
            "waist_pitch_joint": G1_29_JointIndex.kWaistPitch,
        }
        
        # Arms mapping (15-21 for left arm, 29-35 for right arm)
        arm_mapping = {
            "left_shoulder_pitch_joint": G1_29_JointIndex.kLeftShoulderPitch,
            "left_shoulder_roll_joint": G1_29_JointIndex.kLeftShoulderRoll,
            "left_shoulder_yaw_joint": G1_29_JointIndex.kLeftShoulderYaw,
            "left_elbow_joint": G1_29_JointIndex.kLeftElbow,
            "left_wrist_roll_joint": G1_29_JointIndex.kLeftWristRoll,
            "left_wrist_pitch_joint": G1_29_JointIndex.kLeftWristPitch,
            "left_wrist_yaw_joint": G1_29_JointIndex.kLeftWristyaw,
            "right_shoulder_pitch_joint": G1_29_JointIndex.kRightShoulderPitch,
            "right_shoulder_roll_joint": G1_29_JointIndex.kRightShoulderRoll,
            "right_shoulder_yaw_joint": G1_29_JointIndex.kRightShoulderYaw,
            "right_elbow_joint": G1_29_JointIndex.kRightElbow,
            "right_wrist_roll_joint": G1_29_JointIndex.kRightWristRoll,
            "right_wrist_pitch_joint": G1_29_JointIndex.kRightWristPitch,
            "right_wrist_yaw_joint": G1_29_JointIndex.kRightWristYaw,
        }
        
        self.ark_to_g1_mapping.update(leg_waist_mapping)
        self.ark_to_g1_mapping.update(arm_mapping)
        
        # Create reverse mapping
        for ark_name, g1_index in self.ark_to_g1_mapping.items():
            self.g1_to_ark_mapping[g1_index.value] = ark_name

        # Hand joint indices in Ark joint array (29-35 for left hand, 36-42 for right hand)
        self.hand_joint_indices = {
            "left_hand_thumb_0_joint": 29,
            "left_hand_thumb_1_joint": 30,
            "left_hand_thumb_2_joint": 31,
            "left_hand_middle_0_joint": 32,
            "left_hand_middle_1_joint": 33,
            "left_hand_index_0_joint": 34,
            "left_hand_index_1_joint": 35,
            "right_hand_thumb_0_joint": 36,
            "right_hand_thumb_1_joint": 37,
            "right_hand_thumb_2_joint": 38,
            "right_hand_middle_0_joint": 39,
            "right_hand_middle_1_joint": 40,
            "right_hand_index_0_joint": 41,
            "right_hand_index_1_joint": 42,
        }

    def _init_low_level_communication(self):
        """Initialize low-level communication for legs and waist"""
        # ChannelFactoryInitialize is already called by arm_controller, so we don't need to call it again
        
        # Get subscribers for low-level state
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._low_state_callback, 10)

        # Get publisher for low-level commands
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # Initialize low-level command message
        self.crc = CRC()
        self.low_cmd_msg = unitree_hg_msg_dds__LowCmd_()
        self.low_cmd_msg.mode_pr = 0

    def _low_state_callback(self, msg: LowState_):
        """Callback for low-level state messages (legs, waist)"""
        # Update joint positions and velocities for legs and waist (0-14)
        for ark_name, g1_index in self.ark_to_g1_mapping.items():
            if g1_index.value < 15:  # Only legs and waist
                ark_index = self.joint_names.index(ark_name)
                self.joint_positions[ark_index] = msg.motor_state[g1_index.value].q
                self.joint_velocities[ark_index] = msg.motor_state[g1_index.value].dq
                self.joint_accelerations[ark_index] = msg.motor_state[g1_index.value].ddq

    def _update_joint_states(self):
        """Update joint states from arm and hand controllers"""
        while self._running:
            try:
                # Update arm joint states (left arm: 15-21, right arm: 29-35)
                arm_q = self.arm_controller.get_current_dual_arm_q()
                arm_dq = self.arm_controller.get_current_dual_arm_dq()
                
                # Map arm states to Ark joint array
                for i, g1_arm_index in enumerate(G1_29_JointArmIndex):
                    ark_name = self.g1_to_ark_mapping[g1_arm_index.value]
                    ark_index = self.joint_names.index(ark_name)
                    self.joint_positions[ark_index] = arm_q[i]
                    self.joint_velocities[ark_index] = arm_dq[i]

                # Update hand joint states (left hand: 22-28, right hand: 36-42)
                with self.dual_hand_data_lock:
                    hand_states = np.array(self.dual_hand_state_array[:])
                
                # Map hand states to Ark joint array
                for ark_name, ark_index in self.hand_joint_indices.items():
                    if "left" in ark_name:
                        # Left hand joints (0-6 in hand_states)
                        hand_idx = self._get_left_hand_index(ark_name)
                        self.joint_positions[ark_index] = hand_states[hand_idx]
                    else:
                        # Right hand joints (7-13 in hand_states)
                        hand_idx = self._get_right_hand_index(ark_name)
                        self.joint_positions[ark_index] = hand_states[hand_idx + 7]

            except Exception as e:
                log.error(f"Error updating joint states: {e}")
            
            time.sleep(0.01)  # 100Hz update rate

    def _get_left_hand_index(self, joint_name: str) -> int:
        """Get index in left hand array for given joint name"""
        left_hand_mapping = {
            "left_hand_thumb_0_joint": 0,
            "left_hand_thumb_1_joint": 1,
            "left_hand_thumb_2_joint": 2,
            "left_hand_middle_0_joint": 3,
            "left_hand_middle_1_joint": 4,
            "left_hand_index_0_joint": 5,
            "left_hand_index_1_joint": 6,
        }
        return left_hand_mapping[joint_name]

    def _get_right_hand_index(self, joint_name: str) -> int:
        """Get index in right hand array for given joint name"""
        right_hand_mapping = {
            "right_hand_thumb_0_joint": 0,
            "right_hand_thumb_1_joint": 1,
            "right_hand_thumb_2_joint": 2,
            "right_hand_middle_0_joint": 3,
            "right_hand_middle_1_joint": 4,
            "right_hand_index_0_joint": 5,
            "right_hand_index_1_joint": 6,
        }
        return right_hand_mapping[joint_name]

    # Ark RobotDriver interface methods
    def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
        """Return joint positions for specified joints"""
        joint_positions = {}
        for joint in joints:
            if joint in self.joint_names:
                index = self.joint_names.index(joint)
                joint_positions[joint] = self.joint_positions[index]
            else:
                log.error(f"Joint {joint} not found in joint names.")
        return joint_positions

    def pass_joint_velocities(self, joints: List[str]) -> Dict[str, float]:
        """Return joint velocities for specified joints"""
        joint_velocities = {}
        for joint in joints:
            if joint in self.joint_names:
                index = self.joint_names.index(joint)
                joint_velocities[joint] = self.joint_velocities[index]
            else:
                log.error(f"Joint {joint} not found in joint names.")
        return joint_velocities

    def pass_joint_acceleration(self, joints: List[str]) -> Dict[str, float]:
        """Return joint accelerations for specified joints"""
        joint_accelerations = {}
        for joint in joints:
            if joint in self.joint_names:
                index = self.joint_names.index(joint)
                joint_accelerations[joint] = self.joint_accelerations[index]
            else:
                log.error(f"Joint {joint} not found in joint names.")
        return joint_accelerations

    def pass_joint_efforts(self, joints: List[str]) -> Dict[str, float]:
        """Return joint efforts for specified joints"""
        # Not implemented in the current packages
        raise NotImplementedError("Joint efforts not available in current implementation")

    def pass_joint_group_control_cmd(
        self, control_mode: str, cmd: Dict[str, float], **kwargs
    ) -> None:
        """Send control commands to the robot"""
        if control_mode == "position":
            # Separate commands by joint type
            leg_waist_cmd = {}
            arm_cmd = {}
            hand_cmd = {}

            for joint_name, target_pos in cmd.items():
                if joint_name in self.ark_to_g1_mapping:
                    g1_index = self.ark_to_g1_mapping[joint_name]
                    if g1_index.value < 15:  # Legs and waist
                        leg_waist_cmd[joint_name] = target_pos
                    elif g1_index.value < 29:  # Arms (both left and right arms)
                        arm_cmd[joint_name] = target_pos
                elif joint_name in self.hand_joint_indices:
                    hand_cmd[joint_name] = target_pos

            # Minimal output for visibility during development
            if arm_cmd:
                print(f"Sending arm commands: {list(arm_cmd.keys())}")
            if hand_cmd:
                print(f"Sending hand commands: {list(hand_cmd.keys())}")
            if leg_waist_cmd:
                print(f"Sending leg/waist commands: {list(leg_waist_cmd.keys())}")

            # Send commands to appropriate controllers
            if leg_waist_cmd:
                self._send_leg_waist_commands(leg_waist_cmd)
            
            if arm_cmd:
                self._send_arm_commands(arm_cmd)
            
            if hand_cmd:
                self._send_hand_commands(hand_cmd)

    def _send_leg_waist_commands(self, cmd: Dict[str, float]):
        """Send commands to legs and waist via low-level interface"""
        for joint_name, target_pos in cmd.items():
            g1_index = self.ark_to_g1_mapping[joint_name]
            self.low_cmd_msg.motor_cmd[g1_index.value].mode = 0x01
            self.low_cmd_msg.motor_cmd[g1_index.value].q = target_pos
            self.low_cmd_msg.motor_cmd[g1_index.value].dq = 0.0
            self.low_cmd_msg.motor_cmd[g1_index.value].kp = 60.0
            self.low_cmd_msg.motor_cmd[g1_index.value].kd = 5.0
            self.low_cmd_msg.motor_cmd[g1_index.value].tau = 0.0

        # Send command
        self.low_cmd_msg.crc = self.crc.Crc(self.low_cmd_msg)
        self.lowcmd_publisher.Write(self.low_cmd_msg)

    def _send_arm_commands(self, cmd: Dict[str, float]):
        """Send commands to arms via arm controller"""
        # Prepare arm command array (14 elements: 7 left + 7 right)
        arm_q_target = np.zeros(14)
        
        for joint_name, target_pos in cmd.items():
            g1_index = self.ark_to_g1_mapping[joint_name]
            # Find index in G1_29_JointArmIndex
            for i, arm_index in enumerate(G1_29_JointArmIndex):
                if arm_index.value == g1_index.value:
                    arm_q_target[i] = target_pos
                    break

        # print(f"Sending arm commands: {arm_q_target}")
        # Send to arm controller
        self.arm_controller.ctrl_dual_arm(arm_q_target, np.zeros(14))

    def _send_hand_commands(self, cmd: Dict[str, float]):
        """Send commands to hands via hand controller"""
        # Start from current targets to avoid resetting the other hand to zeros
        with self.dual_hand_data_lock:
            left_hand_target = np.array(self.left_hand_array[:])
            right_hand_target = np.array(self.right_hand_array[:])

        for joint_name, target_pos in cmd.items():
            if "left" in joint_name:
                hand_idx = self._get_left_hand_index(joint_name)
                left_hand_target[hand_idx] = target_pos
            else:
                hand_idx = self._get_right_hand_index(joint_name)
                right_hand_target[hand_idx] = target_pos

        # Update shared arrays
        with self.dual_hand_data_lock:
            self.left_hand_array[:] = left_hand_target
            self.right_hand_array[:] = right_hand_target

    # ---- Direct arm control with tau feedforward ----
    def arm_ctrl_q_tau(self, q14: np.ndarray, tau14: np.ndarray):
        """Directly command both arms with desired joint positions and tau feedforward.

        Expects 14-length vectors: left arm (7) followed by right arm (7).
        """
        try:
            q14 = np.array(q14, dtype=float).reshape(14)
            tau14 = np.array(tau14, dtype=float).reshape(14)
        except Exception:
            print("[UnitreeG1Driver] arm_ctrl_q_tau: invalid input shapes; expected 14 each")
            return
        self.arm_controller.ctrl_dual_arm(q14, tau14)

    def pass_lidar_data(self):
        """Return lidar data"""
        # Not implemented in current packages
        return None

    def pass_camera_image(self):
        """Return camera image"""
        # Not implemented in current packages
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def check_torque_status(self, joints: List[str]) -> Dict[str, bool]:
        """Check torque status for joints"""
        # Not implemented in current packages
        raise NotImplementedError("Torque status not available in current implementation")

    def shutdown_driver(self):
        """Shutdown the driver and all controllers"""
        self._running = False
        
        # Shutdown arm controller
        if hasattr(self, 'arm_controller'):
            self.arm_controller.ctrl_dual_arm_go_home()
            time.sleep(2.0)
            self.arm_controller.shutdown()
        
        # Shutdown hand controller
        if hasattr(self, 'hand_controller'):
            self.hand_controller.shutdown()
        
        print("UnitreeG1Driver shutdown complete")
