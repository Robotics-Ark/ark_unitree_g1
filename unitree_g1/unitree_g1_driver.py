import sys
import time
import copy
import numpy as np
from pathlib import Path
from typing import Dict, Any, List
import cv2

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
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from ark.tools.log import log
from ark.system.driver.robot_driver import RobotDriver


class UnitreeG1Driver(RobotDriver):
    def __init__(
        self, component_name: str, component_config: Dict[str, Any] = None
    ) -> None:
        """!
        Initialze the Unitree G1 driver

        @param component_name Name of the robot.
        @param component_config Configuration dictionary for the robot.
        """
        super().__init__(component_name, component_config, False)
        self.network_interface = self.config.get("network_interface", "")
        self.name = "unitree_g1"
        print("here")
        # G1 Joint names matching the URDF structure (29 DOF)
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
        ]

        if self.network_interface != "":
            ChannelFactoryInitialize(0, self.network_interface)
            print("ChannelFactoryInitialize", self.network_interface)
        else:
            ChannelFactoryInitialize(1)

        # TODO; Add high level control

        # Note: G1 doesn't have a dedicated video client like GO2 (?)
        # Camera functionality would need to be implemented differently
        # self.video_client = VideoClient()  # Not available for G1
        # self.video_client.SetTimeout(3.0)
        # self.video_client.Init()

        # TODO: Add odometry

        # TODO: Add IMU

        self.num_joints = len(self.joint_names)

        msc = MotionSwitcherClient()
        msc.Init()
        msc.ReleaseMode()

        # Use G1 LocoClient instead of GO2 SportClient
        self.loco_client = LocoClient()
        self.loco_client.Init()
        # Note: G1 doesn't have StandDown, might need different initialization

        # self.odom_subscriber = ChannelSubscriber("rt/utlidar/robot_odom", Odometry_)
        # self.odom_subscriber.Init(self.odom_callback)

        self.joint_positions = [0.0] * self.num_joints
        self.joint_velocities = [0.0] * self.num_joints
        self.jont_accelerations = [0.0] * self.num_joints

        # Get subscribers
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        self.lidar_subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
        self.lidar_subscriber.Init(self.LidarMessageHandler, 10)

        self.lowstate_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowstate_publisher.Init()

    def LidarMessageHandler(self, msg: PointCloud2_):
        self.lidar_data = msg

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        print("LowStateMessageHandler")
        for i in range(self.num_joints):
            self.joint_positions[i] = msg.motor_state[i].q
            self.joint_velocities[i] = msg.motor_state[i].dq
            self.jont_accelerations[i] = msg.motor_state[i].ddq

        # print("JP:", self.joint_positions)

    def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
        # pack self.joint_positions into a dictionary
        joint_positions = {}
        for joint in joints:
            if joint in self.joint_names:
                index = self.joint_names.index(joint)
                joint_positions[joint] = self.joint_positions[index]
            else:
                log.error(f"Joint {joint} not found in joint names.")
        print("Joint Positions:", joint_positions)
        return joint_positions

    def pass_joint_velocities(self, joints: List[str]) -> Dict[str, float]:
        joint_velocities = {}
        for joint in joints:
            if joint in self.joint_names:
                index = self.joint_names.index(joint)
                joint_velocities[joint] = self.joint_velocities[index]
            else:
                log.error(f"Joint {joint} not found in joint names.")
        # print("Joint Positions:", joint_positions)
        return joint_velocities

    def pass_joint_acceleration(self, joints: List[str]) -> Dict[str, float]:
        jont_accelerations = {}
        for joint in joints:
            if joint in self.joint_names:
                index = self.joint_names.index(joint)
                jont_accelerations[joint] = self.jont_accelerations[index]
            else:
                log.error(f"Joint {joint} not found in joint names.")
        # print("Joint Positions:", joint_positions)
        return jont_accelerations

    def pass_joint_efforts(self, joints: [List[str]]) -> Dict[str, float]:
        raise NotImplementedError

    def pass_joint_group_control_cmd(
        self, control_mode: str, cmd: Dict[str, float], **kwargs
    ) -> None:
        low_cmd = unitree_hg_msg_dds__LowCmd_()
        print("Control Mode:", control_mode)
        print("Command:", cmd)
        crc = CRC()
        # extract all the cmd as a list
        cmd_list = list(cmd.values())

        if control_mode == "position":
            for i in range(self.num_joints):
                low_cmd.motor_cmd[i].mode = 0x01
                low_cmd.motor_cmd[i].q = cmd_list[i]
                low_cmd.motor_cmd[i].dq = 0.0
                low_cmd.motor_cmd[i].kp = 60.0
                low_cmd.motor_cmd[i].kd = 5.0
                low_cmd.motor_cmd[i].tau = 0.0
        # velocity control

        # torque control

        # send the command to the robot dog
        low_cmd.crc = crc.Crc(low_cmd)
        self.lowstate_publisher.Write(low_cmd)

    def pass_lidar_data(self):
        return self.lidar_data

    def pass_camera_image(self):
        # G1 doesn't have the same video client as GO2
        # Camera functionality needs to be implemented differently for G1
        log.warning("Camera functionality not implemented for G1 yet")
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def check_torque_status(self, joints: List[str]) -> Dict[str, bool]:
        raise NotImplementedError

    def shutdown_driver(self):
        # G1 shutdown logic - might need different approach than GO2
        if hasattr(self, 'loco_client'):
            # Add proper G1 shutdown commands here
            pass
