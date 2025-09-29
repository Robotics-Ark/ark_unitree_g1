from typing import Any, Dict, Optional
import sys
import signal
from dataclasses import dataclass
from enum import Enum
import numpy as np
from ark.client.comm_infrastructure.base_node import main
from ark.system.component.robot import Robot
from ark.system.driver.robot_driver import RobotDriver
from arktypes import (
    joint_state_t,
    force_t,
    imu_t,
    joint_group_command_t,
    string_t,
    pose_2d_t,
    velocity_2d_t,
    float_t,
    image_t,
    point_cloud2_t,
    point_field_t,
)
from arktypes.utils import pack, unpack


from ark.system.pybullet.pybullet_robot_driver import BulletRobotDriver
from ark.tools.log import log

SIM = True
@dataclass
class Drivers(Enum):
    PYBULLET_DRIVER = BulletRobotDriver

# Attempt to import UnitreeG1Driver for real robot control (optional)
try:
    from unitree_g1_driver import UnitreeG1Driver  # type: ignore
    print("UnitreeG1Driver found")
except Exception as error:
    print(error)
    log.warning("libraries for the UnitreeG1Driver not found")

class UnitreeG1(Robot):
    def __init__(
        self,
        name: str,
        global_config: Dict[str, Any] = None,
        driver: RobotDriver = None,
    ) -> None:
        super().__init__(
            name=name,
            global_config=global_config,
            driver=driver,
        )
        # Override joint_groups with proper configuration for G1
        self._configure_joint_groups()
        self.joint_group_command = None
        
        self.state = None
        control_mode = driver.config.get("control", "joint_space")
        # Check if running in simulation mode
        self.sim = "sim" in name.lower() or driver.config.get("simulation", False)
        print(f"Simulation mode: {self.sim}")
        # Create names
        self.joint_pub_name = self.name + "/joint_states"
        self.camera_pub_name = self.name + "/camera"
        self.lidar_pub_name = self.name + "/lidar"

        if control_mode == "joint_space":
            self.subscriber_name = self.name + "/joint_group_command"
            self.subscriber_type = joint_group_command_t
            subscriber_callback = self._joint_group_command_callback
            self.create_subscriber(
                self.subscriber_name, self.subscriber_type, subscriber_callback
            )
        else:
            log.error("Control mode not supported: " + control_mode)
            self.kill_node()

        if self.sim == True:
            self.joint_pub_name = self.joint_pub_name + "/sim"
            self.camera_pub_name = self.camera_pub_name + "/sim"
            self.lidar_pub_name = self.lidar_pub_name + "/sim"

        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.joint_positions = [0] * 43

        self.joint_publisher = self.create_publisher(self.joint_pub_name, joint_state_t)
        self.create_stepper(240, self.get_joint_state)

        if self.sim:
            log.warn(
                "UnitreeG1 in Simulation does not support Lidar or Camera publishing"
            )

        if self.robot_config["camera"]:
            self.camera_publisher = self.create_publisher(self.camera_pub_name, image_t)
            self.create_stepper(60, self.get_camera_data)

        if self.robot_config["lidar"]:
            self.lidar_publisher = self.create_publisher(
                self.lidar_pub_name, point_cloud2_t
            )
            self.create_stepper(240, self.get_lidar_data)

    def _configure_joint_groups(self):
        """Configure joint groups for G1 robot with proper actuated joints."""
        # G1 joint names (43 total, excluding pelvis_contour_joint)
        g1_joint_names = [
            "left_hip_pitch_joint",
            "left_hip_roll_joint", 
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
            "left_hand_thumb_0_joint",
            "left_hand_thumb_1_joint",
            "left_hand_thumb_2_joint",
            "left_hand_middle_0_joint",
            "left_hand_middle_1_joint",
            "left_hand_index_0_joint",
            "left_hand_index_1_joint",
            "right_hand_thumb_0_joint",
            "right_hand_thumb_1_joint",
            "right_hand_thumb_2_joint",
            "right_hand_middle_0_joint",
            "right_hand_middle_1_joint",
            "right_hand_index_0_joint",
            "right_hand_index_1_joint"
        ]
        print("Number of joints:", len(g1_joint_names))
        
        # Fixed joints that are not actuated
        
        # Create joint groups configuration
        joints = {}
        actuated_joints = {}
        
        for i, joint_name in enumerate(g1_joint_names):
            joints[joint_name] = i
            actuated_joints[joint_name] = i
        
        # Configure the "all" joint group
        self.joint_groups["all"] = {
            "control_mode": "position",
            "control_type": "position",  # You might need to import ControlType enum
            "joints": joints,
            "actuated_joints": actuated_joints
        }
        
        # Configure left hand joint group
        left_hand_joints = {}
        left_hand_actuated = {}
        for i, joint_name in enumerate(g1_joint_names[29:36]):  # Left hand joints (indices 29-35)
            left_hand_joints[joint_name] = i
            left_hand_actuated[joint_name] = i + 29  # Actual index in full joint array
        
        self.joint_groups["left_hand"] = {
            "control_mode": "position",
            "control_type": "position",
            "joints": left_hand_joints,
            "actuated_joints": left_hand_actuated
        }
        
        # Configure right hand joint group
        right_hand_joints = {}
        right_hand_actuated = {}
        for i, joint_name in enumerate(g1_joint_names[36:43]):  # Right hand joints (indices 36-42)
            right_hand_joints[joint_name] = i
            right_hand_actuated[joint_name] = i + 36  # Actual index in full joint array
        
        self.joint_groups["right_hand"] = {
            "control_mode": "position",
            "control_type": "position",
            "joints": right_hand_joints,
            "actuated_joints": right_hand_actuated
        }

        # It's possible to configure other joint groups here, if needed
        
        print(f"Configured joint groups with {len(actuated_joints)} actuated joints")
        print(f"Available joint groups: {list(self.joint_groups.keys())}")

    def control_robot(self):
        #print(self.joint_groups)
        if self.joint_group_command:
            # Copy incoming command to local variables to avoid races with other threads
            incoming = self.joint_group_command
            group_name = incoming["name"]
            cmd_array = list(incoming["cmd"]) if incoming.get("cmd") is not None else []
            # Mark consumed early to prevent concurrent reads using a partially updated state
            self.joint_group_command = None

            cmd_dict = {}
            print(f"Processing joint group command for: {group_name}")
            print(f"Command length: {len(cmd_array)}")

            if group_name == "arms_q_tau":
                # Expect 28 values: [q14, tau14]; bypass mapping and send directly to driver
                if len(cmd_array) == 28 and hasattr(self._driver, 'arm_ctrl_q_tau'):
                    q14 = np.array(cmd_array[:14], dtype=float)
                    tau14 = np.array(cmd_array[14:], dtype=float)
                    try:
                        self._driver.arm_ctrl_q_tau(q14, tau14)
                    except Exception as e:
                        log.error(f"Error in arm_ctrl_q_tau: {e}")
                else:
                    log.error("arms_q_tau expects 28 values and driver support.")
            elif group_name in self.joint_groups:
                actuated = self.joint_groups[group_name]["actuated_joints"]
                for joint, goal in zip(list(actuated), cmd_array):
                    cmd_dict[joint] = goal
                self._joint_cmd_msg = None
                control_mode = self.joint_groups[group_name]["control_mode"]
                # Minimal command visibility
                print(f"Control mode: {control_mode}")
                self.control_joint_group(control_mode, cmd_dict)
            else:
                print(f"Unknown joint group: {group_name}")
                print(f"Available groups: {list(self.joint_groups.keys())}")

    def get_lidar_data(self):
        lidar_data = self._driver.pass_lidar_data()

        point_cloud = point_cloud2_t()

        point_cloud.height = lidar_data.height
        point_cloud.width = lidar_data.width

        point_cloud.num_fields = len(lidar_data.fields)

        point_field_array = []
        for field in lidar_data.fields:
            point_field = point_field_t()
            point_field.name = field.name
            point_field.offset = field.offset
            point_field.datatype = field.datatype
            point_field.count = field.count
            point_field_array.append(point_field)

        point_cloud.fields = point_field_array

        point_cloud.is_bigendian = lidar_data.is_bigendian
        point_cloud.point_step = lidar_data.point_step
        point_cloud.row_step = lidar_data.row_step

        point_cloud.num_bytes = len(lidar_data.data)
        point_cloud.data = lidar_data.data

        point_cloud.is_dense = lidar_data.is_dense

        self.lidar_publisher.publish(point_cloud)

    def get_camera_data(self):
        self.image = self._driver.pass_camera_image()
        try:
            image_msg = pack.image(self.image, "front camera")
            self.camera_publisher.publish(image_msg)
        except:
            log.error("Could not get camera Image")

    def get_joint_state(self):
        joint_positions = self.get_joint_positions()
        # print(joint_positions)
        joint_velocities = self._driver.pass_joint_velocities(self._all_actuated_joints)
        msg = joint_state_t()
        msg.n = len(joint_positions)
        msg.name = list(joint_positions.keys())
        msg.position = list(joint_positions.values())
        msg.velocity = list(joint_velocities.values())
        msg.effort = [0.0] * 43
        self.joint_publisher.publish(msg)

    def _joint_group_command_callback(self, t, channel_name, msg):
        cmd, name = unpack.joint_group_command(msg)
        print(f"Received joint group command: {name}, cmd length: {len(cmd)}")
        self.joint_group_command = {
            "cmd": cmd,
            "name": name,
        }

    def get_state(self):
        pass

    def pack_data(self, state):
        pass

    def step_component(self):
        # overrides the parent definition
        self.control_robot()


CONFIG_PATH = "unitree_g1.yaml" if not SIM else "unitree_g1_sim.yaml"

if __name__ == "__main__":
    from unitree_g1_driver import UnitreeG1Driver

    name = "unitree_g1"
    driver = UnitreeG1Driver(name, CONFIG_PATH)

    def _graceful_exit(signum, frame):
        try:
            print(f"\nSignal {signum} received: shutting down UnitreeG1Driver...")
            driver.shutdown_driver()
        except Exception as e:
            print(f"Error during shutdown_driver: {e}")
        finally:
            sys.exit(0)

    try:
        signal.signal(signal.SIGINT, _graceful_exit)
        signal.signal(signal.SIGTERM, _graceful_exit)
    except Exception:
        pass

    try:
        main(UnitreeG1, name, CONFIG_PATH, driver)
    except KeyboardInterrupt:
        _graceful_exit(signal.SIGINT, None)
    finally:
        try:
            driver.shutdown_driver()
        except Exception:
            pass
