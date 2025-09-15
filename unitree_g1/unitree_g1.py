from typing import Any, Dict, Optional
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


@dataclass
class Drivers(Enum):
    PYBULLET_DRIVER = BulletRobotDriver
    try: 
        from unitree_g1_driver import UnitreeG1Driver
        DRIVER = UnitreeG1Driver
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
        self.state = None
        control_mode = driver.config.get("control", "joint_space")
        self.sim = True
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

        self.joint_group_command = None
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

    def control_robot(self):
        if self.joint_group_command:
            cmd_dict = {}
            group_name = self.joint_group_command["name"]
            for joint, goal in zip(
                list(
                    self.joint_groups[self.joint_group_command["name"]][
                        "actuated_joints"
                    ]
                ),
                self.joint_group_command["cmd"],
            ):
                cmd_dict[joint] = goal
            self._joint_cmd_msg = None
            control_mode = self.joint_groups[group_name]["control_mode"]
            self.control_joint_group(control_mode, cmd_dict)

        self.joint_group_command = None

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
        #print(joint_positions)
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
        pass

CONFIG_PATH = "unitree_g1.yaml"

if __name__ == "__main__":
    from unitree_g1_driver import UnitreeG1Driver

    name = "unitree_g1"
    driver = UnitreeG1Driver(name, CONFIG_PATH)
    main(UnitreeG1, name, CONFIG_PATH, driver)
