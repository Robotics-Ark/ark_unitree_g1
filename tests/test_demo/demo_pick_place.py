import time
import numpy as np
from typing import Optional

import os
import sys
from robot_arm_ik import G1_29_ArmIK
import pinocchio as pin
from ark.client.comm_infrastructure.instance_node import InstanceNode
from arktypes import joint_group_command_t, joint_state_t
from arktypes.utils import pack, unpack


SIM = False  # set True to target sim topics


class AllGroupArmsHandsDemo(InstanceNode):
    def __init__(self) -> None:
        super().__init__("G1AllGroupArmsHandsDemo")

        ns = "unitree_g1_sim" if SIM else "unitree_g1"
        self.pub = self.create_publisher(f"{ns}/joint_group_command", joint_group_command_t)
        # Subscribe to joint states via callback to maintain the latest vector
        self.create_subscriber(f"{ns}/joint_states", joint_state_t, self._state_callback)

        self.latest_positions: Optional[np.ndarray] = None
        self.num_joints = 43

    def _state_callback(self, t, channel_name, msg):
        try:
            js = unpack.joint_state(msg)
            self.latest_positions = np.array(js.position, dtype=float)
        except Exception:
            pass

    def get_current_positions(self, timeout_s: float = 2.0) -> np.ndarray:
        start = time.time()
        while self.latest_positions is None and (time.time() - start) < timeout_s:
            time.sleep(0.01)

        if self.latest_positions is None:
            # Fallback to zeros, but keep size consistent
            self.latest_positions = np.zeros(self.num_joints, dtype=float)
        return self.latest_positions.copy()

    def send_all_command(self, full_positions: np.ndarray):
        assert full_positions.shape[0] == self.num_joints
        self.pub.publish(pack.joint_group_command(full_positions.tolist(), "all"))

    def send_arms_q_tau(self, q14: np.ndarray, tau14: np.ndarray):
        assert q14.shape[0] == 14 and tau14.shape[0] == 14
        payload = np.concatenate([q14, tau14]).tolist()
        self.pub.publish(pack.joint_group_command(payload, "arms_q_tau"))

    def send_right_hand(self, hand7: np.ndarray):
        assert hand7.shape[0] == 7
        self.pub.publish(pack.joint_group_command(hand7.tolist(), "right_hand"))

    def run_demo(self):
        # Fetch current to preserve legs/waist and any unmodified joints
        current = self.get_current_positions()

        # Indices per `unitree_g1/unitree_g1.py` joint order
        LEFT_ARM = slice(15, 22)     # 7 joints
        RIGHT_ARM = slice(22, 29)    # 7 joints
        LEFT_HAND = slice(29, 36)    # 7 joints
        RIGHT_HAND = slice(36, 43)   # 7 joints

        def hold(target: np.ndarray, seconds: float, rate_hz: float = 50.0):
            period = 1.0 / rate_hz
            until = time.time() + seconds
            while time.time() < until:
                self.send_all_command(target)
                time.sleep(period)

        print("Starting IK-driven arms sequence demo")

        # Initialize IK with working directory adjusted so URDF paths resolve
        base_dir = os.path.dirname(__file__)
        prev_cwd = os.getcwd()
        try:
            os.chdir(base_dir)
            ik = G1_29_ArmIK(Unit_Test=False, Visualization=False)
        finally:
            os.chdir(prev_cwd)

        # Helper to build SE3 from position with identity orientation
        def se3(px, py, pz):
            return pin.SE3(pin.Quaternion(1.0, 0.0, 0.0, 0.0), np.array([px, py, pz]))

        # Waypoints for left and right wrists
        waypoints = [
            (se3(0.30, +0.25, 0.10), se3(0.30, -0.25, 0.10)),
            (se3(0.35, +0.20, 0.15), se3(0.35, -0.20, 0.15)),
            (se3(0.32, +0.10, 0.20), se3(0.32, -0.10, 0.20)),
            (se3(0.28, +0.18, 0.12), se3(0.28, -0.18, 0.12)),
        ]

        # Current arms as IK seed (14 values)
        current_lr = np.concatenate([current[LEFT_ARM], current[RIGHT_ARM]])
        target = current.copy()

        for L_tf, R_tf in waypoints:
            sol_q, sol_tauff = ik.solve_ik(L_tf.homogeneous, R_tf.homogeneous, current_lr)

            # Optional debug: current seed and IK solution (disabled)

            # Interpolate between current and next for smoother motion
            n_interps = 20
            rate_hz = 10.0
            period = 1.0 / rate_hz
            for alpha in np.linspace(0.0, 1.0, n_interps + 1, endpoint=True):
                q_interp = (1.0 - alpha) * current_lr + alpha * sol_q

                # Stream arms via q+tau interface for gravity compensation
                self.send_arms_q_tau(q_interp, sol_tauff)
                time.sleep(period)

            # Update seed for next waypoint
            current_lr = sol_q.copy()

        print("IK-driven arms sequence complete")

    def run_pick_and_place_demo(self):
        # Fetch current to preserve legs/waist and any unmodified joints
        current = self.get_current_positions()

        # Indices per `unitree_g1/unitree_g1.py` joint order
        LEFT_ARM = slice(15, 22)     # 7 joints
        RIGHT_ARM = slice(22, 29)    # 7 joints
        LEFT_HAND = slice(29, 36)    # 7 joints
        RIGHT_HAND = slice(36, 43)   # 7 joints

        print("Starting Right-Arm Pick-and-Place demo (grasp from above)")

        # Initialize IK
        base_dir = os.path.dirname(__file__)
        prev_cwd = os.getcwd()
        try:
            os.chdir(base_dir)
            ik = G1_29_ArmIK(Unit_Test=False, Visualization=True)
        finally:
            os.chdir(prev_cwd)

        # Helper: SE3 from position and quaternion
        def se3_from_pos_quat(px, py, pz, quat: pin.Quaternion):
            return pin.SE3(quat, np.array([px, py, pz]))

        # Helper: quaternion for a tilt about X by angle_deg (degrees)
        def quat_about_x_deg(angle_deg: float) -> pin.Quaternion:
            angle_rad = np.deg2rad(angle_deg)
            ch = np.cos(0.5 * angle_rad)
            sh = np.sin(0.5 * angle_rad)
            return pin.Quaternion(ch, sh, 0.0, 0.0)

        def quat_side_then_front(side_deg=-90.0, front_deg=10.0) -> pin.Quaternion:
            rx = np.deg2rad(side_deg)
            ry = np.deg2rad(front_deg)
            cx, sx = np.cos(0.5*rx), np.sin(0.5*rx)
            cy, sy = np.cos(0.5*ry), np.sin(0.5*ry)
            # q = q_y ⊗ q_x
            w = cy*cx
            x = cy*sx
            y = sy*cx
            z = -sy*sx
            return pin.Quaternion(w, x, y, z)

        # Grasp-from-above orientation: right-hand tilt by specified angle (deg) about X
        right_tilt_deg = -90.0
        grasp_above_q = quat_side_then_front(side_deg=right_tilt_deg, front_deg=20.0)
        default_quat = pin.Quaternion(1.0, 0.0, 0.0, 0.0)
        # Positions (meters)
        x_pick = 0.35
        y_pick = -0.15
        z_above = 0.20
        z_grasp = 0.02
        x_place = x_pick
        y_place = -0.05
        rate_hz = 20.0
        # Keep left arm fixed to its current configuration
        left_fixed = current[LEFT_ARM].copy()
        right_curr = current[RIGHT_ARM].copy()
        target_full = current.copy()

        def move_right_to(px, py, pz, quat: pin.Quaternion, n_interps: int = 20, rate_hz: float = 10.0):
            nonlocal right_curr, target_full
            # Solve IK with seed combining fixed-left and current-right
            seed_lr = np.concatenate([left_fixed, right_curr])
            L_tf = se3_from_pos_quat(0.30, +0.25, z_above, pin.Quaternion(1.0, 0.0, 0.0, 0.0))
            R_tf = se3_from_pos_quat(px, py, pz, quat)
            sol_q, sol_tauff = ik.solve_ik(L_tf.homogeneous, R_tf.homogeneous, seed_lr)

            next_right = sol_q[7:14]

            # Optional debug for seed/target (disabled)

            # Interpolate only right arm; keep left fixed
            period = 1.0 / rate_hz
            for alpha in np.linspace(0.0, 1.0, n_interps + 1, endpoint=True):
                q_r = (1.0 - alpha) * right_curr + alpha * next_right
                # Build combined 14-dof vector with left fixed + right interpolated
                q14 = np.concatenate([left_fixed, q_r])
                # Use the latest IK tau for gravity comp while interpolating
                self.send_arms_q_tau(q14, sol_tauff)
                time.sleep(period)

            right_curr = next_right.copy()

        def set_right_hand(values: np.ndarray, seconds: float = 1.0, rate_hz: float = 10.0):
            nonlocal target_full
            period = 1.0 / rate_hz
            steps = max(1, int(seconds * rate_hz))
            start = target_full[RIGHT_HAND].copy()
            for alpha in np.linspace(0.0, 1.0, steps, endpoint=True):
                target_full[RIGHT_HAND] = (1.0 - alpha) * start + alpha * values
                # Send only right hand group to avoid overriding arm torques
                self.send_right_hand(target_full[RIGHT_HAND])
                time.sleep(period)

        # Hand presets
        right_hand_open = np.zeros(7)
        right_hand_close = np.array([0.0, -0.3, -1, 0.8, 1., 0.8, 1])

        
        # 1) Approach above pick
        move_right_to(x_pick, y_pick, z_above, grasp_above_q, rate_hz=rate_hz)
        print("1) Approach above pick")
        # 2) Descend to grasp height
        move_right_to(x_pick, y_pick, z_grasp, grasp_above_q, rate_hz=rate_hz)
        print("2) Descend to grasp height")
        # 3) Close hand
        set_right_hand(right_hand_close, seconds=1.5, rate_hz=10.0)
        print("3) Close hand")
        # 4) Lift
        move_right_to(x_pick, y_pick, z_above, grasp_above_q, rate_hz=rate_hz)
        print("4) Lift")
        # 5) Move to place location (left)
        move_right_to(x_place, y_place, z_above, grasp_above_q, rate_hz=rate_hz)
        print("5) Move to place location (left)")
        # 6) Lower to place height
        move_right_to(x_place, y_place, z_grasp, grasp_above_q, rate_hz=rate_hz)
        move_right_to(x_place, y_place, z_grasp, grasp_above_q, rate_hz=rate_hz)
        print("6) Lower to place height")
        # 7) Open hand to release
        set_right_hand(right_hand_open, seconds=1.0, rate_hz=10.0)
        print("7) Open hand to release")
        # 8) Lift back up
        move_right_to(x_place, y_place, z_above, grasp_above_q, rate_hz=rate_hz)
        print("8) Lift back up")

        # Move back to initial position
        move_right_to(0.25, -0.25, 0.15, default_quat, rate_hz=rate_hz)
        move_right_to(0.25, -0.25, 0.15, default_quat, rate_hz=rate_hz)
        move_right_to(0.25, -0.25, 0.15, default_quat, rate_hz=rate_hz)
        print("9) Move back to initial position")

        print("Pick-and-Place demo complete")


if __name__ == "__main__":
    demo = AllGroupArmsHandsDemo()
    # Give time for comms to settle
    time.sleep(2.0)
    demo.run_pick_and_place_demo()


