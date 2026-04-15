import sys
import os
import time
import select
import numpy as np
import numpy.typing as npt
import click
import lcm
from typing import Any, cast
import traceback

# Path setup
file_path = os.path.abspath(__file__)
LCM_DIR = os.path.dirname(file_path)
COMM_DIR = os.path.dirname(LCM_DIR)
PYTHON_DIR = os.path.dirname(COMM_DIR)
sys.path.append(PYTHON_DIR)
sys.path.append(LCM_DIR)

import arx5_interface as arx5
from msg.arx5_command_t import arx5_command_t
from msg.arx5_response_t import arx5_response_t
from msg.arx5_state_t import arx5_state_t
from msg.arx5_gain_t import arx5_gain_t


def echo_exception():
    exc_type, exc_value, exc_traceback = sys.exc_info()
    tb_lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
    return "".join(tb_lines)


class Arx5LcmServer:
    def __init__(
        self, model: str, interface: str, lcm_address: str = "239.255.76.67", lcm_port: int = 7667, lcm_ttl: int = 1, no_cmd_timeout: float = 600.0
    ):
        self.model = model
        self.interface = interface
        self.arx5_cartesian_controller = arx5.Arx5CartesianController(model, interface)
        print(f"Arx5LcmServer is initialized with {model} on {interface}. Typed messages enabled.")

        lcm_url = f"udpm://{lcm_address}:{lcm_port}?ttl={lcm_ttl}"
        print(f"LCM URL: {lcm_url}")
        self.lc = lcm.LCM(lcm_url)

        self.request_channel = "ARX5_REQUEST"
        self.response_channel = "ARX5_RESPONSE"
        self.lc.subscribe(self.request_channel, self._handler)

        self.no_cmd_timeout = no_cmd_timeout
        self.is_reset_to_home = False
        self.last_eef_cmd: npt.NDArray[np.float64] | None = None

    def _pack_state(self, eef_state, low_state) -> arx5_state_t:
        msg = arx5_state_t()
        msg.timestamp = eef_state.timestamp
        msg.ee_pose = eef_state.pose_6d().copy()
        msg.num_joints = len(low_state.pos())
        msg.joint_pos = low_state.pos().copy()
        msg.joint_vel = low_state.vel().copy()
        msg.joint_torque = low_state.torque().copy()
        msg.gripper_pos = low_state.gripper_pos
        msg.gripper_vel = low_state.gripper_vel
        msg.gripper_torque = low_state.gripper_torque
        return msg

    def _pack_gain(self, gain) -> arx5_gain_t:
        msg = arx5_gain_t()
        msg.num_joints = len(gain.kp())
        msg.kp = gain.kp().copy()
        msg.kd = gain.kd().copy()
        msg.gripper_kp = gain.gripper_kp
        msg.gripper_kd = gain.gripper_kd
        return msg

    def _send_error(self, message: str):
        resp = arx5_response_t()
        resp.resp_type = arx5_response_t.TYPE_ERROR
        resp.error_msg = message
        # Fill dummy structs to satisfy LCM non-null requirement if needed
        resp.state = arx5_state_t()
        resp.state.num_joints = 0
        resp.state.joint_pos = []
        resp.state.joint_vel = []
        resp.state.joint_torque = []
        resp.state.ee_pose = [0.0] * 6

        resp.gain = arx5_gain_t()
        resp.gain.num_joints = 0
        resp.gain.kp = []
        resp.gain.kd = []

        self.lc.publish(self.response_channel, resp.encode())

    def _handler(self, channel, data):
        try:
            if self.arx5_cartesian_controller is None:
                print(f"Reestablishing high level controller")
                self.arx5_cartesian_controller = arx5.Arx5CartesianController(self.model, self.interface)

            cmd_msg = arx5_command_t.decode(data)

            resp = arx5_response_t()
            # Initialize empty structs for safety
            resp.state = arx5_state_t()
            resp.state.num_joints = 0
            resp.state.joint_pos = []
            resp.state.joint_vel = []
            resp.state.joint_torque = []
            resp.state.ee_pose = [0.0] * 6
            resp.gain = arx5_gain_t()
            resp.gain.num_joints = 0
            resp.gain.kp = []
            resp.gain.kd = []
            resp.error_msg = ""

            if cmd_msg.cmd_type == arx5_command_t.CMD_GET_STATE:
                eef_state = self.arx5_cartesian_controller.get_eef_state()
                low_state = self.arx5_cartesian_controller.get_joint_state()
                resp.resp_type = arx5_response_t.TYPE_STATE
                resp.state = self._pack_state(eef_state, low_state)
                self.lc.publish(self.response_channel, resp.encode())

            elif cmd_msg.cmd_type == arx5_command_t.CMD_SET_EE_POSE:
                if self.last_eef_cmd is None:
                    print("Error: Cannot set EE pose before RESET_TO_HOME. Please check the input.")
                    self._send_error("Error: Cannot set EE pose before RESET_TO_HOME. Please check the input.")
                    return

                target_ee_pose = np.array(cmd_msg.ee_pose)
                self.last_eef_cmd = target_ee_pose.copy()

                if np.isnan(cmd_msg.gripper_pos):
                    target_gripper_pos = self.arx5_cartesian_controller.get_eef_state().gripper_pos
                else:
                    target_gripper_pos = cmd_msg.gripper_pos

                eef_cmd = arx5.EEFState(target_ee_pose, target_gripper_pos)
                if cmd_msg.preview_time > 0.0:
                    current_time = self.arx5_cartesian_controller.get_timestamp()
                    eef_cmd.timestamp = current_time + cmd_msg.preview_time
                self.arx5_cartesian_controller.set_eef_cmd(eef_cmd)

                # Reply with STATE
                eef_state = self.arx5_cartesian_controller.get_eef_state()
                low_state = self.arx5_cartesian_controller.get_joint_state()
                resp.resp_type = arx5_response_t.TYPE_STATE
                resp.state = self._pack_state(eef_state, low_state)
                self.lc.publish(self.response_channel, resp.encode())

                self.is_reset_to_home = False

            elif cmd_msg.cmd_type == arx5_command_t.CMD_RESET_TO_HOME:
                print(f"Received RESET_TO_HOME message")
                self.arx5_cartesian_controller.reset_to_home()
                resp.resp_type = arx5_response_t.TYPE_OK
                self.lc.publish(self.response_channel, resp.encode())

                self.last_eef_cmd = self.arx5_cartesian_controller.get_eef_cmd().pose_6d().copy()
                self.is_reset_to_home = True

            elif cmd_msg.cmd_type == arx5_command_t.CMD_SET_TO_DAMPING:
                print(f"Received SET_TO_DAMPING message")
                self.arx5_cartesian_controller.set_to_damping()
                resp.resp_type = arx5_response_t.TYPE_OK
                self.lc.publish(self.response_channel, resp.encode())
                self.is_reset_to_home = False

            elif cmd_msg.cmd_type == arx5_command_t.CMD_GET_GAIN:
                print(f"Received GET_GAIN message")
                gain = self.arx5_cartesian_controller.get_gain()
                resp.resp_type = arx5_response_t.TYPE_GAIN
                resp.gain = self._pack_gain(gain)
                self.lc.publish(self.response_channel, resp.encode())

            elif cmd_msg.cmd_type == arx5_command_t.CMD_SET_GAIN:
                print(f"Received SET_GAIN message")
                # Unpack gain from msg
                g = cmd_msg.gain
                kp = np.array(g.kp)
                kd = np.array(g.kd)
                self.arx5_cartesian_controller.set_gain(arx5.Gain(kp, kd, g.gripper_kp, g.gripper_kd))
                resp.resp_type = arx5_response_t.TYPE_OK
                self.lc.publish(self.response_channel, resp.encode())

            else:
                self._send_error(f"Unknown message type: {cmd_msg.cmd_type}")

        except Exception as e:
            exception_str = echo_exception()
            print(f"Error: {exception_str}")
            self._send_error(f"ERROR: {exception_str}")

    def run(self):
        print(f"Arx5LcmServer is running.")
        try:
            while True:
                rfds, _, _ = select.select([self.lc.fileno()], [], [], self.no_cmd_timeout)
                if rfds:
                    self.lc.handle()
                else:
                    if self.arx5_cartesian_controller is not None:
                        print(f"Timeout: No command received for {self.no_cmd_timeout} sec. ARX5 arm is reset to home position.")
                        self.arx5_cartesian_controller.reset_to_home()
                        self.arx5_cartesian_controller.set_to_damping()
                        self.last_eef_cmd = None
                        self.arx5_cartesian_controller = None
                        print("Controller is reset to None")
        except KeyboardInterrupt:
            print("Arx5LcmServer stopped by KeyboardInterrupt")
        except Exception:
            print(echo_exception())

    def __del__(self):
        print("Arx5LcmServer is terminated")


@click.command()
@click.argument("model")
@click.argument("interface")
@click.option("--address", default="239.255.76.67", help="LCM multicast address")
@click.option("--port", default=7667, help="LCM multicast port")
@click.option("--ttl", default=1, help="LCM multicast TTL")
def main(model: str, interface: str, address: str, port: int, ttl: int):
    server = Arx5LcmServer(model=model, interface=interface, lcm_address=address, lcm_port=port, lcm_ttl=ttl)
    server.run()


if __name__ == "__main__":
    main()
