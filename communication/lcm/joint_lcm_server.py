import os
import select
import sys
import traceback

import click
import lcm
import numpy as np

# Path setup
file_path = os.path.abspath(__file__)
LCM_DIR = os.path.dirname(file_path)
COMM_DIR = os.path.dirname(LCM_DIR)
PYTHON_DIR = os.path.dirname(COMM_DIR)
sys.path.append(PYTHON_DIR)
sys.path.append(LCM_DIR)

import arx5_interface as arx5
from msg.arx5_joint_command_t import arx5_joint_command_t
from msg.arx5_response_t import arx5_response_t
from msg.arx5_state_t import arx5_state_t


def echo_exception():
    exc_type, exc_value, exc_traceback = sys.exc_info()
    tb_lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
    return "".join(tb_lines)


class Arx5JointLcmServer:
    def __init__(
        self,
        model: str,
        interface: str,
        lcm_address: str = "239.255.76.67",
        lcm_port: int = 7667,
        lcm_ttl: int = 1,
        no_cmd_timeout: float = 600.0,
    ):
        self.model = model
        self.interface = interface
        self.arx5_joint_controller = arx5.Arx5JointController(model, interface)
        print(f"Arx5JointLcmServer is initialized with {model} on {interface}. Typed messages enabled.")

        lcm_url = f"udpm://{lcm_address}:{lcm_port}?ttl={lcm_ttl}"
        print(f"LCM URL: {lcm_url}")
        self.lc = lcm.LCM(lcm_url)

        self.request_channel = "ARX5_JOINT_REQUEST"
        self.response_channel = "ARX5_JOINT_RESPONSE"
        self.lc.subscribe(self.request_channel, self._handler)

        self.no_cmd_timeout = no_cmd_timeout
        self.has_reset_to_home = False

    def _ensure_controller(self):
        if self.arx5_joint_controller is None:
            print("Reestablishing joint controller")
            self.arx5_joint_controller = arx5.Arx5JointController(self.model, self.interface)
            self.has_reset_to_home = False

    def _empty_state(self) -> arx5_state_t:
        msg = arx5_state_t()
        msg.num_joints = 0
        msg.joint_pos = []
        msg.joint_vel = []
        msg.joint_torque = []
        msg.ee_pose = [0.0] * 6
        return msg

    def _pack_state(self) -> arx5_state_t:
        joint_state = self.arx5_joint_controller.get_joint_state()
        eef_state = self.arx5_joint_controller.get_eef_state()

        msg = arx5_state_t()
        msg.timestamp = joint_state.timestamp
        msg.ee_pose = eef_state.pose_6d().copy()
        msg.num_joints = len(joint_state.pos())
        msg.joint_pos = joint_state.pos().copy()
        msg.joint_vel = joint_state.vel().copy()
        msg.joint_torque = joint_state.torque().copy()
        msg.gripper_pos = joint_state.gripper_pos
        msg.gripper_vel = joint_state.gripper_vel
        msg.gripper_torque = joint_state.gripper_torque
        return msg

    def _make_response(self, resp_type: int) -> arx5_response_t:
        resp = arx5_response_t()
        resp.resp_type = resp_type
        resp.state = self._empty_state()
        resp.error_msg = ""
        return resp

    def _send_error(self, message: str):
        resp = self._make_response(arx5_response_t.TYPE_ERROR)
        resp.error_msg = message
        self.lc.publish(self.response_channel, resp.encode())

    def _validate_joint_command(self, cmd_msg: arx5_joint_command_t):
        expected_dof = self.arx5_joint_controller.get_robot_config().joint_dof
        if cmd_msg.num_joints != expected_dof:
            raise ValueError(f"Expected {expected_dof} joints, got {cmd_msg.num_joints}.")
        if len(cmd_msg.joint_pos) != cmd_msg.num_joints:
            raise ValueError(f"joint_pos length {len(cmd_msg.joint_pos)} does not match num_joints {cmd_msg.num_joints}.")

    def _build_joint_state(self, cmd_msg: arx5_joint_command_t):
        joint_cmd = arx5.JointState(cmd_msg.num_joints)
        joint_cmd.pos()[:] = np.array(cmd_msg.joint_pos, dtype=np.float64)

        if np.isnan(cmd_msg.gripper_pos):
            joint_cmd.gripper_pos = self.arx5_joint_controller.get_joint_state().gripper_pos
        else:
            joint_cmd.gripper_pos = cmd_msg.gripper_pos

        if cmd_msg.preview_time > 0.0:
            joint_cmd.timestamp = self.arx5_joint_controller.get_timestamp() + cmd_msg.preview_time
        return joint_cmd

    def _handler(self, channel, data):
        try:
            self._ensure_controller()
            cmd_msg = arx5_joint_command_t.decode(data)

            if cmd_msg.cmd_type == arx5_joint_command_t.CMD_GET_STATE:
                resp = self._make_response(arx5_response_t.TYPE_STATE)
                resp.state = self._pack_state()
                self.lc.publish(self.response_channel, resp.encode())

            elif cmd_msg.cmd_type == arx5_joint_command_t.CMD_SET_JOINT_CMD:
                if not self.has_reset_to_home:
                    self._send_error("Error: Cannot set joint command before RESET_TO_HOME. Please check the input.")
                    return
                self._validate_joint_command(cmd_msg)
                self.arx5_joint_controller.set_joint_cmd(self._build_joint_state(cmd_msg))
                resp = self._make_response(arx5_response_t.TYPE_STATE)
                resp.state = self._pack_state()
                self.lc.publish(self.response_channel, resp.encode())

            elif cmd_msg.cmd_type == arx5_joint_command_t.CMD_RESET_TO_HOME:
                print("Received RESET_TO_HOME message")
                self.has_reset_to_home = False
                self.arx5_joint_controller.reset_to_home()
                self.has_reset_to_home = True
                resp = self._make_response(arx5_response_t.TYPE_OK)
                self.lc.publish(self.response_channel, resp.encode())

            elif cmd_msg.cmd_type == arx5_joint_command_t.CMD_SET_TO_DAMPING:
                print("Received SET_TO_DAMPING message")
                self.has_reset_to_home = False
                self.arx5_joint_controller.set_to_damping()
                resp = self._make_response(arx5_response_t.TYPE_OK)
                self.lc.publish(self.response_channel, resp.encode())

            else:
                self._send_error(f"Unknown message type: {cmd_msg.cmd_type}")

        except Exception:
            exception_str = echo_exception()
            print(f"Error: {exception_str}")
            self._send_error(f"ERROR: {exception_str}")

    def run(self):
        print("Arx5JointLcmServer is running.")
        try:
            while True:
                rfds, _, _ = select.select([self.lc.fileno()], [], [], self.no_cmd_timeout)
                if rfds:
                    self.lc.handle()
                else:
                    if self.arx5_joint_controller is not None:
                        print(f"Timeout: No command received for {self.no_cmd_timeout} sec. ARX5 arm is reset to home position.")
                        self.has_reset_to_home = False
                        self.arx5_joint_controller.reset_to_home()
                        self.arx5_joint_controller.set_to_damping()
                        self.arx5_joint_controller = None
                        print("Joint controller is reset to None")
        except KeyboardInterrupt:
            print("Arx5JointLcmServer stopped by KeyboardInterrupt")
        except Exception:
            print(echo_exception())

    def __del__(self):
        print("Arx5JointLcmServer is terminated")


@click.command()
@click.argument("model")
@click.argument("interface")
@click.option("--address", default="239.255.76.67", help="LCM multicast address")
@click.option("--port", default=7667, help="LCM multicast port")
@click.option("--ttl", default=1, help="LCM multicast TTL")
def main(model: str, interface: str, address: str, port: int, ttl: int):
    server = Arx5JointLcmServer(model=model, interface=interface, lcm_address=address, lcm_port=port, lcm_ttl=ttl)
    server.run()


if __name__ == "__main__":
    main()
