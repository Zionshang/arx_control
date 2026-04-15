import os
import sys
import traceback
from typing import Any, Optional, TypedDict, cast

import lcm
import numpy as np
import numpy.typing as npt

# Ensure LCM-generated modules can resolve `import msg.*`
file_path = os.path.abspath(__file__)
LCM_DIR = os.path.dirname(file_path)
if LCM_DIR not in sys.path:
    sys.path.append(LCM_DIR)

from .msg.arx5_joint_command_t import arx5_joint_command_t
from .msg.arx5_response_t import arx5_response_t


def echo_exception():
    exc_type, exc_value, exc_traceback = sys.exc_info()
    tb_lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
    return "".join(tb_lines)


class StateDict(TypedDict):
    timestamp: float
    ee_pose: npt.NDArray[np.float64]
    joint_pos: npt.NDArray[np.float64]
    joint_vel: npt.NDArray[np.float64]
    joint_torque: npt.NDArray[np.float64]
    gripper_pos: float
    gripper_vel: float
    gripper_torque: float


class Arx5JointLcmClient:
    """LCM client for the ARX5 joint-space controller.

    Usage:
    - Create: client = Arx5JointLcmClient(address="239.255.76.67", port=7667, ttl=1)
    - State: state = client.get_state()
    - Move: client.set_joint_cmd(state["joint_pos"], gripper_pos=None, preview_time=0.1)
    - Safety: client.reset_to_home(); client.set_to_damping()
    """

    def __init__(self, url: str = "", address: str = "239.255.76.67", port: int = 7667, ttl: int = 1):
        if url:
            lcm_url = url
        else:
            lcm_url = f"udpm://{address}:{port}?ttl={ttl}"

        self.lc = lcm.LCM(lcm_url)
        print(f"Arx5JointLcmClient initialized with {lcm_url}")

        self.request_channel = "ARX5_JOINT_REQUEST"
        self.response_channel = "ARX5_JOINT_RESPONSE"
        self.lc.subscribe(self.response_channel, self._handler)
        self.latest_state: StateDict
        self.current_response: Optional[arx5_response_t] = None

        print(f"Arx5JointLcmClient initialized on channels {self.request_channel}/{self.response_channel}. Typed.")
        self.get_state()
        print("Initial state fetched")

    def _handler(self, channel, data):
        try:
            msg = arx5_response_t.decode(data)
            self.current_response = msg
        except Exception:
            print(echo_exception())

    def _convert_response_to_dict(self, resp: arx5_response_t, cmd: str) -> dict:
        if resp.resp_type == arx5_response_t.TYPE_ERROR:
            return {"cmd": cmd, "data": resp.error_msg}

        if resp.resp_type == arx5_response_t.TYPE_OK:
            return {"cmd": cmd, "data": "OK"}

        if resp.resp_type == arx5_response_t.TYPE_STATE:
            st = resp.state
            state_data: StateDict = {
                "timestamp": st.timestamp,
                "ee_pose": np.array(st.ee_pose),
                "joint_pos": np.array(st.joint_pos),
                "joint_vel": np.array(st.joint_vel),
                "joint_torque": np.array(st.joint_torque),
                "gripper_pos": st.gripper_pos,
                "gripper_vel": st.gripper_vel,
                "gripper_torque": st.gripper_torque,
            }
            return {"cmd": cmd, "data": state_data}

        return {"cmd": cmd, "data": "Unknown Response Type"}

    def _empty_command(self, cmd_type: int) -> arx5_joint_command_t:
        lcm_cmd = arx5_joint_command_t()
        lcm_cmd.cmd_type = cmd_type
        lcm_cmd.preview_time = 0.0
        lcm_cmd.num_joints = 0
        lcm_cmd.joint_pos = []
        lcm_cmd.gripper_pos = 0.0
        return lcm_cmd

    def _fill_joint_command(self, lcm_cmd: arx5_joint_command_t, data: dict[str, Any]):
        joint_pos = np.asarray(data["joint_pos"], dtype=np.float64)

        if joint_pos.ndim != 1:
            raise ValueError("joint_pos must be a 1-D array.")

        lcm_cmd.preview_time = float(data["preview_time"])
        lcm_cmd.num_joints = int(joint_pos.shape[0])
        lcm_cmd.joint_pos = list(joint_pos)

        if data["gripper_pos"] is None:
            lcm_cmd.gripper_pos = float("nan")
        else:
            lcm_cmd.gripper_pos = float(data["gripper_pos"])

    def send_recv(self, cmd_type: int, data: Optional[dict[str, Any]] = None) -> dict:
        try:
            self.current_response = None
            lcm_cmd = self._empty_command(cmd_type)

            cmd_str_map = {
                arx5_joint_command_t.CMD_GET_STATE: "GET_STATE",
                arx5_joint_command_t.CMD_SET_JOINT_CMD: "SET_JOINT_CMD",
                arx5_joint_command_t.CMD_RESET_TO_HOME: "RESET_TO_HOME",
                arx5_joint_command_t.CMD_SET_TO_DAMPING: "SET_TO_DAMPING",
            }
            cmd_str = cmd_str_map.get(cmd_type, "UNKNOWN")

            if cmd_type == arx5_joint_command_t.CMD_SET_JOINT_CMD:
                assert data is not None
                self._fill_joint_command(lcm_cmd, data)

            self.lc.publish(self.request_channel, lcm_cmd.encode())

            while self.current_response is None:
                self.lc.handle()

            return self._convert_response_to_dict(self.current_response, cmd_str)

        except KeyboardInterrupt:
            print("Arx5JointLcmClient: KeyboardInterrupt.")
            return {"cmd": "UNKNOWN", "data": "KeyboardInterrupt"}
        except Exception as e:
            print(f"Arx5JointLcmClient: Error {e}")
            print(echo_exception())
            return {"cmd": "UNKNOWN", "data": "LcmError"}

    def get_state(self) -> StateDict:
        """Return the latest robot state."""
        reply_msg = self.send_recv(arx5_joint_command_t.CMD_GET_STATE)
        assert reply_msg["cmd"] == "GET_STATE"

        if isinstance(reply_msg["data"], str):
            if hasattr(self, "latest_state"):
                return self.latest_state
            raise ValueError(f"Error: {reply_msg['data']}")
        if type(reply_msg["data"]) != dict:
            raise ValueError(f"Error: {reply_msg['data']}")

        state = cast(StateDict, reply_msg["data"])
        self.latest_state = state
        return state

    def set_joint_cmd(
        self,
        joint_pos: npt.NDArray[np.float64],
        gripper_pos: Optional[float] = None,
        preview_time: float = 0.0,
    ) -> StateDict:
        """Send one joint-space position command.

        `gripper_pos=None` keeps the current gripper position on the server.
        `preview_time=0.0` sends an immediate command; positive values schedule the command relative to server time.
        """
        joint_pos_arr = np.asarray(joint_pos, dtype=np.float64)

        reply_msg = self.send_recv(
            arx5_joint_command_t.CMD_SET_JOINT_CMD,
            {
                "joint_pos": joint_pos_arr,
                "gripper_pos": gripper_pos,
                "preview_time": preview_time,
            },
        )
        assert reply_msg["cmd"] == "SET_JOINT_CMD"

        if isinstance(reply_msg["data"], str):
            raise ValueError(f"Error: {reply_msg['data']}")

        if type(reply_msg["data"]) != dict:
            raise ValueError(f"Error: {reply_msg['data']}")

        state = cast(StateDict, reply_msg["data"])
        self.latest_state = state
        return state

    def reset_to_home(self):
        reply_msg = self.send_recv(arx5_joint_command_t.CMD_RESET_TO_HOME)
        assert reply_msg["cmd"] == "RESET_TO_HOME"
        if reply_msg["data"] != "OK":
            raise ValueError(f"Error: {reply_msg['data']}")
        self.get_state()

    def set_to_damping(self):
        reply_msg = self.send_recv(arx5_joint_command_t.CMD_SET_TO_DAMPING)
        assert reply_msg["cmd"] == "SET_TO_DAMPING"
        if reply_msg["data"] != "OK":
            raise ValueError(f"Error: {reply_msg['data']}")
        self.get_state()

    @property
    def timestamp(self):
        timestamp = self.latest_state["timestamp"]
        return cast(float, timestamp)

    @property
    def ee_pose(self):
        ee_pose = self.latest_state["ee_pose"]
        return cast(npt.NDArray[np.float64], ee_pose)

    @property
    def joint_pos(self):
        joint_pos = self.latest_state["joint_pos"]
        return cast(npt.NDArray[np.float64], joint_pos)

    @property
    def joint_vel(self):
        joint_vel = self.latest_state["joint_vel"]
        return cast(npt.NDArray[np.float64], joint_vel)

    @property
    def joint_torque(self):
        joint_torque = self.latest_state["joint_torque"]
        return cast(npt.NDArray[np.float64], joint_torque)

    @property
    def gripper_pos(self):
        gripper_pos = self.latest_state["gripper_pos"]
        return cast(float, gripper_pos)

    @property
    def gripper_vel(self):
        gripper_vel = self.latest_state["gripper_vel"]
        return cast(float, gripper_vel)

    @property
    def gripper_torque(self):
        gripper_torque = self.latest_state["gripper_torque"]
        return cast(float, gripper_torque)

    def __del__(self):
        print("Arx5JointLcmClient is closed")
