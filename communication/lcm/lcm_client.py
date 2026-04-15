from typing import Any, Optional, TypedDict, Union, cast
import os
import sys
import lcm
import numpy.typing as npt
import numpy as np
import traceback
import select

# Ensure LCM-generated modules can resolve `import msg.*`
file_path = os.path.abspath(__file__)
LCM_DIR = os.path.dirname(file_path)
if LCM_DIR not in sys.path:
    sys.path.append(LCM_DIR)

from .msg.arx5_command_t import arx5_command_t
from .msg.arx5_response_t import arx5_response_t
from .msg.arx5_state_t import arx5_state_t
from .msg.arx5_gain_t import arx5_gain_t


def echo_exception():
    exc_type, exc_value, exc_traceback = sys.exc_info()
    tb_lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
    return "".join(tb_lines)

class GainDict(TypedDict):
    kp: npt.NDArray[np.float64]
    kd: npt.NDArray[np.float64]
    gripper_kp: float
    gripper_kd: float


class StateDict(TypedDict):
    timestamp: float
    ee_pose: npt.NDArray[np.float64]
    joint_pos: npt.NDArray[np.float64]
    joint_vel: npt.NDArray[np.float64]
    joint_torque: npt.NDArray[np.float64]
    gripper_pos: float
    gripper_vel: float
    gripper_torque: float


class Arx5LcmClient:
    """LCM client for the ARX5 Cartesian controller.

    Usage:
    - Create: client = Arx5LcmClient(address="239.255.76.67", port=7667, ttl=1)
    - State: state = client.get_state()
    - Move: client.set_ee_pose(state["ee_pose"], gripper_pos=None, preview_time=0.1)
    - Gains: gain = client.get_gain(); client.set_gain(gain)
    - Safety: client.reset_to_home(); client.set_to_damping()
    """
    def __init__(self, url: str = "", address: str = "239.255.76.67", port: int = 7667, ttl: int = 1):
        if url:
            # Use user provided URL string directly
            lcm_url = url
        else:
            # Construct standard UDPM URL
            lcm_url = f"udpm://{address}:{port}?ttl={ttl}"

        self.lc = lcm.LCM(lcm_url)
        print(f"Arx5LcmClient initialized with {lcm_url}")

        self.request_channel = "ARX5_REQUEST"
        self.response_channel = "ARX5_RESPONSE"
        self.lc.subscribe(self.response_channel, self._handler)
        self.latest_state: StateDict
        self.current_response: Optional[arx5_response_t] = None

        print(f"Arx5LcmClient initialized on channels {self.request_channel}/{self.response_channel}. Typed.")
        self.get_state()
        print(f"Initial state fetched")

    def _handler(self, channel, data):
        try:
            msg = arx5_response_t.decode(data)
            self.current_response = msg
        except Exception:
            print(echo_exception())

    def _convert_response_to_dict(self, resp: arx5_response_t, cmd: str) -> dict:
        if resp.resp_type == arx5_response_t.TYPE_ERROR:
            # Mimic ZMQ error structure somewhat?
            # ZMQ returns { "cmd": cmd, "data": "Error string"}
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

        if resp.resp_type == arx5_response_t.TYPE_GAIN:
            g = resp.gain
            gain_data: GainDict = {
                "kp": np.array(g.kp),
                "kd": np.array(g.kd),
                "gripper_kp": g.gripper_kp,
                "gripper_kd": g.gripper_kd,
            }
            return {"cmd": cmd, "data": gain_data}

        return {"cmd": cmd, "data": "Unknown Response Type"}

    def send_recv(self, cmd_type: int, data: dict = {}) -> dict:
        try:
            self.current_response = None

            lcm_cmd = arx5_command_t()
            lcm_cmd.cmd_type = cmd_type

            # Fill default/dummy
            lcm_cmd.ee_pose = [0.0] * 6
            lcm_cmd.gripper_pos = 0.0
            lcm_cmd.preview_time = 0.0
            lcm_cmd.gain = arx5_gain_t()
            lcm_cmd.gain.num_joints = 0
            lcm_cmd.gain.kp = []
            lcm_cmd.gain.kd = []

            # Map command
            cmd_str_map = {
                arx5_command_t.CMD_GET_STATE: "GET_STATE",
                arx5_command_t.CMD_SET_EE_POSE: "SET_EE_POSE",
                arx5_command_t.CMD_RESET_TO_HOME: "RESET_TO_HOME",
                arx5_command_t.CMD_SET_TO_DAMPING: "SET_TO_DAMPING",
                arx5_command_t.CMD_GET_GAIN: "GET_GAIN",
                arx5_command_t.CMD_SET_GAIN: "SET_GAIN",
            }
            cmd_str = cmd_str_map.get(cmd_type, "UNKNOWN")

            if cmd_type == arx5_command_t.CMD_SET_EE_POSE:
                lcm_cmd.ee_pose = list(data["ee_pose"])
                if data["gripper_pos"] is None:
                    lcm_cmd.gripper_pos = float("nan")
                else:
                    lcm_cmd.gripper_pos = float(data["gripper_pos"])
                if data.get("preview_time") is not None:
                    lcm_cmd.preview_time = float(data["preview_time"])

            elif cmd_type == arx5_command_t.CMD_SET_GAIN:
                g = arx5_gain_t()
                kp = data["kp"]
                kd = data["kd"]
                g.num_joints = len(kp)
                g.kp = list(kp)
                g.kd = list(kd)
                g.gripper_kp = float(data["gripper_kp"])
                g.gripper_kd = float(data["gripper_kd"])
                lcm_cmd.gain = g

            self.lc.publish(self.request_channel, lcm_cmd.encode())

            while self.current_response is None:
                self.lc.handle()

            return self._convert_response_to_dict(self.current_response, cmd_str)

        except KeyboardInterrupt:
            print("Arx5LcmClient: KeyboardInterrupt.")
            return {"cmd": "UNKNOWN", "data": "KeyboardInterrupt"}
        except Exception as e:
            print(f"Arx5LcmClient: Error {e}")
            print(echo_exception())
            return {"cmd": "UNKNOWN", "data": "LcmError"}

    def get_state(self) -> StateDict:
        """Return the latest robot state.

        Keys:
        - timestamp: seconds since controller start
        - ee_pose: 6D end-effector pose (x, y, z, rx, ry, rz)
        - joint_pos: joint positions
        - joint_vel: joint velocities
        - joint_torque: joint torques
        - gripper_pos: gripper position
        - gripper_vel: gripper velocity
        - gripper_torque: gripper torque
        """
        reply_msg = self.send_recv(arx5_command_t.CMD_GET_STATE)
        assert reply_msg["cmd"] == "GET_STATE"
        assert isinstance(reply_msg["data"], dict)

        # Check for errors passed as string in data
        if isinstance(reply_msg["data"], str):
            # Could be "LcmError" or "KeyboardInterrupt" or "Error: ..."
            return self.latest_state

        state = cast(StateDict, reply_msg["data"])
        self.latest_state = state
        return state

    def set_ee_pose(
        self,
        pose_6d: npt.NDArray[np.float64],
        gripper_pos: Optional[float] = None,
        preview_time: Optional[float] = None,
    ):
        reply_msg = self.send_recv(
            arx5_command_t.CMD_SET_EE_POSE,
            {
                "ee_pose": pose_6d,
                "gripper_pos": gripper_pos,
                "preview_time": preview_time,
            },
        )
        assert reply_msg["cmd"] == "SET_EE_POSE"

        if isinstance(reply_msg["data"], str):
            return self.latest_state

        if type(reply_msg["data"]) != dict:
            raise ValueError(f"Error: {reply_msg['data']}")

        state = cast(dict[str, Union[npt.NDArray[np.float64], float]], reply_msg["data"])
        self.latest_state = state
        return state


    def reset_to_home(self):
        reply_msg = self.send_recv(arx5_command_t.CMD_RESET_TO_HOME)
        assert reply_msg["cmd"] == "RESET_TO_HOME"
        if reply_msg["data"] != "OK":
            raise ValueError(f"Error: {reply_msg['data']}")
        self.get_state()

    def set_to_damping(self):
        reply_msg = self.send_recv(arx5_command_t.CMD_SET_TO_DAMPING)
        assert reply_msg["cmd"] == "SET_TO_DAMPING"
        if reply_msg["data"] != "OK":
            raise ValueError(f"Error: {reply_msg['data']}")
        self.get_state()

    def get_gain(self) -> GainDict:
        """Return the current controller gains.

        Keys:
        - kp: joint proportional gains
        - kd: joint derivative gains
        - gripper_kp: gripper proportional gain
        - gripper_kd: gripper derivative gain
        """
        reply_msg = self.send_recv(arx5_command_t.CMD_GET_GAIN)
        assert reply_msg["cmd"] == "GET_GAIN"
        if type(reply_msg["data"]) != dict:
            raise ValueError(f"Error: {reply_msg['data']}")
        return cast(GainDict, reply_msg["data"])

    def set_gain(self, gain: GainDict):
        reply_msg = self.send_recv(arx5_command_t.CMD_SET_GAIN, gain)
        assert reply_msg["cmd"] == "SET_GAIN"
        if reply_msg["data"] != "OK":
            raise ValueError(f"Error: {reply_msg['data']}")

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
        # self.lc.close()
        print("Arx5LcmClient is closed")
