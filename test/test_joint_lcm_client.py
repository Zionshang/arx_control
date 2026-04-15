import os
import sys
import time

import numpy as np

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from communication.lcm.joint_lcm_client import Arx5JointLcmClient


def main():
    client = Arx5JointLcmClient()
    client.reset_to_home()
    time.sleep(2.0)
    target_joint_pos = np.array([-0.7, 0.85, 0.8, 0.39, 0.9, 0.5])
    target_gripper_pos = 0.5
    arrival_time_s = 5.0

    state = client.set_joint_cmd(
        target_joint_pos,
        gripper_pos=target_gripper_pos,
        preview_time=arrival_time_s,
    )

    print("Joint LCM command sent.")
    print(f"target_joint_pos: {target_joint_pos}")
    print(f"target_gripper_pos: {target_gripper_pos}")
    print(f"arrival_time_s: {arrival_time_s}")
    print(f"state_timestamp: {state['timestamp']}")

    sleep_time_s = arrival_time_s + 1.0
    print(f"Sleeping {sleep_time_s} seconds before reset_to_home.")
    time.sleep(sleep_time_s)

    client.reset_to_home()
    print("Reset to home.")
    client.set_to_damping()
    print("Set to damping.")


if __name__ == "__main__":
    main()
