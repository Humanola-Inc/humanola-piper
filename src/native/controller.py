import math
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Protocol

import numpy as np
import numpy.typing as npt
from piper_sdk import C_PiperInterface as Piper

from .state import ArmState


class Arm(Protocol):
    def __init__(self, channel: str):
        pass

    def start(self):
        pass

    def restart(self):
        pass

    def update(self, joints: npt.NDArray[np.float64]):
        pass

    def update_over_time(self, target_joints: npt.NDArray[np.float64], duration: float):
        pass

    def get_joints(self) -> npt.NDArray[np.float64]:
        pass

    def close(self):
        pass


class PiperSim:
    def __init__(self, channel: str):
        self.channel = channel
        self.joints = np.zeros(7)

    def start(self):
        pass

    def restart(self):
        pass

    def update(self, joints: npt.NDArray[np.float64]):
        self.joints = joints

    def update_over_time(self, target_joints: npt.NDArray[np.float64], duration: float):
        start_time = 0
        delta_joints = (target_joints - self.joints) / duration
        while start_time < duration:
            self.joints = self.joints + delta_joints * 0.1
            start_time += 0.1
            time.sleep(0.1)
        self.joints = target_joints

    def get_joints(self) -> npt.NDArray[np.float64]:
        return self.joints

    def close(self):
        pass


class PiperNotSim:
    # rad -> 0.001 deg, the unit JointCtrl / GetArmJointMsgs use
    RAD_TO_MDEG = 1000 * 180 / math.pi
    # joints[6] is the gripper opening from 0 (closed) to 1 (open), the sdk wants
    # micro meters of total opening, 70mm being 2x joint7's upper limit in the urdf
    GRIPPER_TO_UM = 70_000

    def __init__(self, channel: str):
        self.channel = channel
        self.piper = Piper(channel)

    def start(self):
        self.piper.ConnectPort()
        while not all(self.piper.GetArmEnableStatus()[:6]):
            self.piper.EnableArm()
            time.sleep(0.1)
        self.piper.ModeCtrl(0x01, 0x01, 100, 0)
        self.piper.GripperCtrl(0, 1000, 0x01, 0)

    def restart(self):
        self.close()
        subprocess.run(["ip", "link", "set", self.channel, "down"], check=True)
        subprocess.run(
            [
                "ip",
                "link",
                "set",
                self.channel,
                "up",
                "type",
                "can",
                "bitrate",
                "1000000",
            ],
            check=True,
        )
        self.start()

    def update(self, joints: npt.NDArray[np.float64]):
        # the sdk protects against going outside the joint limits, no need to clamp here
        self.piper.JointCtrl(*(int(j * self.RAD_TO_MDEG) for j in joints[:6]))
        # micro meters
        self.piper.GripperCtrl(int(joints[6] * self.GRIPPER_TO_UM), 1000, 0x01, 0)

    def update_over_time(self, target_joints: npt.NDArray[np.float64], duration: float):
        start_joints = self.get_joints()
        steps = max(1, int(duration / 0.01))
        for i in range(1, steps + 1):
            self.update(start_joints + (target_joints - start_joints) * i / steps)
            time.sleep(0.01)

    def get_joints(self) -> npt.NDArray[np.float64]:
        js = self.piper.GetArmJointMsgs().joint_state
        gripper = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle
        return np.array(
            [
                js.joint_1 / self.RAD_TO_MDEG,
                js.joint_2 / self.RAD_TO_MDEG,
                js.joint_3 / self.RAD_TO_MDEG,
                js.joint_4 / self.RAD_TO_MDEG,
                js.joint_5 / self.RAD_TO_MDEG,
                js.joint_6 / self.RAD_TO_MDEG,
                gripper / self.GRIPPER_TO_UM,
            ]
        )

    def close(self):
        self.piper.DisconnectPort()


class ArmController:
    def __init__(
        self,
        channel: str,
        state: ArmState,
        init_state: bool = True,
        sim: bool = False,
    ):
        self.channel = channel
        self.sim = sim
        self.state = state
        self.arm: Arm = PiperSim(channel) if sim else PiperNotSim(channel)
        self.arm.start()
        if init_state:
            joints = self.get_joints()
            self.state.set_with_joints(joints)
        else:
            self.update()

    def update(self):
        self.arm.update(self.state.joints)

    def update_over_time(self, target_joints: npt.NDArray[np.float64], duration: float):
        self.state.set_with_joints(target_joints)
        self.arm.update_over_time(target_joints, duration)

    def get_joints(self) -> npt.NDArray[np.float64]:
        return self.arm.get_joints()

    def reset_flat(self):
        self.update_over_time(np.array([0, 1, 1, -1, 1, 1, 0]) * math.pi / 180, 1)

    def restart(self):
        self.reset_flat()
        self.arm.restart()

    def close(self):
        self.reset_flat()
        self.arm.close()
