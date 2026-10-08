import math
import multiprocessing as mp
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional, Protocol

import numpy as np
import numpy.typing as npt
from piper_sdk import C_PiperInterface as Piper

from .state import ArmState

# fork is unsafe once the humanola runtime (grpc) threads are running, which is
# the case when restarting
mp_ctx = mp.get_context("spawn")


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


@dataclass
class JointCtrl:
    joints: npt.NDArray[np.float64]
    duration: Optional[float]


class GetJoint:
    pass


class Close:
    pass


def offshore_controller(
    channel: str,
    sim: bool,
    restart: bool,
    is_ready,
    ctrl_rx,
    joints_rx,
):
    # the sdk parses every can frame in python, keeping it in its own process
    # stops it from starving the ik of the gil
    arm: Arm = PiperSim(channel) if sim else PiperNotSim(channel)
    if restart:
        arm.restart()
    else:
        arm.start()

    def joints_loop():
        while True:
            ev = joints_rx.recv()
            if isinstance(ev, Close):
                break
            elif isinstance(ev, GetJoint):
                joints_rx.send(arm.get_joints())

    def ctrl_loop():
        while True:
            ev = ctrl_rx.recv()
            if isinstance(ev, Close):
                break
            elif isinstance(ev, JointCtrl):
                if ev.duration is None:
                    arm.update(ev.joints)
                else:
                    arm.update_over_time(ev.joints, ev.duration)

    joints_thread = threading.Thread(target=joints_loop, daemon=True)
    ctrl_thread = threading.Thread(target=ctrl_loop, daemon=True)
    joints_thread.start()
    ctrl_thread.start()
    is_ready.set()
    try:
        joints_thread.join()
        ctrl_thread.join()
    finally:
        arm.close()


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
        self.__start_process()
        if init_state:
            joints = self.get_joints()
            self.state.set_with_joints(joints)
        else:
            self.update()

    def __start_process(self, restart: bool = False):
        self.is_ready = mp_ctx.Event()
        ctrl_rx, self.ctrl_tx = mp_ctx.Pipe(duplex=False)
        self.joints_tx, joints_rx = mp_ctx.Pipe()
        self.process = mp_ctx.Process(
            target=offshore_controller,
            args=(
                self.channel,
                self.sim,
                restart,
                self.is_ready,
                ctrl_rx,
                joints_rx,
            ),
        )
        self.process.start()
        self.is_ready.wait()

    def update(self):
        self.ctrl_tx.send(JointCtrl(self.state.joints, None))

    def update_over_time(self, target_joints: npt.NDArray[np.float64], duration: float):
        self.state.set_with_joints(target_joints)
        self.ctrl_tx.send(JointCtrl(target_joints, duration))

    def get_joints(self) -> npt.NDArray[np.float64]:
        self.joints_tx.send(GetJoint())
        return self.joints_tx.recv()

    def restart(self):
        self.close()
        self.__start_process(restart=True)

    def close(self):
        self.update_over_time(np.zeros(7), 1)
        self.joints_tx.send(Close())
        self.ctrl_tx.send(Close())
        self.process.join()
