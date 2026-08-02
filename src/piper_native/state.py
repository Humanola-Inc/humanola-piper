from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Tuple

import numpy as np
from humanola import robo
from piper_sdk import ArmMsgFeedbackStatusEnum
from piper_sdk import C_PiperInterface as Piper


@dataclass
class ArmState:
    pose: np.ndarray
    gripper: float
    joints: Tuple[float, float, float, float, float, float]


@dataclass
class PiperSnapshot:
    timestamp: int
    joint1: int
    joint2: int
    joint3: int
    joint4: int
    joint5: int
    joint6: int  # joint (mdegs)
    gripper: int  # milli meters

    @staticmethod
    def arm_fields(first: int) -> list[robo.Field]:
        fields = [
            robo.Field.tensor(f"joint{first + i}", robo.TDType.Angle, 1e-3, "mdeg", [1])
            for i in range(6)
        ]
        fields.append(
            robo.Field.tensor(f"joint{first + 6}", robo.TDType.Length, 1e-6, "um", [1])
        )
        fields.append(
            robo.Field.tensor(f"joint{first + 7}", robo.TDType.Length, 1e-6, "um", [1])
        )
        return fields

    @staticmethod
    def left_fields() -> list[robo.Field]:
        return PiperSnapshot.arm_fields(1)

    @staticmethod
    def right_fields() -> list[robo.Field]:
        return PiperSnapshot.arm_fields(9)

    @staticmethod
    def add_to_row(snapshot: PiperSnapshot | None, row: robo.Row) -> robo.Row:
        if snapshot is None:
            for _ in range(8):
                row.add([0.0])
            return row
        row.add([float(snapshot.joint1)])
        row.add([float(snapshot.joint2)])
        row.add([float(snapshot.joint3)])
        row.add([float(snapshot.joint4)])
        row.add([float(snapshot.joint5)])
        row.add([float(snapshot.joint6)])
        row.add([float(snapshot.gripper // 2)])
        row.add([float(-snapshot.gripper // 2)])
        return row

    @staticmethod
    def create_row(left: PiperSnapshot | None, right: PiperSnapshot | None) -> robo.Row:
        stamps = [s.timestamp for s in (left, right) if s is not None]
        row = robo.Row(sum(stamps) // len(stamps)) if stamps else robo.Row()
        PiperSnapshot.add_to_row(left, row)
        PiperSnapshot.add_to_row(right, row)
        return row


class PiperState:
    def __init__(self, piper: Piper):
        self.piper = piper
        self.is_resetting = False

    def disable_arm(self):
        self.piper.DisableArm()
        self.piper.ModeCtrl(0, 0)

    def enable_arm(self):
        self.piper.EnableArm()
        self.piper.ModeCtrl(0x01, 0x01, 100, 0)
        self.piper.GripperCtrl(0, 1000, 0x01, 0)

    def emergency_reset(self, timeout: float = 7):
        logging.info("EMERGENCY STOP ARM [x]")
        self.is_resetting = True
        self.piper.EmergencyStop(0x01)
        logging.info("EMERGENCY STOP ARM [√]")
        time.sleep(timeout)
        logging.info("EMERGENCY RESUME ARM [x]")
        self.piper.EmergencyStop(0x02)
        logging.info("EMERGENCY RESUME ARM [√]")
        logging.info("Wait For Normal [x]")
        while (
            self.piper.GetArmStatus().arm_status.arm_status
            != ArmMsgFeedbackStatusEnum.ArmStatus.NORMAL
        ):
            time.sleep(0.1)
        logging.info("Wait For Normal [√]")
        logging.info("Wait For Enable [x]")
        while not all(self.piper.GetArmEnableStatus()[:6]):
            self.piper.EnableArm()
            time.sleep(0.1)
        logging.info("Wait For Enable [√]")
        self.enable_arm()
        self.is_resetting = False

    def snapshot(self) -> PiperSnapshot:
        joints = self.piper.GetArmJointMsgs()
        gripper = self.piper.GetArmGripperMsgs()
        return PiperSnapshot(
            timestamp=int(time.time_ns() / 1000),
            joint1=joints.joint_state.joint_1,
            joint2=joints.joint_state.joint_2,
            joint3=joints.joint_state.joint_3,
            joint4=joints.joint_state.joint_4,
            joint5=joints.joint_state.joint_5,
            joint6=joints.joint_state.joint_6,
            gripper=gripper.gripper_state.grippers_angle,
        )

    def apply(self, v: Tuple[float, float, float, float, float, float, float]):
        if self.is_resetting:
            return
        # the sdk protects again navigating outside the angles by default. hence, there is no need to set them
        # over here
        self.piper.JointCtrl(
            int(v[0] * 1000 / math.pi * 180),
            int(v[1] * 1000 / math.pi * 180),
            int(v[2] * 1000 / math.pi * 180),
            int(v[3] * 1000 / math.pi * 180),
            int(v[4] * 1000 / math.pi * 180),
            int(v[5] * 1000 / math.pi * 180),
        )
        # to micro meters from meters
        self.piper.GripperCtrl(int(v[6] * 1e6), 1000, 0x01, 0)

    def apply_arm_state(self, state: ArmState):
        self.apply(
            (
                state.joints[0],
                state.joints[1],
                state.joints[2],
                state.joints[3],
                state.joints[4],
                state.joints[5],
                state.gripper,
            )
        )

    def zero_pos(self):
        self.apply((0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0))

    @staticmethod
    def connect(channel: str) -> PiperState | None:
        try:
            piper = Piper(channel)
            piper.ConnectPort()
            return PiperState(piper)
        except Exception as e:
            logging.error(f"{e}")
            return None
