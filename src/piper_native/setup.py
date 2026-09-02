import threading
from typing import Tuple

import numpy as np
from humanola import dataset, robo

from .event import PiperEvent, PiperGrip, PiperMove, PiperReposition, PiperReset
from .solver import PiperSolver
from .state import ArmState, PiperSnapshot, PiperState
from .utils import convert_to_robot_convention, pose2transform


class PiperSetupState:
    def __init__(
        self,
        left_arm: PiperState | None,
        right_arm: PiperState | None,
        solver: PiperSolver,
    ):
        self.left_arm = left_arm
        self.right_arm = right_arm
        self.solver = solver

    def disable_arms(self):
        if self.left_arm is not None:
            self.left_arm.disable_arm()
        if self.right_arm is not None:
            self.right_arm.disable_arm()

    def enable_arms(self):
        if self.left_arm is not None:
            self.left_arm.enable_arm()
        if self.right_arm is not None:
            self.right_arm.enable_arm()

    def fields(self) -> list[dataset.Field]:
        return PiperSnapshot.left_fields() + PiperSnapshot.right_fields()

    def snapshot(self) -> robo.DataFrame:
        left_snapshot = self.left_arm.snapshot() if self.left_arm is not None else None
        right_snapshot = (
            self.right_arm.snapshot() if self.right_arm is not None else None
        )
        return PiperSnapshot.create_row(left_snapshot, right_snapshot)

    def handle_event(self, e: PiperEvent, arm_state: Tuple[ArmState, ArmState]):
        state = arm_state[0] if e.hand == "left" else arm_state[1]
        arm = self.left_arm if e.hand == "left" else self.right_arm
        if arm is None:
            return
        if isinstance(e, PiperMove):
            vr_src = convert_to_robot_convention(
                pose2transform(
                    np.array(e.prev_pos),
                    np.array(e.prev_rot),
                )
            )
            vr_tgt = convert_to_robot_convention(
                pose2transform(
                    np.array(e.pos),
                    np.array(e.rot),
                )
            )
            tf = np.linalg.inv(vr_src) @ vr_tgt
            state.pose = state.pose @ tf
            state.joints = self.solver.inverse(state.pose, np.array(state.joints))
            arm.apply_arm_state(state)
        elif isinstance(e, PiperGrip):
            state.gripper = 0.1 if e.open else 0.0
            arm.apply_arm_state(state)
        elif isinstance(e, PiperReposition):
            state.gripper = 0
            state.pose = self.solver.forward(np.array(e.joints))
            state.joints = e.joints
            arm.apply_arm_state(state)
        elif isinstance(e, PiperReset):
            threading.Thread(target=arm.emergency_reset, daemon=True).start()

    def zero_pos(self):
        if self.left_arm is not None:
            self.left_arm.zero_pos()
        if self.right_arm is not None:
            self.right_arm.zero_pos()

    def apply_arm_state(self, arm_state: Tuple[ArmState, ArmState]):
        left_arm_state, right_arm_state = arm_state
        if self.left_arm is not None:
            self.left_arm.apply_arm_state(left_arm_state)
        if self.right_arm is not None:
            self.right_arm.apply_arm_state(right_arm_state)
