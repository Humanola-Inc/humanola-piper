import logging
import math
from typing import List

import numpy as np
from humanola import controllers, robo
from piper_native import (
    ArmState,
    PiperEvent,
    PiperGrip,
    PiperMove,
    PiperReposition,
    PiperReset,
    PiperSetupState,
)


class PiperControllerStream:
    def __init__(
        self,
        state: PiperSetupState,
        init_joints: np.ndarray = np.array([0, 60000, -80000, 0, 90000, 0])
        * math.pi
        / 180_000,
    ):
        self.init_state = init_joints
        self.state = state
        self.left_state = ArmState(
            pose=self.state.solver.forward(init_joints),
            gripper=0,
            joints=tuple(init_joints),
        )
        self.right_state = ArmState(
            pose=self.state.solver.forward(init_joints),
            gripper=0,
            joints=tuple(init_joints),
        )

    def ctrl_beg(self) -> None:
        self.state.enable_arms()
        self.state.apply_arm_state((self.left_state, self.right_state))

    def emit_reset(
        self,
        hand: str,
        prev_device: controllers.Device,
        device: controllers.Device,
        goals: list[PiperEvent],
    ) -> None:
        prev_primary_btn = prev_device.get(
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name(f"*.{hand}.primary")
        )
        cur_primary_btn = device.get(
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name(f"*.{hand}.primary")
        )
        prev_secondary_btn = prev_device.get(
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name(f"*.{hand}.secondary")
        )
        cur_secondary_btn = device.get(
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name(f"*.{hand}.secondary")
        )
        if (
            prev_primary_btn is not None
            and cur_primary_btn is not None
            and not prev_primary_btn.as_btn().pressed
            and cur_primary_btn.as_btn().pressed
        ):
            logging.info("TURNING OFF PIPER")
            goals.append(PiperReset(hand=hand))
        if (
            prev_secondary_btn is not None
            and cur_secondary_btn is not None
            and prev_secondary_btn.as_btn().pressed
            and not cur_secondary_btn.as_btn().pressed
        ):
            logging.info("RESET PIPER POSITION")
            goals.append(PiperReposition(hand=hand, joints=tuple(self.init_state)))

    def emit_grip(
        self,
        hand: str,
        prev_device: controllers.Device,
        device: controllers.Device,
        goals: list[PiperEvent],
    ) -> None:
        prev_aim_btn = prev_device.get(
            controllers.Query().kind(controllers.SensorKind.Btn).name(f"*.{hand}.aim")
        )
        cur_aim_btn = device.get(
            controllers.Query().kind(controllers.SensorKind.Btn).name(f"*.{hand}.aim")
        )
        if prev_aim_btn is not None and cur_aim_btn is not None:
            prev_aim_pressed = prev_aim_btn.as_btn().value > 0.5
            cur_aim_pressed = cur_aim_btn.as_btn().value > 0.5
            if prev_aim_pressed and not cur_aim_pressed:
                logging.info(f"GRIP RELEASE {hand}")
                goals.append(PiperGrip(hand=hand, open=False))
            if not prev_aim_pressed and cur_aim_pressed:
                logging.info(f"GRIP TRIGGER {hand}")
                goals.append(PiperGrip(hand=hand, open=True))

    def emit_move(
        self,
        hand: str,
        prev_device: controllers.Device,
        device: controllers.Device,
        goals: list[PiperEvent],
    ) -> None:
        prev_squeeze_btn = prev_device.get(
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name(f"*.{hand}.squeeze")
        )
        cur_squeeze_btn = device.get(
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name(f"*.{hand}.squeeze")
        )
        prev_position = prev_device.get(
            controllers.Query().kind(controllers.SensorKind.Pos).name(f"*.{hand}.bod")
        )
        prev_rotation = prev_device.get(
            controllers.Query().kind(controllers.SensorKind.Rot).name(f"*.{hand}.bod")
        )
        cur_position = device.get(
            controllers.Query().kind(controllers.SensorKind.Pos).name(f"*.{hand}.bod")
        )
        cur_rotation = device.get(
            controllers.Query().kind(controllers.SensorKind.Rot).name(f"*.{hand}.bod")
        )

        if prev_squeeze_btn is not None and cur_squeeze_btn is not None:
            cur_squeeze_pressed = cur_squeeze_btn.as_btn().value > 0.5
            prev_squeeze_btn = prev_squeeze_btn.as_btn()
            if prev_squeeze_btn.pressed <= 0.5 and cur_squeeze_pressed:
                logging.info(f"PIPER MOTION END {hand}")
            elif prev_squeeze_btn.pressed > 0.5 and not cur_squeeze_pressed:
                logging.info(f"PIPER MOTION START {hand}")
            if (
                cur_squeeze_pressed
                and prev_position is not None
                and prev_rotation is not None
                and cur_position is not None
                and cur_rotation is not None
            ):
                prev_position = prev_position.as_pos()
                prev_rotation = prev_rotation.as_rot()
                cur_position = cur_position.as_pos()
                cur_rotation = cur_rotation.as_rot()
                goals.append(
                    PiperMove(
                        hand=hand,
                        prev_rot=(
                            prev_rotation.x,
                            prev_rotation.y,
                            prev_rotation.z,
                            prev_rotation.w,
                        ),
                        rot=(
                            cur_rotation.x,
                            cur_rotation.y,
                            cur_rotation.z,
                            cur_rotation.w,
                        ),
                        prev_pos=(
                            prev_position.x,
                            prev_position.y,
                            prev_position.z,
                        ),
                        pos=(
                            cur_position.x,
                            cur_position.y,
                            cur_position.z,
                        ),
                    )
                )

    def ctrl(self, prev: "controllers.Device", cur: "controllers.Device") -> None:
        goals: List[PiperEvent] = []
        self.emit_move("left", prev, cur, goals)
        self.emit_move("right", prev, cur, goals)
        self.emit_grip("left", prev, cur, goals)
        self.emit_grip("right", prev, cur, goals)
        self.emit_reset("left", prev, cur, goals)
        self.emit_reset("right", prev, cur, goals)

        for goal in goals:
            self.state.handle_event(goal, (self.left_state, self.right_state))

    def ctrl_end(self) -> None:
        self.state.zero_pos()


class PiperXr:
    def __init__(self, state: PiperSetupState):
        self.state = state

    def desc(self):
        return robo.LoopDesc(
            frame_rate=60,
            name="Piper dual arm controller",
            desc="Controls two piper arm with meta quest",
        )

    def open(self):
        return PiperControllerStream(self.state)
