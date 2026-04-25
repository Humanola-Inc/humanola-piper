from __future__ import annotations

import logging

from battery import PiperBattery
from controllers import PiperXr
from humanola import robo
from piper_native import PiperSetupState, PiperSolver, PiperSolverConfig, PiperState
from sources import PiperDataSource


def on_error(err: str) -> None:
    logging.error("Robo error: %s", err)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(message)s",
        datefmt="%H:%M:%S",
    )
    state = PiperSetupState(
        left_arm=PiperState.connect("can1"),
        right_arm=PiperState.connect("can0"),
        solver=PiperSolver(config=PiperSolverConfig()),
    )
    channel, runtime = (
        robo.Robo.new_default()
        .add_controller("controller", PiperXr(state))
        .set_battery(PiperBattery())
        .add_source("data", PiperDataSource(state))
        .auto_discover_cameras()
        .run(on_error)
    )
    runtime.wait_for_interrupt()
