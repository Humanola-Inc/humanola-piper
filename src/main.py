from __future__ import annotations

import logging

from humanola import robo

from battery import PiperBattery
from controllers import PiperXr
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
        left_arm=PiperState.connect("can0"),
        right_arm=PiperState.connect("can1"),
        solver=PiperSolver(config=PiperSolverConfig()),
    )
    channel, runtime = (
        robo.Robo(
            url="https://grpc.humanola.com", api_key="<API_KEY>", robo_id="<ROBO_ID>"
        )
        .add_controller("controller", PiperXr(state))
        .set_battery(PiperBattery())
        .add_source("data", PiperDataSource(state))
        .auto_discover_cameras()
        .verbose()
        .run(on_error)
    )
    runtime.wait_for_interrupt()
