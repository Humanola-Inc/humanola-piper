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
        left_arm=PiperState.connect("can0"),
        right_arm=PiperState.connect("can1"),
        solver=PiperSolver(config=PiperSolverConfig()),
    )
    channel, runtime = (
        robo.Robo(
            url="https://grpc.humanola.com",
            api_key="I3MObARleuenXMhRo4gATDG5tBrSYk11Su03YWxO",
            robo_id="019cb288-a9b3-7dc3-9249-d8e7bd936549",
        )
        # robo.Robo(
        #     url="http://192.168.1.119:8001",
        #     api_key="Fe1J3LXWZafMLEzND0QN0hzKLP452Mij7DMQpDfU",
        #     robo_id="019c529b-0bab-7db0-af4c-20a87b97cbd1",
        # )
        .add_controller("controller", PiperXr(state))
        .set_battery(PiperBattery())
        .add_source("data", PiperDataSource(state))
        .auto_discover_cameras()
        .verbose()
        .run(on_error)
    )
    runtime.wait_for_interrupt()
