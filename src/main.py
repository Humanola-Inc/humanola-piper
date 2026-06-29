from __future__ import annotations

import logging

from humanola import constants, robo

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
    robo = (
        robo.Robo(url="https://grpc.humanola.com", api_key="<YOUR_API_KEY>")
        .attach_controller(
            robo.LoopDesc(
                topic=constants.DEV_XR_CONTROLLER,
                rate=60,
                name="Piper dual arm controller",
                desc="Controls two piper arm with meta quest",
            ),
            PiperXr(state),
        )
        .attach_battery(PiperBattery())
        .attach_source(
            robo.LoopDesc(
                topic="src:data",
                rate=60,
                name="Piper dual arm source",
                desc="Records the joint position of piper arms",
            ),
            PiperDataSource(state),
        )
        .auto_discover_cameras()
    )
    channel, runtime = robo.run(on_error)
    runtime.wait_for_interrupt()
