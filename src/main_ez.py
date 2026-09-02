from __future__ import annotations

import logging
import os

from humanola import constants, robo

from battery import PiperBattery
from controllers import PiperXr
from main import ROBO_ID, attach_cameras
from piper_native import PiperSetupState, PiperSolver, PiperSolverConfig, PiperState
from sources import PiperDataSource

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
    config = attach_cameras(
        robo.RoboConfig(
            os.environ["HUMANOLA_API_ENDPOINT"],
            os.environ["HUMANOLA_API_KEY"],
            os.environ.get("HUMANOLA_ROBO_ID", ROBO_ID),
        )
        .attach_device_subscriber(
            constants.DEV_XR_CONTROLLER_TOPIC,
            PiperXr(state),
            60,
            name="Piper dual arm controller",
            desc="Controls two piper arm with meta quest",
        )
        .attach_battery(PiperBattery())
        .attach_data(
            constants.SRC_DATA,
            PiperDataSource(state),
            60,
            state.fields(),
            name="Piper dual arm source",
            desc="Records the joint position of piper arms",
        )
    )
    channel, runtime = config.run()
    runtime.wait_for_interrupt()
