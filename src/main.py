from __future__ import annotations

import logging

from humanola import constants, robo

from battery import PiperBattery
from controllers import PiperXr
from piper_native import PiperSetupState, PiperSolver, PiperSolverConfig, PiperState
from sources import PiperDataSource

ROBO_ID = "019cb287-0bb9-7452-96ea-459a69598003"


def attach_cameras(
    config: robo.RoboConfig, width: int = 1280, height: int = 720, rate: int = 30
) -> robo.RoboConfig:
    for cam_id in range(4):
        try:
            config.attach_usb_camera(f"cam:{cam_id}", cam_id, width, height, rate)
        except RuntimeError:
            continue
    return config


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
        robo.RoboConfig("https://grpc.humanola.com", "<YOUR_API_KEY>", ROBO_ID)
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
