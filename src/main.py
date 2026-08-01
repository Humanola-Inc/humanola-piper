from __future__ import annotations

import logging

from humanola import constants, robo

from battery import PiperBattery
from controllers import PiperXr
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
    r = robo.Robo(api_url="https://grpc.humanola.com", api_key="<YOUR_API_KEY>")
    r.attach_controller(
        robo.LoopDesc(
            name="Piper dual arm controller",
            desc="Controls two piper arm with meta quest",
            topic=constants.DEV_XR_CONTROLLER_TOPIC,
            rate=60,
        ),
        PiperXr(state),
    )
    r.attach_battery(PiperBattery())
    r.attach_source(
        robo.LoopDesc(
            name="Piper dual arm source",
            desc="Records the joint position of piper arms",
            topic="src:data",
            rate=60,
            fields=state.fields(),
        ),
        PiperDataSource(state),
    )
    r.auto_discover_cameras()
    channel, runtime = r.run()
    runtime.wait_for_interrupt()
