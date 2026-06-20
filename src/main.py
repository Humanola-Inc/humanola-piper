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
        # robo.Robo(
        #     url="https://grpc.humanola.com",
        #     api_key="I3MObARleuenXMhRo4gATDG5tBrSYk11Su03YWxO",
        #     robo_id="019cb288-a9b3-7dc3-9249-d8e7bd936549",
        # )
        # robo.Robo(
        #     url="https://product-jitters-refried.ngrok-free.dev",
        #     api_key="Fe1J3LXWZafMLEzND0QN0hzKLP452Mij7DMQpDfU",
        # )
        robo.Robo(
            url="https://grpc.dev.humanola.com",
            api_key="robo_6JShjggJT2EMenshIMgkwfOqb2Pwc77ipx9AFyx5",
        )
        # .add_controller("controller", PiperXr(state))
        .attach_controller(
            robo.LoopDesc(
                topic="dev:controller",
                desc="Some weird shit controller",
                rate=60,
                name="XR Controller",
            ),
            PiperXr(state),
        )
        .attach_usb_camera(
            kind=robo.CameraKind.Stereo,
            width=2560,
            height=720,
            topic="ego_lol_camera",
            name="Lol Egocentric Camera",
            rate=60,
            id=2,
        )
        .attach_usb_camera(
            kind=robo.CameraKind.Stereo,
            width=2560,
            height=720,
            topic="ego_lol_camera",
            name="Lol Egocentric Camera",
            rate=30,
            id=2,
        )
        .attach_usb_camera(
            kind=robo.CameraKind.Stereo,
            width=2560,
            height=720,
            topic="ego_camera",
            name="Egocentric Camera",
            rate=60,
            id=0,
        )
        .attach_usb_camera(
            kind=robo.CameraKind.Stereo,
            width=2560,
            height=720,
            topic="ego_camera",
            name="Egocentric Camera",
            rate=30,
            id=0,
        )
        .attach_battery(PiperBattery())
        .attach_source(
            robo.LoopDesc(
                topic="src:data",
                desc="some weird shit joint source",
                rate=60,
                name="Joint source",
            ),
            PiperDataSource(state),
        )
        .verbose()
        .run(on_error)
    )
    runtime.wait_for_interrupt()
