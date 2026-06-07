from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Dict

from humanola import robo

from battery import PiperBattery
from controllers import PiperXr
from piper_native import PiperSetupState, PiperSolver, PiperSolverConfig, PiperState
from sources import PiperDataSource


def on_error(err: str) -> None:
    logging.error("Robo error: %s", err)


@dataclass
class Camera:
    id: int
    desc: robo.CameraDesc
    cam: "robo.CameraSpec"


def is_better(prev: robo.CameraDesc, cur: robo.CameraDesc):
    if cur.width > prev.width:
        return True
    elif cur.width == prev.width and cur.height > prev.height:
        return True
    elif (
        cur.width == prev.width
        and cur.height == prev.height
        and cur.frame_rate > prev.frame_rate
    ):
        return True
    return False


if __name__ == "__main__":
    cameras = robo.list_cameras()
    cam_ids: Dict[int, Camera] = {}
    for id, spec in cameras:
        desc = spec.desc()
        if id not in cam_ids:
            cam_ids[id] = Camera(id=id, desc=desc, cam=spec)
        elif id in cam_ids and is_better(cam_ids[id].desc, desc):
            cam_ids[id] = Camera(id=id, desc=desc, cam=spec)

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
        robo.Robo.new_default()
        .add_controller("controller", PiperXr(state))
        .set_battery(PiperBattery())
        .add_source("data", PiperDataSource(state))
    )
    for id, s in cam_ids.items():
        robo.add_camera(s.desc.name, s.cam)
    channel, runtime = robo.run(on_error)
    runtime.wait_for_interrupt()
