from __future__ import annotations

import argparse

import numpy as np
from humanola import robo
from native import Arm, PiperNotSim, PiperSim

LEADER_TOPIC = "dev:piper-leader"


class FollowerCtrl:
    QUERIES: list[robo.Query] = [
        robo.Query(kind=robo.SensorKind.RawDouble, name=name)
        for name in [f"joint{i + 1}" for i in range(6)] + ["gripper"]
    ]

    def __init__(self, arm: Arm):
        self.arm = arm

    def open(self) -> FollowerCtrl:
        return self

    def recv_delta(self, prev: robo.Device, cur: robo.Device) -> None:
        sensors = [cur.get(q) for q in FollowerCtrl.QUERIES]
        if any(s is None for s in sensors):
            return
        self.arm.update(np.array([s.as_raw().value for s in sensors]))

    def close_stream(self) -> None:
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--channel", default="can0")
    args = parser.parse_args()

    arm: Arm = PiperSim(args.channel) if args.sim else PiperNotSim(args.channel)
    arm.start()

    channel, runtime = (
        robo.RoboConfig(
            api_url="https://grpc.humanola.com",
            api_key="<YOUR_API_KEY>",
            robo_id="<YOUR_ROBO_ID>",
            name="Piper follower",
        )
        .attach_device_subscriber(
            topic=LEADER_TOPIC,
            name="Piper follower arm",
            desc="Mirrors the joints of the piper leader arm",
            rate=60,
            v=FollowerCtrl(arm),
        )
        .run()
    )
    try:
        runtime.wait_for_interrupt()
    finally:
        arm.update(np.zeros(7))
        arm.close()
