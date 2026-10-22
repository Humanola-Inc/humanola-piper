from __future__ import annotations

import argparse

from humanola import robo
from native import Arm, PiperNotSim, PiperSim

LEADER_TOPIC = "dev:piper-leader"


class LeaderDevice:
    def __init__(self, arm: Arm):
        self.arm = arm

    def open(self) -> LeaderDevice:
        return self

    def get_device(self) -> robo.Device:
        device = robo.Device(kind="piper", name="leader")
        device.timestamp_now()
        for i, joint in enumerate(self.arm.get_joints()[:6]):
            device.add_reading(f"joint{i + 1}", robo.RawDoubleState(value=float(joint)))
        return device

    def close_stream(self) -> None:
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--channel", default="can0")
    args = parser.parse_args()

    arm: Arm = PiperSim(args.channel) if args.sim else PiperNotSim(args.channel)
    arm.connect()

    channel, runtime = (
        robo.RoboConfig(
            api_url="https://grpc.humanola.com",
            api_key="<YOUR_API_KEY>",
            robo_id="<YOUR_ROBO_ID>",
            name="Piper leader",
        )
        .attach_device(
            topic=LEADER_TOPIC,
            name="Piper leader arm",
            desc="Publishes the joints of the piper leader arm",
            rate=60,
            v=LeaderDevice(arm),
        )
        .run()
    )
    try:
        runtime.wait_for_interrupt()
    finally:
        arm.close()
