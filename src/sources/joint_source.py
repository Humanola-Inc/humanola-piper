from humanola import robo, streaming

from piper_native import PiperSetupState


class PiperDataSource:
    def __init__(self, state: PiperSetupState):
        self.state = state

    def src_beg(self):
        pass

    def src(self):
        snapshot = self.state.snapshot()
        return streaming.DataFrame.binary(snapshot.proto_encode())

    def src_end(self):
        pass

    def desc(self):
        return robo.LoopDesc(
            frame_rate=60,
            name="Piper dual arm source",
            desc="Records the joint position of piper arms",
        )

    def open(self):
        return self
