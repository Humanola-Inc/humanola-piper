from humanola import transport

from piper_native import PiperSetupState


class PiperDataSource:
    def __init__(self, state: PiperSetupState):
        self.state = state

    def src_beg(self):
        pass

    def src(self):
        snapshot = self.state.snapshot()
        return transport.RawPacket(snapshot.proto_encode())

    def src_end(self):
        pass

    def open(self):
        return self
