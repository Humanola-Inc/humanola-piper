from humanola import robo

from piper_native import PiperSetupState


class PiperDataSource:
    def __init__(self, state: PiperSetupState):
        self.state = state

    def get_data(self) -> robo.DataFrame:
        return self.state.snapshot()

    def close_stream(self) -> None:
        pass

    def open(self):
        return self
