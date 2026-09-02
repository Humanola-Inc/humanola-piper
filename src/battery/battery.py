from humanola import robo


class PiperBattery:
    def get_battery(self) -> robo.Battery:
        return robo.Battery().attach_plugged_in("left").attach_plugged_in("right")
