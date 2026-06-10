from humanola import robo


class PiperBattery:
    def get_battery_status(self) -> robo.Battery:
        return (
            robo.Battery()
            .add_battery("left", robo.BatteryStatus.plugged_in())
            .add_battery("right", robo.BatteryStatus.plugged_in())
        )
