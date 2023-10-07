from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from px4_msgs.msg import Wind, VehicleAirData, VehicleGlobalPosition
from diagnostic_updater import Updater, DiagnosticTask, CompositeDiagnosticTask
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
import rclpy


WIND_TOPIC = "/fmu/out/wind"
AIR_TOPIC = "/fmu/out/VehicleAirData"
POSITION_TOPIC = "/fmu/out/VehicleGlobalPosition"


class WeatherDiagnosticsNode(Node):
    def __init__(self):
        super().__init__("weather_diagnostic")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("timeout", 10),
                ("err_altitude", 2500.0),
                ("warn_wind_speed", 13.0),
                ("err_wind_speed", 15.0),
                ("warn_min_temperature", -30.0),
                ("err_min_temperature", -40.0),
                ("warn_max_temperature", 40.0),
                ("err_max_temperature", 40.0),
            ],
        )
        self.wind_speed = None
        self.wind_speed_stamp = 0
        self.temperature = None
        self.temperature_stamp = 0
        self.altitude = None
        self.altitude_stamp = 0

        self.create_subscription(Wind, WIND_TOPIC, self.wind_callback, 10)
        self.create_subscription(VehicleAirData, AIR_TOPIC, self.air_data_callback, 10)
        self.create_subscription(VehicleGlobalPosition, POSITION_TOPIC, self.global_position_callback, 10)

        # diagnostic_update block
        updater = Updater(self, period=self.get_parameter("timeout").value)
        updater.setHardwareID("none")

        update_wind = DiagnosticTaskWind(node=self)
        update_temperature = DiagnosticTaskTemperature(node=self)
        update_altitude = DiagnosticTaskAltitude(node=self)
        updater.add(update_wind)
        updater.add(update_temperature)
        updater.add(update_altitude)
        
        # within using aggregator node
        # compose_updater = CompositeDiagnosticTask("Weather status")
        # compose_updater.addTask(update_wind)
        # compose_updater.addTask(update_temperature)
        # compose_updater.addTask(update_altitude)
        # updater.add(compose_updater)

    def wind_callback(self, msg):
        # TODO Choice One
        # self.wind_speed = (msg.windspeed_north ** 2 + msg.windspeed_east ** 2) ** 0.5
        # self.wind_speed = msg.windspeed_north
        self.wind_speed = msg.tas_innov

        # TODO px4 used timestamp from system start
        # if we want a extend timestamp diagnostic, we can use TopicDiagnostic(TimestampStatusParam)
        # self.wind_speed_stamp = msg.timestamp
        self.wind_speed_stamp = Clock(clock_type=ClockType.ROS_TIME).now().nanoseconds

    def air_data_callback(self, msg):
        self.temperature = msg.baro_temp_celcius
        self.temperature_stamp = Clock(clock_type=ClockType.ROS_TIME).now().nanoseconds

    def global_position_callback(self, msg):
        self.altitude = msg.alt
        self.altitude_stamp = Clock(clock_type=ClockType.ROS_TIME).now().nanoseconds


class DiagnosticTaskWind(DiagnosticTask):
    def __init__(self, node: WeatherDiagnosticsNode):
        super().__init__("Wind speed status")
        self.node = node

    def run(self, status):
        wind_speed = self.node.wind_speed
        wind_timestamp = self.node.wind_speed_stamp
        max_wind_speed = self.node.get_parameter("err_wind_speed").value
        warn_wind_speed = self.node.get_parameter("warn_wind_speed").value
        timeout = self.node.get_parameter("timeout").value * 1e9

        curtime = Clock(clock_type=ClockType.ROS_TIME).now().nanoseconds
        if abs(curtime - wind_timestamp) > timeout:
            status.summary(
                DiagnosticStatus.STALE,
                f"No wind data last {(curtime - wind_timestamp) / 1e9:.2f} sec",
            )
        elif wind_speed > max_wind_speed:
            status.summary(DiagnosticStatus.ERROR, f"Excessive wind speed: {wind_speed} m/s")
        elif wind_speed > warn_wind_speed:
            status.summary(DiagnosticStatus.WARN, f"High wind speed: {wind_speed} m/s")
        else:
            status.summary(DiagnosticStatus.OK, f"Normal wind speed")
        return status


class DiagnosticTaskTemperature(DiagnosticTask):
    def __init__(self, node: WeatherDiagnosticsNode):
        super().__init__("Air Temperature Status")
        self.node = node

    def run(self, status):
        temperature = self.node.temperature
        temperature_timestamp = self.node.temperature_stamp
        warn_min_temperature = self.node.get_parameter("warn_min_temperature").value
        err_min_temperature = self.node.get_parameter("err_min_temperature").value
        warn_max_temperature = self.node.get_parameter("warn_max_temperature").value
        err_max_temperature = self.node.get_parameter("err_max_temperature").value
        timeout = self.node.get_parameter("timeout").value * 1e9

        curtime = Clock(clock_type=ClockType.ROS_TIME).now().nanoseconds
        if abs(curtime - temperature_timestamp) > timeout:
            status.summary(
                DiagnosticStatus.STALE,
                f"No temperature data last {(curtime - temperature_timestamp) / 1e9:.2f} sec",
            )
        elif temperature < err_min_temperature or temperature > err_max_temperature:
            status.summary(
                DiagnosticStatus.ERROR, f"Extreme temperature: {temperature} °C"
            )
        elif temperature < warn_min_temperature or temperature > warn_max_temperature:
            status.summary(
                DiagnosticStatus.WARN, f"Uncomfortable temperature: {temperature} °C"
            )
        else:
            status.summary(DiagnosticStatus.OK, "Comfortable temperature")
        return status


class DiagnosticTaskAltitude(DiagnosticTask):
    def __init__(self, node: WeatherDiagnosticsNode):
        super().__init__("Altitude Status")
        self.node = node

    def run(self, status):
        altitude = self.node.altitude
        altitude_timestamp = self.node.altitude_stamp
        err_altitude = self.node.get_parameter("err_altitude").value
        timeout = self.node.get_parameter("timeout").value * 1e9

        curtime = Clock(clock_type=ClockType.ROS_TIME).now().nanoseconds
        if abs(curtime - altitude_timestamp) > timeout:
            status.summary(
                DiagnosticStatus.STALE,
                f"No altitude data last {(curtime - altitude_timestamp) / 1e9:.2f} sec",
            )
        elif altitude > err_altitude:
            status.summary(DiagnosticStatus.ERROR, f"High altitude: {altitude} m")
        else:
            status.summary(DiagnosticStatus.OK, "Normal altitude")
        return status


def main():
    rclpy.init()
    node = WeatherDiagnosticsNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
