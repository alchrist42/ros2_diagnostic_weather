import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import Wind, VehicleAirData, VehicleGlobalPosition

class WeatherDiagnosticsNode(Node):

    def __init__(self):
        super().__init__('weather_diagnostics_native')
        self.wind_speed = 0
        
        # Initialize ROS parameters for thresholds
        self.declare_parameters(
            namespace='',
            parameters=[
                ('err_altitude', 2500.0),
                ('warn_wind_speed', 13.0),
                ('err_wind_speed', 15.0),
                ('warn_min_temperature', -30.0),
                ('err_min_temperature', -40.0),
                ('warn_max_temperature', 40.0),
                ('err_max_temperature', 40.0),
            ]
        )
        # Initialize diagnostic messages
        self.diagnostic_array = DiagnosticArray()

        # Subscribe to input topics
        self.wind_sub = self.create_subscription(Wind, '/fmu/out/wind', self.wind_callback, 10)
        self.air_data_sub = self.create_subscription(VehicleAirData, '/fmu/out/VehicleAirData', self.air_data_callback, 10)
        self.global_position_sub = self.create_subscription(VehicleGlobalPosition, '/fmu/out/VehicleGlobalPosition', self.global_position_callback, 10)

        self.diagnostic_publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)

    def wind_callback(self, msg):
        # Check wind speed
        max_wind_speed = self.get_parameter('err_wind_speed').value
        warn_wind_speed = self.get_parameter('warn_wind_speed').value

        wind_speed = (msg.windspeed_north ** 2 + msg.windspeed_east ** 2) ** 0.5    
        if wind_speed > max_wind_speed:
            status = DiagnosticStatus.ERROR
            message = f"Excessive wind speed: {wind_speed} m/s"
        elif wind_speed > warn_wind_speed:
            status = DiagnosticStatus.WARN
            message = f"High wind speed: {wind_speed} m/s"
        else: # TODO add low border wind speed
            status = DiagnosticStatus.OK
            message = "Normal wind speed"
        
        # Create diagnostic message
        wind_status = DiagnosticStatus()
        wind_status.name = "Wind Status"
        wind_status.level = status
        wind_status.message = message
        wind_status.values.append(KeyValue(key="Wind Speed", value=str(wind_speed)))

        # Append to diagnostic array
        self.diagnostic_array.status.append(wind_status)

    def air_data_callback(self, msg):
        # Check temperature
        warn_min_temperature = self.get_parameter('warn_min_temperature').value
        err_min_temperature = self.get_parameter('err_min_temperature').value
        warn_max_temperature = self.get_parameter('warn_max_temperature').value
        err_max_temperature = self.get_parameter('err_max_temperature').value
        temperature = msg.baro_temp_celcius
        
        if temperature < err_min_temperature or temperature > err_max_temperature:
            status = DiagnosticStatus.ERROR
            message = f"Extreme temperature: {temperature} °C"
        elif temperature < warn_min_temperature or temperature > warn_max_temperature:
            status = DiagnosticStatus.WARN
            message = f"Uncomfortable temperature: {temperature} °C"
        else:
            status = DiagnosticStatus.OK
            message = "Comfortable temperature"
        
        # Create diagnostic message
        air_status = DiagnosticStatus()
        air_status.name = "Air Temperature Status"
        air_status.level = status
        air_status.message = message
        air_status.values.append(KeyValue(key="Temperature", value=str(temperature)))

        # Append to diagnostic array
        self.diagnostic_array.status.append(air_status)

    def global_position_callback(self, msg):
        # Check altitude
        err_altitude = self.get_parameter('err_altitude').value
        altitude = msg.alt

        if altitude > err_altitude:
            status = DiagnosticStatus.ERROR
            message = f"High altitude: {altitude} m"
        else:
            status = DiagnosticStatus.OK
            message = "Normal altitude"
        
        # Create diagnostic message
        altitude_status = DiagnosticStatus()
        altitude_status.name = "Altitude Status"
        altitude_status.level = status
        altitude_status.message = message
        altitude_status.values.append(KeyValue(key="Altitude", value=str(altitude)))

        # Append to diagnostic array
        self.diagnostic_array.status.append(altitude_status)

    def publish_diagnostics(self):
        # Publish diagnostic array
        self.diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        self.diagnostic_array.header.frame_id = "weather_diagnostics"
        # self.diagnostic_array.status.sort(key=lambda x: x.name)
        self.diagnostic_publisher.publish(self.diagnostic_array)
        self.diagnostic_array = DiagnosticArray()

def main(args=None):
    rclpy.init(args=args)
    node = WeatherDiagnosticsNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()