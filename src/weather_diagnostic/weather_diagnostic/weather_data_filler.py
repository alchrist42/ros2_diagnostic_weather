from random import random

from px4_msgs.msg import Wind, VehicleAirData, VehicleGlobalPosition
import rclpy
from rclpy.node import Node

WIND_TOPIC = "/fmu/out/wind"
AIR_TOPIC = "/fmu/out/VehicleAirData"
POSITION_TOPIC = "/fmu/out/VehicleGlobalPosition"


class DataFiller(Node):

    def __init__(self):
        super().__init__('data_filler')
        self.pub_wind = self.create_publisher(Wind, WIND_TOPIC, 10)
        self.pub_air = self.create_publisher(VehicleAirData, AIR_TOPIC, 10)
        self.pub_gpos = self.create_publisher(VehicleGlobalPosition, POSITION_TOPIC, 10)
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"send data from example")
        # stamp = self.get_clock().now().to_msg()
        msg_wind = Wind()
        msg_air = VehicleAirData()
        msg_gpos = VehicleGlobalPosition()

        level = random()
        msg_wind.tas_innov = float(16 * level)
        msg_air.baro_temp_celcius = float(60 * level)
        msg_gpos.alt = 3000 * float(level)


        if not (0.05 < level < 0.1 or 0.93 < level < 0.99):
            self.pub_wind.publish(msg_wind)
        self.pub_air.publish(msg_air)
        self.pub_gpos.publish(msg_gpos)


def main():
    rclpy.init()
    node = DataFiller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()