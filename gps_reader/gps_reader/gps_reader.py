import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpsdclient import GPSDClient
from sensor_msgs.msg import NavSatFix, NavSatStatus

def safe_float(value):
    try:
        return float(value)
    except ValueError:
        print(f"Could not convert '{value}' to float.")
        return -1.0

class GpsReader(Node):
    def __init__(self):
        super().__init__('gps_reader')
        self.publisher_ = self.create_publisher(NavSatFix, '/ugv/gps/fix', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):

        with GPSDClient() as client:
            for result in client.dict_stream(convert_datetime=True, filter=["TPV"]):
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'gps_ugv'

                msg.status.status = NavSatStatus.STATUS_FIX
                msg.status.service = NavSatStatus.SERVICE_GPS

                msg.latitude = safe_float(result.get("lat", "n/a"))
                msg.longitude = safe_float(result.get("lon", "n/a"))
                msg.altitude = safe_float(result.get("alt", "n/a"))

                msg.position_covariance = [0.5, 0.0, 0.0,
                                        0.0, 0.5, 0.0,
                                        0.0, 0.0, 1.0]
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
