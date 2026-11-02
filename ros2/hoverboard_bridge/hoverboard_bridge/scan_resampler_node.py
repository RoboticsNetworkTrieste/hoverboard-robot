"""Republishes /scan on a fixed angular grid as /scan_fixed (slam_toolbox needs a constant beam count)."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from hoverboard_bridge.scan_resample import resample


class ScanResampler(Node):

    def __init__(self):
        super().__init__('scan_resampler')
        self.beams = self.declare_parameter('beams', 360).value
        # Flat [min, max, min, max, ...] in degrees within [-180, 180), LIDAR frame: masks parts of the robot
        sectors = self.declare_parameter('excluded_sectors_deg', Parameter.Type.DOUBLE_ARRAY).value or []
        if len(sectors) % 2 != 0:
            raise ValueError('excluded_sectors_deg needs pairs of angles')
        self.excluded = [(math.radians(sectors[i]), math.radians(sectors[i + 1])) for i in range(0, len(sectors), 2)]
        self.pub = self.create_publisher(LaserScan, 'scan_fixed', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

    def on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_increment = 2.0 * math.pi / self.beams
        out.angle_min = -math.pi
        out.angle_max = math.pi - out.angle_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = resample(msg.angle_min, msg.angle_increment, msg.ranges,
                              msg.range_min, msg.range_max, self.beams, self.excluded)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanResampler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
