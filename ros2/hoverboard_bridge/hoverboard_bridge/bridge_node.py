"""ROS2 node bridging /cmd_vel and robot telemetry to the ESP32-S3 over UDP."""
import math
import signal
import socket

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, JointState, Temperature
from tf2_ros import TransformBroadcaster

from hoverboard_bridge import imu_convert
from hoverboard_bridge import link_protocol as lp
from hoverboard_bridge.kinematics import (
    DiffDriveParams, OdometryIntegrator, TickContinuity, rpm_to_rad_s, twist_to_commands)

POSE_COVARIANCE_DIAG = (0.01, 0.01, 1e6, 1e6, 1e6, 0.05)
# Pose covariance is constant, so consumers should fuse the twist only; vy is exactly 0 for a diff drive
TWIST_COVARIANCE_DIAG = (0.01, 1e-3, 1e6, 1e6, 1e6, 0.05)
IMU_ROLL_PITCH_VARIANCE = 0.01
IMU_YAW_VARIANCE = 1e6  # Yaw is not absolute: 6-axis drifts, hub motor magnets spoil 9-axis
IMU_GYRO_VARIANCE = 0.001
IMU_ACC_VARIANCE = 0.05
LIDAR_TX_CHUNK = 512


def diagonal(values, size):
    matrix = [0.0] * (size * size)
    for i, value in enumerate(values):
        matrix[i * (size + 1)] = value
    return matrix


class BridgeNode(Node):

    def __init__(self):
        super().__init__('hoverboard_bridge')
        defaults = {
            'esp_host': '192.168.1.50',
            'esp_port': 8888,
            'wheel_radius': 0.0825,
            'wheel_separation': 0.40,
            'ticks_per_rev': 90,
            'n_mot_max': 200.0,
            'max_cmd': 1000,
            'left_cmd_sign': 1,
            'right_cmd_sign': 1,
            'left_ticks_sign': 1,
            'right_ticks_sign': 1,
            'left_rpm_sign': 1,
            'right_rpm_sign': -1,
            'cmd_rate': 20.0,
            'cmd_vel_timeout': 0.5,
            'link_timeout': 1.0,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'imu_frame': 'imu_link',
            'publish_tf': True,
            'enable_lidar': True,
            'lidar_pty_link': '/tmp/ttyLIDAR',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ('left_cmd_sign', 'right_cmd_sign', 'left_ticks_sign', 'right_ticks_sign',
                     'left_rpm_sign', 'right_rpm_sign'):
            if self._param(name) not in (-1, 1):
                raise ValueError(f'{name} must be -1 or 1')

        self.kin = DiffDriveParams(self._param('wheel_radius'), self._param('wheel_separation'),
                                   self._param('ticks_per_rev'), self._param('n_mot_max'), self._param('max_cmd'))
        self.odom = OdometryIntegrator(self.kin)
        self.continuity = TickContinuity()
        self.cmd_signs = (self._param('left_cmd_sign'), self._param('right_cmd_sign'))
        self.ticks_signs = (self._param('left_ticks_sign'), self._param('right_ticks_sign'))
        self.rpm_signs = (self._param('left_rpm_sign'), self._param('right_rpm_sign'))
        self.cmd_vel_timeout_ns = int(self._param('cmd_vel_timeout') * 1e9)
        self.link_timeout_ns = int(self._param('link_timeout') * 1e9)
        self.odom_frame = self._param('odom_frame')
        self.base_frame = self._param('base_frame')
        self.imu_frame = self._param('imu_frame')

        host = self._param('esp_host')
        self.esp_addr = (socket.gethostbyname(host), self._param('esp_port'))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        self.seq = 0
        self.twist = (0.0, 0.0)
        self.last_cmd_vel_time = None
        self.last_rx_time = None
        self.link_up = False
        self.last_status = None
        self.imu_seen = False
        self.imu_ok = False
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.start_time = self.steady_clock.now()
        self.no_rx_warned = False

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'hoverboard/battery', 10)
        self.temp_pub = self.create_publisher(Temperature, 'hoverboard/temperature', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self._param('publish_tf') else None
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.lidar = None
        if self._param('enable_lidar'):
            from hoverboard_bridge.lidar_pty import LidarPty
            self.lidar = LidarPty(self._param('lidar_pty_link'))
            self.get_logger().info(f'LIDAR stream at {self.lidar.link_path}')

        self.create_timer(1.0 / self._param('cmd_rate'), self.send_command)
        self.create_timer(0.005, self.poll)
        self.create_timer(1.0, self.check_link)
        self.get_logger().info(f'Bridging to ESP at {host} ({self.esp_addr[0]}:{self.esp_addr[1]})')

    def _param(self, name):
        return self.get_parameter(name).value

    def _send(self, datagram):
        try:
            self.sock.sendto(datagram, self.esp_addr)
        except OSError as err:
            self.get_logger().warn(f'UDP send failed: {err}', throttle_duration_sec=5.0)

    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq

    def on_cmd_vel(self, msg):
        self.twist = (msg.linear.x, msg.angular.z)
        self.last_cmd_vel_time = self.steady_clock.now()

    def send_command(self):
        left = right = 0
        now = self.steady_clock.now()
        if self.last_cmd_vel_time is not None and \
                (now - self.last_cmd_vel_time).nanoseconds < self.cmd_vel_timeout_ns:
            left, right = twist_to_commands(self.kin, *self.twist)
            left *= self.cmd_signs[0]
            right *= self.cmd_signs[1]
        self._send(lp.encode_cmd_wheels(self._next_seq(), left, right))

    def poll(self):
        for _ in range(200):  # Bounded so the executor stays responsive
            try:
                data, source = self.sock.recvfrom(lp.MAX_DATAGRAM)
            except (BlockingIOError, ConnectionResetError):
                break
            if source[0] != self.esp_addr[0]:
                continue
            msg = lp.decode(data)
            if msg is None:
                continue
            self.last_rx_time = self.steady_clock.now()
            if msg.type == lp.MSG_WHEEL_STATE:
                state = lp.parse_wheel_state(msg.body)
                if state is not None:
                    self.publish_wheel_state(msg.stamp_ms, state)
            elif msg.type == lp.MSG_IMU:
                imu = lp.parse_imu(msg.body)
                if imu is not None:
                    self.publish_imu(imu)
            elif msg.type == lp.MSG_LIDAR_RX and self.lidar is not None:
                self.lidar.write(msg.body)
            elif msg.type == lp.MSG_LOG:
                self.get_logger().info('[esp] ' + msg.body.decode('ascii', errors='replace').rstrip())

        if self.lidar is not None:
            while True:
                chunk = self.lidar.read(LIDAR_TX_CHUNK)
                if not chunk:
                    break
                self._send(lp.encode(lp.MSG_LIDAR_TX, self._next_seq(), 0, chunk))

    def publish_wheel_state(self, stamp_ms, state):
        stamp = self.get_clock().now().to_msg()
        ticks_l = state.ticks_l * self.ticks_signs[0]
        ticks_r = state.ticks_r * self.ticks_signs[1]
        ticks_l, ticks_r, rebooted = self.continuity.update(stamp_ms, ticks_l, ticks_r)
        if rebooted:
            self.get_logger().warn('ESP restarted, odometry kept continuous')
        w_left = rpm_to_rad_s(state.rpm_l * self.rpm_signs[0])
        w_right = rpm_to_rad_s(state.rpm_r * self.rpm_signs[1])
        self.odom.update(ticks_l, ticks_r)
        linear = (w_left + w_right) * self.kin.wheel_radius / 2.0
        angular = (w_right - w_left) * self.kin.wheel_radius / self.kin.wheel_separation
        qz, qw = math.sin(self.odom.theta / 2.0), math.cos(self.odom.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom.x
        odom.pose.pose.position.y = self.odom.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = diagonal(POSE_COVARIANCE_DIAG, 6)
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        odom.twist.covariance = diagonal(TWIST_COVARIANCE_DIAG, 6)
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.odom.x
            tf.transform.translation.y = self.odom.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['left_wheel_joint', 'right_wheel_joint']
        joints.position = [self.odom.ticks_to_angle(ticks_l), self.odom.ticks_to_angle(ticks_r)]
        joints.velocity = [w_left, w_right]
        self.joint_pub.publish(joints)

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = state.bat_cv / 100.0
        battery.current = battery.charge = battery.capacity = battery.design_capacity = math.nan
        battery.percentage = battery.temperature = math.nan
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery.present = True
        self.battery_pub.publish(battery)

        temperature = Temperature()
        temperature.header.stamp = stamp
        temperature.temperature = state.temp_dc / 10.0
        self.temp_pub.publish(temperature)

        self.report_status_changes(state.status)

    def report_status_changes(self, status):
        previous = self.last_status
        hover_ok = bool(status & lp.STATUS_HOVER_OK)
        if previous is None or hover_ok != bool(previous & lp.STATUS_HOVER_OK):
            if hover_ok:
                self.get_logger().info('Hoverboard feedback OK')
            else:
                self.get_logger().warn('No hoverboard feedback on the ESP (check wiring / power)')
        imu_ok = bool(status & lp.STATUS_IMU_OK)
        if imu_ok != self.imu_ok:
            if imu_ok:
                self.imu_seen = True
                self.get_logger().info('IMU data OK')
            elif self.imu_seen:
                self.get_logger().warn('IMU data lost')
            self.imu_ok = imu_ok
        self.last_status = status

    def publish_imu(self, imu):
        roll, pitch, yaw = (imu_convert.angle_to_rad(a) for a in imu.angle)
        x, y, z, w = imu_convert.euler_to_quaternion(roll, pitch, yaw)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = x, y, z, w
        msg.orientation_covariance = diagonal([IMU_ROLL_PITCH_VARIANCE, IMU_ROLL_PITCH_VARIANCE, IMU_YAW_VARIANCE], 3)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            imu_convert.gyro_to_rad_s(g) for g in imu.gyro)
        msg.angular_velocity_covariance = diagonal([IMU_GYRO_VARIANCE] * 3, 3)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
            imu_convert.acc_to_m_s2(a) for a in imu.acc)
        msg.linear_acceleration_covariance = diagonal([IMU_ACC_VARIANCE] * 3, 3)
        self.imu_pub.publish(msg)

    def check_link(self):
        now = self.steady_clock.now()
        if self.last_rx_time is None:
            if not self.no_rx_warned and (now - self.start_time).nanoseconds >= self.link_timeout_ns:
                self.no_rx_warned = True
                self.get_logger().warn(f'No telemetry from ESP at {self.esp_addr[0]}:{self.esp_addr[1]}')
            return
        up = (now - self.last_rx_time).nanoseconds < self.link_timeout_ns
        if up != self.link_up:
            if up:
                self.get_logger().info('ESP telemetry received')
            else:
                self.get_logger().warn(f'No telemetry from ESP at {self.esp_addr[0]}:{self.esp_addr[1]}')
            self.link_up = up

    def shutdown(self):
        try:
            for _ in range(3):
                self._send(lp.encode_cmd_wheels(self._next_seq(), 0, 0))
        finally:
            if self.lidar is not None:
                self.lidar.close()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A second SIGINT forwarded by ros2 launch must not interrupt the cleanup
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
