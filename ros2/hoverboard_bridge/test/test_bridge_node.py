"""BridgeNode against a fake ESP on localhost (needs rclpy: run in the container)."""
import math
import socket
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

from hoverboard_bridge import link_protocol as lp  # noqa: E402
from hoverboard_bridge.bridge_node import BridgeNode  # noqa: E402

FORWARD_0_5 = (289, 289)  # Default parameters, linear.x = 0.5 m/s (see test_kinematics.py)


@pytest.fixture
def esp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    sock.setblocking(False)
    yield sock
    sock.close()


@pytest.fixture
def bridge(esp):
    rclpy.init(args=['--ros-args', '-p', 'esp_host:=127.0.0.1', '-p', f'esp_port:={esp.getsockname()[1]}',
                     '-p', 'enable_lidar:=false', '-p', 'publish_tf:=false'])
    node = BridgeNode()
    helper = rclpy.create_node('test_helper')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    yield helper, executor
    executor.shutdown()
    helper.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def spin_until(executor, predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return True
    return False


def receive_commands(esp):
    """Returns [((left, right), sender)] for the CMD_WHEELS datagrams waiting on the fake ESP."""
    commands = []
    while True:
        try:
            data, sender = esp.recvfrom(lp.MAX_DATAGRAM)
        except BlockingIOError:
            return commands
        msg = lp.decode(data)
        if msg is not None and msg.type == lp.MSG_CMD_WHEELS:
            commands.append((lp.CMD_WHEELS.unpack(msg.body), sender))


def wheel_state(stamp_ms, ticks_l, ticks_r):
    body = lp.WHEEL_STATE.pack(ticks_l, ticks_r, 0, 0, 4000, 300, lp.STATUS_HOVER_OK, 0, 0)
    return lp.encode(lp.MSG_WHEEL_STATE, 0, stamp_ms, body)


def test_cmd_vel_becomes_wheel_command(bridge, esp):
    helper, executor = bridge
    pub = helper.create_publisher(Twist, 'cmd_vel', 10)
    twist = Twist()
    twist.linear.x = 0.5
    seen = []

    def forward_seen():
        pub.publish(twist)
        seen.extend(cmd for cmd, _ in receive_commands(esp))
        return FORWARD_0_5 in seen

    assert spin_until(executor, forward_seen)


def test_commands_stop_after_cmd_vel_timeout(bridge, esp):
    helper, executor = bridge
    pub = helper.create_publisher(Twist, 'cmd_vel', 10)
    twist = Twist()
    twist.linear.x = 0.5
    pub.publish(twist)
    assert spin_until(executor, lambda: any(cmd == FORWARD_0_5 for cmd, _ in receive_commands(esp)))

    seen = []
    spin_until(executor, lambda: seen.extend(cmd for cmd, _ in receive_commands(esp)) and False, timeout=1.0)
    assert seen and seen[-1] == (0, 0)  # cmd_vel_timeout is 0.5 s


def test_wheel_state_becomes_odometry(bridge, esp):
    helper, executor = bridge
    odoms = []
    helper.create_subscription(Odometry, 'odom', odoms.append, 10)
    senders = []
    assert spin_until(executor, lambda: senders.extend(s for _, s in receive_commands(esp)) or senders)

    esp.sendto(wheel_state(1000, 0, 0), senders[0])
    esp.sendto(wheel_state(1020, 90, 90), senders[0])  # One wheel revolution
    assert spin_until(executor, lambda: odoms and odoms[-1].pose.pose.position.x > 0.0)
    assert odoms[-1].pose.pose.position.x == pytest.approx(2 * math.pi * 0.0825)
