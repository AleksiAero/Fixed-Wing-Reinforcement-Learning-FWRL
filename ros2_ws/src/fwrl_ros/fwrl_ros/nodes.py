import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, TwistStamped
from std_msgs.msg import String
from fwrl.analysis import analyze_files
from fwrl.dynamics import step, guidance
from fwrl.world import load_world, collision


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class Simulator(Node):
    def __init__(self):
        super().__init__('fwrl_simulator')
        self.declare_parameter('world', '')
        self.world = load_world(self.get_parameter('world').value)
        self.state = np.array([*self.world['start'], 0., 22., 0., 0.])
        self.command = np.array([0., 0., 22.])
        self.received = None
        self.index = 0
        self.ended = False
        self.pub = self.create_publisher(Odometry, '/fwrl/odometry', 10)
        self.target_pub = self.create_publisher(PointStamped, '/fwrl/waypoint', 10)
        self.status_pub = self.create_publisher(String, '/fwrl/status', 10)
        self.create_subscription(TwistStamped, '/fwrl/command', self.on_command, 10)
        self.create_timer(.05, self.tick)

    def on_command(self, msg):
        command = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.linear.x])
        if msg.header.frame_id == 'fwrl_setpoint' and np.isfinite(command).all():
            self.command = command
            self.received = time.monotonic()

    def tick(self):
        if self.ended:
            return
        stamp = self.get_clock().now().to_msg()
        # Pause physics until a controller is connected; stale commands freeze the research simulator.
        active = self.received is not None and time.monotonic() - self.received < .5
        if active:
            self.state = step(self.state, self.command)
            if collision(self.state[:3], self.world):
                self.ended = True
                status = 'collision'
            elif np.linalg.norm(self.state[:3] - self.world['waypoints'][self.index]) < self.world['goal_radius_m']:
                self.index += 1
                self.ended = self.index == len(self.world['waypoints'])
                status = 'success' if self.ended else 'running'
            else:
                status = 'running'
        else:
            status = 'paused: waiting for fresh controller command'
        self.status_pub.publish(String(data=json.dumps({'state': status, 'waypoints_reached': self.index})))
        s = self.state
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'map', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, s[:3])
        # FLU body: positive Euler pitch points nose down; positive guidance bank turns left.
        q = quaternion(-s[5], -s[6], s[3])
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        msg.twist.twist.linear.x = float(s[4])
        self.pub.publish(msg)
        if not self.ended:
            target = PointStamped()
            target.header.stamp, target.header.frame_id = stamp, 'map'
            target.point.x, target.point.y, target.point.z = map(float, self.world['waypoints'][self.index])
            self.target_pub.publish(target)
        else:
            self.get_logger().info(status)


class Controller(Node):
    def __init__(self):
        super().__init__('fwrl_controller')
        self.target = None
        self.target_time = None
        self.pub = self.create_publisher(TwistStamped, '/fwrl/command', 10)
        self.create_subscription(PointStamped, '/fwrl/waypoint', self.on_target, 10)
        self.create_subscription(Odometry, '/fwrl/odometry', self.on_state, 10)

    def on_target(self, msg):
        if msg.header.frame_id == 'map':
            self.target = [msg.point.x, msg.point.y, msg.point.z]
            self.target_time = time.monotonic()

    def on_state(self, msg):
        if self.target is None or time.monotonic() - self.target_time > .5:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        state = np.array([p.x, p.y, p.z, yaw, msg.twist.twist.linear.x, 0., 0.])
        command = guidance(state, self.target)
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'fwrl_setpoint'
        out.twist.angular.x, out.twist.angular.y, out.twist.linear.x = map(float, command)
        self.pub.publish(out)


class Analyzer(Node):
    def __init__(self):
        super().__init__('fwrl_analyzer')
        self.declare_parameter('input_dir', '')
        self.pub = self.create_publisher(String, '/fwrl/analysis', 10)
        self.create_timer(5., self.analyze)

    def analyze(self):
        folder = self.get_parameter('input_dir').value
        if not folder:
            return
        try:
            report = analyze_files(sorted(Path(folder).glob('*.csv')))
            self.pub.publish(String(data=json.dumps(report)))
        except (ValueError, OSError) as exc:
            self.get_logger().error(str(exc))


def run(cls):
    rclpy.init()
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def simulator_main(): run(Simulator)
def controller_main(): run(Controller)
def analyzer_main(): run(Analyzer)
