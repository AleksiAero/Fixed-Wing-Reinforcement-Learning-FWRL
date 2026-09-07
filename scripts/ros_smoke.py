"""Verify closed-loop ROS messaging and stale-command behavior."""
import json
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from fwrl_ros.nodes import Simulator, Controller

rclpy.init(args=['--ros-args', '-p', 'world:=worlds/training_airfield.json'])
sim, controller = Simulator(), Controller()
executor = SingleThreadedExecutor()
executor.add_node(sim)
executor.add_node(controller)
initial = sim.state.copy()
deadline = time.monotonic() + 2
while time.monotonic() < deadline:
    executor.spin_once(timeout_sec=.05)
distance = float(np.linalg.norm(sim.state[:3] - initial[:3]))
assert distance > 10, f'Controller did not move simulator: {distance}'
executor.remove_node(controller)
controller.destroy_node()
deadline = time.monotonic() + .7
while time.monotonic() < deadline:
    executor.spin_once(timeout_sec=.05)
frozen = sim.state.copy()
deadline = time.monotonic() + .2
while time.monotonic() < deadline:
    executor.spin_once(timeout_sec=.05)
assert np.array_equal(frozen, sim.state), 'Stale-command watchdog failed'
print(json.dumps({'ros_closed_loop': 'passed', 'distance_m': distance, 'watchdog': 'passed'}))
executor.remove_node(sim)
sim.destroy_node()
executor.shutdown()
rclpy.shutdown()
