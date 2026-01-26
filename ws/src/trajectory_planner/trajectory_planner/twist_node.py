import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

import numpy as np
from trajectory_planner.core.factory import make_interpolator
from trajectory_planner.core.types import Waypoint

class InterpTrajectoryNode(Node):
    def __init__(self):
        super().__init__('interp_trajectory_node')

        # Params
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('start_now', True)
        self.declare_parameter('interpolator', 'catmull_rom')
        self.declare_parameter('waypoints', [])  # list of dicts via YAML

        self.frame_id = self.get_parameter('frame_id').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.dt = float(self.get_parameter('dt').value)
        self.start_now = bool(self.get_parameter('start_now').value)
        self.interpolator_name = self.get_parameter('interpolator').value
        wp_list = self.get_parameter('waypoints').value
        self.waypoints = self._parse_waypoints(wp_list)

        self.interpolator = make_interpolator(self.interpolator_name)
        self.interpolator.set_waypoints(self.waypoints)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 1)

        self.t0 = self.get_clock().now() if self.start_now else None
        self.timer = self.create_timer(self.dt, self._tick)

        # Publish full path once for RViz
        self._publish_path()

        self.get_logger().info(
            f"Publishing cmd on {self.cmd_topic}, path on {self.path_topic}, dt={self.dt}s, Nwp={len(self.waypoints)}"
        )

    def _parse_waypoints(self, wp_list):
        if not wp_list or len(wp_list) < 2:
            raise RuntimeError("Parameter 'waypoints' must contain at least 2 waypoints")

        wps = []
        for w in wp_list:
            p = np.array([float(w['x']), float(w['y']), float(w.get('z', 0.0))], dtype=float)
            wps.append(Waypoint(
                t=float(w['t']),
                p=p,
                yaw=float(w.get('yaw', 0.0)),
            ))
        # set_waypoints() will sort/check monotonic time, but we can keep it clean:
        wps.sort(key=lambda ww: ww.t)
        return wps

    def _publish_path(self):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        for w in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = msg.header.stamp
            ps.pose.position.x = w.x
            ps.pose.position.y = w.y
            ps.pose.position.z = w.z
            # yaw -> orientation could be added later
            msg.poses.append(ps)

        self.path_pub.publish(msg)

    def _tick(self):
        if self.t0 is None:
            self.t0 = self.get_clock().now()

        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        sample = self.interpolator.sample(t)

        cmd = TwistStamped()
        cmd.header.frame_id = self.frame_id
        cmd.header.stamp = now.to_msg()
        cmd.twist.linear.x = float(sample.v[0])
        cmd.twist.linear.y = float(sample.v[1])
        cmd.twist.linear.z = float(sample.v[2])
        cmd.twist.angular.z = float(sample.yaw_rate)

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = InterpTrajectoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
