import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from trajectory_planner.core.factory import make_interpolator
from trajectory_planner.core.types import Waypoint


def yaw_to_quat(yaw: float):
    # planar yaw quaternion
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)

def clamp(a, lo, hi):
    return max(lo, min(hi, a))

def interp_pose(waypoints, t: float):
    if len(waypoints) < 2:
        raise ValueError("Need at least 2 waypoints")

    if t <= waypoints[0].t:
        return waypoints[0]
    if t >= waypoints[-1].t:
        return waypoints[-1]

    i = 0
    while i < len(waypoints) - 1 and not (waypoints[i].t <= t <= waypoints[i+1].t):
        i += 1

    w0, w1 = waypoints[i], waypoints[i+1]
    dt = max(1e-9, (w1.t - w0.t))
    a = clamp((t - w0.t) / dt, 0.0, 1.0)

    # position
    x = w0.x + a * (w1.x - w0.x)
    y = w0.y + a * (w1.y - w0.y)
    z = w0.z + a * (w1.z - w0.z)

    # yaw unwrap + interp
    dyaw = w1.yaw - w0.yaw
    while dyaw > math.pi: dyaw -= 2*math.pi
    while dyaw < -math.pi: dyaw += 2*math.pi
    yaw = w0.yaw + a * dyaw

    return Waypoint(t=t, x=x, y=y, z=z, yaw=yaw)

class InterpPoseNode(Node):
    def __init__(self):
        super().__init__('interp_pose_node')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pose_topic', '/setpoint_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('start_now', True)
        self.declare_parameter('interpolator', 'catmull_rom')
        self.declare_parameter('waypoints', [])

        self.frame_id = self.get_parameter('frame_id').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.dt = float(self.get_parameter('dt').value)
        self.start_now = bool(self.get_parameter('start_now').value)
        self.interpolator_name = self.get_parameter('interpolator').value
        wp_list = self.get_parameter('waypoints').value
        self.waypoints = self._parse_waypoints(wp_list)
        self.interpolator = make_interpolator(self.interpolator_name)
        self.interpolator.set_waypoints(self.waypoints)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 1)

        self.t0 = self.get_clock().now() if self.start_now else None
        self.timer = self.create_timer(self.dt, self._tick)

        self._publish_path()
        self.get_logger().info(
            f"Publishing pose setpoints on {self.pose_topic}, dt={self.dt}s, Nwp={len(self.waypoints)}"
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
            qx, qy, qz, qw = yaw_to_quat(w.yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            msg.poses.append(ps)

        self.path_pub.publish(msg)

    def _tick(self):
        if self.t0 is None:
            self.t0 = self.get_clock().now()

        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        sample = self.interpolator.sample(t)

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = now.to_msg()
        msg.pose.position.x = float(sample.p[0])
        msg.pose.position.y = float(sample.p[1])
        msg.pose.position.z = float(sample.p[2])
        qx, qy, qz, qw = yaw_to_quat(sample.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_pub.publish(msg)

def main():
    rclpy.init()
    node = InterpPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
