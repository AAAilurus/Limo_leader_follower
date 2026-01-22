import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Deque, Tuple

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose as TurtlePose
from geometry_msgs.msg import Twist, PoseStamped

from lf_turtlesim.safety_filter_core import wrap_to_pi


@dataclass
class Stamped2DPose:
    t_sec: int
    t_nsec: int
    x: float
    y: float
    yaw: float


def quat_to_yaw(qz: float, qw: float) -> float:
    # yaw from quaternion (0,0,z,w)
    return 2.0 * math.atan2(qz, qw)


def time_to_float(t_sec: int, t_nsec: int) -> float:
    return float(t_sec) + 1e-9 * float(t_nsec)


class FollowerPathTracker(Node):
    def __init__(self):
        super().__init__("follower_path_tracker")

        # Delay requirement (3–5 seconds)
        self.delay_sec = 4.0

        # Start follower only after enough history exists
        self.min_history_sec = 3.0

        # Buffer of leader poses
        self.buf: Deque[Stamped2DPose] = deque(maxlen=30000)  # ~10min at 50Hz

        self.leader_started = False
        self.follower_pose: Optional[TurtlePose] = None

        # Controller gains (pose tracking, not cmd replay)
        self.k_r = 1.2          # distance gain
        self.k_alpha = 4.0      # heading-to-target gain
        self.k_beta = -1.2      # final heading gain (negative stabilizes)

        self.v_max = 2.0
        self.w_max = 4.0

        self.sub_leader_path = self.create_subscription(PoseStamped, "/leader/path_pose", self.cb_leader_pose, 50)
        self.sub_follower_pose = self.create_subscription(TurtlePose, "/turtle2/pose", self.cb_follower_pose, 50)

        self.pub_raw = self.create_publisher(Twist, "/follower/cmd_vel_raw", 10)

        self.timer = self.create_timer(0.02, self.step)  # 50 Hz
        self.get_logger().info("Follower path tracker running. Output: /follower/cmd_vel_raw")

    def cb_leader_pose(self, msg: PoseStamped):
        yaw = quat_to_yaw(msg.pose.orientation.z, msg.pose.orientation.w)
        sp = Stamped2DPose(
            t_sec=msg.header.stamp.sec,
            t_nsec=msg.header.stamp.nanosec,
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            yaw=yaw,
        )
        self.buf.append(sp)

    def cb_follower_pose(self, msg: TurtlePose):
        self.follower_pose = msg

    def get_delayed_target(self, now_sec: int, now_nsec: int) -> Optional[Stamped2DPose]:
        """
        Returns the most recent leader pose with timestamp <= now - delay.
        """
        if len(self.buf) < 10:
            return None

        now = time_to_float(now_sec, now_nsec)
        target_time = now - self.delay_sec

        # Ensure enough history exists before starting
        oldest = time_to_float(self.buf[0].t_sec, self.buf[0].t_nsec)
        newest = time_to_float(self.buf[-1].t_sec, self.buf[-1].t_nsec)
        if (newest - oldest) < self.min_history_sec:
            return None

        # Find best index from the back (fast)
        for i in range(len(self.buf) - 1, -1, -1):
            ti = time_to_float(self.buf[i].t_sec, self.buf[i].t_nsec)
            if ti <= target_time:
                return self.buf[i]

        # If delay is too large, nothing found
        return None

    def step(self):
        if self.follower_pose is None:
            return
        if len(self.buf) < 10:
            return

        now_msg = self.get_clock().now().to_msg()
        target = self.get_delayed_target(now_msg.sec, now_msg.nanosec)
        if target is None:
            # Not enough history yet: publish zero
            cmd = Twist()
            self.pub_raw.publish(cmd)
            return

        # Pose tracking controller (unicycle pose stabilization)
        x = float(self.follower_pose.x)
        y = float(self.follower_pose.y)
        th = float(self.follower_pose.theta)

        xd, yd, thd = target.x, target.y, target.yaw

        dx = xd - x
        dy = yd - y

        # Transform error to follower frame
        ex = math.cos(th) * dx + math.sin(th) * dy
        ey = -math.sin(th) * dx + math.cos(th) * dy

        rho = math.hypot(ex, ey)
        alpha = wrap_to_pi(math.atan2(ey, ex))
        beta = wrap_to_pi(thd - th - alpha)

        # Key idea to avoid corner cutting:
        # - v is reduced when alpha is large (so robot turns first)
        # - using thd in beta helps match the recorded heading
        v = self.k_r * rho * math.cos(alpha)
        w = self.k_alpha * alpha + self.k_beta * beta

        # Clamp
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_raw.publish(cmd)


def main():
    rclpy.init()
    node = FollowerPathTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
