import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from lf_turtlesim.safety_filter_core import Rect, SafetyParams, apply_safety_filter_unicycle


class LeaderSafetyFilter(Node):
    def __init__(self):
        super().__init__("leader_safety_filter")

        # Forbidden rectangles in world coordinates (edit as needed)
        # Example: two rectangles in the middle area
        self.rects = [
            Rect(xmin=4.0, xmax=6.0, ymin=4.0, ymax=6.0),
            Rect(xmin=7.5, xmax=9.5, ymin=1.0, ymax=3.0),
        ]

        # turtlesim typical world roughly [0, 11] x [0, 11]
        self.world = Rect(xmin=0.0, xmax=11.0, ymin=0.0, ymax=11.0)

        self.params = SafetyParams(
            d_safe=0.45,
            alpha=2.0,
            k_rep=2.0,
            rep_range=1.25,
            v_min=-1.5,
            v_max=2.0,
            w_min=-4.0,
            w_max=4.0,
        )

        self.pose = None
        self.last_raw = Twist()

        self.sub_pose = self.create_subscription(Pose, "/turtle1/pose", self.cb_pose, 10)
        self.sub_raw = self.create_subscription(Twist, "/leader/cmd_vel_raw", self.cb_raw, 10)
        self.pub_safe = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.timer = self.create_timer(0.02, self.step)  # 50 Hz safety filtering loop
        self.get_logger().info("Leader safety filter running. Filtering /leader/cmd_vel_raw -> /turtle1/cmd_vel")

    def cb_pose(self, msg: Pose):
        self.pose = msg

    def cb_raw(self, msg: Twist):
        self.last_raw = msg

    def step(self):
        if self.pose is None:
            return

        v_des = float(self.last_raw.linear.x)
        w_des = float(self.last_raw.angular.z)

        v_safe, w_safe = apply_safety_filter_unicycle(
            px=self.pose.x,
            py=self.pose.y,
            theta=self.pose.theta,
            v_des=v_des,
            w_des=w_des,
            rects=self.rects,
            world_bounds=self.world,
            params=self.params
        )

        cmd = Twist()
        cmd.linear.x = v_safe
        cmd.angular.z = w_safe
        self.pub_safe.publish(cmd)


def main():
    rclpy.init()
    node = LeaderSafetyFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

