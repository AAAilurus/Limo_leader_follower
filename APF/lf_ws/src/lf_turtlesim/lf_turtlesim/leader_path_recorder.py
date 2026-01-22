import csv
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


class LeaderPathRecorder(Node):
    def __init__(self):
        super().__init__("leader_path_recorder")

        # Stream recorded poses for follower
        self.pub_path_pose = self.create_publisher(PoseStamped, "/leader/path_pose", 50)

        self.sub_pose = self.create_subscription(Pose, "/turtle1/pose", self.cb_pose, 50)

        # CSV logging
        self.csv_path = "leader_path.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["stamp_sec", "stamp_nsec", "x", "y", "theta"])

        self.last_pose = None
        self.get_logger().info(f"Recording leader path to {self.csv_path} and publishing /leader/path_pose")

    def cb_pose(self, msg: Pose):
        now = self.get_clock().now().to_msg()

        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "turtlesim_world"  # informational only (no TF used)

        ps.pose.position.x = float(msg.x)
        ps.pose.position.y = float(msg.y)

        # store theta in quaternion (z,w) for convention
        # yaw -> quaternion (0,0,sin(y/2),cos(y/2))
        yaw = float(msg.theta)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub_path_pose.publish(ps)

        # Log to CSV
        self.writer.writerow([now.sec, now.nanosec, msg.x, msg.y, msg.theta])
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = LeaderPathRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
