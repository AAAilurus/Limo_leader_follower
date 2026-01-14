#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class RectSafetyFilter:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/follower/odom")
        self.cmd_in     = rospy.get_param("~cmd_in", "/follower/cmd_vel_raw")

        # IMPORTANT: follower driver listens to /cmd_vel (global)
        self.cmd_out    = rospy.get_param("~cmd_out", "/cmd_vel")

        # forbidden rectangle (in follower odom frame)
        self.xmin = float(rospy.get_param("~xmin", 1.0))
        self.xmax = float(rospy.get_param("~xmax", 3.0))
        self.ymin = float(rospy.get_param("~ymin", 1.0))
        self.ymax = float(rospy.get_param("~ymax", 3.0))
        self.margin = float(rospy.get_param("~margin", 0.30))

        self.lookahead = float(rospy.get_param("~lookahead", 1.0))
        self.turn_away_gain = float(rospy.get_param("~turn_away_gain", 1.5))
        self.min_forward = float(rospy.get_param("~min_forward", 0.06))

        self.last_odom = None
        self.last_cmd = Twist()

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber(self.cmd_in, Twist, self.cmd_cb, queue_size=20)
        self.pub = rospy.Publisher(self.cmd_out, Twist, queue_size=10)

        # RViz marker (optional)
        self.marker_pub = rospy.Publisher("~constraint_marker", Marker, queue_size=1, latch=True)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.step)
        rospy.loginfo("RectSafetyFilter ON. cmd_in=%s cmd_out=%s odom=%s",
                      self.cmd_in, self.cmd_out, self.odom_topic)

    def odom_cb(self, msg):
        self.last_odom = msg
        # publish marker in correct frame
        self.publish_marker(msg.header.frame_id)

    def cmd_cb(self, msg):
        self.last_cmd = msg

    def inside(self, x, y):
        return (self.xmin - self.margin <= x <= self.xmax + self.margin) and \
               (self.ymin - self.margin <= y <= self.ymax + self.margin)

    def yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def step(self, _evt):
        if self.last_odom is None:
            return

        x = self.last_odom.pose.pose.position.x
        y = self.last_odom.pose.pose.position.y
        yaw = self.yaw_from_quat(self.last_odom.pose.pose.orientation)

        v = float(self.last_cmd.linear.x)
        w = float(self.last_cmd.angular.z)

        # predict forward
        dt = self.lookahead
        if abs(w) < 1e-6:
            x_pred = x + v * math.cos(yaw) * dt
            y_pred = y + v * math.sin(yaw) * dt
        else:
            x_pred = x + (v / w) * (math.sin(yaw + w*dt) - math.sin(yaw))
            y_pred = y - (v / w) * (math.cos(yaw + w*dt) - math.cos(yaw))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w

        if self.inside(x, y) or self.inside(x_pred, y_pred):
            cx = 0.5*(self.xmin + self.xmax)
            cy = 0.5*(self.ymin + self.ymax)

            away = math.atan2(y - cy, x - cx)
            err = math.atan2(math.sin(away - yaw), math.cos(away - yaw))

            cmd.linear.x = max(self.min_forward, 0.3 * max(0.0, v))
            cmd.angular.z = clamp(w + self.turn_away_gain * err, -2.0, 2.0)

        self.pub.publish(cmd)

    def publish_marker(self, frame_id):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "constraints"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD

        m.pose.position.x = 0.5*(self.xmin + self.xmax)
        m.pose.position.y = 0.5*(self.ymin + self.ymax)
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0

        m.scale.x = (self.xmax - self.xmin) + 2.0*self.margin
        m.scale.y = (self.ymax - self.ymin) + 2.0*self.margin
        m.scale.z = 0.05

        m.color.r = 1.0
        m.color.a = 0.35

        self.marker_pub.publish(m)

if __name__ == "__main__":
    rospy.init_node("rect_safety_filter")
    RectSafetyFilter()
    rospy.spin()
