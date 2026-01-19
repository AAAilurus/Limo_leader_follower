Make sure ROS2 Humble is sourced in every terminal:

source /opt/ros/humble/setup.bash

1) Run turtlesim with two turtles (test first)
Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

Terminal 2: Spawn turtle2
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"

Terminal 3: Teleop leader (turtle1)
ros2 run turtlesim turtle_teleop_key


✅ Confirm you can move turtle1 and turtle2 exists.

2) Create a workspace and package (from scratch)
2.1 Create workspace folders
mkdir -p ~/apf_ws/src
cd ~/apf_ws/src

2.2 Create Python package
ros2 pkg create apf_leader_follower --build-type ament_python --dependencies rclpy turtlesim geometry_msgs


You now have:

~/apf_ws/src/apf_leader_follower/

3) Create the complete code files

Go into the Python module folder:

cd ~/apf_ws/src/apf_leader_follower/apf_leader_follower


You should see:

__init__.py


We will create two nodes:

trail_recorder.py

follower_apf.py

3.1 Create trail_recorder.py
nano trail_recorder.py


Paste everything:

#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Point


class TrailRecorder(Node):
    """
    Records leader path from /turtle1/pose.
    Publishes points to /leader_trail_point (geometry_msgs/Point).
    Stores points only when leader moved >= min_dist to avoid too many points.
    """

    def __init__(self):
        super().__init__('trail_recorder')

        # Parameters
        self.declare_parameter('min_dist', 0.20)
        self.declare_parameter('max_points', 3000)

        self.min_dist = float(self.get_parameter('min_dist').value)
        self.max_points = int(self.get_parameter('max_points').value)

        # Internal storage
        self.trail = deque(maxlen=self.max_points)  # list of (x,y)

        # ROS pub/sub
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.cb_pose, 10)
        self.pub_point = self.create_publisher(Point, '/leader_trail_point', 50)

        self.get_logger().info("TrailRecorder running: subscribe /turtle1/pose, publish /leader_trail_point")

    def cb_pose(self, msg: Pose):
        x, y = float(msg.x), float(msg.y)

        if not self.trail:
            self.trail.append((x, y))
            self._publish_point(x, y)
            return

        last_x, last_y = self.trail[-1]
        d = math.hypot(x - last_x, y - last_y)

        if d >= self.min_dist:
            self.trail.append((x, y))
            self._publish_point(x, y)

    def _publish_point(self, x: float, y: float):
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        self.pub_point.publish(p)


def main():
    rclpy.init()
    node = TrailRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


Save: CTRL+O, Enter, then exit CTRL+X.

3.2 Create follower_apf.py
nano follower_apf.py


Paste everything:

#!/usr/bin/env python3
import math
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point


def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FollowerAPF(Node):
    """
    Follower turtle2:
    - Receives leader trail points on /leader_trail_point
    - Chooses a delayed point behind leader (delay_points)
    - Tracks that target using a simple distance + heading controller
    - Applies APF repulsion from forbidden rectangles (constraint-aware safety)
    """

    def __init__(self):
        super().__init__('follower_apf')

        # ---------------- Parameters ----------------
        # Trail following
        self.declare_parameter('delay_points', 15)    # how far behind leader (in saved points)
        self.declare_parameter('k_lin', 1.2)          # linear gain
        self.declare_parameter('k_ang', 4.0)          # angular gain
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('w_max', 2.5)

        # APF safety
        self.declare_parameter('k_rep', 2.0)          # repulsive strength
        self.declare_parameter('d0', 1.2)             # influence distance around forbidden areas

        # Escape local minima
        self.declare_parameter('stuck_eps', 0.03)     # if |v| and |w| are both below this => stuck
        self.declare_parameter('noise_mag', 0.18)     # how strong the random nudge is

        # Forbidden rectangles: [xmin,xmax,ymin,ymax, xmin,xmax,ymin,ymax, ...]
        # Example: center box and a top-left strip
        self.declare_parameter('forbidden', [
            4.5, 6.5, 4.5, 6.5,
            1.0, 2.0, 8.5, 10.0
        ])

        self.delay_points = int(self.get_parameter('delay_points').value)
        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)

        self.k_rep = float(self.get_parameter('k_rep').value)
        self.d0 = float(self.get_parameter('d0').value)

        self.stuck_eps = float(self.get_parameter('stuck_eps').value)
        self.noise_mag = float(self.get_parameter('noise_mag').value)

        forb = self.get_parameter('forbidden').value
        if len(forb) % 4 != 0:
            raise ValueError("forbidden must be multiple of 4: xmin,xmax,ymin,ymax ...")

        self.forbidden_rects = []
        for i in range(0, len(forb), 4):
            xmin, xmax, ymin, ymax = map(float, forb[i:i+4])
            self.forbidden_rects.append((xmin, xmax, ymin, ymax))

        # ---------------- State ----------------
        self.follower_pose = None
        self.trail = deque(maxlen=4000)  # list of (x,y) points from leader

        # ---------------- ROS I/O ----------------
        self.sub_pose = self.create_subscription(Pose, '/turtle2/pose', self.cb_follower_pose, 10)
        self.sub_trail = self.create_subscription(Point, '/leader_trail_point', self.cb_trail_point, 50)
        self.pub_cmd = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("FollowerAPF running: /turtle2/pose + /leader_trail_point -> /turtle2/cmd_vel")

    def cb_follower_pose(self, msg: Pose):
        self.follower_pose = msg

    def cb_trail_point(self, msg: Point):
        self.trail.append((float(msg.x), float(msg.y)))

    # ---------------- Geometry helpers for rectangles ----------------
    def _closest_point_on_rect(self, x, y, xmin, xmax, ymin, ymax):
        cx = min(max(x, xmin), xmax)
        cy = min(max(y, ymin), ymax)
        return cx, cy

    def _inside_rect(self, x, y, xmin, xmax, ymin, ymax):
        return (xmin <= x <= xmax) and (ymin <= y <= ymax)

    def repulsion_vector(self, x: float, y: float) -> np.ndarray:
        """
        Compute repulsive vector away from forbidden rectangles.
        - If inside rectangle: push outward strongly toward nearest exit side.
        - If outside but within d0: push away from closest boundary point.
        """
        v = np.array([0.0, 0.0], dtype=float)

        for (xmin, xmax, ymin, ymax) in self.forbidden_rects:
            inside = self._inside_rect(x, y, xmin, xmax, ymin, ymax)
            cx, cy = self._closest_point_on_rect(x, y, xmin, xmax, ymin, ymax)

            dx = x - cx
            dy = y - cy
            d = math.hypot(dx, dy)

            if inside:
                # distance to each side
                left = abs(x - xmin)
                right = abs(xmax - x)
                bottom = abs(y - ymin)
                top = abs(ymax - y)

                m = min(left, right, bottom, top)
                # direction to move OUT
                if m == left:
                    dir_vec = np.array([+1.0, 0.0])
                    d_eff = max(left, 1e-3)
                elif m == right:
                    dir_vec = np.array([-1.0, 0.0])
                    d_eff = max(right, 1e-3)
                elif m == bottom:
                    dir_vec = np.array([0.0, +1.0])
                    d_eff = max(bottom, 1e-3)
                else:
                    dir_vec = np.array([0.0, -1.0])
                    d_eff = max(top, 1e-3)

                v += self.k_rep * (1.0 / d_eff) * dir_vec
                continue

            # Outside but near: APF-like push away from closest point on rectangle
            if d < self.d0 and d > 1e-6:
                strength = self.k_rep * (1.0 / d - 1.0 / self.d0) * (1.0 / (d * d))
                v += strength * np.array([dx / d, dy / d])

        return v

    def control_loop(self):
        if self.follower_pose is None:
            return
        if len(self.trail) < (self.delay_points + 1):
            return

        # -------- Target selection: delayed waypoint --------
        tx, ty = self.trail[-1 - self.delay_points]

        # -------- Current follower pose --------
        x = float(self.follower_pose.x)
        y = float(self.follower_pose.y)
        theta = float(self.follower_pose.theta)

        # -------- Base trail-follow controller --------
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_err = wrap_angle(desired_heading - theta)

        # Smooth motion: reduce forward speed on sharp turns
        turn_slowdown = max(0.0, 1.0 - abs(heading_err) / math.pi)

        v = self.k_lin * dist * turn_slowdown
        w = self.k_ang * heading_err

        # -------- APF safety adjustment --------
        rep = self.repulsion_vector(x, y)
        rep_norm = float(np.linalg.norm(rep))

        if rep_norm > 1e-6:
            # convert repulsion vector into a heading we want to turn toward (away from rectangle)
            rep_heading = math.atan2(rep[1], rep[0])
            rep_err = wrap_angle(rep_heading - theta)

            # steer away + slow down
            w += 1.0 * rep_err
            v *= 1.0 / (1.0 + rep_norm)

        # -------- Escape local minima --------
        if abs(v) < self.stuck_eps and abs(w) < self.stuck_eps:
            w += float(np.random.uniform(-self.noise_mag, self.noise_mag))
            v += float(np.random.uniform(0.0, self.noise_mag))

        # -------- Clamp speeds --------
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = FollowerAPF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


Save and exit.

4) Wire the package to run these nodes
4.1 Make scripts executable
chmod +x ~/apf_ws/src/apf_leader_follower/apf_leader_follower/trail_recorder.py
chmod +x ~/apf_ws/src/apf_leader_follower/apf_leader_follower/follower_apf.py

4.2 Edit setup.py

Open:

nano ~/apf_ws/src/apf_leader_follower/setup.py


Find the entry_points part and replace it with this:

entry_points={
    'console_scripts': [
        'trail_recorder = apf_leader_follower.trail_recorder:main',
        'follower_apf = apf_leader_follower.follower_apf:main',
    ],
},


Save/exit.

5) Build the workspace
cd ~/apf_ws
colcon build
source install/setup.bash


Optional: auto-source this workspace every time:

echo "source ~/apf_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

6) Run the full system (final run steps)

Make sure turtlesim is running + turtle2 spawned + teleop is running (from Section 1).

Then:

Terminal A: Trail recorder
source ~/apf_ws/install/setup.bash
ros2 run apf_leader_follower trail_recorder

Terminal B: Follower APF
source ~/apf_ws/install/setup.bash
ros2 run apf_leader_follower follower_apf


Now drive turtle1 — turtle2 should:

follow the same path (delayed waypoint tracking)

avoid the forbidden rectangles

7) Customize constraints (forbidden rectangles)

In follower_apf.py, parameter forbidden is:

[xmin, xmax, ymin, ymax, xmin, xmax, ymin, ymax, ...]


Example: forbid a big center region:

self.declare_parameter('forbidden', [3.5, 7.5, 3.5, 7.5])

8) Quick debugging commands (when something doesn’t move)
Check follower is publishing cmd_vel:
ros2 topic echo /turtle2/cmd_vel

Check trail points are being published:
ros2 topic echo /leader_trail_point

Check turtle2 pose exists:
ros2 topic echo /turtle2/pose
