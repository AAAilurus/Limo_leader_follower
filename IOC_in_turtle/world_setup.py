1. Create: nano ~/ros2_ws/src/ioc_turtlesim/ioc_turtlesim/world_setup.py
-----------------------------------------
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from turtlesim.srv import Spawn, TeleportAbsolute, SetPen


class WorldSetup(Node):
    """
    Sets up a "puzzle" turtlesim world:
    - reset sim
    - leader turtle1 at start
    - follower turtle2 at start
    - a goal turtle at goal
    - obstacle turtles as visible circular obstacles
    - a drawer turtle that draws rectangular maze walls on the canvas
    """

    def __init__(self):
        super().__init__('world_setup')

        # ---------- YOU CAN EDIT THESE ----------
        self.leader_start = (1.0, 1.0, 0.0)
        self.follower_start = (1.0, 1.0, 0.0)  # same as leader for first version
        self.goal_pose = (10.0, 10.0, 0.0)

        # Obstacle turtles (circular “keep-out” zones conceptually)
        # (x, y)
        self.obstacles = [
            (4.0, 2.5),
            (6.0, 3.5),
            (7.5, 6.0),
            (5.0, 7.5),
            (3.5, 6.0),
        ]

        # Maze walls as rectangles: (xmin, ymin, xmax, ymax)
        # These are drawn lines only (visual), but we will treat them later as constraints in cost.
        self.walls = [
            # corridor-ish walls
            (2.0, 2.0, 9.0, 2.2),   # bottom wall strip
            (2.0, 8.8, 9.0, 9.0),   # top wall strip
            (2.0, 2.0, 2.2, 9.0),   # left wall strip
            (8.8, 2.0, 9.0, 9.0),   # right wall strip

            # internal hurdles (like puzzle gates)
            (3.0, 3.0, 7.8, 3.2),
            (3.0, 5.0, 7.8, 5.2),
            (3.0, 7.0, 7.8, 7.2),
        ]
        # ----------------------------------------

        # Service clients
        self.reset_cli = self.create_client(Empty, '/reset')
        self.clear_cli = self.create_client(Empty, '/clear')
        self.spawn_cli = self.create_client(Spawn, '/spawn')

        self.get_logger().info("Waiting for turtlesim services...")
        for cli in [self.reset_cli, self.clear_cli, self.spawn_cli]:
            cli.wait_for_service()

        self.get_logger().info("Setting up world...")
        self.setup_world()

    def call_empty(self, client: rclpy.client.Client, name: str):
        req = Empty.Request()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            raise RuntimeError(f"Service call failed: {name}")

    def spawn(self, x: float, y: float, theta: float, name: str):
        req = Spawn.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        req.name = name
        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            raise RuntimeError(f"Spawn failed for: {name}")
        return fut.result().name

    def teleport(self, turtle_name: str, x: float, y: float, theta: float):
        cli = self.create_client(TeleportAbsolute, f'/{turtle_name}/teleport_absolute')
        cli.wait_for_service()
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            raise RuntimeError(f"Teleport failed for: {turtle_name}")

    def set_pen(self, turtle_name: str, r: int, g: int, b: int, width: int, off: int):
        cli = self.create_client(SetPen, f'/{turtle_name}/set_pen')
        cli.wait_for_service()
        req = SetPen.Request()
        req.r = int(r); req.g = int(g); req.b = int(b)
        req.width = int(width)
        req.off = int(off)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            raise RuntimeError(f"SetPen failed for: {turtle_name}")

    def draw_rectangle_outline(self, turtle_name: str, xmin: float, ymin: float, xmax: float, ymax: float):
        # Draw by teleporting around corners (turtlesim draws lines when pen is on)
        corners = [
            (xmin, ymin),
            (xmax, ymin),
            (xmax, ymax),
            (xmin, ymax),
            (xmin, ymin),
        ]
        # Teleport to first corner without drawing
        self.set_pen(turtle_name, 0, 0, 0, 3, 1)  # pen off
        self.teleport(turtle_name, corners[0][0], corners[0][1], 0.0)

        # Turn pen on and trace corners
        self.set_pen(turtle_name, 0, 0, 0, 3, 0)  # pen on, black
        for (x, y) in corners[1:]:
            self.teleport(turtle_name, x, y, 0.0)

        # Pen off again
        self.set_pen(turtle_name, 0, 0, 0, 3, 1)

    def setup_world(self):
        # Reset + clear canvas
        self.call_empty(self.reset_cli, '/reset')
        self.call_empty(self.clear_cli, '/clear')

        # After /reset, turtle1 exists again at default center.
        # Teleport leader to start
        self.teleport('turtle1', *self.leader_start)
        self.set_pen('turtle1', 255, 0, 0, 2, 1)  # leader pen off (no trail)

        # Spawn follower turtle2 and teleport
        self.spawn(5.5, 5.5, 0.0, 'turtle2')
        self.teleport('turtle2', *self.follower_start)
        self.set_pen('turtle2', 0, 255, 0, 2, 1)  # follower pen off

        # Spawn goal turtle (visual marker)
        self.spawn(self.goal_pose[0], self.goal_pose[1], self.goal_pose[2], 'goal')
        self.set_pen('goal', 0, 0, 255, 2, 1)  # pen off

        # Spawn drawer turtle for maze walls
        self.spawn(1.0, 10.5, 0.0, 'drawer')
        self.set_pen('drawer', 0, 0, 0, 3, 1)  # start pen off

        # Draw puzzle walls (visual)
        for (xmin, ymin, xmax, ymax) in self.walls:
            self.draw_rectangle_outline('drawer', xmin, ymin, xmax, ymax)

        # Spawn obstacle turtles (visual circular obstacles)
        for i, (ox, oy) in enumerate(self.obstacles, start=1):
            name = f'obs{i}'
            self.spawn(ox, oy, 0.0, name)
            self.set_pen(name, 120, 120, 120, 2, 1)  # pen off

        self.get_logger().info("World ready: leader=turtle1, follower=turtle2, goal=goal, obstacles=obs*, walls drawn.")
        self.get_logger().info("Next: teleop turtle1 and record data.")


def main():
    rclpy.init()
    node = WorldSetup()
    # no spin needed; setup is immediate
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
