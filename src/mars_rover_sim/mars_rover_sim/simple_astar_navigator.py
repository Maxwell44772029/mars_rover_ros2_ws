#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, TransformException, TimeoutException
import tf_transformations
import numpy as np
import math
import heapq
import time

class SimpleAStarPlanner(Node):
    def __init__(self):
        super().__init__('simple_astar_planner')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(OccupancyGrid, '/costmap_2d', self.map_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)

        self.goal_dx = -1000.0
        self.goal_dy = 0.0
        self.starting_pose_set = False
        self.goal_x = None
        self.goal_y = None

        self.current_path = []
        self.path_index = 0
        self.local_radius = 50.0  # meters

    def map_callback(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = tf.transform.translation.x
            robot_y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        except (LookupException, TransformException, TimeoutException):
            self.get_logger().warn('TF lookup failed')
            return

        if not self.starting_pose_set:
            self.goal_x = robot_x + self.goal_dx
            self.goal_y = robot_y + self.goal_dy
            self.starting_pose_set = True
            self.get_logger().info(f"Set full goal at (x={self.goal_x:.2f}, y={self.goal_y:.2f})")

        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        raw_grid = np.copy(grid)
        # Inflate black cells to give safety buffer around obstacles
        inflation_radius = 3  # number of cells to expand; tune based on resolution and rover size

        inflated_grid = np.copy(grid)

        for y in range(height):
            for x in range(width):
                if grid[y, x] > 50:  # consider this cell an obstacle
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        for dx in range(-inflation_radius, inflation_radius + 1):
                            nx = x + dx
                            ny = y + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                dist = math.hypot(dx, dy)
                                decay_cost = max(100 - int(dist * 25), grid[ny, nx])
                                inflated_grid[ny, nx] = max(inflated_grid[ny, nx], decay_cost)

        # Use inflated grid for A*
        grid = inflated_grid


        gx = int((self.goal_x - origin_x) / resolution)
        gy = int((self.goal_y - origin_y) / resolution)
        rx = int((robot_x - origin_x) / resolution)
        ry = int((robot_y - origin_y) / resolution)

        dx = self.goal_x - robot_x
        dy = self.goal_y - robot_y
        distance_to_goal = math.hypot(dx, dy)

        # Scale goal into local A* radius
        if distance_to_goal < self.local_radius:
            tx, ty = gx, gy
        else:
            scale = self.local_radius / distance_to_goal
            temp_x = robot_x + dx * scale
            temp_y = robot_y + dy * scale
            tx = int((temp_x - origin_x) / resolution)
            ty = int((temp_y - origin_y) / resolution)
            self.get_logger().info(f"Using local goal at (x={temp_x:.2f}, y={temp_y:.2f})")

        self.get_logger().info(
            f"[GRID] A* from ({rx}, {ry}) to ({tx}, {ty}) | Grid size: {width}x{height}"
        )

        path = self.astar(grid, rx, ry, tx, ty)
        if not path:
            self.get_logger().warn("No A* path found! Attempting fallback...")

            fallback_scale = 0.75 * self.local_radius / distance_to_goal
            temp_x = robot_x + dx * fallback_scale
            temp_y = robot_y + dy * fallback_scale
            tx = int((temp_x - origin_x) / resolution)
            ty = int((temp_y - origin_y) / resolution)
            self.get_logger().info(f"[FALLBACK] New temp goal at (x={temp_x:.2f}, y={temp_y:.2f}) → grid ({tx}, {ty})")

            if 0 <= tx < width and 0 <= ty < height and grid[ty, tx] <= 50 and grid[ty, tx] != -1:
                path = self.astar(grid, rx, ry, tx, ty)
            else:
                self.get_logger().warn("Fallback goal cell is invalid or out-of-bounds. Trying raw grid...")

        if not path:
            self.get_logger().warn("Trying raw grid A* without inflated safety margin...")
            path = self.astar(raw_grid, rx, ry, tx, ty)

        if not path and self.last_good_path:
            self.get_logger().warn("Even raw grid failed. Reusing last known good path.")
            path = self.last_good_path

        if not path:
            self.get_logger().warn("Even raw grid A* failed. Holding position.")
            return

        self.current_path = [(x * resolution + origin_x, y * resolution + origin_y) for x, y in path]
        self.last_good_path = path
        self.path_index = 0
        self.publish_path(path, resolution, origin_x, origin_y)

        if self.path_index >= len(self.current_path):
            self.get_logger().info("Goal reached!")
            return

        target_x, target_y = self.current_path[min(self.path_index + 3, len(self.current_path) - 1)]
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(target_theta - yaw), math.cos(target_theta - yaw))

        twist = Twist()
        # Minimum speed enforcement
        if distance > 0.5:
            twist.linear.x = max(0.2, 1.0)  # Ensure ≥ 0.2 m/s
            twist.angular.z = 1.5 * math.tanh(angle_diff)
        else:
            self.path_index += 1
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

        # === DEBUG LOGGING ===
        self.get_logger().info(
            f"[POSE] Robot @ ({robot_x:.1f}, {robot_y:.1f}) → Full Goal @ ({self.goal_x:.1f}, {self.goal_y:.1f})"
        )
        self.get_logger().info(
            f"[NAV] Target Waypoint: ({target_x:.1f}, {target_y:.1f}) | "
            f"Δθ: {math.degrees(angle_diff):.1f}°, Heading: {math.degrees(yaw):.1f}°, "
            f"Distance: {distance:.2f}, Waypoint {self.path_index}/{len(self.current_path)}"
        )

        self.get_logger().info("Grid values around robot:")
        for dy in range(-2, 3):
            row = ""
            for dx in range(-2, 3):
                y = ry + dy
                x = rx + dx
                if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]:
                    row += f"{grid[y, x]:>4}"
                else:
                    row += "   ."
            self.get_logger().info(row)

            if not path:
                self.get_logger().warn("No A* path found!")
                return

            for i, (px, py) in enumerate(path[:10]):
                val = grid[py, px]
                self.get_logger().info(f"Path[{i}] = ({px}, {py}) → grid val = {val}")


    def astar(self, grid, start_x, start_y, goal_x, goal_y):
        height, width = grid.shape
        open_set = [(0, (start_x, start_y))]
        came_from = {}
        g_score = { (start_x, start_y): 0 }
        f_score = { (start_x, start_y): self.heuristic(start_x, start_y, goal_x, goal_y) }

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == (goal_x, goal_y):
                return self.reconstruct_path(came_from, current)

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (1,1), (-1,1), (1,-1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                x, y = neighbor
                if 0 <= x < width and 0 <= y < height:
                    val = grid[y, x]
                    # Try strict:
                    if val == 100:
                        penalty = 1000.0  
                    elif val >= 50:
                        penalty = 10.0  
                    elif val >= 1:
                        penalty = val / 10.0 
                    else:
                        penalty = 0.0  
                    tentative_g = g_score[current] + math.hypot(dx, dy) + penalty
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(x, y, goal_x, goal_y)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None

    def heuristic(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, path, resolution, origin_x, origin_y):
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x * resolution + origin_x
            pose.pose.position.y = y * resolution + origin_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        self.path_pub.publish(ros_path)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()