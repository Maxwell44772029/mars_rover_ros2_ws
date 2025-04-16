#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, LookupException, TransformException, TimeoutException
from std_srvs.srv import Empty
import numpy as np

class OccupiedCellsProjector(Node):
    def __init__(self):
        super().__init__('occupied_cells_projector')

        self.declare_parameter('resolution', 2.5)
        self.declare_parameter('radius', 50.0)
        self.declare_parameter('slope_threshold', 2.5)  # in meters

        self.resolution = self.get_parameter('resolution').value
        self.radius = self.get_parameter('radius').value
        self.slope_thresh = self.get_parameter('slope_threshold').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(MarkerArray, '/occupied_cells_vis_array', self.marker_callback, 10)
        self.pub = self.create_publisher(OccupancyGrid, '/costmap_2d', 10)

        self.get_logger().info('Slope-aware costmap node started.')

        # OctoMap reset client
        self.reset_client = self.create_client(Empty, '/octomap_server/reset')
        self.reset_timer = self.create_timer(30.0, self.reset_octomap)

    def reset_octomap(self):
        if self.reset_client.service_is_ready():
            self.reset_client.call_async(Empty.Request())
            self.get_logger().info('Sent OctoMap reset request.')
        else:
            self.get_logger().warn('OctoMap reset service not ready.')

    def marker_callback(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = tf.transform.translation.x
            robot_y = tf.transform.translation.y
        except (LookupException, TransformException, TimeoutException):
            self.get_logger().warn('TF lookup failed.')
            return

        if len(msg.markers) == 0:
            self.get_logger().warn("Empty marker array received — skipping.")
            return

        size = int((2 * self.radius) / self.resolution)
        grid = np.full((size, size), -1, dtype=np.int8)
        z_min = np.full((size, size), np.inf)
        z_max = np.full((size, size), -np.inf)
        hits = np.zeros((size, size), dtype=int)

        # Snap origin to grid to prevent flicker
        origin_x = np.floor((robot_x - self.radius) / self.resolution) * self.resolution
        origin_y = np.floor((robot_y - self.radius) / self.resolution) * self.resolution


        for marker in msg.markers:
            for point in marker.points:
                x, y, z = point.x, point.y, point.z
                if (x - robot_x) ** 2 + (y - robot_y) ** 2 > self.radius ** 2:
                    continue

                gx = int((x - origin_x) / self.resolution)
                gy = int((y - origin_y) / self.resolution)

                if 0 <= gx < size and 0 <= gy < size:
                    z_min[gy, gx] = min(z_min[gy, gx], z)
                    z_max[gy, gx] = max(z_max[gy, gx], z)
                    hits[gy, gx] += 1

        for y in range(size):
            for x in range(size):
                if hits[y, x] == 0:
                    grid[y, x] = -1
                elif (z_max[y, x] - z_min[y, x]) > self.slope_thresh:
                    grid[y, x] = 100
                else:
                    grid[y, x] = 0

        costmap = OccupancyGrid()
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.header.frame_id = 'map'  # map frame is correct
        costmap.info.resolution = self.resolution
        costmap.info.width = size
        costmap.info.height = size
        costmap.info.origin.position.x = origin_x
        costmap.info.origin.position.y = origin_y
        costmap.info.origin.orientation.w = 1.0
        costmap.data = grid.flatten().tolist()

        self.pub.publish(costmap)

def main(args=None):
    rclpy.init(args=args)
    node = OccupiedCellsProjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
