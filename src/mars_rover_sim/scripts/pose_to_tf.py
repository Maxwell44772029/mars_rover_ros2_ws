#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')

        # Declare sim time parameter so ROS respects use_sim_time
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/pose',
            self.handle_pose,
            10
        )


    def handle_pose(self, msg):
        t = TransformStamped()

        # Add 100ms lead time to help downstream TF consumers (like OctoMap and RViz)
        lead_time = rclpy.duration.Duration(nanoseconds=100_000_000)  # 100ms
        t.header.stamp = (rclpy.time.Time.from_msg(msg.header.stamp) + lead_time).to_msg()

        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

