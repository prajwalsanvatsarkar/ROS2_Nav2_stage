#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_pub')
        # publish to the same topic RViz uses
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # params (you can override with --ros-args -p xmin:=... etc.)
        self.declare_parameter('xmin', -2.0)
        self.declare_parameter('xmax',  2.0)
        self.declare_parameter('ymin', -2.0)
        self.declare_parameter('ymax',  2.0)
        self.declare_parameter('period', 8.0)     # seconds between goals (set 0 to publish once)
        self.declare_parameter('frame_id', 'map') # must match Nav2 global frame

        period = self.get_parameter('period').get_parameter_value().double_value
        if period <= 0.0:
            # publish once then exit
            self.publish_random_goal()
            self.get_logger().info('Published one goal; exiting in 2s…')
            self.create_timer(2.0, lambda: rclpy.shutdown())
        else:
            self.timer = self.create_timer(period, self.publish_random_goal)
            self.get_logger().info(f'Publishing a random goal every {period:.1f}s')

    def publish_random_goal(self):
        xmin = self.get_parameter('xmin').get_parameter_value().double_value
        xmax = self.get_parameter('xmax').get_parameter_value().double_value
        ymin = self.get_parameter('ymin').get_parameter_value().double_value
        ymax = self.get_parameter('ymax').get_parameter_value().double_value
        frame = self.get_parameter('frame_id').get_parameter_value().string_value

        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        # pick a random yaw and convert to quaternion (z,w for 2D)
        yaw = random.uniform(-math.pi, math.pi)
        z = math.sin(yaw/2.0)
        w = math.cos(yaw/2.0)

        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w

        self.pub.publish(msg)
        self.get_logger().info(f'Sent goal → x:{x:.2f} y:{y:.2f} yaw:{yaw:.2f} rad')

def main():
    rclpy.init()
    node = GoalPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

