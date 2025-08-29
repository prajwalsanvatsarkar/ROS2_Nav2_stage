import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitPose(Node):
    def __init__(self):
        super().__init__('init_pose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(0.5, self.publish_once)
        self.sent = False

    def publish_once(self):
        if self.sent:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0   # <— adjust to your spawn location if needed
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        # minimal covariance to satisfy AMCL
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.pub.publish(msg)
        self.get_logger().info('Initial pose published to /initialpose')
        self.sent = True

def main():
    rclpy.init()
    node = InitPose()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

