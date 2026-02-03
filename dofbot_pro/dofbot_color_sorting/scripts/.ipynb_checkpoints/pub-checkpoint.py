import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from shared import SharedState

os.environ['ROS_DOMAIN_ID'] = '25'

class MyPublisher(Node):
    def __init__(self):
        super().__init__('publisher_p')
        self.publisher_ = self.create_publisher(String, 'topic_p', 10)
        timer_period = 0.5  # 0.5 秒触发一次
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, I am MRobot!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

class ROS2Publisher:
    def __init__(self):
        if not SharedState.initialized:
            rclpy.init()
            SharedState.initialized = True
        self.node = MyPublisher()

    def spin(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass

    def shutdown(self):
        self.node.destroy_node()
        if SharedState.initialized:
            rclpy.shutdown()
            SharedState.initialized = False