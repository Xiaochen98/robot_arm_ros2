import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from shared import SharedState

os.environ['ROS_DOMAIN_ID'] = '25'

class MySubscriber(Node):
    def __init__(self):
        super().__init__('subscriber_p')
        self.subscription = self.create_subscription(
            String,
            'topic_p',
            self.listener_callback,
            10)
        self.subscription  # 防止订阅者被垃圾回收

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

class ROS2Subscriber:
    def __init__(self):
        if not SharedState.initialized:
            rclpy.init()
            SharedState.initialized = True
        self.node = MySubscriber()

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