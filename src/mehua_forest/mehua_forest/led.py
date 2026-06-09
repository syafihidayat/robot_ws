import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class LedPublisher(Node):

    def __init__(self):
        super().__init__('led_publisher')

        self.pub = self.create_publisher(
            Bool,
            'led_control',
            10)

        self.state = False

        self.timer = self.create_timer(
            1.0,
            self.timer_callback)

    def timer_callback(self):
        msg = Bool()

        self.state = not self.state
        msg.data = self.state

        self.pub.publish(msg)

        self.get_logger().info(
            f'LED: {"ON" if self.state else "OFF"}')


def main():
    rclpy.init()

    node = LedPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()