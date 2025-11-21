import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class PreSubscriber(Node):

    def _init_(self):
        super()._init_('pre_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topico_steps',
            self.listener_callback,
            10)
        self.subscription  

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    pre_subscriber = PreSubscriber()

    rclpy.spin(pre_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pre_subscriber.destroy_node()
    rclpy.shutdown()


if __name__== '__main__':
    main()
