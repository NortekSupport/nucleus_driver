import rclpy
from rclpy.node import Node

from interfaces.msg import Ahrs


class SubscriberAhrsPackets(Node):

    def __init__(self, callback_function):

        super().__init__('subscriber_ahrs_packets')

        self.callback_function = callback_function
        self.subscription = self.create_subscription(Ahrs, 'ahrs', self.callback_function, 100)





def main(args=None):

    def ahrs_packet_callback(ahrs):
        subscriber.get_logger().info(f'NEW # roll: {ahrs.roll} \t pitch: {ahrs.pitch} \t heading: {ahrs.heading}')

    rclpy.init(args=args)

    subscriber = SubscriberAhrsPackets(callback_function=ahrs_packet_callback)

    rclpy.spin(subscriber)

    subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
