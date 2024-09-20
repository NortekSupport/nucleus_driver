import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.msg import AHRS


class SubscriberAhrsPackets(Node):

    def __init__(self, callback_function, qos_profile=100):

        super().__init__('ahrs_packets')

        self.subscription = self.create_subscription(AHRS, topic='nucleus_node/ahrs_packets', callback=callback_function, qos_profile=qos_profile)

    def subscribe(self):
        
        if self.executor is not None:
            self.executor.spin()
        else:
            self.get_logger().warning('This subscriber is not added to an executor. Establishing a temporary SingleThreadedExecutor for this subscription')
            executor = SingleThreadedExecutor()
            executor.add_node(self)
            executor.spin()
            executor.shutdown()


def main(args=None):

    def ahrs_packet_callback(ahrs):
        
        try:
            DIGIT_LENGTH = 5
            formated_ahrs = list()

            for data in [ahrs.roll, ahrs.pitch, ahrs.heading]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formated_ahrs.append(f'{data:.{decimal_length}f}')

            subscriber.get_logger().info(f'roll: {formated_ahrs[0]} | pitch: {formated_ahrs[1]} | heading: {formated_ahrs[2]}')
        
        except Exception:
            subscriber.get_logger().info(f'roll: {round(ahrs.roll, 3)} | pitch: {round(ahrs.pitch, 3)} | heading: {round(ahrs.heading, 3)}')
            
    rclpy.init(args=args)

    subscriber = SubscriberAhrsPackets(callback_function=ahrs_packet_callback)

    executor = SingleThreadedExecutor()

    try:
        executor.add_node(subscriber)
        subscriber.subscribe()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        subscriber.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
