import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.msg import Altimeter


class SubscriberAltimeterPackets(Node):

    def __init__(self, callback_function):

        super().__init__('subscriber_altimeter_packets')

        self.callback_function = callback_function
        self.subscription = self.create_subscription(Altimeter, 'altimeter', self.callback_function, 100)

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

    def altimeter_packet_callback(altimeter):
        
        try:
            DIGIT_LENGTH = 5
            formated_altimeter = list()

            for data in [altimeter.altimeter_distance, altimeter.altimeter_quality, altimeter.temperature, altimeter.pressure]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formated_altimeter.append(f'{data:.{decimal_length}f}')

            subscriber.get_logger().info(f'distance: {formated_altimeter[0]} | quality: {formated_altimeter[1]} | temperature: {formated_altimeter[2]} | pressure: {formated_altimeter[3]}')
        
        except Exception:
            subscriber.get_logger().info(f'distance: {round(altimeter.altimeter_distance, 3)} | quality: {round(altimeter.altimeter_quality, 3)} | temperature: {round(altimeter.temperature, 3)} | pressure: {round(altimeter.pressure, 3)}')
            
    rclpy.init(args=args)

    subscriber = SubscriberAltimeterPackets(callback_function=altimeter_packet_callback)

    executor = SingleThreadedExecutor()
    executor.add_node(subscriber)

    subscriber.subscribe()

    executor.shutdown()

    subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
