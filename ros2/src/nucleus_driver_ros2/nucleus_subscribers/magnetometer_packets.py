import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.msg import Magnetometer


class SubscriberMagnetometerPackets(Node):

    def __init__(self, callback_function, qos_profile=100):

        super().__init__('magnetometer_packets')

        self.subscription = self.create_subscription(Magnetometer, topic='nucleus_node/magnetometer_packets', callback=callback_function, qos_profile=qos_profile)

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

    def mag_packet_callback(mag):
        
        try:
            DIGIT_LENGTH = 5
            formated_mag = list()

            for data in [mag.magnetometer_x, mag.magnetometer_y, mag.magnetometer_z]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formated_mag.append(f'{data:.{decimal_length}f}')

            subscriber.get_logger().info(f'magnetometer_x: {formated_mag[0]} | magnetometer_y: {formated_mag[1]} | magnetometer_z: {formated_mag[2]}')
        
        except Exception:
            subscriber.get_logger().info(f'magnetometer_x: {round(mag.magnetometer_x, 3)} | magnetometer_y: {round(mag.magnetometer_y, 3)} | magnetometer_z: {round(mag.magnetometer_z, 3)}')
            
    rclpy.init(args=args)

    subscriber = SubscriberMagnetometerPackets(callback_function=mag_packet_callback)

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
