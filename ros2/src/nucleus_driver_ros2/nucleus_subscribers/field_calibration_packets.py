import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.msg import FieldCalibration


class SubscriberFieldCalibrationPackets(Node):

    def __init__(self, callback_function, qos_profile=100):

        super().__init__('field_calibration_packets')

        self.subscription = self.create_subscription(FieldCalibration, topic='nucleus_node/field_calibration_packets', callback=callback_function, qos_profile=qos_profile)

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

    def field_calibration_packet_callback(field_calibration):
        
        try:
            DIGIT_LENGTH = 5
            formated_field_calibration = list()

            for data in [field_calibration.hard_iron_x, field_calibration.hard_iron_y, field_calibration.hard_iron_z]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formated_field_calibration.append(f'{data:.{decimal_length}f}')

            subscriber.get_logger().info(f'hard_iron_x: {formated_field_calibration[0]} | hard_iron_y: {formated_field_calibration[1]} | hard_iron_z: {formated_field_calibration[2]}')
        
        except Exception:
            subscriber.get_logger().info(f'hard_iron_x: {round(field_calibration.hard_iron_x, 3)} | hard_iron_y: {round(field_calibration.hard_iron_y, 3)} | hard_iron_z: {round(field_calibration.hard_iron_z, 3)}')
            
    rclpy.init(args=args)

    subscriber = SubscriberFieldCalibrationPackets(callback_function=field_calibration_packet_callback)

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
