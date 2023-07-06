import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.msg import AHRS


class SubscriberAhrsPackets(Node):

    def __init__(self, callback_function):

        super().__init__('subscriber_ahrs_packets')

        self.callback_function = callback_function
        self.subscription = self.create_subscription(AHRS, 'ahrs', self.callback_function, 100)

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

            for data in [ahrs.roll, ahrs.pitch, ahrs.heading, ahrs.quaternion_w, ahrs.quaternion_x, ahrs.quaternion_y, ahrs.quaternion_z]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formated_ahrs.append(f'{data:.{decimal_length}f}')

            subscriber.get_logger().info(f'roll: {formated_ahrs[0]} | pitch: {formated_ahrs[1]} | heading: {formated_ahrs[2]} | quat w: {formated_ahrs[3]} | quat x: {formated_ahrs[4]} | quat y: {formated_ahrs[5]} | quat z: {formated_ahrs[6]}')
        
        except Exception:
            subscriber.get_logger().info(f'roll: {round(ahrs.roll, 3)} | pitch: {round(ahrs.pitch, 3)} | heading: {round(ahrs.heading, 3)} | quat w: {round(ahrs.quaternion_w, 3)} | quat x: {round(ahrs.quaternion_x, 3)} | quat y: {round(ahrs.quaternion_y, 3)} | quat z: {round(ahrs.quaternion_z, 3)}')
            
    rclpy.init(args=args)

    subscriber = SubscriberAhrsPackets(callback_function=ahrs_packet_callback)

    executor = SingleThreadedExecutor()
    executor.add_node(subscriber)

    subscriber.subscribe()

    executor.shutdown()

    subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
