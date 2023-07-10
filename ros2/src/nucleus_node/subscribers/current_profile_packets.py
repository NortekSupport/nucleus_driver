import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.msg import CurrentProfile


class SubscriberCurrentProfilePackets(Node):

    def __init__(self, callback_function):

        super().__init__('subscriber_current_profile_packets')

        self.callback_function = callback_function
        self.subscription = self.create_subscription(CurrentProfile, 'current_profile', self.callback_function, 100)

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

    def current_profile_packet_callback(current_profile):
        '''
        try:
            DIGIT_LENGTH = 5
            formated_current_profile = list()

            for data in [current_profile.velocity_x, current_profile.velocity_y, current_profile.velocity_z, current_profile.fom_x, current_profile.fom_y, current_profile.fom_z]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formated_current_profile.append(f'{data:.{decimal_length}f}')

            subscriber.get_logger().info(f'velocity_x: {formated_current_profile[0]} | velocity_y: {formated_current_profile[1]} | velocity_z: {formated_current_profile[2]} | fom_x: {formated_current_profile[3]} | fom_y: {formated_current_profile[4]} | fom_z: {formated_current_profile[5]}')
        
        except Exception:
            subscriber.get_logger().info(f'velocity_data: {current_profile.velocity_x} | amplitude_data: {current_profile.amplitude_data} | correlation_data: {current_profile.correlation_data}')
        '''
        subscriber.get_logger().info(f'velocity_data: {current_profile.velocity_data} | amplitude_data: {current_profile.amplitude_data} | correlation_data: {current_profile.correlation_data}')
            
    rclpy.init(args=args)

    subscriber = SubscriberCurrentProfilePackets(callback_function=current_profile_packet_callback)

    executor = SingleThreadedExecutor()
    executor.add_node(subscriber)

    subscriber.subscribe()

    executor.shutdown()

    subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
