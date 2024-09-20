import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.msg import BottomTrack


class SubscriberWaterTrackPackets(Node):

    def __init__(self, callback_function, qos_profile=100):

        super().__init__('water_track_packets')

        self.subscription = self.create_subscription(BottomTrack, topic='nucleus_node/water_track_packets', callback=callback_function, qos_profile=qos_profile)

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

    def water_track_packet_callback(water_track):
        
        try:
            DIGIT_LENGTH = 5
            formated_water_track = list()

            for data in [water_track.velocity_x, water_track.velocity_y, water_track.velocity_z, water_track.fom_x, water_track.fom_y, water_track.fom_z]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formated_water_track.append(f'{data:.{decimal_length}f}')

            subscriber.get_logger().info(f'velocity_x: {formated_water_track[0]} | velocity_y: {formated_water_track[1]} | velocity_z: {formated_water_track[2]} | fom_x: {formated_water_track[3]} | fom_y: {formated_water_track[4]} | fom_z: {formated_water_track[5]}')
        
        except Exception:
            subscriber.get_logger().info(f'velocity_x: {round(water_track.velocity_x, 3)} | velocity_y: {round(water_track.velocity_y, 3)} | velocity_z: {round(water_track.velocity_z, 3)} | fom_x: {round(water_track.fom_x, 3)} | fom_y: {round(water_track.fom_y, 3)} | fom_z: {round(water_track.fom_z, 3)}')
            
    rclpy.init(args=args)

    subscriber = SubscriberWaterTrackPackets(callback_function=water_track_packet_callback)

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
