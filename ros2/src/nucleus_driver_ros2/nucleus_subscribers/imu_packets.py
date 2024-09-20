import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.msg import IMU


class SubscriberImuPackets(Node):

    def __init__(self, callback_function, qos_profile=100):

        super().__init__('imu_packets')

        self.subscription = self.create_subscription(IMU, topic='nucleus_node/imu_packets', callback=callback_function, qos_profile=qos_profile)

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

    def imu_packet_callback(imu):
        
        try:
            DIGIT_LENGTH = 5
            formated_imu = list()

            for data in [imu.accelerometer_x, imu.accelerometer_y, imu.accelerometer_z, imu.gyro_x, imu.gyro_y, imu.gyro_z]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formated_imu.append(f'{data:.{decimal_length}f}')

            subscriber.get_logger().info(f'accelerometer_x: {formated_imu[0]} | accelerometer_y: {formated_imu[1]} | accelerometer_z: {formated_imu[2]} | gyro_x: {formated_imu[3]} | gyro_y: {formated_imu[4]} | gyro_z: {formated_imu[5]}')
        
        except Exception:
            subscriber.get_logger().info(f'accelerometer_x: {round(imu.accelerometer_x, 3)} | accelerometer_y: {round(imu.accelerometer_y, 3)} | accelerometer_z: {round(imu.accelerometer_z, 3)} | gyro_x: {round(imu.gyro_x, 3)} | gyro_y: {round(imu.gyro_y, 3)} | gyro_z: {round(imu.gyro_z, 3)}')
            
    rclpy.init(args=args)

    subscriber = SubscriberImuPackets(callback_function=imu_packet_callback)

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
