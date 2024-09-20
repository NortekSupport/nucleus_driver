import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.msg import INS


class SubscriberInsPackets(Node):

    def __init__(self, callback_function, qos_profile=100):

        super().__init__("ins_packets")

        self.subscription = self.create_subscription(
            INS,
            topic="nucleus_node/ins_packets",
            callback=callback_function,
            qos_profile=qos_profile,
        )

    def subscribe(self):

        if self.executor is not None:
            self.executor.spin()
        else:
            self.get_logger().warning(
                "This subscriber is not added to an executor. Establishing a temporary SingleThreadedExecutor for this subscription"
            )
            executor = SingleThreadedExecutor()
            executor.add_node(self)
            executor.spin()
            executor.shutdown()


def main(args=None):

    def ins_packet_callback(ins):

        try:
            DIGIT_LENGTH = 5
            formated_ins = list()

            for data in [
                ins.position_frame_x,
                ins.position_frame_y,
                ins.position_frame_z,
                ins.altitude,
                ins.latitude,
                ins.longitude,
            ]:
                integral_length = len(str(data).split(".")[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formated_ins.append(f"{data:.{decimal_length}f}")

            subscriber.get_logger().info(
                f"position x: {formated_ins[0]} | position y: {formated_ins[1]} | position z: {formated_ins[2]} | altitude: {formated_ins[3]} | latitude: {formated_ins[4]} | longitude: {formated_ins[5]}"
            )

        except Exception:
            subscriber.get_logger().info(
                f"position x: {round(ins.roll, 3)} | position y: {round(ins.pitch, 3)} | position z: {round(ins.heading, 3)} | altitude: {round(ins.quaternion_w, 3)} | latitude: {round(ins.quaternion_x, 3)} | longitude: {round(ins.quaternion_y, 3)}"
            )

    rclpy.init(args=args)

    subscriber = SubscriberInsPackets(callback_function=ins_packet_callback)

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


if __name__ == "__main__":
    main()
