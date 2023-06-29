import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import Stop

class ClientStop(Node):

    def __init__(self):

        super().__init__('client_stop')

        self.client_stop = self.create_client(Stop, 'stop')
        while not self.client_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('stop service not available. Waiting...')
        self.request = Stop.Request()

    def send_request(self):

        self.call = self.client_stop.call_async(self.request)

def main(args=None):

    rclpy.init(args=args)

    client_stop = ClientStop()
    client_stop.send_request()

    while rclpy.ok():

        rclpy.spin_once(client_stop)

        if client_stop.call.done():
            try:
                response = client_stop.call.result()
            except Exception as e:
                client_stop.get_logger().info(f'stop call failed: {e}')
            else:
                client_stop.get_logger().info(f'Successfully made the stop call with status: {response.reply}')

            break
    
    client_stop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()