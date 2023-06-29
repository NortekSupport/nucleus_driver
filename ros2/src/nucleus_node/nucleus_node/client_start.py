import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import Start

class ClientStart(Node):

    def __init__(self):

        super().__init__('client_start')

        self.client_start = self.create_client(Start, 'start')
        while not self.client_start.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('start service not available. Waiting...')
        self.request = Start.Request()

    def send_request(self):

        self.call = self.client_start.call_async(self.request)

def main(args=None):

    rclpy.init(args=args)

    client_start = ClientStart()
    client_start.send_request()

    while rclpy.ok():

        rclpy.spin_once(client_start)

        if client_start.call.done():
            try:
                response = client_start.call.result()
            except Exception as e:
                client_start.get_logger().info(f'start call failed: {e}')
            else:
                client_start.get_logger().info(f'Successfully made the start call with status: {response.reply}')

            break
    
    client_start.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()