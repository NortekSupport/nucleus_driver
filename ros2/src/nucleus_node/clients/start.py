import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import Start

class ClientStart(Node):

    def __init__(self):

        super().__init__('client_start')

        self.client = self.create_client(Start, 'start')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('start service not available. Waiting...')

        self.request = Start.Request()

    def send_request(self):

        self.call = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.call)
        
        return self.call.result()


def call() -> bool:

    rclpy.init()

    client = ClientStart()
    response = client.send_request()

    client.get_logger().info(f'Successfully made the start call with status: {response.reply}')

    client.destroy_node()
    rclpy.shutdown()

    return response

def main():

    response = call()

    print(f'call response: {response.reply}')


if __name__ == '__main__':
    main()