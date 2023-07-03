import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import Stop

class ClientStop(Node):

    def __init__(self):

        super().__init__('client_stop')

        self.client = self.create_client(Stop, 'stop')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('stop service not available. Waiting...')

        self.request = Stop.Request()

    def send_request(self):

        self.call = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.call)
        
        return self.call.result()

def call() -> bool:

    rclpy.init()

    client = ClientStop()
    response = client.send_request()

    client.get_logger().info(f'Successfully made the stop call with status: {response.reply}')

    client.destroy_node()
    rclpy.shutdown()

    return response

def main():

    response = call()

    print(f'call response: {response.reply}')


if __name__ == '__main__':
    main()