import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import Disconnect

class ClientDisconnect(Node):

    def __init__(self):

        super().__init__('client_disconnect')

        self.client = self.create_client(Disconnect, 'disconnect')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('disconnect service not available. Waiting...')
        self.request = Disconnect.Request()

    def send_request(self):

        self.call = self.client.call_async(self.request)

def main(args=None):

    rclpy.init(args=args)

    client = ClientDisconnect()
    client.send_request()

    while rclpy.ok():

        rclpy.spin_once(client)

        if client.call.done():
            try:
                response = client.call.result()
            except Exception as e:
                client.get_logger().info(f'disconnect call failed: {e}')
            else:
                client.get_logger().info(f'Successfully made the disconnect call with status: {response.status}')

            break
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()