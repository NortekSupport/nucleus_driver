import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import Disconnect

class ClientDisconnect(Node):

    def __init__(self):

        super().__init__('client_disconnect')

        self.client_disconnect = self.create_client(Disconnect, 'disconnect')
        while not self.client_disconnect.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('disconnect service not available. Waiting...')
        self.request = Disconnect.Request()

    def send_request(self):

        self.call = self.client_disconnect.call_async(self.request)

def main(args=None):

    rclpy.init(args=args)

    client_disconnect = ClientDisconnect()
    client_disconnect.send_request()

    while rclpy.ok():

        rclpy.spin_once(client_disconnect)

        if client_disconnect.call.done():
            try:
                response = client_disconnect.call.result()
            except Exception as e:
                client_disconnect.get_logger().info(f'disconnect call failed: {e}')
            else:
                client_disconnect.get_logger().info(f'Successfully made the disconnect call with status: {response.status}')

            break
    
    client_disconnect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()