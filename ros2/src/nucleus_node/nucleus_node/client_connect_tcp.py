import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import ConnectTcp

class ClientConnectTcp(Node):

    def __init__(self):

        super().__init__('client_connect_tcp')

        self.client = self.create_client(ConnectTcp, 'connect_tcp')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('connect tcp service not available. Waiting...')
        self.request = ConnectTcp.Request()

    def send_request(self):
        self.request.host = str(sys.argv[1])
        self.request.password = str(sys.argv[2])

        self.call = self.client.call_async(self.request)

def main(args=None):

    rclpy.init(args=args)

    client = ClientConnectTcp()
    client.send_request()

    while rclpy.ok():

        rclpy.spin_once(client)

        if client.call.done():
            try:
                response = client.call.result()
            except Exception as e:
                client.get_logger().info(f'connect tcp call failed: {e}')
            else:
                client.get_logger().info(f'Successfully made the connect tcp call with status: {response.status}')

            break
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()