import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import ConnectSerial

class ClientConnectSerial(Node):

    def __init__(self):

        super().__init__('client_connect_serial')

        self.client = self.create_client(ConnectSerial, 'connect_serial')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('connect serial service not available. Waiting...')
        self.request = ConnectSerial.Request()

    def send_request(self):
        self.request.serial_port = str(sys.argv[1])

        self.call = self.client.call_async(self.request)

def main(args=None):

    rclpy.init(args=args)

    client = ClientConnectSerial()
    client.send_request()

    while rclpy.ok():

        rclpy.spin_once(client)

        if client.call.done():
            try:
                response = client.call.result()
            except Exception as e:
                client.get_logger().info(f'connect serial call failed: {e}')
            else:
                client.get_logger().info(f'Successfully made the connect serial call with status: {response.status}')

            break
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()