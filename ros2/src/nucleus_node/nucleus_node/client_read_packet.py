import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import ReadPacket

class ClientReadPacket(Node):

    def __init__(self):

        super().__init__('client_read_packet')

        self.client = self.create_client(ReadPacket, 'read_packet')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('read_packet service not available. Waiting...')
        self.request = ReadPacket.Request()

    def send_request(self):

        self.request.size = int(sys.argv[1])
        
        self.call = self.client.call_async(self.request)

def main(args=None):

    rclpy.init(args=args)

    client = ClientReadPacket()
    client.send_request()

    while rclpy.ok():

        rclpy.spin_once(client)

        if client.call.done():
            try:
                response = client.call.result()
            except Exception as e:
                client.get_logger().info(f'read_packet call failed: {e}')
            else:
                client.get_logger().info(f'Successfully made the read_packet call with status: {response.packet_list}')

            break
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()