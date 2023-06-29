import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import ReadPacket

class ClientReadPacket(Node):

    def __init__(self):

        super().__init__('client_read_packet')

        self.client_read_packet = self.create_client(ReadPacket, 'read_packet')
        while not self.client_read_packet.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('read_packet service not available. Waiting...')
        self.request = ReadPacket.Request()

    def send_request(self):

        self.request.size = int(sys.argv[1])
        
        self.call = self.client_read_packet.call_async(self.request)

def main(args=None):

    rclpy.init(args=args)

    client_read_packet = ClientReadPacket()
    client_read_packet.send_request()

    while rclpy.ok():

        rclpy.spin_once(client_read_packet)

        if client_read_packet.call.done():
            try:
                response = client_read_packet.call.result()
            except Exception as e:
                client_read_packet.get_logger().info(f'read_packet call failed: {e}')
            else:
                client_read_packet.get_logger().info(f'Successfully made the read_packet call with status: {response.packet_list}')

            break
    
    client_read_packet.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()