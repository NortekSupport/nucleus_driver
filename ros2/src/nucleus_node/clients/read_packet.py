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

    def send_request(self, size):

        self.request.size = size
        
        self.call = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.call)
        
        return self.call.result()


def call(size: int):

    rclpy.init()

    client = ClientReadPacket()
    response = client.send_request(size=size)

    client.get_logger().info(f'Successfully made the connect serial call with status: {response.packet_list}')

    client.destroy_node()
    rclpy.shutdown()

    return response

def main():

    try:
        size = int(sys.argv[1])
    except IndexError:
        print(f'Argument "size" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument for size: {e}')
        return
    
    response = call(size=size)

    print(f'call response: {response.packet_list}')

if __name__ == '__main__':
    main()