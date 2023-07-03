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

    def send_request(self, serial_port: str):

        self.request.serial_port = serial_port

        self.call = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.call)
        
        return self.call.result()


def call(serial_port: str) -> bool:

    rclpy.init()

    client = ClientConnectSerial()
    response = client.send_request(serial_port=serial_port)

    client.get_logger().info(f'Successfully made the connect serial call with status: {response.status}')

    client.destroy_node()
    rclpy.shutdown()

    return response

def main():

    try:
        serial_port = str(sys.argv[1])
    except IndexError:
        print(f'Argument "serial_port" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument for serial_port: {e}')
        return
    
    response = call(serial_port=serial_port)

    print(f'call response: {response.status}')

if __name__ == '__main__':
    main()