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

    def send_request(self, host: str, password: str = None):
        
        self.request.host = host
        self.request.password = password  

        self.call = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.call)
        
        return self.call.result()


def call(host: str, password: str) -> bool:

    rclpy.init()

    client = ClientConnectTcp()
    response = client.send_request(host=host, password=password)

    client.get_logger().info(f'Successfully made the connect tcp call with status: {response.status}')

    client.destroy_node()
    rclpy.shutdown()

    return response

def main():

    try:
        host = str(sys.argv[1])
    except IndexError:
        print(f'First argument "host" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument for host: {e}')
        return

    try:
        password = str(sys.argv[2])
    except IndexError:
        print(f'Second argument "password" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument for password: {e}')
        return
    
    # password may be None

    response = call(host=host, password=password)

    print(f'call response: {response.status}')

if __name__ == '__main__':
    main()