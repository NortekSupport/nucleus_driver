import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.srv import ConnectTcp


class ClientConnectTcp(Node):

    def __init__(self):

        super().__init__('connect_tcp')

        self.client = self.create_client(ConnectTcp, srv_name='nucleus_node/connect_tcp')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('connect tcp service not available. Waiting...')

        self.request = ConnectTcp.Request()

    def send_request(self, host: str, password: str = None, timeout_sec=None):
        
        self.request.host = host
        self.request.password = password  

        self.call = self.client.call_async(self.request)
        
        if self.executor is not None:
            self.executor.spin_until_future_complete(self.call, timeout_sec=timeout_sec)
        else:
            self.get_logger().warning('This client is not added to an executor. Establishing a temporary SingleThreadedExecutor for this call')
            executor = SingleThreadedExecutor()
            executor.add_node(self)
            executor.spin_until_future_complete(self.call, timeout_sec=timeout_sec)
            executor.shutdown()

        return self.call.result()


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
    
    rclpy.init()

    client = ClientConnectTcp()

    executor = SingleThreadedExecutor()
    executor.add_node(client)

    response = client.send_request(host=host, password=password)

    client.get_logger().info(f'Successfully made the connect tcp call with status: {response.status}')

    executor.shutdown()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()