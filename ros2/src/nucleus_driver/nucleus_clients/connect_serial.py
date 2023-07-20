import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.srv import ConnectSerial


class ClientConnectSerial(Node):

    def __init__(self):

        super().__init__('connect_serial')

        self.client = self.create_client(ConnectSerial, srv_name='nucleus_node/connect_serial')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('connect serial service not available. Waiting...')

        self.request = ConnectSerial.Request()

    def send_request(self, serial_port: str, timeout_sec=None):

        self.request.serial_port = serial_port

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
        serial_port = str(sys.argv[1])
    except IndexError:
        print(f'Argument "serial_port" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument for serial_port: {e}')
        return
    
    rclpy.init()

    client = ClientConnectSerial()

    executor = SingleThreadedExecutor()
    executor.add_node(client)

    response = client.send_request(serial_port=serial_port)

    client.get_logger().info(f'{response.status}')

    executor.shutdown()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()