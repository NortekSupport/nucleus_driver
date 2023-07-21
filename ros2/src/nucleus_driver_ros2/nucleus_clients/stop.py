import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.srv import Stop


class ClientStop(Node):

    def __init__(self):

        super().__init__('stop')

        self.client = self.create_client(Stop, srv_name='nucleus_node/stop')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('stop service not available. Waiting...')

        self.request = Stop.Request()

    def send_request(self, timeout_sec=None):

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

    rclpy.init()

    client = ClientStop()

    executor = SingleThreadedExecutor()
    executor.add_node(client)

    response = client.send_request()

    client.get_logger().info(f'Successfully made the stop call with status: {response.reply}')

    executor.shutdown()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()