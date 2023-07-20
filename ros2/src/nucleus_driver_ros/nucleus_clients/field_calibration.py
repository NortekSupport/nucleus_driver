import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.srv import StartFieldCalibration


class ClientFieldCalibration(Node):

    def __init__(self):

        super().__init__('field_calibration')

        self.client = self.create_client(StartFieldCalibration, srv_name='nucleus_node/field_calibration')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('field_calibration service not available. Waiting...')

        self.request = StartFieldCalibration.Request()

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

    client = ClientFieldCalibration()

    executor = SingleThreadedExecutor()
    executor.add_node(client)

    response = client.send_request()

    client.get_logger().info(f'Successfully made the field_calibration call with status: {response.reply}')

    executor.shutdown()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()