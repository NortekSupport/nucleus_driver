import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import Disconnect

class ClientDisconnect(Node):

    def __init__(self):

        super().__init__('client_disconnect')

        self.client = self.create_client(Disconnect, 'disconnect')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('disconnect service not available. Waiting...')
        
        self.request = Disconnect.Request()

    def send_request(self):

        self.call = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.call)
        
        return self.call.result()
   

def call() -> bool:

    rclpy.init()

    client = ClientDisconnect()
    response = client.send_request()

    client.get_logger().info(f'Successfully made the disconnect serial call with status: {response.status}')

    client.destroy_node()
    rclpy.shutdown()

    return response
 
def main():

    response = call()

    print(f'call response: {response.status}')

if __name__ == '__main__':
    main()