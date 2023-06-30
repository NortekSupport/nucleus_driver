import sys
import rclpy
from rclpy.node import Node

from interfaces.srv import Command

class ClientCommand(Node):

    def __init__(self):

        super().__init__('client_command')

        self.client = self.create_client(Command, 'command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command service not available. Waiting...')
        self.request = Command.Request()

    def send_request(self):
        
        self.request.command = str(sys.argv[1])

        self.call = self.client.call_async(self.request)


def main(args=None):

    rclpy.init(args=args)

    client = ClientCommand()
    client.send_request()

    while rclpy.ok():

        rclpy.spin_once(client)

        if client.call.done():
            try:
                response = client.call.result()
            except Exception as e:
                client.get_logger().info(f'command call failed: {e}')
            else:
                client.get_logger().info(f'Successfully made the command call with status: {response.reply}')

            break
    
    client.destroy_node()
    rclpy.shutdown()
    
    return response

if __name__ == '__main__':
    main()