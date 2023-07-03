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

    def send_request(self, command):
        
        self.request.command = command

        self.call = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.call)
        
        return self.call.result()


def call(command: str) -> object:

    rclpy.init()

    client = ClientCommand()
    response = client.send_request(command=command)

    client.get_logger().info(f'Successfully made the command call with reply: {response.reply}')

    client.destroy_node()
    rclpy.shutdown()

    return response

def main():

    try:
        command=str(sys.argv[1])
    except IndexError:
        print(f'Argument "command" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument: {e}')
        return
    
    response = call(command=command)

    print(f'call response: {response.reply}')

if __name__ == '__main__':
    main()