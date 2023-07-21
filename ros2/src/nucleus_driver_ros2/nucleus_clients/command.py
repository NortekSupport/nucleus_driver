import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from interfaces.srv import Command


class ClientCommand(Node):

    def __init__(self):

        super().__init__('command')

        self.client = self.create_client(Command, srv_name='nucleus_node/command')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command service not available. Waiting...')

        self.request = Command.Request()

    def send_request(self, command, timeout_sec=None):
        
        self.request.command = command

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
        command=str(sys.argv[1])
    except IndexError:
        print(f'Argument "command" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument: {e}')
        return

    rclpy.init()

    client = ClientCommand()

    executor = SingleThreadedExecutor()
    executor.add_node(client)

    response = client.send_request(command=command)

    client.get_logger().info(f'{response.reply}')

    executor.shutdown()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()