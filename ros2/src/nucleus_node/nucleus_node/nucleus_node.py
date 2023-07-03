import rclpy
from rclpy.node import Node

from interfaces.srv import ConnectTcp, ConnectSerial, Disconnect, Start, Stop, ReadPacket, Command
from interfaces.msg import Ahrs

import json

from nucleus_driver import NucleusDriver


class NucleusNode(Node):

    def __init__(self, nucleus_driver):
        super().__init__("nucleus_node")

        self.nucleus_driver = nucleus_driver

        self.connect_tcp_service = self.create_service(ConnectTcp, 'connect_tcp', self.connect_tcp_callback)
        self.connect_serial_service = self.create_service(ConnectSerial, 'connect_serial', self.connect_serial_callback)
        self.disconnect_service = self.create_service(Disconnect, 'disconnect', self.disconnect_callback)
        self.start_service = self.create_service(Start, 'start', self.start_callback)
        self.stop_service = self.create_service(Stop, 'stop', self.stop_callback)
        self.read_packet_service = self.create_service(ReadPacket, 'read_packet', self.read_packet_callback)
        self.command_service = self.create_service(Command, 'command', self.command_callback)

        self.ahrs_publisher = self.create_publisher(Ahrs, 'ahrs', 100)
        self.packet_timer = self.create_timer(0.01, self.packet_callback)

    def connect_tcp_callback(self, request, response):

        self.nucleus_driver.set_tcp_configuration(host=request.host)
        status = self.nucleus_driver.connect(connection_type='tcp', password=request.password)

        if status:
            self.get_logger().info(f'Connected through TCP with host: {request.host}')
        else:
            self.get_logger().info(f'Failed to connect with host: {request.host}')

        response.status = status

        return response

    def connect_serial_callback(self, request, response):
        
        self.nucleus_driver.set_serial_configuration(port=request.serial_port)
        status = self.nucleus_driver.connect(connection_type='serial')

        if status:
            self.get_logger().info(f'connected through serial with serial port: {request.serial_port}')
        else:
            self.get_logger().info(f'failed to connect through serial port: {request.serial_port}')

        response.status = status

        return response

    def disconnect_callback(self, request, response):

        status = self.nucleus_driver.disconnect()

        response.status = status

        if status:
            self.get_logger().info(f'Successfully disconnected from Nucleus')
        else:
            self.get_logger().info(f'Failed to disconnect from Nucleus')

        return response

    def start_callback(self, request, response):

        if not self.nucleus_driver.connection.get_connection_status():
            self.get_logger().info(f'Nucleus is not connected')
            response.reply = f'Nucleus is not connected'

        else:
            reply = self.nucleus_driver.start_measurement()
            
            try:
                response.reply = reply[0].decode()
                self.get_logger().info(f'start reply: {response.reply}')
            except Exception as e:
                response.reply = f'Failed to decode response from start command: {e}'

        return response
    
    def stop_callback(self, request, response):

        if not self.nucleus_driver.connection.get_connection_status():
            self.get_logger().info(f'Nucleus is not connected')
            response.reply = f'Nucleus is not connected'

        else:
            reply = self.nucleus_driver.stop()
            
            try:
                response.reply = reply[0].decode()
                self.get_logger().info(f'stop reply: {response.reply}')
            except Exception as e:
                response.reply = f'Failed to decode response from start command: {e}'

        return response

    def read_packet_callback(self, request, response):

        if not self.nucleus_driver.connection.get_connection_status():
            self.get_logger().info(f'Nucleus is not connected')
            #response.reply = f'Nucleus is not connected'

        self.get_logger().info(f'request.size: {request.size}')

        if int(request.size) > 1:
            size = request.size

        else:
            size = 1

        packet_list = list()

        self.get_logger().info(f'size: {size}')

        for _ in range(size):
            
            self.get_logger().info(f'reading packet')

            packet = self.nucleus_driver.read_packet()

            if packet is None:
                break

            packet_list.append(packet)

        response.packet_list = json.dumps(packet_list)

        return response
    
    def command_callback(self, request, response):

        if not self.nucleus_driver.connection.get_connection_status():
            self.get_logger().info(f'Nucleus is not connected')
            response.reply = f'Nucleus is not connected'

        else:
            command = request.command

            reply = self.nucleus_driver.send_command(command=command)
            
            try:
                for entry in reply:
                    response.reply += entry.decode()
                
                self.get_logger().info(f'command reply: {response.reply}')
            except Exception as e:
                response.reply = f'Failed to decode response from start command: {e}'

        return response

    def packet_callback(self):

        packet = self.nucleus_driver.read_packet()

        if packet is None:
            return
        
        if packet['id'] == 0xd2:

            ahrs_packet = Ahrs()

            ahrs_packet.roll = packet['ahrsData.roll']
            ahrs_packet.pitch = packet['ahrsData.pitch']
            ahrs_packet.heading = packet['ahrsData.heading']

            self.ahrs_publisher.publish(ahrs_packet)




def main():

    print('Hi from nucleus_driver.')

    nucleus_driver = NucleusDriver()

    rclpy.init()

    nucleus_node = NucleusNode(nucleus_driver=nucleus_driver)

    rclpy.spin(nucleus_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
