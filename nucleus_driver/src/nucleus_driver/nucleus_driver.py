import argparse
import time

from messages import Messages
from connection import Connection
from logger import Logger
from parser import Parser
from commands import Commands
from flash import Flash


class NucleusDriver:

    def __init__(self):

        self.messages = Messages()
        self.connection = Connection(self.messages)
        self.logger = Logger(messages=self.messages, connection=self.connection)
        self.parser = Parser(self.messages, self.logger, self.connection)
        self.commands = Commands(messages=self.messages, connection=self.connection, parser=self.parser)
        self.flash = Flash(self.messages, self.connection, self.commands)

        self.connection.commands = self.commands
        self.logger.commands = self.commands

        self.logging_fieldcal = False

    ###########################################
    # Connection
    ###########################################

    def set_serial_configuration(self, port=None):

        self.connection.set_serial_configuration(port=port)

    def set_tcp_configuration(self, host=None):

        self.connection.set_tcp_configuration(host=host)

    def connect(self, connection_type) -> bool:

        CONNECTION_TYPES = ['serial', 'tcp']

        if connection_type not in CONNECTION_TYPES:
            self.messages.write_warning('Connection type {} not in {}'.format(connection_type, CONNECTION_TYPES))
            return False

        return self.connection.connect(connection_type=connection_type)

    def disconnect(self) -> bool:

        return self.connection.disconnect()

    ###########################################
    # Command
    ###########################################

    def send_command(self, command: str):

        command = command.rstrip('\n').rstrip('\r')

        if command.upper() == 'START':
            self.messages.write_message('start command is only supported through logging')

        elif command.upper() == 'FIELDCAL':

            self.messages.write_message('fieldcal command is only supported through logging')

        elif command.upper() == 'STARTSPECTRUM':

            self.messages.write_message('startspectrum command is only supported through logging')

        elif command.upper() == 'STOP':

            self.messages.write_message('stop command is only supported through logging')

        else:
            self.connection.write(command.encode() + b'\r\n')
            for i in range(100):
                command_reply = self.connection.readline()
                if command_reply == b'':
                    break
                if command_reply:
                    try:
                        self.messages.write_message(command_reply.decode())
                    except Exception as e:
                        self.messages.write_exception('Could not decode reply: {}'.format(command_reply))
                        self.messages.write_exception('Received error: {}'.format(e))

    ###########################################
    # Logging
    ###########################################

    def set_log_path(self, path):

        self.logger.set_path(path=path)

    def start_logging(self):

        self.logger.start()

    def stop_logging(self):

        self.logger.stop()

    ###########################################
    # Parser
    ###########################################

    def read_packet(self):

        packet = self.parser.read_packet()

        return packet

    ###########################################
    # Measurement
    ###########################################

    def start_measurement(self):

        self.logger.get_cp_nc()
        self.parser.start()
        #self.logger.start()
        self.commands.start()

    def start_spectrum_analyzer(self):

        self.parser.start()
        #self.logger.start()
        self.commands.start_spectrum()

    def start_fieldcal(self):

        self.parser.start()
        #self.logger.start()
        self.commands.fieldcal()

        self.logging_fieldcal = True

    def stop(self):

        #self.logger.stop()
        self.parser.stop()
        if self.logging_fieldcal:
            time.sleep(0.5)
        self.commands.stop()

        self.logging_fieldcal = False

    ###########################################
    # Flash
    ###########################################

    def set_flash_files(self, path: str):

        self.flash.set_flash_files(path=path)

    def reset_flash_files(self):

        self.flash.reset_flash_files()

    def flash_firmware(self):

        result = self.flash.flash_firmware()

        if result == 0:
            self.messages.write_message('Successfully flashed firmware')
        else:
            self.messages.write_warning('Failed to flash firmware with error: {}'.format(result))


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--connection-type', default=None, help='connection type (default=None)')
    parser.add_argument('--connection-serial-port', default=None, help='serial com port (default=None)')
    parser.add_argument('--connection-serial-baudrate', default=None, help='serial baudrate (default=None)')
    parser.add_argument('--connection-tcp-host', default=None, help='tcp host (default=None)')
    parser.add_argument('--connection-tcp-port', default=None, help='tcp port (default=None)')
    parser.add_argument('--logging-set-path', default=None, help='set log files path (default=None)')
    args = parser.parse_args()

    nucleus_driver = NucleusDriver()

    if args.connection_serial_port is not None:
        nucleus_driver.connection.set_serial_configuration(port=args.connection_serial_port)

    if args.connection_serial_baudrate is not None:
        nucleus_driver.connection.set_serial_configuration(baudrate=int(args.connection_serial_baudrate))

    if args.connection_tcp_host is not None:
        nucleus_driver.connection.set_tcp_configuration(host=args.connection_tcp_host)

    if args.connection_tcp_port is not None:
        nucleus_driver.connection.set_tcp_configuration(port=int(args.connection_tcp_port))

    if args.connection_type is not None:
        if args.connection_type == 'serial':

            if nucleus_driver.connection.serial_configuration.port is None:
                serial_port = nucleus_driver.connection.select_serial_port()
                nucleus_driver.connection.set_serial_configuration(port=serial_port)

            if nucleus_driver.connection.serial_configuration.baudrate is None:
                baudrate = input('serial baudrate: ')
                nucleus_driver.connection.set_serial_configuration(baudrate=int(baudrate))

        elif args.connection_type == 'tcp':

            if nucleus_driver.connection.tcp_configuration.host is None:
                host = input('TCP host: ')

                if host == '':
                    host = None

                nucleus_driver.connection.set_tcp_configuration(host=host)

            if nucleus_driver.connection.tcp_configuration.port is None:
                port = input('TCP port: ')

                nucleus_driver.connection.set_tcp_configuration(port=int(port))

        nucleus_driver.connection.connect(connection_type=args.connection_type)

    if args.logging_set_path is not None:
        nucleus_driver.logger.set_path(path=args.logging_set_path)

    nucleus_driver.parser.set_queuing(ascii=True)

    nucleus_driver.run_classes.main()
