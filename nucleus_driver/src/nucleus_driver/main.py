import argparse
import os
import cmd2
from cmd2 import Fg, style, Cmd2ArgumentParser, with_argparser, with_category

from nucleus_driver import NucleusDriver


class App(cmd2.Cmd):

    CMD_CAT_CONNECTING = 'Connection'
    CMD_CAT_LOGGING = 'Logging'
    CMD_CAT_COMMAND = 'Command'
    CMD_CAT_FLASH = 'Flash'

    def __init__(self, nucleus_driver):
        super().__init__()

        self.nucleus_driver = nucleus_driver

        self.prompt = style('nucleus> ', fg=Fg.GREEN)

        intro = ["  _   _            _                  _____       _                \r\n",
                 " | \ | |          | |                |  __ \     (_)               \r\n",
                 " |  \| |_   _  ___| | ___ _   _ ___  | |  | |_ __ ___   _____ _ __ \r\n",
                 " | . ` | | | |/ __| |/ _ \ | | / __| | |  | | '__| \ \ / / _ \ '__|\r\n",
                 " | |\  | |_| | (__| |  __/ |_| \__ \ | |__| | |  | |\ V /  __/ |   \r\n",
                 " |_| \_|\__,_|\___|_|\___|\__,_|___/ |_____/|_|  |_| \_/ \___|_|   \r\n"]

        self.intro = style(''.join(intro), fg=Fg.BLUE, bold=True)

        del cmd2.Cmd.do_edit
        del cmd2.Cmd.do_alias
        del cmd2.Cmd.do_macro
        del cmd2.Cmd.do_run_pyscript
        del cmd2.Cmd.do_run_script
        #del cmd2.Cmd.do_set
        del cmd2.Cmd.do_shell
        del cmd2.Cmd.do_shortcuts

    connect_parser = Cmd2ArgumentParser(description='Connect to nucleus device')
    connect_parser.add_argument('connection_type', choices=['serial', 'tcp'], help='connect to nucleus device through serial or tcp')

    @with_argparser(connect_parser)
    @with_category(CMD_CAT_CONNECTING)
    def do_connect(self, connect_args):

        def serial_configuration() -> bool:

            if args.connection_serial_port is None:

                port = self.nucleus_driver.connection.select_serial_port()

                if port is None:
                    return False

                self.nucleus_driver.connection.set_serial_configuration(port=port)

            return True

        def tcp_configuration() -> bool:

            if args.connection_tcp_host is None:

                self.nucleus_driver.messages.write_message('\nConnect through TCP with host(hostname/ip) or serial number: ')
                self.nucleus_driver.messages.write_message('[0] Host')
                self.nucleus_driver.messages.write_message('[1] Serial_number')
                reply = input('Input integer value in range [0:1]: ')

                if reply == '0':
                    host = input('\ntcp - host: ')

                    if host == '':
                        return False

                    self.nucleus_driver.connection.set_tcp_configuration(host=host)

                elif reply == '1':
                    serial_number = input('\ntcp - serial number: ')

                    try:
                        int(serial_number)
                        condition = self.nucleus_driver.connection.set_tcp_hostname_from_serial_number(serial_number=serial_number)
                    except ValueError:
                        self.nucleus_driver.messages.write_warning('serial number is not integer, host name will not be set')
                        condition = False

                    if not condition:
                        return condition

                else:
                    self.nucleus_driver.messages.write_message('Invalid selection')
                    return False

            return True

        config_status = False

        if connect_args.connection_type == 'serial':
            config_status = serial_configuration()

        elif connect_args.connection_type == 'tcp':
            config_status = tcp_configuration()

        if config_status:
            status = self.nucleus_driver.connect(connection_type=connect_args.connection_type)
        else:
            status = False

        print('Connection status: {}'.format(status))

    disconnect_parser = Cmd2ArgumentParser(description='Disconnect from the nucleus device')

    @with_argparser(disconnect_parser)
    @with_category(CMD_CAT_CONNECTING)
    def do_disconnect(self, _):

        status = self.nucleus_driver.disconnect()
        print('Connection status: {}'.format(not status))

    start_measurement_parser = Cmd2ArgumentParser(description='Start measurement')

    @with_argparser(start_measurement_parser)
    @with_category(CMD_CAT_LOGGING)
    def do_start_measurement(self, _):

        if self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.start_logging()
            self.nucleus_driver.start_measurement()
        else:
            self.nucleus_driver.messages.write_message('Nucleus not connected')

    start_spectrum_analyzer_parser = Cmd2ArgumentParser(description='Start spectrum analyzer')

    @with_argparser(start_spectrum_analyzer_parser)
    @with_category(CMD_CAT_LOGGING)
    def do_start_spectrum_analyzer(self, _):

        if self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.start_logging()
            self.nucleus_driver.start_spectrum_analyzer()
        else:
            self.nucleus_driver.messages.write_message('Nucleus not connected')

    start_field_calibration_parser = Cmd2ArgumentParser(description='Start field calibration')

    @with_argparser(start_field_calibration_parser)
    @with_category(CMD_CAT_LOGGING)
    def do_start_field_calibration(self, _):

        if self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.start_logging()
            self.nucleus_driver.start_fieldcal()
        else:
            self.nucleus_driver.messages.write_message('Nucleus not connected')

    stop_measurement_parser = Cmd2ArgumentParser(description='Stop measurement')

    @with_argparser(stop_measurement_parser)
    @with_category(CMD_CAT_LOGGING)
    def do_stop(self, _):

        if self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.stop_logging()
            self.nucleus_driver.stop()
        else:
            self.nucleus_driver.messages.write_message('Nucleus not connected')

    set_log_path_parser = Cmd2ArgumentParser(description='Set path for log files')
    set_log_path_parser.add_argument('path', help='Set path for log files', completer=cmd2.Cmd.path_complete)

    @with_argparser(set_log_path_parser)
    @with_category(CMD_CAT_LOGGING)
    def do_set_log_path(self, set_log_path_args):

        path = set_log_path_args.path

        if not os.access(path, os.W_OK):
            self.nucleus_driver.messages.write_warning('You do not have writing access to specified path')
        else:
            self.nucleus_driver.set_log_path(path=path)

    command_parser = Cmd2ArgumentParser(description='Send command to nucleus')
    command_parser.add_argument('command', help='send command to nucleus device and print response')

    @with_argparser(command_parser)
    @with_category(CMD_CAT_COMMAND)
    def do_command(self, command_args):

        command = command_args.command

        if self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.send_command(command)
        else:
            self.nucleus_driver.messages.write_message('Nucleus not connected')

    set_flash_file_parser = Cmd2ArgumentParser(description='Set path for flash files')
    set_flash_file_parser.add_argument('path', help='Set flash file. Extension must be in [.bin, .ldr, .zip]', completer=cmd2.Cmd.path_complete)

    @with_argparser(set_flash_file_parser)
    @with_category(CMD_CAT_FLASH)
    def do_set_flash_file(self, set_flash_path_args):

        path = set_flash_path_args.path

        if not os.access(path, os.R_OK):
            self.nucleus_driver.messages.write_warning('You do not have writing access to specified path')
        else:
            self.nucleus_driver.set_flash_files(path=path)

    reset_flash_files_parser = Cmd2ArgumentParser(description='Reset flash files')

    @with_argparser(reset_flash_files_parser)
    @with_category(CMD_CAT_FLASH)
    def do_reset_flash_file(self, _):

        self.nucleus_driver.reset_flash_files()

    flash_firmware_parser = Cmd2ArgumentParser(description='Flash firmware')

    @with_argparser(flash_firmware_parser)
    @with_category(CMD_CAT_FLASH)
    def do_flash_firmware(self, _):

        self.nucleus_driver.flash_firmware()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--connection-type', default=None, help='connection type (default=None)')
    parser.add_argument('--connection-serial-port', default=None, help='serial com port (default=None)')
    parser.add_argument('--connection-tcp-host', default=None, help='tcp host (default=None)')
    parser.add_argument('--logging-set-path', default=None, help='set log files path (default=None)')
    args = parser.parse_args()

    nucleus_driver = NucleusDriver()
    nucleus_driver.parser.set_queuing(ascii=True)

    if args.connection_serial_port is not None:
        nucleus_driver.connection.set_serial_configuration(port=args.connection_serial_port)

    if args.connection_tcp_host is not None:
        nucleus_driver.connection.set_tcp_configuration(host=args.connection_tcp_host)

    if args.connection_type is not None:
        if args.connection_type == 'serial':

            if nucleus_driver.connection.serial_configuration.port is None:
                serial_port = nucleus_driver.connection.select_serial_port()

                if serial_port is not None:
                    nucleus_driver.set_serial_configuration(port=serial_port)

        elif args.connection_type == 'tcp':

            if nucleus_driver.connection.tcp_configuration.host is None:
                nucleus_driver.messages.write_message('\nConnect through TCP with host(hostname/ip) or serial number: ')
                nucleus_driver.messages.write_message('[0] Host')
                nucleus_driver.messages.write_message('[1] Serial_number')
                reply = input('Input integer value in range [0:1]: ')

                if reply == '0':
                    host = input('\nhost: ')

                    if not host == '':
                        nucleus_driver.connection.set_tcp_configuration(host=host)
                    else:
                        nucleus_driver.messages.write_message('No host specified')

                elif reply == '1':
                    serial_number = input('\nserial number: ')

                    try:
                        int(serial_number)
                        condition = nucleus_driver.connection.set_tcp_hostname_from_serial_number(serial_number=serial_number)
                    except ValueError:
                        nucleus_driver.messages.write_warning('serial number is not integer, host name will not be set')

                else:
                    nucleus_driver.messages.write_message('Invalid selection')

        nucleus_driver.connection.connect(connection_type=args.connection_type)

    if args.logging_set_path is not None:
        nucleus_driver.logger.set_path(path=args.logging_set_path)

    nucleus_driver.parser.set_queuing(ascii=True)

    app = App(nucleus_driver=nucleus_driver)
    app.cmdloop()