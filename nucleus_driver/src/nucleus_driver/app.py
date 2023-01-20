import os
import cmd2
from cmd2 import Fg, style, Cmd2ArgumentParser, with_argparser, with_category

from nucleus_driver import NucleusDriver


class App(cmd2.Cmd):

    CMD_CAT_CONNECTING = 'Connection'
    CMD_CAT_LOGGING = 'Logging'
    CMD_CAT_COMMAND = 'Command'
    CMD_CAT_FLASH = 'Flash'
    CMD_CAT_SYSLOG = 'Syslog'
    CMD_CAT_ASSERT = 'Assert'
    CMD_CAT_DOWNLOAD = 'Download'

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

            port = self.nucleus_driver.connection.select_serial_port()

            if port is None:
                return False

            self.nucleus_driver.connection.set_serial_configuration(port=port)

            return True

        def tcp_configuration() -> bool:

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
        password = None

        if connect_args.connection_type == 'serial':
            config_status = serial_configuration()

        elif connect_args.connection_type == 'tcp':
            config_status = tcp_configuration()
            if config_status:
                password = input('\ntcp - password: ')

        if not config_status:
            self.nucleus_driver.messages.write_warning('Failed to set connection configuration')
            return

        if self.nucleus_driver.connect(connection_type=connect_args.connection_type, password=password):
            self.nucleus_driver.messages.write_message('\r\nSuccessfully connected to Nucleus device\r\n')
            self.nucleus_driver.messages.write_message('ID:    {}'.format(self.nucleus_driver.connection.nucleus_id))
            self.nucleus_driver.messages.write_message('GETFW: {}\r\n'.format(self.nucleus_driver.connection.firmware_version))
        else:
            self.nucleus_driver.messages.write_warning('\r\nFailed to connect to Nucleus device\r\n')

    disconnect_parser = Cmd2ArgumentParser(description='Disconnect from the nucleus device')

    @with_argparser(disconnect_parser)
    @with_category(CMD_CAT_CONNECTING)
    def do_disconnect(self, _):

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('No device connected')
            return

        if self.nucleus_driver.disconnect():
            self.nucleus_driver.messages.write_message('Disconnected from Nucleus device')
        else:
            self.nucleus_driver.messages.write_warning('Failed to disconnect from Nucleus Device')

    start_measurement_parser = Cmd2ArgumentParser(description='Start measurement')
    start_measurement_parser.add_argument('-p', '--path', help='Set path for log files', completer=cmd2.Cmd.path_complete)

    @with_argparser(start_measurement_parser)
    @with_category(CMD_CAT_LOGGING)
    def do_start(self, start_measurement_args):

        path = start_measurement_args.path

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Logging thread is already running')
            return

        if path is not None:
            self.nucleus_driver.set_log_path(path=path)

        self.nucleus_driver.start_logging()
        self.nucleus_driver.start_measurement()

    start_field_calibration_parser = Cmd2ArgumentParser(description='Start field calibration')
    start_field_calibration_parser.add_argument('-p', '--path', help='Set path for log files', completer=cmd2.Cmd.path_complete)

    @with_argparser(start_field_calibration_parser)
    @with_category(CMD_CAT_LOGGING)
    def do_fieldcal(self, start_field_calibration_args):

        path = start_field_calibration_args.path

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Logging thread is already running')
            return

        if path is not None:
            self.nucleus_driver.set_log_path(path=path)

        self.nucleus_driver.start_logging()
        self.nucleus_driver.start_fieldcal()

    stop_measurement_parser = Cmd2ArgumentParser(description='Stop measurement')

    @with_argparser(stop_measurement_parser)
    @with_category(CMD_CAT_LOGGING)
    def do_stop(self, _):

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        self.nucleus_driver.stop_logging()
        self.nucleus_driver.stop()

    command_parser = Cmd2ArgumentParser(description='Send command to nucleus')
    command_parser.add_argument('command', help='send command to nucleus device and print response')

    @with_argparser(command_parser)
    @with_category(CMD_CAT_COMMAND)
    def do_command(self, command_args):

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Can not send command to Nucleus while logging thread is running')
            return

        command = command_args.command

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        self.nucleus_driver.send_command(command)

    flash_firmware_parser = Cmd2ArgumentParser(description='Flash firmware')
    flash_firmware_parser.add_argument('path', help='Set flash file. Extension must be in [.bin, .ldr, .zip]', completer=cmd2.Cmd.path_complete)

    @with_argparser(flash_firmware_parser)
    @with_category(CMD_CAT_FLASH)
    def do_flash_firmware(self, set_flash_path_args):

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Can not flash Nucleus while logging thread is running')
            return

        path = set_flash_path_args.path

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        password = None
        if self.nucleus_driver.connection.get_connection_type() == 'tcp':
            password = input('password: ')

        self.nucleus_driver.flash_firmware(path=path, password=password)

    ###########################################
    # Assert
    ###########################################

    download_asserts_parser = Cmd2ArgumentParser(description='Download asserts')
    download_asserts_parser.add_argument('-p', '--path', help='Set path for assert file download', completer=cmd2.Cmd.path_complete)

    @with_argparser(download_asserts_parser)
    @with_category(CMD_CAT_ASSERT)
    def do_assert_download(self, download_asserts_args):

        path = download_asserts_args.path

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Can not download asserts while logging thread is running')
            return

        if not self.nucleus_driver.assert_download(path=path):
            self.nucleus_driver.messages.write_warning('Did not receive any asserts from Nucleus')

    clear_asserts_parser = Cmd2ArgumentParser(description='Clear asserts from Nucleus device')

    @with_argparser(clear_asserts_parser)
    @with_category(CMD_CAT_ASSERT)
    def do_assert_clear(self, _):

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Can not clear asserts while logging thread is running')
            return

        if b'OK\r\n' in self.nucleus_driver.clear_assert():
            self.nucleus_driver.messages.write_message('Successfully cleared asserts')
        else:
            self.nucleus_driver.messages.write_warning('Clear assert did not reply with OK: {}'.format(response))

    ##########################################
    # Syslog
    ###########################################

    download_syslog_parser = Cmd2ArgumentParser(description='Download syslog')
    download_syslog_parser.add_argument('-p', '--path', help='Set path for syslog file download', completer=cmd2.Cmd.path_complete)

    @with_argparser(download_syslog_parser)
    @with_category(CMD_CAT_SYSLOG)
    def do_syslog_download(self, download_syslog_args):

        path = download_syslog_args.path

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Can not download syslog while logging thread is running')
            return

        if not self.nucleus_driver.syslog_download(path=path):
            self.nucleus_driver.messages.write_warning('Did not receive any syslog entries from Nucleus')

    clear_syslog_parser = Cmd2ArgumentParser(description='Clear syslog from Nucleus device')

    @with_argparser(clear_syslog_parser)
    @with_category(CMD_CAT_SYSLOG)
    def do_syslog_clear(self, _):

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Can not clear syslog while logging thread is running')
            return

        response = self.nucleus_driver.clear_syslog()

        for message in response:
            self.nucleus_driver.messages.write_message(message)

    ##########################################
    # Download
    ###########################################

    download_dvl_parser = Cmd2ArgumentParser(description='Download dvl diagnostics data')
    download_dvl_parser.add_argument('-f', '--fid', help='id of file to download')
    download_dvl_parser.add_argument('-s', '--sa', help='start byte address of file to download')
    download_dvl_parser.add_argument('-l', '--length', help='length of bytes in file to download')
    download_dvl_parser.add_argument('-p', '--path', help='path for file to be downloaded', completer=cmd2.Cmd.path_complete)

    @with_argparser(download_dvl_parser)
    @with_category(CMD_CAT_DOWNLOAD)
    def do_download_dvl_data(self, download_dvl_args):

        fid = download_dvl_args.fid
        sa = download_dvl_args.sa
        length = download_dvl_args.length
        path = download_dvl_args.path

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Can not download DVL diagnostics data while logging thread is running')
            return

        if fid is not None:
            fid = int(fid)

        if sa is not None:
            sa = int(sa)

        if length is not None:
            length = int(length)

        status = self.nucleus_driver.download_dvl_data(fid=fid, sa=sa, length=length, path=path)

        if status:
            self.nucleus_driver.messages.write_message('Successfully downloaded data')
        else:
            self.nucleus_driver.messages.write_warning('Failed to completely download data')

    download_nucleus_parser = Cmd2ArgumentParser(description='Download nucleus data')
    download_nucleus_parser.add_argument('-f', '--fid', help='id of file to download')
    download_nucleus_parser.add_argument('-s', '--sa', help='start byte address of file to download')
    download_nucleus_parser.add_argument('-l', '--length', help='length of bytes in file to download')
    download_nucleus_parser.add_argument('-p', '--path', help='path for file to be downloaded', completer=cmd2.Cmd.path_complete)

    @with_argparser(download_nucleus_parser)
    @with_category(CMD_CAT_DOWNLOAD)
    def do_download_nucleus_data(self, download_nucleus_args):

        fid = download_nucleus_args.fid
        sa = download_nucleus_args.sa
        length = download_nucleus_args.length
        path = download_nucleus_args.path

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Can not download Nucleus data while logging thread is running')
            return

        if fid is not None:
            fid = int(fid)

        if sa is not None:
            sa = int(sa)

        if length is not None:
            length = int(length)

        status = self.nucleus_driver.download_nucleus_data(fid=fid, sa=sa, length=length, path=path)

        if status:
            self.nucleus_driver.messages.write_message('Successfully downloaded data')
        else:
            self.nucleus_driver.messages.write_warning('Failed to completely download data')

    convert_nucleus_data_parser = Cmd2ArgumentParser(description='Nucleus binary file path')
    convert_nucleus_data_parser.add_argument('path', help='Select path for nucleus binary file', completer=cmd2.Cmd.path_complete)

    @with_argparser(convert_nucleus_data_parser)
    @with_category(CMD_CAT_DOWNLOAD)
    def do_convert_nucleus_data(self, convert_nucleus_data_args):

        path = convert_nucleus_data_args.path

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Can not convert Nucleus data while logging thread is running')
            return

        status = self.nucleus_driver.convert_nucleus_data(path=path)

        if status:
            self.nucleus_driver.messages.write_message('Data conversion complete')
        else:
            self.nucleus_driver.messages.write_warning('Failed to convert data')

    list_files_parser = Cmd2ArgumentParser(description='List Nucleus or DVL files')
    list_files_parser.add_argument('data_type', choices=['nucleus', 'dvl', 'getall'], help='List Nucleus data or DVL diagnostics data')

    @with_argparser(list_files_parser)
    @with_category(CMD_CAT_DOWNLOAD)
    def do_list_files(self, list_files_args):

        data_type = list_files_args.data_type

        if not self.nucleus_driver.connection.get_connection_status():
            self.nucleus_driver.messages.write_message('Nucleus not connected')
            return

        if self.nucleus_driver.parser.thread.is_alive():
            self.nucleus_driver.messages.write_message('Can not get file list while logging thread is running')
            return

        src = None
        if data_type == 'nucleus':
            src = 0
        if data_type == 'dvl':
            src = 1
        if data_type == 'getall':
            src = 2

        response = self.nucleus_driver.commands.list_files(src=src)

        for entry in response:
            self.nucleus_driver.messages.write_message(entry)


def nucleus_driver_console():

    nucleus_driver = NucleusDriver()
    nucleus_driver.parser.set_queuing(packet=False, ascii=True, condition=False)

    app = App(nucleus_driver=nucleus_driver)

    app.cmdloop()
