import argparse
import time

from nucleus_driver._messages import Messages
from nucleus_driver._connection import Connection
from nucleus_driver._logger import Logger
from nucleus_driver._parser import Parser
from nucleus_driver._commands import Commands
from nucleus_driver._flash import Flash
from nucleus_driver._assert import Assert
from nucleus_driver._syslog import Syslog
from nucleus_driver._download import Download


class NucleusDriver:

    def __init__(self):

        self.messages = Messages()
        self.connection = Connection(messages=self.messages)
        self.logger = Logger(messages=self.messages, connection=self.connection)
        self.parser = Parser(messages=self.messages, logger=self.logger, connection=self.connection)
        self.commands = Commands(messages=self.messages, connection=self.connection, parser=self.parser)
        self.flash = Flash(messages=self.messages, connection=self.connection, commands=self.commands)
        self.asserts = Assert(messages=self.messages, connection=self.connection, commands=self.commands)
        self.syslog = Syslog(messages=self.messages, connection=self.connection, commands=self.commands)
        self.download = Download(messages=self.messages, connection=self.connection, commands=self.commands, parser=self.parser, logger=self.logger)

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

    def connect(self, connection_type, password=None) -> bool:

        CONNECTION_TYPES = ['serial', 'tcp']

        if connection_type not in CONNECTION_TYPES:
            self.messages.write_warning('Connection type {} not in {}'.format(connection_type, CONNECTION_TYPES))
            return False

        return self.connection.connect(connection_type=connection_type, password=password)

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

    def read_ascii(self):

        packet = self.parser.read_ascii()

        return packet

    def read_condition(self):

        packet = self.parser.read_condition()

        return packet

    ###########################################
    # Measurement
    ###########################################

    def start_measurement(self):

        self.logger.get_cp_nc()
        self.parser.start()
        response = self.commands._start()

        return response

    def start_fieldcal(self):

        self.parser.start()
        response = self.commands._fieldcal()

        self.logging_fieldcal = True

        return response

    def stop(self):

        self.parser.stop()
        if self.logging_fieldcal:
            time.sleep(0.5)
        response = self.commands._stop(timeout=3)

        self.logging_fieldcal = False

        return response

    ###########################################
    # Flash
    ###########################################

    def flash_firmware(self, path: str, password: str=None) -> bool:

        self.flash.reset_flash_files()

        if not self.flash.set_flash_files(path=path):
            self.messages.write_warning('Was not able to set flash file: {}'.format(path))
            return False

        result = self.flash.flash_firmware(password=password)

        if result == 0:
            self.messages.write_message('Successfully flashed firmware')
            return True
        else:
            self.messages.write_warning('Failed to flash firmware with error: {}'.format(result))
            return True

    ###########################################
    # Assert
    ###########################################

    def set_assert_path(self, path: str):

        return self.asserts.set_file_path(path=path)

    def clear_assert(self) -> [bytes]:

        return self.asserts.clear_assert()

    def assert_download(self, path: str=None):

        self.asserts.read_assert()

        status = False
        if len(self.asserts.assert_encrypted) > 1:
            self.asserts.write_encrypted_assert_to_file(path=path)
            status = True

        return status

    ###########################################
    # Syslog
    ###########################################

    def set_syslog_path(self, path: str):

        return self.syslog.set_file_path(path=path)

    def clear_syslog(self) -> [bytes]:

        return self.syslog.clear_syslog()

    def syslog_download(self, path: str=None):

        self.syslog.read_syslog()

        status = False
        if len(self.syslog.syslog_encrypted) > 1:
            self.syslog.write_encrypted_syslog_to_file(path=path)
            status = True

        return status

    ###########################################
    # Download
    ###########################################

    def set_download_path(self, path: str):

        return self.download.set_path(path=path)

    def download_dvl_data(self, fid=None, sa=None, length=None, path=None):

        return self.download.download_dvl_data(fid=fid, sa=sa, length=length, path=path)

    def download_nucleus_data(self, fid=None, sa=None, length=None, path=None):

        return self.download.download_nucleus_data(fid=fid, sa=sa, length=length, path=path)

    def convert_nucleus_data(self, path):

        return self.download.convert_nucleus_data(path=path)

    def list_files(self, src=None):

        return self.commands.list_files(src=src)
