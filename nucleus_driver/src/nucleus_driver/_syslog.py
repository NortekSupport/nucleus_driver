from pathlib import Path
from datetime import datetime
import os


class Syslog:

    def __init__(self, **kwargs):

        self.messages = kwargs.get('messages')
        self.connection = kwargs.get('connection')
        self.commands = kwargs.get('commands')

        self.syslog_encrypted = b''
        self._path = str(Path.cwd()) + '/syslog'

    def set_file_path(self, path: str):

        self._path = path.rstrip('/') + '/syslog'

        return self._path

    def list_syslog(self):

        if not self.connection.get_connection_status():
            self.messages.write_message('Nucleus is not connected')
            return

        syslog_list = self.commands.list_syslog()

        for entry in syslog_list:
            if entry != b'OK\r\n':
                self.messages.write_message(entry)

    def read_syslog(self):

        if not self.connection.get_connection_status():
            self.messages.write_message('Nucleus is not connected')
            return list()

        reply = self.commands.read_syslog()

        if len(reply) < 2:
            self.messages.write_message('Did not receive any syslog entries')
            return list()

        if reply[1] != b'OK\r\n':
            self.messages.write_message('Did not receive OK after syslog readout')
            return list()

        if len(reply[0]) == 0:
            self.messages.write_message('No syslog entry')
            return list()

        if len(reply[0]) < 16:
            self.messages.write_message('syslog entries too short')
            return list()

        self.syslog_encrypted = reply[0]

        return self.syslog_encrypted

    def clear_syslog(self) -> [bytes]:

        self.syslog_encrypted = b''
        return self.commands.clear_syslog()

    def write_encrypted_syslog_to_file(self, path=None):

        if len(self.syslog_encrypted) < 1:
            self.messages.write_message('Encrypted syslog is empty')
            return

        if path is None:
            path = self._path

        if path.endswith('.txt'):
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            file_path = path
        else:
            Path(path).mkdir(parents=True, exist_ok=True)
            file_path = '{}/{}_syslog_encrypted.txt'.format(path.rstrip('/'), datetime.now().strftime('%y%m%d_%H%M%S'))

        with open(file_path, 'wb') as file:
            file.write(self.syslog_encrypted)
            self.messages.write_message('Wrote data to file: {}'.format(file_path))

        return file_path
