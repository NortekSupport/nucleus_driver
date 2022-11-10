from datetime import datetime
from pathlib import Path
import os


class Assert:

    def __init__(self, **kwargs):

        self.messages = kwargs.get('messages')
        self.connection = kwargs.get('connection')
        self.commands = kwargs.get('commands')

        self.assert_encrypted = b''
        self._path = str(Path.cwd()) + '/assert'

    def set_file_path(self, path: str):

        self._path = path.rstrip('/') + '/assert'

        return self._path

    def read_assert(self, timeout=1) -> bytes:

        if not self.connection.get_connection_status():
            self.messages.write_message('Nucleus is not connected')
            return b''

        reply = self.commands.read_assert(timeout=timeout)

        if len(reply) < 2:
            self.messages.write_message('Did not receive any assert messages')
            return b''

        if reply[1] != b'OK\r\n':
            self.messages.write_message('Did not receive OK after assert readout')
            return b''

        if len(reply[0]) == 0:
            self.messages.write_message('No assert message')
            return b''

        if len(reply[0]) < 16:
            self.messages.write_message('Assert message too short')
            return b''

        self.assert_encrypted = reply[0]

        return self.assert_encrypted

    def clear_assert(self) -> [bytes]:

        self.assert_encrypted = b''
        return self.commands.clear_assert()

    def write_encrypted_assert_to_file(self):

        if len(self.assert_encrypted) < 1:
            self.messages.write_message('assert encrypted is empty')
            return

        file_path = self._path + '/' + datetime.now().strftime('%y%m%d_%H%M%S') + '_assert_encrypted.txt'

        os.makedirs(self._path, exist_ok=True)

        with open(file_path, 'wb') as file:
            file.write(self.assert_encrypted)
            self.messages.write_message('Wrote data to file: {}'.format(file_path))

        return file_path

