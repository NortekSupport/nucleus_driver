from datetime import datetime
import time
from dataclasses import dataclass
import serial
from serial.tools import list_ports
import socket
import select
import errno


class Connection:
    CONNECTION_TYPES = ['serial', 'tcp']

    @dataclass
    class SerialConfiguration:
        port: str = None
        baudrate: int = 115200

    @dataclass
    class TcpConfiguration:
        host: str = None
        port: int = 9000

    def __init__(self, **kwargs):

        self.messages = kwargs.get('messages')
        self.commands = None

        self._connection_type = None
        self._connected = False

        self.serial = serial.Serial()
        self.tcp = socket.socket()

        self.serial_configuration = self.SerialConfiguration()
        self.tcp_configuration = self.TcpConfiguration()
        self.timeout = 1

        self.tcp_buffer = b''

        self.nucleus_id = None
        self.firmware_version = None
        self.get_all = None

    def get_connection_type(self) -> str:

        return self._connection_type

    def get_connection_status(self) -> bool:

        return self._connected

    def get_serial_port(self):

        port_info = list_ports.comports(include_links=False)
        ports = [port.device for port in port_info]

        return ports

    def select_serial_port(self):

        serial_port = None

        ports = self.get_serial_port()

        if bool(ports):
            self.messages.write_message('\nserial - port:')

            for i, key in enumerate(ports):
                self.messages.write_message('[{}] {}'.format(i, key))

            device_selection = input('Input integer value in the range [0:{}]: '.format(str(len(ports) - 1)))

            if device_selection == '':
                return None

            try:
                int(device_selection)
            except ValueError:
                self.messages.write_message('input value is not integer')
                return None

            if not 0 <= int(device_selection) < len(ports):
                self.messages.write_message('input value out of range')
                return None

            serial_port = ports[int(device_selection)]

        return serial_port

    def set_serial_configuration(self, port: str = None, baudrate: int = None):

        if port is not None:
            self.serial_configuration.port = port

        if baudrate is not None:
            self.serial_configuration.baudrate = baudrate

    def set_tcp_configuration(self, host: str = None, port: int = None):

        if host is not None:
            self.tcp_configuration.host = host

        if port is not None:
            self.tcp_configuration.port = port

    def get_serial_number_from_tcp_hostname(self) -> int:

        serial_number = None

        if self.tcp_configuration.host is None:
            self.messages.write_warning('Could not extract serial number from hostname. Hostname is None')
            return None

        try:
            serial_number_str = self.tcp_configuration.host.split('-')[-1].split('.local')[0]
            serial_number = int(serial_number_str)
        except ValueError as e:
            self.messages.write_warning('Could not extract serial number from tcp hostname: {}'.format(e))
            return None

        return serial_number

    def set_tcp_hostname_from_serial_number(self, serial_number, name: str = 'NORTEK') -> bool:

        supported_host_names = ['NortekFusion',
                                'NORTEK']

        if name not in supported_host_names:
            return False

        try:
            int(serial_number)
        except ValueError:
            self.messages.write_warning('Serial number is not integer')
            return False

        if name == 'NortekFusion':
            hostname = 'NortekFusion-{}.local'.format(str.zfill(str(serial_number), 4))

        elif name == 'NORTEK':
            hostname = 'NORTEK-{}.local'.format(str.zfill(str(serial_number), 6))

        else:
            self.messages.write_warning('Invalid value for name argument')
            return False

        self.set_tcp_configuration(host=hostname)

        return True

    def connect(self, connection_type: str, get_device_info=True) -> bool:

        def _set_connection_type() -> bool:
            if connection_type in self.CONNECTION_TYPES:
                self._connection_type = connection_type

                if self._connection_type == 'serial':
                    self.serial = serial.Serial()

                if self._connection_type == 'tcp':
                    self.tcp = socket.socket()

                return True

            else:
                self.messages.write_message(message='Invalid connection type "{}". Connection type must be in {}'.format(connection_type, self.CONNECTION_TYPES))

                return False

        def _connect_serial() -> bool:

            if self.serial_configuration.port is None:
                self.messages.write_message(message='serial_configuration.port is not defined')
                return False

            if self.serial_configuration.baudrate is None:
                self.messages.write_message(message='serial_configuration.baudrate is not defined')
                return False

            try:
                self.serial = serial.Serial(port=self.serial_configuration.port,
                                            baudrate=self.serial_configuration.baudrate,
                                            timeout=self.timeout)

                return True

            except serial.SerialException as exception:
                self.messages.write_exception(message='Failed to connect through serial: {}'.format(exception))

                return False

        def _connect_tcp() -> bool:

            def _login_tcp() -> bool:

                login = self.readline()

                if b'Please enter password:\r\n' not in login:
                    self.messages.write_warning(message='Did not recevie login promp when connecting to TCP')
                    return False

                self.write(command=b'nortek\r\n')

                reply = self.readline()

                if b'Welcome to Nortek' not in reply:
                    self.messages.write_warning(message='Did not recevie welcome message after login attempt')
                    return False

                return True

            if self.tcp_configuration.host is None:
                self.messages.write_message(message='tcp_configuration.host is not defined')
                return False

            if self.tcp_configuration.port is None:
                self.messages.write_message(message='tcp_configuration.port is not defined')
                return False

            # This gives the Nucleus 35 seconds to become visible on the network after power on
            for i in range(8):
                if i >= 7:
                    self.messages.write_warning('Failed to discover Nucleus on the network')
                    return False
                try:
                    socket.getaddrinfo(self.tcp_configuration.host, self.tcp_configuration.port)  # 5 sec timeout
                    break
                except socket.gaierror:
                    continue

            try:
                self.tcp = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
                self.tcp.connect((self.tcp_configuration.host, self.tcp_configuration.port))
                self.tcp.settimeout(self.timeout)

            except Exception as exception:
                self.messages.write_exception(message='Failed to connect through TCP: {}'.format(exception))
                return False

            self._connected = True
            if not _login_tcp():
                self.disconnect()
                return False

            return True

        if self.get_connection_status() is True:
            self.messages.write_message(message='Nucleus is already connected')
            return False

        if _set_connection_type() is not True:
            self.messages.write_message(message='Failed to connect to Nucleus')
            return False

        if self.get_connection_type() == 'serial':
            self._connected = _connect_serial()

        if self.get_connection_type() == 'tcp':
            self._connected = _connect_tcp()

        if self.get_connection_status() and self.commands is not None and get_device_info:
            if not self.set_clockstring():
                self.messages.write_warning('Failed to set current time on Nucleus device')
            self.get_info()

        return self.get_connection_status()

    def set_clockstring(self):

        clockstring = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")

        status = False
        if b'OK\r\n' in self.commands.set_clockstring(clockstring=clockstring):
            status = True

        return status

    def get_info(self):

        get_all = self.commands.get_all()

        if len(get_all) >= 15 and get_all[-1] == b'OK\r\n':

            self.get_all = list()

            try:
                for entry in get_all:
                    if b'ID' in entry:
                        self.nucleus_id = entry.lstrip(b'ID,').rstrip(b'\r\n').decode()

                    if b'GETFW' in entry:
                        self.firmware_version = entry.lstrip(b'GETFW,').rstrip(b'\r\n').decode()

                    if b'OK\r\n' in entry:
                        break

                    self.get_all.append(entry.decode())

            except UnicodeDecodeError:
                self.messages.write_warning('Failed to decode GETALL message')

    def disconnect(self) -> bool:

        if self.get_connection_status() is False:
            self.messages.write_message(message='Nucleus is already disconnected')
            return False

        def _disconnect_serial():

            try:
                self.serial.close()
                self._connection_type = None
                self.serial = serial.Serial()

                return True

            except Exception as exception:
                self.messages.write_exception(message='Failed to disconnet from serial connection: {}'.format(exception))

                return False

        def _disconnect_tcp():

            try:
                self.tcp.close()
                self._connection_type = None
                self.tcp = socket.socket()

                return True

            except Exception as exception:
                self.messages.write_exception(message='Failed to disconnet from serial connection: {}'.format(exception))

                return False

        disconnected = False

        if self.get_connection_type() == 'serial':
            disconnected = _disconnect_serial()

        if self.get_connection_type() == 'tcp':
            disconnected = _disconnect_tcp()

        self._connected = not disconnected

        return disconnected

    def write(self, command: bytes) -> bool:

        sending_successful = False

        def _send_serial_command():

            try:
                self.serial.write(command)
                return True
            except Exception as exception:
                self.messages.write_exception(message='Failed to send "{}" over serial: {}'.format(command, exception))
                return False

        def _send_tcp_command():

            try:
                data = command

                while len(data):
                    try:
                        sent = self.tcp.send(data)
                        data = data[sent:]
                    except socket.error as e:
                        if e.errno != errno.EAGAIN:
                            raise e
                        select.select([], [self.tcp], [])

                return True
            except Exception as exception:
                self.messages.write_exception(message='Failed to send "{}" over tcp: {}'.format(command[:20], exception))
                return False

        if not self.get_connection_status():
            self.messages.write_message(message='Nucleus is not connected. Can not send command: {}'.format(command))
            return False

        if self.get_connection_type() == 'serial':
            sending_successful = _send_serial_command()

        if self.get_connection_type() == 'tcp':
            sending_successful = _send_tcp_command()

        return sending_successful

    def read(self, size=None, terminator: bytes = None, timeout: int = 1) -> bytes:

        read_data = b''

        def _serial_read() -> bytes:
            serial_data = b''

            try:
                if terminator is not None:
                    init_time = datetime.now()
                    while (datetime.now() - init_time).seconds < timeout:
                        serial_data += self.serial.read_until(terminator, size)
                        if terminator in serial_data or b'ERROR\r\n' in serial_data:
                            break

                elif size is not None:
                    init_time = datetime.now()
                    while (datetime.now() - init_time).seconds < timeout:
                        serial_data += self.serial.read(size=max(0, size - len(serial_data)))
                        if len(serial_data) >= size:
                            break

                else:
                    if self.serial.in_waiting:
                        serial_data += self.serial.read(self.serial.in_waiting)

            except Exception as exception:
                self.messages.write_exception(message='Failed to read serial data from Nucleus: {}'.format(exception))

            return serial_data

        def _tcp_read() -> bytes:

            def _read() -> bool:
                try:
                    self.tcp_buffer += self.tcp.recv(4096)
                    return True
                except socket.timeout:
                    return True
                except Exception as exception:
                    if self.get_connection_status():
                        self.messages.write_exception(message='Failed to read tcp data from Nucleus: {}'.format(exception))

                    return False

            if terminator is not None:

                init_time = datetime.now()
                while (datetime.now() - init_time).seconds < timeout:
                    if terminator in self.tcp_buffer:
                        break
                    else:
                        if not _read():
                            break

                line, separator, self.tcp_buffer = self.tcp_buffer.partition(terminator)
                tcp_data = line + separator

            elif size is not None:

                init_time = datetime.now()
                while (datetime.now() - init_time).seconds < timeout:
                    if len(self.tcp_buffer) >= size:
                        break
                    else:
                        if not _read():
                            break

                tcp_data = self.tcp_buffer[:size]
                self.tcp_buffer = self.tcp_buffer[size:]

            else:
                _read()
                tcp_data = self.tcp_buffer
                self.tcp_buffer = b''

            return tcp_data

        if self.get_connection_type() == 'serial':
            read_data = _serial_read()

        if self.get_connection_type() == 'tcp':
            read_data = _tcp_read()

        return read_data

    def readline(self, timeout: int = 1) -> bytes:

        return self.read(terminator=b'\r\n', timeout=timeout)

    def reset_buffers(self):

        if self.get_connection_type() == 'serial':
            self.serial.reset_output_buffer()
            self.serial.reset_input_buffer()
            time.sleep(0.01)  # Double reset with delay since serial buffer reset is buggy
            self.serial.reset_output_buffer()
            self.serial.reset_input_buffer()

        elif self.get_connection_type() == 'tcp':
            self.tcp_buffer = b''
            for i in range(100):
                try:
                    self.tcp.recv(4096)
                except socket.timeout as e:
                    break
                except Exception as e:
                    self.messages.write_warning('Got an unexpected exception when resetting TCP buffers: {}'.format(e))
                    break

