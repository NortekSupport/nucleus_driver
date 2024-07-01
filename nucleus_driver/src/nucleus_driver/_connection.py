from datetime import datetime
import time
from dataclasses import dataclass
import serial
from serial.tools import list_ports
from serial.serialutil import SerialException
import socket
import select
import errno


class Connection:

    CONNECTION_TYPES = ['serial', 'tcp']
    TIMEOUT = 0.01  # This is not the timeout for the read function of this driver as that is handled in the read function, but rather the timeout of the actual pyserial and socket read function

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
        self.parser = None

        self._connection_type = None
        self._connected = False

        self.serial = serial.Serial()
        self.tcp = socket.socket()

        self.serial_configuration = self.SerialConfiguration()
        self.tcp_configuration = self.TcpConfiguration()
        self.timeout = 1

        self.buffer = b''

        self.nucleus_id = None
        self.firmware_version = None
        self.get_all = None
        self.get_all_nmea = b''

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

    def connect(self, connection_type: str, get_device_info=True, password=None) -> bool:

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
                                            timeout=self.TIMEOUT)

                return True

            except serial.SerialException as exception:
                self.messages.write_exception(message='Failed to connect through serial: {}'.format(exception))

                return False

        def _connect_tcp() -> bool:

            def _login_tcp() -> bool:

                login = self.readline()

                if b'Welcome to Nortek' in login:
                    return True

                if b'Please enter password:\r\n' not in login:
                    self.messages.write_warning(message=f'Did not receive login prompt when connecting to TCP. Received: {login}')
                    return False

                if password is not None:
                    self.write(command=password.encode().rstrip(b'\n').rstrip(b'\r') + b'\r\n')
                else:
                    self.write(command=b'nortek\r\n')

                reply = self.readline()

                if b'Welcome to Nortek' not in reply:
                    self.messages.write_warning(message=f'Did not receive welcome message after login attempt. Received: {reply}')
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
                self.tcp.settimeout(self.TIMEOUT)

            except Exception as exception:
                self.messages.write_exception(message='Failed to connect through TCP: {}'.format(exception))
                return False

            self._connected = True
            if not _login_tcp():
                self.disconnect()
                return False

            return True

        self.nucleus_id = None
        self.firmware_version = None

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

        if not self.get_connection_status():
            self.messages.write_warning('Failed to establish connection to device')
            return False

        if get_device_info:
            if not self.set_clockstring():
                get_device_info = False

        if get_device_info:
            self.get_info()

        return self.get_connection_status()

    def set_clockstring(self):

        self.commands._reset_buffer()

        clockstring = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
        command = 'SETCLOCKSTR,TIME="{}"\r\n'.format(clockstring).encode()
        self.write(command)

        reply = self.commands._get_reply(terminator=b'OK\r\n', timeout=1)

        if len(reply) >= 50:
            self.messages.write_message('Nucleus is already running')
            return False

        self.commands._check_reply(data=reply, command=command, terminator=b'OK\r\n')

        if b'OK\r\n' in reply:
            return True

        else:
            self.messages.write_warning('Failed to set current time on Nucleus device')
            return False

    def get_info(self):

        get_all = self.commands.get_all(_nmea=True)

        if len(get_all) >= 15 and b'OK' in get_all[-1]:
            
            self.get_all_nmea = get_all
            self.get_all = list()
            
            try:
                for entry in get_all:

                    stripped_entry = entry.split(b'*')[0].split(b'$PNOR,')[1]

                    if b'ID' in stripped_entry:

                        stripped_id_entry = stripped_entry.split(b'*')[0].lstrip(b'ID,')

                        self.nucleus_id = ','.join([x.split(b'=')[1].decode() for x in stripped_id_entry.split(b',')])

                    if b'GETFW' in stripped_entry:

                        stripped_getfw_entry = stripped_entry.split(b'*')[0].lstrip(b'GETFW,')

                        self.firmware_version = ','.join([x.split(b'=')[1].decode() for x in stripped_getfw_entry.split(b',')])

                    if b'OK\r\n' in stripped_entry:
                        break

                    self.get_all.append(stripped_entry.decode() + '\r\n')

            except UnicodeDecodeError:
                self.messages.write_warning('Failed to decode GETALL message')

        else:
            self.messages.write_warning('Unable to obtain GETALL information')

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
                self.tcp.shutdown(socket.SHUT_RDWR)
                self.tcp.close()

                time.sleep(0.01)

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

    def _read(self) -> bytes:
        
        data = b''

        if self.get_connection_type() == 'tcp':
            try:
                data = self.tcp.recv(4096)
            except socket.timeout:
                data = b''
            except Exception as exception:
                self.messages.write_exception(message='Failed to read tcp data from Nucleus: {}'.format(exception))
                return None
            
        if self.get_connection_type() == 'serial':
            try:
                data = self.serial.read(size=self.serial.in_waiting)
            except SerialException:
                data = b''
        
        return data

    def read(self, size=None, terminator: bytes = None, timeout: float = 1) -> bytes:

        data = b''
        size_satisfied = False
        terminator_satisfied = False

        if timeout is None:

            read_data = self._read()

            if read_data is not None:
                data = read_data

            return data

        init_time = datetime.now()
        while (datetime.now() - init_time).total_seconds() <= timeout:
            
            read_data = self._read()
            
            if read_data is None:
                break

            self.buffer += read_data
            
            if size is not None and len(self.buffer) >= size:
                size_satisfied = True

            if terminator is not None and terminator in self.buffer:
                terminator_satisfied = True

            if b'ERROR\r\n' in self.buffer:
                
                if terminator_satisfied:
                    if self.buffer.find(b'ERROR\r\n') < self.buffer.find(terminator):
                        terminator = b'ERROR\r\n'
                else:
                    terminator_satisfied = True
                    terminator = b'ERROR\r\n'

            if size_satisfied and terminator_satisfied:

                line, separator, self.buffer = self.buffer.partition(terminator)
                data = line + separator

                if len(data) > size:

                    data = data[:size]
                    self.buffer = data[size:] + self.buffer

                break

            elif size_satisfied:

                data = self.buffer[:size]
                self.buffer = self.buffer[size:]

                break

            elif terminator_satisfied:
                
                line, separator, self.buffer = self.buffer.partition(terminator)
                data = line + separator

                break

        else:
            data = self.buffer
            self.buffer = b''

        return data
    
    def readline(self, timeout: int = 1) -> bytes:

        return self.read(terminator=b'\r\n', timeout=timeout)

    def reset_buffers(self):
        
        self.buffer = b''

        if self.get_connection_type() == 'serial':
            self.serial.reset_output_buffer()
            self.serial.reset_input_buffer()
            time.sleep(0.01)  # Double reset with delay since serial buffer reset is buggy
            self.serial.reset_output_buffer()
            self.serial.reset_input_buffer()

        elif self.get_connection_type() == 'tcp':
            
            for _ in range(5):
                try:
                    self.tcp.recv(4096)
                except socket.timeout as e:
                    break
                except Exception as e:
                    self.messages.write_warning('Got an unexpected exception when resetting TCP buffers: {}'.format(e))
                    break
            