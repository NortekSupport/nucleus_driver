from dataclasses import dataclass
import argparse
from threading import Thread
import time
import serial
import socket
from itertools import zip_longest
from datetime import datetime
from struct import unpack, error
import queue
import csv
from pathlib import Path
import io
from serial.tools import list_ports


class FusionDriver:

    def __init__(self):

        self.messages = self.FusionMessages()
        self.connection = self.FusionConnection(self.messages)
        self.logging = self.FusionLogging(self.messages)
        self.parser = self.FusionParser(self.messages, self.logging)
        self.thread = self.FusionThread(self.messages, self.connection, self.parser)
        self.commands = self.FusionCommands(self.messages, self.connection, self.thread, self.parser)
        self.run_classes = self.RunClasses(self.messages, self.connection, self.thread, self.logging, self.commands)

        self.connection.fusion_commands = self.commands

    class FusionMessages:

        def write_message(self, message: str, skip_newline=False):

            if not skip_newline:
                print(message)
            else:
                print(message, end='')

        def write_warning(self, message: str):

            print('WARNING: {}'.format(message))

        def write_exception(self, message: str):

            print('EXCEPTION: {}'.format(message))

    class FusionParser:

        ID_IMU = 0x82
        ID_MAGNETOMETER = 0x87
        ID_BOTTOMTRACK = 0xb4
        ID_WATERTRACK = 0xbe
        ID_ALTIMETER = 0xaa
        ID_AHRS = 0xd2
        ID_FIELD_CALIBRATION = 0x8B

        def __init__(self, fusion_message, fusion_logging):

            self.fusion_message = fusion_message
            self.fusion_logging = fusion_logging

            self.packet_queue = queue.Queue()
            self.ascii_queue = queue.Queue()
            self.condition_queue = queue.Queue()

            self._queuing = {'packet': False,
                             'ascii': False,
                             'condition': False}

        def set_queuing(self, packet: bool = None, ascii: bool = None, condition: bool = None):

            if packet is not None:
                self._queuing['packet'] = packet

            if ascii is not None:
                self._queuing['ascii'] = ascii

            if condition is not None:
                self._queuing['condition'] = condition

        def get_queuing(self):

            return self._queuing

        def clear_queue(self, queue_name):

            if queue_name == 'packet' or queue_name == 'all':
                for i in range(self.packet_queue.qsize()):
                    self.packet_queue.get_nowait()

            if queue_name == 'condition' or queue_name == 'all':
                for i in range(self.condition_queue.qsize()):
                    self.condition_queue.get_nowait()

            if queue_name == 'ascii' or queue_name == 'all':
                for i in range(self.ascii_queue.qsize()):
                    self.ascii_queue.get_nowait()

        def write_packet(self, packet):

            if self._queuing['packet'] is True:
                self.packet_queue.put_nowait(packet)

            if self.fusion_logging._logging is True:

                self.fusion_logging._writing_packet = True

                try:
                    self.fusion_logging.packet_writer.writerow(packet)
                except ValueError as exception:
                    self.fusion_message.write_warning('Failed to write package to csv file: {}'.format(exception))

                self.fusion_logging._writing_packet = False

        def read_packet(self, timeout=None):

            packet = None

            try:
                if timeout is not None:
                    packet = self.packet_queue.get(timeout=timeout)

                elif not self.packet_queue.empty():
                    packet = self.packet_queue.get_nowait()

            except Exception as exception:
                self.fusion_message.write_warning('failed to retrieve packet from packet queue: {}'.format(exception))

            return packet

        def write_ascii(self, packet):

            if self._queuing['ascii'] is True:
                ascii_bytes = bytes(packet)
                ascii_packet = {'timestamp_python': datetime.now().timestamp(),
                                'bytes': ascii_bytes}

                self.ascii_queue.put_nowait(ascii_packet)

            if self.fusion_logging._logging is True:
                ascii_message = ''.join(chr(i) for i in packet if 0 <= i <= 0x7e).rstrip('\r\n')
                ascii_packet = {'timestamp_python': datetime.now().timestamp(),
                                'message': ascii_message}

                self.fusion_logging._writing_ascii = True

                try:
                    self.fusion_logging.ascii_writer.writerow(ascii_packet)
                except ValueError as exception:
                    self.fusion_message.write_warning('Failed to write ascii message to csv file: {}'.format(exception))

                self.fusion_logging._writing_ascii = False

        def read_ascii(self, timeout=None):

            packet = None

            try:
                if timeout is not None:
                    packet = self.ascii_queue.get(timeout=timeout)

                elif not self.ascii_queue.empty():
                    packet = self.ascii_queue.get_nowait()

            except Exception as exception:
                self.fusion_message.write_warning('failed to retrieve packet from ascii queue: {}'.format(exception))

            return packet

        def write_condition(self, error_message, packet):

            failed_packet = {'timestamp_python': datetime.now().timestamp(),
                             'error_message': error_message,
                             'failed_packet': packet}

            if self._queuing['condition'] is True:
                self.condition_queue.put_nowait(failed_packet)

            if self.fusion_logging._logging is True:

                self.fusion_logging._writing_condition = True

                try:
                    self.fusion_logging.condition_writer.writerow(failed_packet)
                except ValueError as exception:
                    self.fusion_message.write_warning('Failed to write condition to csv file: {}'.format(exception))

                self.fusion_logging._writing_condition = False

        def read_condition(self, timeout=None):

            packet = None

            try:
                if timeout is not None:
                    packet = self.condition_queue.get(timeout=timeout)

                elif not self.condition_queue.empty():
                    packet = self.condition_queue.get_nowait()

            except Exception as exception:
                self.fusion_message.write_warning('failed to retrieve packet from condition queue: {}'.format(exception))

            return packet

        @staticmethod
        def checksum(packet):

            checksum = 0xb58c

            for u, v in zip_longest(packet[::2], packet[1::2], fillvalue=None):

                if v is not None:
                    checksum += u | v << 8
                else:
                    checksum += u << 8 | 0x00

                checksum &= 0xffff

            return checksum

        def check_header(self, data):

            checksum_result = False
            header_size = data[1]

            if header_size >= len(data):
                return checksum_result

            if self.checksum(data[:header_size - 2]) == data[header_size - 2] | data[header_size - 1] << 8:
                checksum_result = True

            return checksum_result

        def check_packet(self, data):

            checksum_result = False
            header_size = data[1]

            packet_size = (data[4] & 0xff) | ((data[5] & 0xff) << 8)

            if header_size + packet_size != len(data):
                return checksum_result

            if self.checksum(data[header_size:]) == data[header_size - 4] | data[header_size - 3] << 8:
                checksum_result = True

            return checksum_result

        def add_binary_packet(self, binary_packet, ascii_packet):

            header_length = binary_packet[1]
            package_length = header_length + (binary_packet[4] & 0xff) | ((binary_packet[5] & 0xff) << 8)

            if self.check_header(binary_packet[:package_length]):

                if self.check_packet(binary_packet[:package_length]):
                    packet, error_message, condition = self.extract_packet(binary_packet[:package_length])
                else:
                    condition = False
                    packet = binary_packet[:package_length]
                    error_message = 'data checksum failed'

                if condition is True:
                    self.write_packet(packet=packet)

                else:
                    self.write_condition(error_message=error_message, packet=packet)

                binary_packet = binary_packet[package_length:]
                if len(binary_packet) == 0:
                    reading_packet = False
                else:
                    reading_packet = True

            elif 0xa5 in binary_packet[1:]:
                start_index = next(i + 1 for i, x in enumerate(binary_packet[1:]) if x == 0xa5)
                ascii_packet.extend(binary_packet[:start_index])
                binary_packet = binary_packet[start_index:]
                reading_packet = True

            else:
                self.write_condition(error_message='header checksum failed', packet=binary_packet[:package_length])
                binary_packet = binary_packet[package_length:]
                reading_packet = False

            return binary_packet, ascii_packet, reading_packet

        def extract_packet(self, data):

            header_size = data[1]
            id = data[2]

            sensor_data, error_message, condition = self.get_sensor_data(id, data[header_size:])

            return sensor_data, error_message, condition

        def get_sensor_data(self, sensor_id, data):

            packet = None
            error_message = 'packet id not recognized'
            condition = False

            if sensor_id == self.ID_IMU:
                try:
                    version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                    if version == 1:
                        unpacked_data = list(unpack('<BBHIIIfffffff', bytearray(data)))
                        packet = {
                            'id': hex(sensor_id),
                            'timestamp_python': datetime.now().timestamp(),
                            'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                            'status': hex(unpacked_data[5]),
                            'accelerometer_x': unpacked_data[6],
                            'accelerometer_y': unpacked_data[7],
                            'accelerometer_z': unpacked_data[8],
                            'gyro_x': unpacked_data[9],
                            'gyro_y': unpacked_data[10],
                            'gyro_z': unpacked_data[11],
                            'temperature': unpacked_data[12]
                        }
                        error_message = ''
                        condition = True
                except error:
                    self.fusion_message.write_warning('failed to unpack IMU data')
                    error_message = 'failed to unpack IMU data'

            elif sensor_id == self.ID_MAGNETOMETER:
                try:
                    version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                    if version == 1:
                        unpacked_data = list(unpack('<BBHIIIfff', bytearray(data)))
                        packet = {
                            'id': hex(sensor_id),
                            'timestamp_python': datetime.now().timestamp(),
                            'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                            'status': hex(unpacked_data[5]),
                            'magnetometer_x': unpacked_data[6],
                            'magnetometer_y': unpacked_data[7],
                            'magnetometer_z': unpacked_data[8],
                        }
                        error_message = ''
                        condition = True
                except error:
                    self.fusion_message.write_warning('failed to unpack magnetometer data')
                    error_message = 'failed to unpack magnetometer data'

            elif sensor_id in (self.ID_BOTTOMTRACK, self.ID_WATERTRACK):
                try:
                    version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                    if version == 1:
                        unpacked_data = list(unpack('<BBHIIIIIffffffffffffffffffffffffff', bytearray(data)))
                        packet = {
                            'id': hex(sensor_id),
                            'timestamp_python': datetime.now().timestamp(),
                            'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                            'status': hex(unpacked_data[5]),
                            'serial_number': unpacked_data[6],
                            'error': hex(unpacked_data[7]),
                            'sound_speed': unpacked_data[8],
                            'temperature': unpacked_data[9],
                            'pressure': unpacked_data[10],
                            'velocity_beam_0': unpacked_data[11],
                            'velocity_beam_1': unpacked_data[12],
                            'velocity_beam_2': unpacked_data[13],
                            'distance_beam_0': unpacked_data[14],
                            'distance_beam_1': unpacked_data[15],
                            'distance_beam_2': unpacked_data[16],
                            'fom_beam_0': unpacked_data[17],
                            'fom_beam_1': unpacked_data[18],
                            'fom_beam_2': unpacked_data[19],
                            'dt_beam_0': unpacked_data[20],
                            'dt_beam_1': unpacked_data[21],
                            'dt_beam_2': unpacked_data[22],
                            'time_velocity_estimate_0': unpacked_data[23],
                            'time_velocity_estimate_1': unpacked_data[24],
                            'time_velocity_estimate_2': unpacked_data[25],
                            'velocity_x': unpacked_data[26],
                            'velocity_y': unpacked_data[27],
                            'velocity_z': unpacked_data[28],
                            'fom_x': unpacked_data[29],
                            'fom_y': unpacked_data[30],
                            'fom_z': unpacked_data[31],
                            'dt_xyz': unpacked_data[32],
                            'time_velocity_estimate_xyz': unpacked_data[33]
                        }
                        error_message = ''
                        condition = True
                except error:
                    self.fusion_message.write_warning('failed to unpack DVL data')
                    error_message = 'failed to unpack DVL data'

            elif sensor_id == self.ID_ALTIMETER:
                try:
                    version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                    if version == 1:
                        unpacked_data = list(unpack('<BBHIIIIIffffH', bytearray(data)))
                        packet = {
                            'id': hex(sensor_id),
                            'timestamp_python': datetime.now().timestamp(),
                            'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                            'status': hex(unpacked_data[5]),
                            'serial_number': unpacked_data[6],
                            'error': hex(unpacked_data[7]),
                            'sound_speed': unpacked_data[8],
                            'temperature': unpacked_data[9],
                            'pressure': unpacked_data[10],
                            'altimeter_distance': unpacked_data[11],
                            'altimeter_quality': unpacked_data[12]
                        }
                        error_message = ''
                        condition = True
                except error:
                    self.fusion_message.write_warning('failed to unpack altimeter data')
                    error_message = 'failed to unpack altimeter data'

            elif sensor_id == self.ID_AHRS:
                try:
                    version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                    if version == 1:
                        unpacked_data = list(unpack('<BBHIIIIIBBBBffffffffffffffffff', bytearray(data)))
                        packet = {
                            'id': hex(sensor_id),
                            'timestamp_python': datetime.now().timestamp(),
                            'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                            'status': hex(unpacked_data[5]),
                            'serial_number': unpacked_data[6],
                            'error': hex(unpacked_data[7]),
                            'operation_mode': unpacked_data[8],
                            'fom_ahrs': unpacked_data[9],
                            'fom_fc1': unpacked_data[10],
                            'euler_angles_roll': unpacked_data[12],
                            'euler_angles_pitch': unpacked_data[13],
                            'euler_angles_heading': unpacked_data[14],
                            'quaternion_0': unpacked_data[15],
                            'quaternion_1': unpacked_data[16],
                            'quaternion_2': unpacked_data[17],
                            'quaternion_3': unpacked_data[18],
                            'dcm_11': unpacked_data[19],
                            'dcm_12': unpacked_data[20],
                            'dcm_13': unpacked_data[21],
                            'dcm_21': unpacked_data[22],
                            'dcm_22': unpacked_data[23],
                            'dcm_23': unpacked_data[24],
                            'dcm_31': unpacked_data[25],
                            'dcm_32': unpacked_data[26],
                            'dcm_33': unpacked_data[27],
                            'declination': unpacked_data[28],
                            'depth': unpacked_data[29],
                        }
                        error_message = ''
                        condition = True
                    elif version == 2:
                        unpacked_data = list(unpack('<BBHIIIIIBBBBffffffffffffffffffff', bytearray(data)))
                        packet = {
                            'id': hex(sensor_id),
                            'timestamp_python': datetime.now().timestamp(),
                            'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                            'status': hex(unpacked_data[5]),
                            'serial_number': unpacked_data[6],
                            'error': hex(unpacked_data[7]),
                            'operation_mode': unpacked_data[8],
                            'fom_ahrs': unpacked_data[12],
                            'fom_fc1': unpacked_data[13],
                            'euler_angles_roll': unpacked_data[14],
                            'euler_angles_pitch': unpacked_data[15],
                            'euler_angles_heading': unpacked_data[16],
                            'quaternion_0': unpacked_data[17],
                            'quaternion_1': unpacked_data[18],
                            'quaternion_2': unpacked_data[19],
                            'quaternion_3': unpacked_data[20],
                            'dcm_11': unpacked_data[21],
                            'dcm_12': unpacked_data[22],
                            'dcm_13': unpacked_data[23],
                            'dcm_21': unpacked_data[24],
                            'dcm_22': unpacked_data[25],
                            'dcm_23': unpacked_data[26],
                            'dcm_31': unpacked_data[27],
                            'dcm_32': unpacked_data[28],
                            'dcm_33': unpacked_data[29],
                            'declination': unpacked_data[30],
                            'depth': unpacked_data[31],
                        }
                        error_message = ''
                        condition = True
                except error:
                    self.fusion_message.write_warning('failed to unpack AHRS data')
                    error_message = 'failed to unpack AHRS data'

            elif sensor_id == self.ID_FIELD_CALIBRATION:
                try:
                    version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                    if version == 1:
                        unpacked_data = list(unpack('<BBHIIIfffffffffffffffff', bytearray(data)))
                        packet = {
                            'id': hex(sensor_id),
                            'timestamp_python': datetime.now().timestamp(),
                            'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                            'status': hex(unpacked_data[5]),
                            'hard_iron_x': unpacked_data[6],
                            'hard_iron_y': unpacked_data[7],
                            'hard_iron_z': unpacked_data[8],
                            's_axis_00': unpacked_data[9],
                            's_axis_01': unpacked_data[10],
                            's_axis_02': unpacked_data[11],
                            's_axis_10': unpacked_data[12],
                            's_axis_11': unpacked_data[13],
                            's_axis_12': unpacked_data[14],
                            's_axis_20': unpacked_data[15],
                            's_axis_21': unpacked_data[16],
                            's_axis_22': unpacked_data[17],
                            'new_point_0': unpacked_data[18],
                            'new_point_1': unpacked_data[19],
                            'new_point_2': unpacked_data[20],
                            'fom': unpacked_data[21],
                            'coverage': unpacked_data[22],
                        }
                        error_message = ''
                        condition = True
                except error:
                    self.fusion_message.write_warning('failed to unpack field_cal data')
                    error_message = 'failed to unpack field_cal data'

            return packet, error_message, condition

        def add_ascii_packet(self, ascii_packet):

            self.write_ascii(packet=ascii_packet)

    class FusionThread:

        MAX_PACKAGE_LENGTH = 360
        # MAX_PACKAGE_LENGTH is given by the following firmware scripts:
        # https://dev.azure.com/NortekGroup/INS/_git/Firmware?path=/src/ofmt.c&version=GBmaster&line=28&lineEnd=29&lineStartColumn=1&lineEndColumn=1&lineStyle=plain&_a=contents
        # https://dev.azure.com/NortekGroup/INS/_git/Firmware?path=/lib/sensor_fusion/src/uns_types.h&version=GBmaster&line=10&lineEnd=11&lineStartColumn=1&lineEndColumn=1&lineStyle=plain&_a=contents

        def __init__(self, fusion_messages, fusion_connection, fusion_parser):

            self.fusion_messages = fusion_messages
            self.fusion_connection = fusion_connection
            self.fusion_parser = fusion_parser

            self.thread = Thread()
            self.thread_running = False

        def start(self) -> bool:

            if self.thread.is_alive():
                self.fusion_messages.write_warning(message='Fusion thread is already alive')
                return False

            self.thread = Thread(target=self.run)
            self.thread.start()

            return True

        def stop(self) -> bool:

            if not self.thread.is_alive():
                self.fusion_messages.write_warning(message='Fusion thread is not alive')
                return False

            self.thread_running = False
            self.thread.join(2)
            self.thread = Thread()

            return True

        def run(self):

            reading_packet = False
            binary_packet = list()
            ascii_packet = list()
            self.thread_running = True

            while self.thread_running:

                if not self.fusion_connection.get_connection_status():
                    time.sleep(0.05)
                    continue

                data = self.fusion_connection.read()

                if data:

                    for value in data:

                        if value == 0xa5 and not reading_packet:
                            reading_packet = True
                            ascii_packet = list()

                        if reading_packet:
                            binary_packet.append(value)
                        else:
                            ascii_packet.append(value)

                        if len(binary_packet) > self.MAX_PACKAGE_LENGTH or len(binary_packet) > 5 and len(binary_packet) >= binary_packet[1] + (binary_packet[4] & 0xff) | ((binary_packet[5] & 0xff) << 8):
                            binary_packet, ascii_packet, reading_packet = self.fusion_parser.add_binary_packet(binary_packet, ascii_packet)

                        if len(ascii_packet) >= 2 and ascii_packet[-2] == 0x0d and ascii_packet[-1] == 0x0a:
                            self.fusion_parser.add_ascii_packet(ascii_packet)
                            ascii_packet = list()
                else:
                    time.sleep(0.01)

    class FusionConnection:

        CONNECTION_TYPES = ['serial', 'udp', 'tcp']

        @dataclass
        class SerialConfiguration:
            port: str = None
            baudrate: int = None
            timeout: float = 1.0

        @dataclass
        class TcpConfiguration:
            #ip: str = None
            #hostname: str = None
            host: str = None
            port: int = None
            timeout: float = 1.0

        @dataclass
        class UdpConfiguration:
            ip: str = '192.168.2.2'  # UDP address of ROV running on ArduSub software
            port: int = None
            timeout: float = 1.0

        def __init__(self, fusion_messages):

            self.fusion_messages = fusion_messages
            self.fusion_commands = None

            self._connection_type = None
            self._connected = False

            self.serial = serial.Serial()
            self.tcp = socket.socket()
            self.udp = socket.socket()

            self.serial_configuration = self.SerialConfiguration()
            self.tcp_configuration = self.TcpConfiguration()
            self.udp_configuration = self.UdpConfiguration()

            self.tcp_buffer = b''
            self.udp_buffer = b''

            self.fusion_id = None
            self.firmware_version = None

        def get_connection_type(self) -> str:

            return self._connection_type

        def get_connection_status(self) -> bool:

            return self._connected

        def select_serial_port(self):

            serial_port = None

            port_info = list_ports.comports(include_links=False)
            ports = [port.device for port in port_info]

            if bool(ports):
                self.fusion_messages.write_message('\nserial - port:')

                for i, key in enumerate(ports):
                    # self.fusion_messages.write_message('[' + str(i) + '] ' + str(key))
                    self.fusion_messages.write_message('[{}] {}'.format(i, key))

                device_selection = input('Input integer value in the range [0:{}]: '.format(str(len(ports) - 1)))

                if device_selection == '':
                    return None

                try:
                    int(device_selection)
                except ValueError:
                    print('input value is not integer')
                    return None

                if not 0 <= int(device_selection) < len(ports):
                    print('input value out of range')
                    return None

                serial_port = ports[int(device_selection)]

            return serial_port

        def set_serial_configuration(self, port: str = None, baudrate: int = None, timeout: float = None):

            if port is not None:
                self.serial_configuration.port = port

            if baudrate is not None:
                self.serial_configuration.baudrate = baudrate

            if timeout is not None:
                self.serial_configuration.timeout = timeout
                self.serial.timeout = self.serial_configuration.timeout

        def set_tcp_configuration(self, host: str = None, port: int = None, timeout: float = None):

            if host is not None:
                self.tcp_configuration.host = host

            if port is not None:
                self.tcp_configuration.port = port

            if timeout is not None:
                self.tcp_configuration.timeout = timeout
                self.tcp.settimeout(self.tcp_configuration.timeout)

        def set_udp_configuration(self, ip: str = None, port: int = None, timeout: float = None):

            if ip is not None:
                self.udp_configuration.ip = ip

            if port is not None:
                self.udp_configuration.port = port

            if timeout is not None:
                self.udp_configuration.timeout = timeout
                self.udp.settimeout(self.udp_configuration.timeout)

        def get_timeout(self) -> float:

            if self.get_connection_type() == 'serial':
                return self.serial_configuration.timeout

            if self.get_connection_type() == 'tcp':
                return self.tcp_configuration.timeout

            if self.get_connection_type() == 'udp':
                return self.udp_configuration.timeout

        def set_timeout(self, timeout):

            if self.get_connection_type() == 'serial':
                self.set_serial_configuration(timeout=timeout)

            if self.get_connection_type() == 'tcp':
                self.set_tcp_configuration(timeout=timeout)

            if self.get_connection_type() == 'udp':
                self.set_udp_configuration(timeout=timeout)

        def connect(self, connection_type: str) -> bool:

            def _set_connection_type() -> bool:
                if connection_type in self.CONNECTION_TYPES:
                    self._connection_type = connection_type

                    if self._connection_type == 'serial':
                        self.serial = serial.Serial()

                    if self._connection_type == 'tcp':
                        self.tcp = socket.socket()

                    if self._connection_type == 'udp':
                        self.udp = socket.socket()

                    return True

                else:
                    self.fusion_messages.write_message(message='Invalid connection type "{}". Connection type must be in {}'.format(connection_type, self.CONNECTION_TYPES))

                    return False

            def _connect_serial() -> bool:

                if self.serial_configuration.port is None:
                    self.fusion_messages.write_message(message='serial_configuration.port is not defined')
                    return False

                if self.serial_configuration.baudrate is None:
                    self.fusion_messages.write_message(message='serial_configuration.baudrate is not defined')
                    return False

                try:
                    self.serial = serial.Serial(port=self.serial_configuration.port,
                                                baudrate=self.serial_configuration.baudrate,
                                                timeout=self.serial_configuration.timeout)

                    return True

                except serial.SerialException as exception:
                    self.fusion_messages.write_exception(message='Failed to connect through serial: {}'.format(exception))

                    return False

            def _connect_tcp() -> bool:

                def _login_tcp() -> bool:

                    login = self.readline()

                    if b'Please enter password:\r\n' not in login:
                        self.fusion_messages.write_warning(message='Did not recevie login promp when connecting to TCP')
                        return False

                    self.write(command=b'nortek\r\n')

                    reply = self.readline()

                    if b'Welcome to Nortek Fusion DVL1000\r\n' not in reply:
                        self.fusion_messages.write_warning(message='Did not recevie welcome message after login attempt')
                        return False

                    return True

                if self.tcp_configuration.host is None:
                    self.fusion_messages.write_message(message='tcp_configuration.host is not defined')
                    return False

                if self.tcp_configuration.port is None:
                    self.fusion_messages.write_message(message='tcp_configuration.port is not defined')
                    return False

                # This gives the Fusion 25 seconds to become visible on the network after power on
                for i in range(6):
                    if i >= 5:
                        self.fusion_messages.write_warning('Failed to discover Fusion on the network')
                        return False
                    try:
                        socket.getaddrinfo(self.tcp_configuration.host, self.tcp_configuration.port)  # 5 sec timeout
                        break
                    except socket.gaierror:
                        continue

                try:
                    self.tcp = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
                    self.tcp.connect((self.tcp_configuration.host, self.tcp_configuration.port))
                    self.tcp.settimeout(self.tcp_configuration.timeout)

                except Exception as exception:
                    self.fusion_messages.write_exception(message='Failed to connect through TCP: {}'.format(exception))
                    return False

                self._connected = True
                if not _login_tcp():
                    self.disconnect()
                    return False

                return True

            def _connect_udp() -> bool:

                if self.tcp_configuration.port is None:
                    self.fusion_messages.write_message(message='tcp_configuration.port is not defined')
                    return False

                try:
                    self.udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

                    self.udp.settimeout(self.udp_configuration.timeout)

                    return True

                except Exception as exception:
                    self.fusion_messages.write_exception(message='Failed to connect through UDP: {}'.format(exception))

                    return False

            if self.get_connection_status() is True:
                self.fusion_messages.write_message(message='Fusion is already connected')
                return False

            if _set_connection_type() is not True:
                self.fusion_messages.write_message(message='Failed to connect to Fusion')
                return False

            if self.get_connection_type() == 'serial':
                self._connected = _connect_serial()

            if self.get_connection_type() == 'tcp':
                self._connected = _connect_tcp()

            if self.get_connection_type() == 'udp':
                self._connected = _connect_udp()

            if self.get_connection_status() and self.fusion_commands is not None:
                fusion_id = self.fusion_commands.get_id()
                if len(fusion_id) == 2 and fusion_id[1] == b'OK\r\n':
                    self.fusion_id = fusion_id[0].rstrip(b'\r\n').decode()

                firmware_version = self.fusion_commands.get_fw()
                if len(firmware_version) == 2 and firmware_version[1] == b'OK\r\n':
                    self.firmware_version = firmware_version[0].rstrip(b'\r\n').decode()

            return self.get_connection_status()

        def disconnect(self) -> bool:

            if self.get_connection_status() is False:
                self.fusion_messages.write_message(message='Fusion is already disconnected')
                return False

            def _disconnect_serial():

                try:
                    self.serial.close()
                    self._connection_type = None
                    self.serial = serial.Serial()

                    return True

                except Exception as exception:
                    self.fusion_messages.write_exception(message='Failed to disconnet from serial connection: {}'.format(exception))

                    return False

            def _disconnect_tcp():

                try:
                    self.tcp.close()
                    self._connection_type = None
                    self.tcp = socket.socket()

                    return True

                except Exception as exception:
                    self.fusion_messages.write_exception(message='Failed to disconnet from serial connection: {}'.format(exception))

                    return False

            def _disconnect_udp():

                try:
                    self.udp.close()
                    self._connection_type = None
                    self.udp = socket.socket()

                    return True

                except Exception as exception:
                    self.fusion_messages.write_exception(message='Failed to disconnet from serial connection: {}'.format(exception))

                    return False

            disconnected = False

            if self.get_connection_type() == 'serial':
                disconnected = _disconnect_serial()

            if self.get_connection_type() == 'tcp':
                disconnected = _disconnect_tcp()

            if self.get_connection_type() == 'udp':
                disconnected = _disconnect_udp()

            self._connected = not disconnected

            return disconnected

        def write(self, command: bytes) -> bool:

            sending_successful = False

            def _send_serial_command():

                try:
                    self.serial.write(command)
                    return True
                except Exception as exception:
                    self.fusion_messages.write_exception(message='Failed to send "{}" over serial: {}'.format(command, exception))
                    return False

            def _send_tcp_command():

                try:
                    self.tcp.sendall(command)
                    return True
                except Exception as exception:
                    self.fusion_messages.write_exception(message='Failed to send "{}" over tcp: {}'.format(command, exception))
                    return False

            def _send_udp_command():

                try:
                    self.udp.sendto(command, (
                    self.udp_configuration.ip, self.connection.udp_configuration.port))
                    return True
                except Exception as exception:
                    self.fusion_messages.write_exception(message='Failed to send "{}" over udp: {}'.format(command, exception))
                    return False

            if not self.get_connection_status():
                self.fusion_messages.write_message(message='Fusion is not connected. Can not send command: {}'.format(command))
                return False

            if self.get_connection_type() == 'serial':
                sending_successful = _send_serial_command()

            if self.get_connection_type() == 'tcp':
                sending_successful = _send_tcp_command()

            if self.get_connection_type() == 'udp':
                sending_successful = _send_udp_command()

            return sending_successful

        def read(self, size=None, terminator=None, timeout=None) -> bytes:

            read_data = b''

            _timeout = self.get_timeout()
            if timeout is not None:
                self.set_timeout(timeout=timeout)

            def _serial_read() -> bytes:

                serial_data = b''

                try:
                    if terminator is not None:
                        serial_data = self.serial.read_until(terminator, size)

                    elif size is not None:
                        serial_data = self.serial.read(size)

                    else:
                        if self.serial.in_waiting:
                            serial_data = self.serial.read(self.serial.in_waiting)

                except Exception as exception:
                    self.fusion_messages.write_exception(message='Failed to read serial data from Fusion: {}'.format(exception))

                return serial_data

            def _tcp_read() -> bytes:

                def _read() -> bool:
                    try:
                        self.tcp_buffer += self.tcp.recv(4096)
                        return True
                    except socket.timeout:
                        return False

                    except Exception as exception:
                        if self.get_connection_status():
                            self.fusion_messages.write_exception(message='Failed to read tcp data from Fusion: {}'.format(exception))

                        return False

                if terminator is not None:

                    while True:
                        if terminator in self.tcp_buffer:
                            break
                        else:
                            if not _read():
                                break

                    line, separator, self.tcp_buffer = self.tcp_buffer.partition(terminator)
                    tcp_data = line + separator

                elif size is not None:

                    while True:
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

            def _udp_read() -> bytes:

                udp_data = b''

                try:
                    self.udp_buffer += self.udp.recv(4096)

                    if terminator is not None:
                        line, separator, self.udp_buffer = self.udp_buffer.partition(terminator)
                        udp_data = line + separator

                    elif size is not None:
                        udp_data = self.udp_buffer[:size]
                        self.udp_buffer = self.udp_buffer[size:]

                    else:
                        udp_data = self.udp_buffer
                        self.tcp_buffer = b''

                except socket.timeout:
                    pass

                except Exception as exception:
                    if self.get_connection_status():
                        self.fusion_messages.write_exception(message='Failed to read udp data from Fusion: {}'.format(exception))

                return udp_data

            if self.get_connection_type() == 'serial':
                read_data = _serial_read()

            if self.get_connection_type() == 'tcp':
                read_data = _tcp_read()

            if self.get_connection_type() == 'udp':
                read_data = _udp_read()

            if timeout is not None:
                self.set_timeout(timeout=_timeout)

            return read_data

        def readline(self, timeout=None) -> bytes:

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
                        self.fusion_messages.write_warning('Got an unexpected exception when resetting TCP buffers: {}'.format(e))
                        break

            elif self.get_connection_type() == 'udp':
                self.udp_buffer = b''

    class FusionCommands:

        def __init__(self, fusion_messages, fusion_connection, fusion_thread, fusion_parser):

            self.fusion_messages = fusion_messages
            self.fusion_connection = fusion_connection
            self.fusion_thread = fusion_thread
            self.fusion_parser = fusion_parser

        def _reset_buffer(self):

            if self.fusion_thread.thread_running is not True:
                self.fusion_connection.reset_buffers()
            elif self.fusion_parser.get_queuing()['ascii'] is True:
                self.fusion_parser.clear_queue(queue_name='ascii')

        def _get_reply(self, terminator, timeout) -> [bytes]:

            get_reply = b''

            if self.fusion_thread.thread_running is not True:
                get_reply = self.fusion_connection.read(terminator=terminator, timeout=timeout)
            elif self.fusion_parser.get_queuing()['ascii'] is True:
                for i in range(100):  # Allow for up to 100 lines of reply before OK reply
                    ascii_packet = self.fusion_parser.read_ascii(timeout=timeout)
                    if ascii_packet is None:
                        break
                    else:
                        get_reply += ascii_packet['bytes']

                    if b'OK\r\n' in get_reply:
                        break

            return get_reply

        def _check_reply(self, data: bytes, terminator: bytes, command: bytes):

            if terminator not in data:
                if b'ERROR\r\n' in data:
                    get_error_reply = self.get_error().rstrip(terminator)
                    self.fusion_messages.write_exception(message='Received ERROR instead of OK after sending {}: {}'.format(command, get_error_reply))

                else:
                    self.fusion_messages.write_warning(message='Did not receive OK after sending {}: {}'.format(command, data))

        def _handle_reply(self, command, terminator, timeout: float = 0.2) -> [bytes]:

            get_reply = self._get_reply(terminator=terminator, timeout=timeout)
            self._check_reply(data=get_reply, terminator=terminator, command=command)

            return [i + b'\r\n' for i in get_reply.split(b'\r\n') if i]

        def get_error(self) -> [bytes]:

            self._reset_buffer()

            command = b'GETERROR\r\n'

            self.fusion_connection.write(command)

            get_reply = self._get_reply(terminator=b'OK\r\n', timeout=0.2)

            return get_reply

        def start(self) -> [bytes]:

            self._reset_buffer()

            command = b'START\r\n'

            self.fusion_connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=1)

            return get_reply

        def stop(self) -> [bytes]:

            self._reset_buffer()

            command = b'STOP\r\n'

            self.fusion_connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=1)

            return get_reply

        def fieldcal(self) -> [bytes]:

            self._reset_buffer()

            fieldcal_command = b'FIELDCAL\r\n'

            self.fusion_connection.write(fieldcal_command)

            get_reply = self._handle_reply(command=fieldcal_command, terminator=b'OK\r\n', timeout=1)

            return get_reply

        def reset(self) -> [bytes]:

            self._reset_buffer()

            command = b'RESET\r\n'

            self.fusion_connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=1)

            return get_reply

        def get_id(self, sn=False, str=False) -> [bytes]:

            self._reset_buffer()

            command = b'ID'

            if sn is True:
                command += b',SN'

            if str is True:
                command += b',STR'

            command += b'\r\n'

            self.fusion_connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=1)

            return get_reply

        def get_fw(self, str=False, major=False, minor=False, patch=False, build=False, hash=False, dvlfw=False, dvlminor=False, dvlboot=False, dvlfpga=False) -> [bytes]:

            self._reset_buffer()

            command = b'GETFW'

            if str is True:
                command += b',STR'

            if major is True:
                command += b',MAJOR'

            if minor is True:
                command += b',MINOR'

            if patch is True:
                command += b',PATCH'

            if build is True:
                command += b',BUILD'

            if hash is True:
                command += b',HASH'

            if dvlfw is True:
                command += b',DVLFW'

            if dvlminor is True:
                command += b',DVLMINOR'

            if dvlboot is True:
                command += b',DVLBOOT'

            if dvlfpga is True:
                command += b',DVLFPGA'

            command += b'\r\n'

            self.fusion_connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=1)

            return get_reply

    class FusionLogging:

        def __init__(self, fusion_message):

            self.fusion_message = fusion_message

            self._logging = False

            self._writing_packet = False
            self._writing_ascii = False
            self._writing_condition = False

            self._path = str(Path.cwd()) + '/logs'
            self.packet_file = None
            self.ascii_file = None
            self.condition_file = None

            self.packet_writer = None
            self.ascii_writer = None
            self.condition_writer = None

        @staticmethod
        def _get_field_names_packet():
            """function to return the fieldnames for the UNS logging"""
            field_names = ['id', 'timestamp_python', 'timestamp', 'serial_number', 'error', 'status',
                           'accelerometer_x', 'accelerometer_y', 'accelerometer_z', 'gyro_x', 'gyro_y', 'gyro_z',
                           'magnetometer_x', 'magnetometer_y', 'magnetometer_z',
                           'sound_speed', 'temperature', 'pressure',
                           'velocity_beam_0', 'velocity_beam_1', 'velocity_beam_2',
                           'distance_beam_0', 'distance_beam_1', 'distance_beam_2',
                           'fom_beam_0', 'fom_beam_1', 'fom_beam_2',
                           'dt_beam_0', 'dt_beam_1', 'dt_beam_2',
                           'time_velocity_estimate_0', 'time_velocity_estimate_1', 'time_velocity_estimate_2',
                           'velocity_x', 'velocity_y', 'velocity_z',
                           'fom_x', 'fom_y', 'fom_z', 'dt_xyz', 'time_velocity_estimate_xyz',
                           'altimeter_distance', 'altimeter_quality',
                           'euler_angles_roll', 'euler_angles_pitch', 'euler_angles_heading',
                           'quaternion_0', 'quaternion_1', 'quaternion_2', 'quaternion_3',
                           'dcm_11', 'dcm_12', 'dcm_13', 'dcm_21', 'dcm_22', 'dcm_23', 'dcm_31', 'dcm_32', 'dcm_33',
                           'declination', 'depth', 'operation_mode', 'fom_ahrs', 'fom_fc1',
                           'delta_quaternion_0', 'delta_quaternion_1', 'delta_quaternion_2', 'delta_quaternion_3',
                           'course_over_ground', 'turn_rate_x', 'turn_rate_y', 'turn_rate_z',
                           'altitude', 'latitude', 'longitude', 'height',
                           'position_frame_x', 'position_frame_y', 'position_frame_z',
                           'delta_position_frame_x', 'delta_position_frame_y', 'delta_position_frame_z',
                           'delta_position_uns_x', 'delta_position_uns_y', 'delta_position_uns_z',
                           'velocity_ned_x', 'velocity_ned_y', 'velocity_ned_z',
                           'velocity_uns_x', 'velocity_uns_y', 'velocity_uns_z',
                           'delta_velocity_ned_x', 'delta_velocity_ned_y', 'delta_velocity_ned_z',
                           'delta_velocity_uns_x', 'delta_velocity_uns_y', 'delta_velocity_uns_z',
                           'velocity_sog',
                           'hard_iron_x', 'hard_iron_y', 'hard_iron_z',
                           's_axis_00', 's_axis_01', 's_axis_02', 's_axis_10', 's_axis_11', 's_axis_12', 's_axis_20',
                           's_axis_21', 's_axis_22',
                           'new_point_0', 'new_point_1', 'new_point_2',
                           'fom', 'coverage']

            return field_names

        @staticmethod
        def _get_field_names_condition():
            """function to return the fieldnames for the condition logging"""
            field_names = ['timestamp_python', 'error_message', 'failed_packet']

            return field_names

        @staticmethod
        def _get_field_names_ascii():
            """function to return the fieldnames for the info logging"""
            field_names = ['timestamp_python', 'message']

            return field_names

        def set_path(self, path: str):

            self._path = path.rstrip('/')

        def start(self) -> str:

            folder = self._path + '/' + datetime.now().strftime('%y%m%d_%H%M%S')

            Path(folder).mkdir(parents=True, exist_ok=True)

            self.fusion_message.write_message('Logging started. Path: {}'.format(folder))

            self.packet_file = open(folder + '/fusion_log.csv', 'w', newline='')
            self.condition_file = open(folder + '/condition_log.csv', 'w', newline='')
            self.ascii_file = open(folder + '/ascii_log.csv', 'w', newline='')

            self.packet_writer = csv.DictWriter(self.packet_file, fieldnames=self._get_field_names_packet())
            self.condition_writer = csv.DictWriter(self.condition_file, fieldnames=self._get_field_names_condition())
            self.ascii_writer = csv.DictWriter(self.ascii_file, fieldnames=self._get_field_names_ascii())

            self.packet_writer.writeheader()
            self.condition_writer.writeheader()
            self.ascii_writer.writeheader()

            self._logging = True

            return folder

        def stop(self):

            self._logging = False

            # give the writers up to 0.1 sec to complete writing
            for i in range(10):

                if not self._writing_packet and not self._writing_ascii and not self._writing_condition:
                    break

                time.sleep(0.01)

            if isinstance(self.packet_file, io.TextIOWrapper):
                self.packet_file.close()

            if isinstance(self.condition_file, io.TextIOWrapper):
                self.condition_file.close()

            if isinstance(self.ascii_file, io.TextIOWrapper):
                self.ascii_file.close()

    class RunClasses:

        def __init__(self, fusion_messages, fusion_connection, fusion_thread, fusion_logging, fusion_commands):

            self.fusion_connection = fusion_connection
            self.fusion_messages = fusion_messages
            self.fusion_thread = fusion_thread
            self.fusion_logging = fusion_logging
            self.fusion_commands = fusion_commands

            self.main_running = True
            self.logging_fieldcal = False

        def main(self):

            def print_title():

                driver_title = [r"  ______         _               _____       _                ",
                                r" |  ____|       (_)             |  __ \     (_)               ",
                                r" | |__ _   _ ___ _  ___  _ __   | |  | |_ __ ___   _____ _ __ ",
                                r" |  __| | | / __| |/ _ \| '_ \  | |  | | '__| \ \ / / _ \ '__|",
                                r" | |  | |_| \__ \ | (_) | | | | | |__| | |  | |\ V /  __/ |   ",
                                r" |_|   \__,_|___/_|\___/|_| |_| |_____/|_|  |_| \_/ \___|_|   "]

                for line in driver_title:
                    self.fusion_messages.write_message(line)

                self.fusion_messages.write_message('')

            def print_commands():

                COMMAND_DICT = {'status': 'returns the status of the connection, information about fusion and logging status',
                                'commands': 'Sends commands to the Fusion. This allows for starting and stoping measurements among other things',
                                'logging': 'logs data from the Fusion to files',
                                'list_commands': 'lists these commands',
                                'help': 'Provides further explenation of the functionality of the driver',
                                'quit': 'Ends this script'}

                self.fusion_messages.write_message('\nInput one of the following commands to enter the respective functionality of the Fusion driver\n')
                for key in COMMAND_DICT:
                    self.fusion_messages.write_message('{} - {}'.format(key, COMMAND_DICT[key]))

            def print_status():

                self.fusion_messages.write_message(
                    '\n######################################################################################################################################################')

                connection_status = self.fusion_connection.get_connection_status()

                if connection_status is True:
                    connection_type = self.fusion_connection.get_connection_type()
                    self.fusion_messages.write_message('\nFusion connected: {}'.format(connection_type))
                    if connection_type == 'serial':
                        self.fusion_messages.write_message(
                            'port: {}'.format(self.fusion_connection.serial_configuration.port))
                        self.fusion_messages.write_message(
                            'baudrate: {}'.format(self.fusion_connection.serial_configuration.baudrate))
                    if connection_type == 'tcp':
                        self.fusion_messages.write_message(
                            'hostname: {}'.format(self.fusion_connection.tcp_configuration.host))
                        self.fusion_messages.write_message(
                            'port: {}'.format(self.fusion_connection.tcp_configuration.port))
                    if connection_type == 'udp':
                        self.fusion_messages.write_message(
                            'port: {}'.format(self.fusion_connection.udp_configuration.port))

                    self.fusion_messages.write_message('\nFusion ID: {}'.format(self.fusion_connection.fusion_id))
                    self.fusion_messages.write_message(
                        'Firmware version: {}'.format(self.fusion_connection.firmware_version))

                else:
                    self.fusion_messages.write_message('\nFusion connected: {}'.format(connection_status))

                logging = self.fusion_logging._logging
                self.fusion_messages.write_message('\nLogging: {}'.format(logging))
                if logging:
                    self.fusion_messages.write_message('Path: {}'.format(self.fusion_logging._path))

                self.fusion_messages.write_message(
                    '\n######################################################################################################################################################')

            print_title()

            self.connection()
            if not self.fusion_connection.get_connection_status():
                self.fusion_messages.write_message('Fusion is not connected. Quitting...')
                return

            print_status()
            print_commands()

            while self.main_running:

                reply = input('\nmain: ')

                if reply == 'status':

                    print_status()

                elif reply == 'commands':

                    self.commands()

                elif reply == 'logging':

                    self.logging()
                    print_status()

                elif reply == 'list_commands':

                    print_commands()

                elif reply == 'help':

                    pass

                elif reply == 'quit':

                    self.main_running = False

                else:

                    self.fusion_messages.write_message('Unknown command')

        def connection(self):

            def serial_configuration():

                port = self.fusion_connection.select_serial_port()
                baudrate = input('\nserial - baudrate (input nothing for 115200): ')

                if baudrate == '':
                    baudrate = 115200
                else:
                    try:
                        baudrate = int(baudrate)
                    except ValueError:
                        self.fusion_messages.write_warning('baudrate is not integer, port value will not be set')
                        baudrate = None

                self.fusion_connection.set_serial_configuration(port=port, baudrate=baudrate)

            def tcp_configuration():

                self.fusion_messages.write_message('\nConnect through TCP with host(hostname/ip) or serial number: ')
                self.fusion_messages.write_message('[0] Host')
                self.fusion_messages.write_message('[1] Serial_number')
                reply = input('Input integer value in range [0:1]: ')

                if reply == '0':
                    host = input('\ntcp - host: ')
                elif reply == '1':
                    serial_number = input('\ntcp - serial number: ')
                    host = 'NortekFusion-{}.local'.format(str.zfill(serial_number, 4))

                port = input('\ntcp - port: ')

                if host == '':
                    host = None

                if port == '':
                    port = None
                else:
                    try:
                        port = int(port)
                    except ValueError:
                        self.fusion_messages.write_warning('port is not integer, port value will not be set')
                        port = None

                self.fusion_connection.set_tcp_configuration(host=host, port=port)

            def udp_configuration():

                port = input('\nudp - port: ')

                if port == '':
                    port = None
                else:
                    try:
                        port = int(port)
                    except ValueError:
                        self.fusion_messages.write_warning('port is not integer, port value will not be set')
                        port = None

                self.fusion_connection.set_udp_configuration(port=port)

            if self.fusion_thread.thread.is_alive():
                self.fusion_messages.write_message('Can not handle connection while logging thread is running')
                return

            running = True

            while running:

                self.fusion_messages.write_message('Select connection type: ')
                self.fusion_messages.write_message('[0] Serial')
                self.fusion_messages.write_message('[1] TCP')
                self.fusion_messages.write_message('[2] UDP')

                reply = input('Input integer value in range [0:2]: ')

                if reply == '0':
                    serial_configuration()
                    self.fusion_connection.connect(connection_type='serial')

                elif reply == '1':
                    tcp_configuration()
                    self.fusion_connection.connect(connection_type='tcp')

                elif reply == '2':
                    udp_configuration()
                    self.fusion_connection.connect(connection_type='udp')

                else:
                    self.fusion_messages.write_message('Invalid selection')

                if self.fusion_connection.get_connection_status():
                    running = False
                else:
                    self.fusion_messages.write_message('Failed to connect to Fusion')

        def commands(self):

            running = True

            self.fusion_messages.write_message('Send commands diectly to Fusion. Input nothing to return')

            while running:

                command = input('\ncommands: ')

                if command == '':
                    running = False

                elif command.upper() == 'START':
                    self.fusion_messages.write_message('start command is only supported through logging')

                elif command.upper() == 'FIELDCAL':

                    self.fusion_messages.write_message('fieldcal command is only supported through logging')

                elif command.upper() == 'STOP':

                    self.fusion_messages.write_message('stop command is only supported through logging')

                else:
                    self.fusion_connection.write(command.encode() + b'\r\n')
                    for i in range(100):
                        command_reply = self.fusion_connection.readline()
                        if command_reply == b'':
                            break
                        if command_reply:
                            try:
                                self.fusion_messages.write_message(command_reply.decode())
                            except Exception as e:
                                self.fusion_messages.write_message('Could not decode reply: {}'.format(command_reply))
                                self.fusion_messages.write_message('Received error: {}'.format(e))

        def logging(self):

            def print_commands():

                COMMAND_DICT = {
                    'start': 'Starts logging',
                    'stop': 'Stops logging',
                    'set_log_path': 'Specify path for where log files should be saved. If not specified a default path will be used',
                    'status': 'Returns status of logging',
                    'list_commands': 'lists these commands',
                    'return': 'Leaves setup and returns to main',
                    'quit': 'Ends this script'}

                self.fusion_messages.write_message(
                    '\nInput one of the following commands to enter the respective functionality of the Fusion driver\n')
                for key in COMMAND_DICT:
                    self.fusion_messages.write_message('{} - {}'.format(key, COMMAND_DICT[key]))

            print_commands()

            running = True

            while running:

                reply = input('\nlogging: ')

                if reply == 'start':

                    self.fusion_thread.start()
                    self.fusion_logging.start()
                    self.fusion_commands.start()

                elif reply == 'fieldcal':

                    self.fusion_thread.start()
                    self.fusion_logging.start()
                    self.fusion_commands.fieldcal()

                    self.logging_fieldcal = True

                elif reply == 'stop':

                    self.fusion_logging.stop()
                    self.fusion_thread.stop()
                    if self.logging_fieldcal:
                        time.sleep(0.5)
                    self.fusion_commands.stop()

                    self.logging_fieldcal = False

                elif reply == 'set_log_path':

                    path = input('logging - set_log_path: ')
                    self.fusion_logging.set_path(path=path)

                elif reply == 'status':

                    self.fusion_messages.write_message('logging running: {}'.format(self.fusion_logging._logging))

                elif reply == 'list_commands':

                    print_commands()

                elif reply == 'return':

                    running = False

                elif reply == 'quit':

                    running = False
                    self.main_running = False

                else:

                    self.fusion_messages.write_message('Unknown command')


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--connection-type', default=None, help='connection type (default=None)')
    parser.add_argument('--connection-serial-port', default=None, help='serial com port (default=None)')
    parser.add_argument('--connection-serial-baudrate', default=None, help='serial baudrate (default=None)')
    parser.add_argument('--connection-tcp-host', default=None, help='tcp host (default=None)')
    parser.add_argument('--connection-tcp-port', default=None, help='tcp port (default=None)')
    parser.add_argument('--connection-udp-port', default=None, help='udp port (default=None)')
    parser.add_argument('--logging-set-path', default=None, help='set log files path (default=None)')
    args = parser.parse_args()

    fusion_driver = FusionDriver()

    if args.connection_serial_port is not None:
        fusion_driver.connection.set_serial_configuration(port=args.connection_serial_port)

    if args.connection_serial_baudrate is not None:
        fusion_driver.connection.set_serial_configuration(baudrate=int(args.connection_serial_baudrate))

    if args.connection_tcp_host is not None:
        fusion_driver.connection.set_tcp_configuration(host=args.connection_tcp_host)

    if args.connection_tcp_port is not None:
        fusion_driver.connection.set_tcp_configuration(port=int(args.connection_tcp_port))

    if args.connection_udp_port is not None:
        fusion_driver.connection.set_udp_configuration(port=int(args.connection_udp_port))

    if args.connection_type is not None:
        if args.connection_type == 'serial':

            if fusion_driver.connection.serial_configuration.port is None:
                serial_port = fusion_driver.connection.select_serial_port()
                fusion_driver.connection.set_serial_configuration(port=serial_port)

            if fusion_driver.connection.serial_configuration.baudrate is None:
                baudrate = input('serial baudrate: ')
                fusion_driver.connection.set_serial_configuration(baudrate=int(baudrate))

        elif args.connection_type == 'tcp':

            if fusion_driver.connection.tcp_configuration.host is None:
                host = input('TCP host: ')

                if host == '':
                    host = None

                fusion_driver.connection.set_tcp_configuration(host=host)

            if fusion_driver.connection.tcp_configuration.port is None:
                port = input('TCP port: ')

                fusion_driver.connection.set_tcp_configuration(port=int(port))

        elif args.connection_type == 'udp':

            if fusion_driver.connection.udp_configuration.port is None:
                port = input('UDP port: ')

                fusion_driver.connection.set_udp_configuration(port=int(port))

        fusion_driver.connection.connect(connection_type=args.connection_type)

    if args.logging_set_path is not None:
        fusion_driver.logging.set_path(path=args.logging_set_path)

    fusion_driver.parser.set_queuing(ascii=True)

    fusion_driver.run_classes.main()
