import os
import sys
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
import select
import errno
from math import ceil
from re import match


class NucleusDriver:

    def __init__(self):

        self.messages = self.Messages()
        self.connection = self.Connection(self.messages)
        self.logging = self.Logging(messages=self.messages, connection=self.connection)
        self.parser = self.Parser(self.messages, self.logging, self.connection)
        self.commands = self.Commands(messages=self.messages, connection=self.connection, parser=self.parser)
        self.run_classes = self.RunClasses(messages=self.messages, connection=self.connection, parser=self.parser,
                                           logging=self.logging, commands=self.commands)

        self.connection.commands = self.commands
        self.logging.commands = self.commands

    class Messages:

        def write_message(self, message: str, skip_newline=False):

            if not skip_newline:
                print(message)
            else:
                print(message, end='')

        def write_warning(self, message: str):

            print('WARNING: {}'.format(message))

        def write_exception(self, message: str):

            print('EXCEPTION: {}'.format(message))

    class Parser:

        FAMILY_ID_NUCLEUS = 0x20
        FAMILY_ID_DVL = 0x16

        ID_IMU = 0x82
        ID_MAGNETOMETER = 0x87
        ID_BOTTOMTRACK = 0xb4
        ID_WATERTRACK = 0xbe
        ID_ALTIMETER = 0xaa
        ID_AHRS = 0xd2
        ID_INS = 0xdc
        ID_FIELD_CALIBRATION = 0x8B
        ID_ASCII = 0xA0
        ID_SPECTRUM_ANALYZER = 0x20
        ID_CURRENT_PROFILE = 0xC0

        DEFAULT_RESPONSE_TIMEOUT = 5

        MAX_PACKAGE_LENGTH = 7000
        # MAX_PACKAGE_LENGTH is given by the following firmware scripts:
        # https://dev.azure.com/NortekGroup/INS/_git/Firmware?path=/src/ofmt.c&version=GBmaster&line=28&lineEnd=29&lineStartColumn=1&lineEndColumn=1&lineStyle=plain&_a=contents
        # https://dev.azure.com/NortekGroup/INS/_git/Firmware?path=/lib/sensor_fusion/src/uns_types.h&version=GBmaster&line=10&lineEnd=11&lineStartColumn=1&lineEndColumn=1&lineStyle=plain&_a=contents

        def __init__(self, messages, logging, connection):

            self.messages = messages
            self.logging = logging
            self.connection = connection

            self.packet_queue = queue.Queue()
            self.ascii_queue = queue.Queue()
            self.condition_queue = queue.Queue()

            self.reading_packet = False
            self.binary_packet = bytearray()
            self.ascii_packet = list()

            self._queuing = {'packet': False,
                             'ascii': False,
                             'condition': False}

            self.thread = Thread()
            self.thread_running = False

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

            if self.logging._logging is True:

                self.logging._writing_packet = True

                try:
                    self.logging.packet_writer.writerow(packet)
                except ValueError as exception:
                    self.messages.write_warning('Failed to write package to csv file: {}'.format(exception))

                self.logging._writing_packet = False

        def read_packet(self, timeout=None):

            packet = None

            try:
                if timeout is not None:
                    packet = self.packet_queue.get(timeout=timeout)

                elif not self.packet_queue.empty():
                    packet = self.packet_queue.get_nowait()

            except Exception as exception:
                self.messages.write_warning('failed to retrieve packet from packet queue: {}'.format(exception))

            return packet

        def write_ascii(self, packet):

            if self._queuing['ascii'] is True:
                ascii_bytes = bytes(packet)
                ascii_packet = {'timestamp_python': datetime.now().timestamp(),
                                'bytes': ascii_bytes}

                self.ascii_queue.put_nowait(ascii_packet)

            if self.logging._logging is True:
                ascii_message = ''.join(chr(i) for i in packet if 0 <= i <= 0x7e).rstrip('\r\n')
                ascii_packet = {'timestamp_python': datetime.now().timestamp(),
                                'message': ascii_message}

                self.logging._writing_ascii = True

                try:
                    self.logging.ascii_writer.writerow(ascii_packet)
                except ValueError as exception:
                    self.messages.write_warning('Failed to write ascii message to csv file: {}'.format(exception))

                self.logging._writing_ascii = False

        def read_ascii(self, timeout=None):

            packet = None

            try:
                if timeout is not None:
                    packet = self.ascii_queue.get(timeout=timeout)

                else:
                    packet = self.ascii_queue.get(timeout=self.connection.timeout)

            except queue.Empty:
                pass

            except Exception as exception:
                self.messages.write_warning('failed to retrieve packet from ascii queue: {}'.format(exception))

            return packet

        def write_condition(self, error_message, packet):

            failed_packet = {'timestamp_python': datetime.now().timestamp(),
                             'error_message': error_message,
                             'failed_packet': packet}

            if self._queuing['condition'] is True:
                self.condition_queue.put_nowait(failed_packet)

            if self.logging._logging is True:

                self.logging._writing_condition = True

                try:
                    self.logging.condition_writer.writerow(failed_packet)
                except ValueError as exception:
                    self.messages.write_warning('Failed to write condition to csv file: {}'.format(exception))

                self.logging._writing_condition = False

        def read_condition(self, timeout=None):

            packet = None

            try:
                if timeout is not None:
                    packet = self.condition_queue.get(timeout=timeout)

                elif not self.condition_queue.empty():
                    packet = self.condition_queue.get_nowait()

            except Exception as exception:
                self.messages.write_warning('failed to retrieve packet from condition queue: {}'.format(exception))

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

        def add_binary_packet(self, binary_packet, ascii_packet):

            header_checksum, data_checksum, packet = self.get_packet(binary_packet)

            if header_checksum:
                if data_checksum and packet['id'] == hex(self.ID_ASCII):
                    self.write_ascii(packet)

                elif data_checksum:
                    self.write_packet(packet)

                else:
                    self.write_condition(error_message='data checksum failed', packet=binary_packet)

                binary_packet = binary_packet[packet['sizeHeader'] + packet['sizeData']:]
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
                self.write_condition(error_message='header checksum failed', packet=binary_packet)
                reading_packet = False
                binary_packet = bytearray()

            return binary_packet, ascii_packet, reading_packet

        def get_packet(self, binary_packet):

            header_checksum = False
            data_checksum = False
            packet = dict()

            if not isinstance(binary_packet, bytearray):
                self.messages.write_exception('packet is not bytearray. Extraction aborted')
                return header_checksum, data_checksum, packet

            def check_header_size():

                status = False

                if len(binary_packet) >= unpack('<B', binary_packet[1:2])[0]:
                    status = True

                return status

            def check_packet_size():

                status = False

                if len(binary_packet) >= header_data['sizeHeader'] + header_data['sizeData']:
                    status = True

                return status

            def get_header_data():

                header = {'sizeHeader': unpack('<B', binary_packet[1:2])[0],
                          'id': unpack('<B', binary_packet[2:3])[0],
                          'family': unpack('<B', binary_packet[3:4])[0],
                          'sizeData': unpack('<H', binary_packet[4:6])[0],
                          'size': unpack('<B', binary_packet[1:2])[0] + unpack('<H', binary_packet[4:6])[0],
                          'dataCheckSum': unpack('<H', binary_packet[6:8])[0],
                          'headerCheckSum': unpack('<H', binary_packet[8:10])[0]}

                return header

            def get_common_data():

                common = {'version': unpack('<B', data[0:1])[0],
                          'offsetOfData': unpack('<B', data[1:2])[0],
                          'timeStamp': unpack('<I', data[4:8])[0],
                          'microSeconds': unpack('<I', data[8:12])[0]}

                return common

            def get_sensor_data():

                def get_status(status_bits, bit):

                    status = False

                    if (status_bits >> bit) & 0x01 == 1:
                        status = True

                    return status

                sensor = None

                if header_data['family'] == self.FAMILY_ID_NUCLEUS:

                    if header_data['id'] == self.ID_AHRS:

                        sensor = {'serialNumber': unpack('<I', data[16:20])[0],
                                  'operationMode': unpack('<B', data[24:25])[0],  # TODO: add operationModeString?
                                  'ahrsData.roll': unpack('<f', data[common_data['offsetOfData']: common_data['offsetOfData'] + 4])[0],
                                  'ahrsData.pitch': unpack('<f', data[common_data['offsetOfData'] + 4: common_data['offsetOfData'] + 8])[0],
                                  'ahrsData.heading': unpack('<f', data[common_data['offsetOfData'] + 8: common_data['offsetOfData'] + 12])[0],
                                  'ahrsData.quaternionW': unpack('<f', data[common_data['offsetOfData'] + 12: common_data['offsetOfData'] + 16])[0],
                                  'ahrsData.quaternionX': unpack('<f', data[common_data['offsetOfData'] + 16: common_data['offsetOfData'] + 20])[0],
                                  'ahrsData.quaternionY': unpack('<f', data[common_data['offsetOfData'] + 20: common_data['offsetOfData'] + 24])[0],
                                  'ahrsData.quaternionZ': unpack('<f', data[common_data['offsetOfData'] + 24: common_data['offsetOfData'] + 28])[0],
                                  'ahrsData.rotationMatrix_0': unpack('<f', data[common_data['offsetOfData'] + 28: common_data['offsetOfData'] + 32])[0],
                                  'ahrsData.rotationMatrix_1': unpack('<f', data[common_data['offsetOfData'] + 32: common_data['offsetOfData'] + 36])[0],
                                  'ahrsData.rotationMatrix_2': unpack('<f', data[common_data['offsetOfData'] + 36: common_data['offsetOfData'] + 40])[0],
                                  'ahrsData.rotationMatrix_3': unpack('<f', data[common_data['offsetOfData'] + 40: common_data['offsetOfData'] + 44])[0],
                                  'ahrsData.rotationMatrix_4': unpack('<f', data[common_data['offsetOfData'] + 44: common_data['offsetOfData'] + 48])[0],
                                  'ahrsData.rotationMatrix_5': unpack('<f', data[common_data['offsetOfData'] + 48: common_data['offsetOfData'] + 52])[0],
                                  'ahrsData.rotationMatrix_6': unpack('<f', data[common_data['offsetOfData'] + 52: common_data['offsetOfData'] + 56])[0],
                                  'ahrsData.rotationMatrix_7': unpack('<f', data[common_data['offsetOfData'] + 56: common_data['offsetOfData'] + 60])[0],
                                  'ahrsData.rotationMatrix_8': unpack('<f', data[common_data['offsetOfData'] + 60: common_data['offsetOfData'] + 64])[0],
                                  'declination': unpack('<f', data[common_data['offsetOfData'] + 64: common_data['offsetOfData'] + 68])[0],
                                  'depth': unpack('<f', data[common_data['offsetOfData'] + 68: common_data['offsetOfData'] + 72])[0]
                                  }

                        if common_data['version'] == 1:

                            sensor['fomAhrs'] = unpack('<B', data[25:26])[0]
                            sensor['fomFc1'] = unpack('<B', data[26:27])[0]

                        if common_data['version'] == 2:

                            sensor['fomAhrs'] = unpack('<f', data[28:32])[0]
                            sensor['fomFc1'] = unpack('<f', data[32:36])[0]

                    if header_data['id'] == self.ID_INS:

                        sensor = {'serialNumber': unpack('<I', data[16:20])[0],
                                  'operationMode': unpack('<B', data[24:25])[0],  # TODO: add operationModeString?
                                  'ahrsData.roll': unpack('<f', data[common_data['offsetOfData']: common_data['offsetOfData'] + 4])[0],
                                  'ahrsData.pitch': unpack('<f', data[common_data['offsetOfData'] + 4: common_data['offsetOfData'] + 8])[0],
                                  'ahrsData.heading': unpack('<f', data[common_data['offsetOfData'] + 8: common_data['offsetOfData'] + 12])[0],
                                  'ahrsData.quaternionW': unpack('<f', data[common_data['offsetOfData'] + 12: common_data['offsetOfData'] + 16])[0],
                                  'ahrsData.quaternionX': unpack('<f', data[common_data['offsetOfData'] + 16: common_data['offsetOfData'] + 20])[0],
                                  'ahrsData.quaternionY': unpack('<f', data[common_data['offsetOfData'] + 20: common_data['offsetOfData'] + 24])[0],
                                  'ahrsData.quaternionZ': unpack('<f', data[common_data['offsetOfData'] + 24: common_data['offsetOfData'] + 28])[0],
                                  'ahrsData.rotationMatrix_0': unpack('<f', data[common_data['offsetOfData'] + 28: common_data['offsetOfData'] + 32])[0],
                                  'ahrsData.rotationMatrix_1': unpack('<f', data[common_data['offsetOfData'] + 32: common_data['offsetOfData'] + 36])[0],
                                  'ahrsData.rotationMatrix_2': unpack('<f', data[common_data['offsetOfData'] + 36: common_data['offsetOfData'] + 40])[0],
                                  'ahrsData.rotationMatrix_3': unpack('<f', data[common_data['offsetOfData'] + 40: common_data['offsetOfData'] + 44])[0],
                                  'ahrsData.rotationMatrix_4': unpack('<f', data[common_data['offsetOfData'] + 44: common_data['offsetOfData'] + 48])[0],
                                  'ahrsData.rotationMatrix_5': unpack('<f', data[common_data['offsetOfData'] + 48: common_data['offsetOfData'] + 52])[0],
                                  'ahrsData.rotationMatrix_6': unpack('<f', data[common_data['offsetOfData'] + 52: common_data['offsetOfData'] + 56])[0],
                                  'ahrsData.rotationMatrix_7': unpack('<f', data[common_data['offsetOfData'] + 56: common_data['offsetOfData'] + 60])[0],
                                  'ahrsData.rotationMatrix_8': unpack('<f', data[common_data['offsetOfData'] + 60: common_data['offsetOfData'] + 64])[0],
                                  'declination': unpack('<f', data[common_data['offsetOfData'] + 64: common_data['offsetOfData'] + 68])[0],
                                  'depth': unpack('<f', data[common_data['offsetOfData'] + 68: common_data['offsetOfData'] + 72])[0],
                                  'deltaQuaternionW': unpack('<f', data[common_data['offsetOfData'] + 72: common_data['offsetOfData'] + 76])[0],
                                  'deltaQuaternionX': unpack('<f', data[common_data['offsetOfData'] + 76: common_data['offsetOfData'] + 80])[0],
                                  'deltaQuaternionY': unpack('<f', data[common_data['offsetOfData'] + 80: common_data['offsetOfData'] + 84])[0],
                                  'deltaQuaternionZ': unpack('<f', data[common_data['offsetOfData'] + 84: common_data['offsetOfData'] + 88])[0],
                                  'courseOverGround': unpack('<f', data[common_data['offsetOfData'] + 88: common_data['offsetOfData'] + 92])[0],
                                  'temperature': unpack('<f', data[common_data['offsetOfData'] + 92: common_data['offsetOfData'] + 96])[0],
                                  'pressure': unpack('<f', data[common_data['offsetOfData'] + 96: common_data['offsetOfData'] + 100])[0],
                                  'altitude': unpack('<f', data[common_data['offsetOfData'] + 100: common_data['offsetOfData'] + 104])[0],
                                  'latitude': unpack('<f', data[common_data['offsetOfData'] + 104: common_data['offsetOfData'] + 108])[0],
                                  'longitude': unpack('<f', data[common_data['offsetOfData'] + 108: common_data['offsetOfData'] + 112])[0],
                                  'height': unpack('<f', data[common_data['offsetOfData'] + 112: common_data['offsetOfData'] + 116])[0],
                                  'positionFrameX': unpack('<f', data[common_data['offsetOfData'] + 116: common_data['offsetOfData'] + 120])[0],
                                  'positionFrameY': unpack('<f', data[common_data['offsetOfData'] + 120: common_data['offsetOfData'] + 124])[0],
                                  'positionFrameZ': unpack('<f', data[common_data['offsetOfData'] + 124: common_data['offsetOfData'] + 128])[0],
                                  'deltaPositionFrameX': unpack('<f', data[common_data['offsetOfData'] + 128: common_data['offsetOfData'] + 132])[0],
                                  'deltaPositionFrameY': unpack('<f', data[common_data['offsetOfData'] + 132: common_data['offsetOfData'] + 136])[0],
                                  'deltaPositionFrameZ': unpack('<f', data[common_data['offsetOfData'] + 136: common_data['offsetOfData'] + 140])[0],
                                  'deltaPositionNucleusX': unpack('<f', data[common_data['offsetOfData'] + 140: common_data['offsetOfData'] + 144])[0],
                                  'deltaPositionNucleusY': unpack('<f', data[common_data['offsetOfData'] + 144: common_data['offsetOfData'] + 148])[0],
                                  'deltaPositionNucleusZ': unpack('<f', data[common_data['offsetOfData'] + 148: common_data['offsetOfData'] + 152])[0],
                                  'velocityNedX': unpack('<f', data[common_data['offsetOfData'] + 152: common_data['offsetOfData'] + 156])[0],
                                  'velocityNedY': unpack('<f', data[common_data['offsetOfData'] + 156: common_data['offsetOfData'] + 160])[0],
                                  'velocityNedZ': unpack('<f', data[common_data['offsetOfData'] + 160: common_data['offsetOfData'] + 164])[0],
                                  'velocityNucleusX': unpack('<f', data[common_data['offsetOfData'] + 164: common_data['offsetOfData'] + 168])[0],
                                  'velocityNucleusY': unpack('<f', data[common_data['offsetOfData'] + 168: common_data['offsetOfData'] + 172])[0],
                                  'velocityNucleusZ': unpack('<f', data[common_data['offsetOfData'] + 172: common_data['offsetOfData'] + 176])[0],
                                  'deltaVelocityNedX': unpack('<f', data[common_data['offsetOfData'] + 176: common_data['offsetOfData'] + 180])[0],
                                  'deltaVelocityNedY': unpack('<f', data[common_data['offsetOfData'] + 180: common_data['offsetOfData'] + 184])[0],
                                  'deltaVelocityNedZ': unpack('<f', data[common_data['offsetOfData'] + 184: common_data['offsetOfData'] + 188])[0],
                                  'deltaVelocityNucleusX': unpack('<f', data[common_data['offsetOfData'] + 188: common_data['offsetOfData'] + 192])[0],
                                  'deltaVelocityNucleusY': unpack('<f', data[common_data['offsetOfData'] + 192: common_data['offsetOfData'] + 196])[0],
                                  'deltaVelocityNucleusZ': unpack('<f', data[common_data['offsetOfData'] + 196: common_data['offsetOfData'] + 200])[0],
                                  'speedOverGround': unpack('<f', data[common_data['offsetOfData'] + 200: common_data['offsetOfData'] + 204])[0],
                                  'turnRateX': unpack('<f', data[common_data['offsetOfData'] + 204: common_data['offsetOfData'] + 208])[0],
                                  'turnRateY': unpack('<f', data[common_data['offsetOfData'] + 208: common_data['offsetOfData'] + 212])[0],
                                  'turnRateZ': unpack('<f', data[common_data['offsetOfData'] + 212: common_data['offsetOfData'] + 216])[0],
                                  }

                        if common_data['version'] == 1:

                            sensor['fomAhrs'] = unpack('<B', data[25:26])[0]
                            sensor['fomFc1'] = unpack('<B', data[26:27])[0]

                        if common_data['version'] == 2:

                            sensor['fomAhrs'] = unpack('<f', data[28:32])[0]
                            sensor['fomFc1'] = unpack('<f', data[32:36])[0]

                    if header_data['id'] == self.ID_IMU:

                        imu_status = unpack('<I', data[12:16])[0]

                        sensor = {'status.isValid': get_status(status_bits=imu_status, bit=0),
                                  'status.hasDataPathOverrun': get_status(status_bits=imu_status, bit=17),
                                  'status.hasFlashUpdateFailure': get_status(status_bits=imu_status, bit=18),
                                  'status.hasSpiComError': get_status(status_bits=imu_status, bit=19),
                                  'status.hasLowVoltage': get_status(status_bits=imu_status, bit=20),
                                  'status.hasSensorFailure': get_status(status_bits=imu_status, bit=21),
                                  'status.hasMemoryFailure': get_status(status_bits=imu_status, bit=22),
                                  'status.hasGyro1Failure': get_status(status_bits=imu_status, bit=23),
                                  'status.hasGyro2Failure': get_status(status_bits=imu_status, bit=24),
                                  'status.hasAccelerometerFailure': get_status(status_bits=imu_status, bit=25),
                                  'accelerometer.x': unpack('<f', data[common_data['offsetOfData']: common_data['offsetOfData'] + 4])[0],
                                  'accelerometer.y': unpack('<f', data[common_data['offsetOfData'] + 4: common_data['offsetOfData'] + 8])[0],
                                  'accelerometer.z': unpack('<f', data[common_data['offsetOfData'] + 8: common_data['offsetOfData'] + 12])[0],
                                  'gyro.x': unpack('<f', data[common_data['offsetOfData'] + 12: common_data['offsetOfData'] + 16])[0],
                                  'gyro.y': unpack('<f', data[common_data['offsetOfData'] + 16: common_data['offsetOfData'] + 20])[0],
                                  'gyro.z': unpack('<f', data[common_data['offsetOfData'] + 20: common_data['offsetOfData'] + 24])[0],
                                  'temperature': unpack('<f', data[common_data['offsetOfData'] + 24: common_data['offsetOfData'] + 28])[0]
                                  }

                    if header_data['id'] == self.ID_MAGNETOMETER:

                        mag_status = unpack('<I', data[12:16])[0]

                        sensor = {'status.isCompensatedForHardIron': get_status(status_bits=mag_status, bit=0),
                                  'status.dvlActive': get_status(status_bits=mag_status, bit=29),
                                  'status.dvlAcousticsActive': get_status(status_bits=mag_status, bit=30),
                                  'status.dvlTransmitterActive': get_status(status_bits=mag_status, bit=31),
                                  'magnetometer.x': unpack('<f', data[common_data['offsetOfData']: common_data['offsetOfData'] + 4])[0],
                                  'magnetometer.y': unpack('<f', data[common_data['offsetOfData'] + 4: common_data['offsetOfData'] + 8])[0],
                                  'magnetometer.z': unpack('<f', data[common_data['offsetOfData'] + 8: common_data['offsetOfData'] + 12])[0]
                                  }

                    if header_data['id'] in (self.ID_BOTTOMTRACK, self.ID_WATERTRACK):

                        status = unpack('<I', data[12:16])[0]

                        sensor = {'status.beam1VelocityValid': get_status(status_bits=status, bit=0),
                                  'status.beam2VelocityValid': get_status(status_bits=status, bit=1),
                                  'status.beam3VelocityValid': get_status(status_bits=status, bit=2),
                                  'status.beam1DistanceValid': get_status(status_bits=status, bit=3),
                                  'status.beam2DistanceValid': get_status(status_bits=status, bit=4),
                                  'status.beam3DistanceValid': get_status(status_bits=status, bit=5),
                                  'status.beam1FomValid': get_status(status_bits=status, bit=6),
                                  'status.beam2FomValid': get_status(status_bits=status, bit=7),
                                  'status.beam3FomValid': get_status(status_bits=status, bit=8),
                                  'status.xVelocityValid': get_status(status_bits=status, bit=9),
                                  'status.yVelocityValid': get_status(status_bits=status, bit=10),
                                  'status.zVelocityValid': get_status(status_bits=status, bit=11),
                                  'status.xFomValid': get_status(status_bits=status, bit=12),
                                  'status.yFomValid': get_status(status_bits=status, bit=13),
                                  'status.zFomValid': get_status(status_bits=status, bit=14),
                                  'serialNumber': unpack('<I', data[16:20])[0],
                                  'soundSpeed': unpack('<f', data[24:28])[0],
                                  'temperature': unpack('<f', data[28:32])[0],
                                  'pressure': unpack('<f', data[32:36])[0],
                                  'velocityBeam1': unpack('<f', data[36:40])[0],
                                  'velocityBeam2': unpack('<f', data[40:44])[0],
                                  'velocityBeam3': unpack('<f', data[44:48])[0],
                                  'distanceBeam1': unpack('<f', data[48:52])[0],
                                  'distanceBeam2': unpack('<f', data[52:56])[0],
                                  'distanceBeam3': unpack('<f', data[56:60])[0],
                                  'fomBeam1': unpack('<f', data[60:64])[0],
                                  'fomBeam2': unpack('<f', data[64:68])[0],
                                  'fomBeam3': unpack('<f', data[68:72])[0],
                                  'dtBeam1': unpack('<f', data[72:76])[0],
                                  'dtBeam2': unpack('<f', data[76:80])[0],
                                  'dtBeam3': unpack('<f', data[80:84])[0],
                                  'timeVelBeam1': unpack('<f', data[84:88])[0],
                                  'timeVelBeam2': unpack('<f', data[88:92])[0],
                                  'timeVelBeam3': unpack('<f', data[92:96])[0],
                                  'velocityX': unpack('<f', data[96:100])[0],
                                  'velocityY': unpack('<f', data[100:104])[0],
                                  'velocityZ': unpack('<f', data[104:108])[0],
                                  'fomX': unpack('<f', data[108:112])[0],
                                  'fomY': unpack('<f', data[112:116])[0],
                                  'fomZ': unpack('<f', data[116:120])[0],
                                  'dtXYZ': unpack('<f', data[120:124])[0],
                                  'timeVelXYZ': unpack('<f', data[124:128])[0]
                                  }

                    if header_data['id'] == self.ID_ALTIMETER:

                        status = unpack('<I', data[12:16])[0]

                        sensor = {'status.altimeterDistanceValid': get_status(status_bits=status, bit=0),
                                  'status.altimeterQualityValid': get_status(status_bits=status, bit=1),
                                  'status.pressureValid': get_status(status_bits=status, bit=16),
                                  'status.temperatureValid': get_status(status_bits=status, bit=17),
                                  'serialNumber': unpack('<I', data[16:20])[0],
                                  'soundSpeed': unpack('<f', data[24:28])[0],
                                  'temperature': unpack('<f', data[28:32])[0],
                                  'pressure': unpack('<f', data[32:36])[0],
                                  'altimeterDistance': unpack('<f', data[36:40])[0],
                                  'altimeterQuality': unpack('<H', data[40:42])[0]
                                  }

                    if header_data['id'] == self.ID_CURRENT_PROFILE:

                        sensor = {'serialNumber': unpack('<I', data[16:20])[0],
                                  'soundVelocity': unpack('<f', data[24:28])[0],
                                  'temperature': unpack('<f', data[28:32])[0],
                                  'pressure': unpack('<f', data[32:36])[0],
                                  'cellSize': unpack('<f', data[36:40])[0],
                                  'blanking': unpack('<f', data[40:44])[0],
                                  'numberOfCells': unpack('<H', data[44:46])[0],
                                  'ambiguityVelocity': unpack('<H', data[46:48])[0]}

                        velocity_data_format = '<' + ''.ljust(3 * sensor['numberOfCells'], 'H')
                        velocity_data_offset = common_data['offsetOfData']
                        velocity_data_size = 2 * 3 * sensor['numberOfCells']
                        #sensor['velocityData'] = unpack(velocity_data_format, data[velocity_data_offset: velocity_data_offset + velocity_data_size])[:velocity_data_size]
                        velocity_data = unpack(velocity_data_format, data[velocity_data_offset: velocity_data_offset + velocity_data_size])[:velocity_data_size]
                        for index, velocity_cell in enumerate(velocity_data):
                            sensor['velocityData_{}'.format(index)] = velocity_cell

                        amplitude_data_format = '<' + ''.ljust(3 * sensor['numberOfCells'], 'B')
                        amplitude_data_offset = common_data['offsetOfData'] + 6 * sensor['numberOfCells']
                        amplitude_data_size = 1 * 3 * sensor['numberOfCells']
                        #sensor['amplitudeData'] = unpack(amplitude_data_format, data[amplitude_data_offset: amplitude_data_offset + amplitude_data_size])[:amplitude_data_size]
                        amplitude_data = unpack(amplitude_data_format, data[amplitude_data_offset: amplitude_data_offset + amplitude_data_size])[:amplitude_data_size]
                        for index, amplitude_cell in enumerate(amplitude_data):
                            sensor['amplitudeData_{}'.format(index)] = amplitude_cell

                        correlation_data_format = '<' + ''.ljust(3 * sensor['numberOfCells'], 'B')
                        correlation_data_offset = common_data['offsetOfData'] + 9 * sensor['numberOfCells']
                        correlation_data_size = 1 * 3 * sensor['numberOfCells']
                        #sensor['correlationData'] = unpack(correlation_data_format, data[correlation_data_offset: correlation_data_offset + correlation_data_size])[:correlation_data_size]
                        correlation_data = unpack(correlation_data_format, data[correlation_data_offset: correlation_data_offset + correlation_data_size])[:correlation_data_size]
                        for index, correlation_cell in enumerate(correlation_data):
                            sensor['correlationData_{}'.format(index)] = correlation_cell

                    if header_data['id'] == self.ID_FIELD_CALIBRATION:

                        status = unpack('<I', data[12:16])[0]

                        sensor = {'status.pointsUsedInEstimation': get_status(status_bits=status, bit=0),
                                  'hardIron.x': unpack('<f', data[common_data['offsetOfData']: common_data['offsetOfData'] + 4])[0],
                                  'hardIron.y': unpack('<f', data[common_data['offsetOfData'] + 4: common_data['offsetOfData'] + 8])[0],
                                  'hardIron.z': unpack('<f', data[common_data['offsetOfData'] + 8: common_data['offsetOfData'] + 12])[0],
                                  'sAxis_0': unpack('<f', data[common_data['offsetOfData'] + 12: common_data['offsetOfData'] + 16])[0],
                                  'sAxis_1': unpack('<f', data[common_data['offsetOfData'] + 16: common_data['offsetOfData'] + 20])[0],
                                  'sAxis_2': unpack('<f', data[common_data['offsetOfData'] + 20: common_data['offsetOfData'] + 24])[0],
                                  'sAxis_3': unpack('<f', data[common_data['offsetOfData'] + 24: common_data['offsetOfData'] + 28])[0],
                                  'sAxis_4': unpack('<f', data[common_data['offsetOfData'] + 28: common_data['offsetOfData'] + 32])[0],
                                  'sAxis_5': unpack('<f', data[common_data['offsetOfData'] + 32: common_data['offsetOfData'] + 36])[0],
                                  'sAxis_6': unpack('<f', data[common_data['offsetOfData'] + 36: common_data['offsetOfData'] + 40])[0],
                                  'sAxis_7': unpack('<f', data[common_data['offsetOfData'] + 40: common_data['offsetOfData'] + 44])[0],
                                  'sAxis_8': unpack('<f', data[common_data['offsetOfData'] + 44: common_data['offsetOfData'] + 48])[0],
                                  'newPoint.x': unpack('<f', data[common_data['offsetOfData'] + 48: common_data['offsetOfData'] + 52])[0],
                                  'newPoint.y': unpack('<f', data[common_data['offsetOfData'] + 52: common_data['offsetOfData'] + 56])[0],
                                  'newPoint.z': unpack('<f', data[common_data['offsetOfData'] + 56: common_data['offsetOfData'] + 60])[0],
                                  'fomFieldCalibration': unpack('<f', data[common_data['offsetOfData'] + 60: common_data['offsetOfData'] + 64])[0],
                                  'coverage': unpack('<f', data[common_data['offsetOfData'] + 64: common_data['offsetOfData'] + 68])[0]
                                  }

                    if header_data['id'] == self.ID_ASCII:

                        sensor = {'string': unpack('<{}s'.format(len(data)), data)}

                if header_data['family'] == self.FAMILY_ID_DVL:

                    if header_data['id'] == self.ID_SPECTRUM_ANALYZER:

                        sensor = {'data': data}

                return sensor

            if not check_header_size():
                self.messages.write_exception('Packet is smaller than specified header length. Extraction aborted')
                return header_checksum, data_checksum, packet

            header_data = get_header_data()

            if self.checksum(binary_packet[:header_data['sizeHeader'] - 2]) == header_data['headerCheckSum']:
                header_checksum = True
            else:
                self.messages.write_exception('Header did not pass checksum. Extraction aborted')
                return header_checksum, data_checksum, packet

            packet.update(header_data)

            if not check_packet_size():
                self.messages.write_exception('Packet is smaller than specified header and data length. Extraction aborted')
                return header_checksum, data_checksum, packet

            if self.checksum(binary_packet[header_data['sizeHeader']: header_data['sizeHeader'] + header_data['sizeData']]) == header_data['dataCheckSum']:
                data_checksum = True
            else:
                self.messages.write_exception('Packet did not pass checksum. Extraction aborted')
                return header_checksum, data_checksum, packet

            data = binary_packet[header_data['sizeHeader']:header_data['sizeHeader'] + header_data['sizeData']]

            common_data = get_common_data()
            packet.update(common_data)

            packet['timestampPython'] = datetime.now().timestamp()

            sensor_data = get_sensor_data()

            if sensor_data is None:
                self.messages.write_exception('Unable to unpack sensor data. Extraction aborted')
                return header_checksum, data_checksum, packet

            packet.update(sensor_data)

            return header_checksum, data_checksum, packet

        def add_ascii_packet(self, ascii_packet):

            self.write_ascii(packet=ascii_packet)

        def add_data(self, data):
            for value in data:
                if value == 0xa5 and not self.reading_packet:
                    self.reading_packet = True
                    self.ascii_packet = list()

                if self.reading_packet:
                    self.binary_packet.append(value)
                else:
                    self.ascii_packet.append(value)

                if len(self.binary_packet) > self.MAX_PACKAGE_LENGTH or len(self.binary_packet) > 5 and len(self.binary_packet) >= self.binary_packet[1] + (self.binary_packet[4] & 0xff) | ((self.binary_packet[5] & 0xff) << 8):
                    self.binary_packet, self.ascii_packet, self.reading_packet = self.add_binary_packet(self.binary_packet, self.ascii_packet)

                if len(self.ascii_packet) >= 2 and self.ascii_packet[-2] == 0x0d and self.ascii_packet[-1] == 0x0a:
                    self.add_ascii_packet(self.ascii_packet)
                    self.ascii_packet = list()

        def start(self) -> bool:

            if self.thread.is_alive():
                self.messages.write_warning(message='Nucleus thread is already alive')
                return False

            self.thread = Thread(target=self.run)
            self.thread.start()

            return True

        def stop(self) -> bool:

            if not self.thread.is_alive():
                self.messages.write_warning(message='Nucleus thread is not alive')
                return False

            self.thread_running = False
            self.thread.join(2)
            self.thread = Thread()

            return True

        def run(self):

            self.thread_running = True

            while self.thread_running:

                if not self.connection.get_connection_status():
                    time.sleep(0.05)
                    continue

                data = self.connection.read()

                if data:

                    self.add_data(data=data)

                else:
                    time.sleep(0.01)

    class Connection:

        CONNECTION_TYPES = ['serial', 'udp', 'tcp']

        @dataclass
        class SerialConfiguration:
            port: str = None
            baudrate: int = None

        @dataclass
        class TcpConfiguration:
            host: str = None
            port: int = None

        @dataclass
        class UdpConfiguration:
            ip: str = '192.168.2.2'  # UDP address of ROV running on ArduSub software
            port: int = None

        def __init__(self, messages):

            self.messages = messages
            self.commands = None

            self._connection_type = None
            self._connected = False

            self.serial = serial.Serial()
            self.tcp = socket.socket()
            self.udp = socket.socket()

            self.serial_configuration = self.SerialConfiguration()
            self.tcp_configuration = self.TcpConfiguration()
            self.udp_configuration = self.UdpConfiguration()
            self.timeout = 1

            self.tcp_buffer = b''
            self.udp_buffer = b''

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

        def set_udp_configuration(self, ip: str = None, port: int = None):

            if ip is not None:
                self.udp_configuration.ip = ip

            if port is not None:
                self.udp_configuration.port = port

        def get_serial_number_from_tcp_hostname(self) -> int:

            serial_number = None

            if self.tcp_configuration.host is None:
                self.messages.write_warning('Could not extract serial number from hostname. Hostname is None')
                return serial_number

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

                    if self._connection_type == 'udp':
                        self.udp = socket.socket()

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

                    # This needs to support:
                    # b'Welcome to Nortek Nucleus1000'
                    # b'Welcome to Nortek Fusion DVL1000'
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

            def _connect_udp() -> bool:

                if self.udp_configuration.port is None:
                    self.messages.write_message(message='tcp_configuration.port is not defined')
                    return False

                try:
                    self.udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

                    self.udp.settimeout(self.timeout)

                    return True

                except Exception as exception:
                    self.messages.write_exception(message='Failed to connect through UDP: {}'.format(exception))

                    return False

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

            if self.get_connection_type() == 'udp':
                self._connected = _connect_udp()

            if self.get_connection_status() and self.commands is not None and get_device_info:

                self.get_info()

            return self.get_connection_status()

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

            def _disconnect_udp():

                try:
                    self.udp.close()
                    self._connection_type = None
                    self.udp = socket.socket()

                    return True

                except Exception as exception:
                    self.messages.write_exception(message='Failed to disconnet from serial connection: {}'.format(exception))

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

            def _send_udp_command():

                try:
                    self.udp.sendto(command, (self.udp_configuration.ip, int(self.udp_configuration.port)))
                    return True
                except Exception as exception:
                    self.messages.write_exception(message='Failed to send "{}" over udp: {}'.format(command, exception))
                    return False

            if not self.get_connection_status():
                self.messages.write_message(message='Nucleus is not connected. Can not send command: {}'.format(command))
                return False

            if self.get_connection_type() == 'serial':
                sending_successful = _send_serial_command()

            if self.get_connection_type() == 'tcp':
                sending_successful = _send_tcp_command()

            if self.get_connection_type() == 'udp':
                sending_successful = _send_udp_command()

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

            def _udp_read() -> bytes:

                def _read() -> bool:
                    try:
                        self.udp_buffer += self.udp.recv(4096)
                        return True
                    except socket.timeout:
                        return True
                    except Exception as exception:
                        if self.get_connection_status():
                            self.messages.write_exception(message='Failed to read udp data from Nucleus: {}'.format(exception))

                        return False

                if terminator is not None:

                    init_time = datetime.now()
                    while (datetime.now() - init_time).seconds < timeout:
                        if terminator in self.udp_buffer:
                            break
                        else:
                            if not _read():
                                break

                    line, separator, self.udp_buffer = self.udp_buffer.partition(terminator)
                    udp_data = line + separator

                elif size is not None:

                    init_time = datetime.now()
                    while (datetime.now() - init_time).seconds < timeout:
                        if len(self.udp_buffer) >= size:
                            break
                        else:
                            if not _read():
                                break

                    udp_data = self.udp_buffer[:size]
                    self.udp_buffer = self.udp_buffer[size:]

                else:
                    _read()
                    udp_data = self.udp_buffer
                    self.udp_buffer = b''

                return udp_data

            if self.get_connection_type() == 'serial':
                read_data = _serial_read()

            if self.get_connection_type() == 'tcp':
                read_data = _tcp_read()

            if self.get_connection_type() == 'udp':
                read_data = _udp_read()

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

            elif self.get_connection_type() == 'udp':
                self.udp_buffer = b''

    class Commands:

        DHCP_STATIC = ['DHCP', 'STATIC']
        ADDRESS_PATTERN = r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$'

        def __init__(self, **kwargs):

            self.connection = kwargs.get('connection')
            self.messages = kwargs.get('messages')
            self.parser = kwargs.get('parser')

        def _reset_buffer(self):

            if self.parser.thread_running is not True:
                self.connection.reset_buffers()
            elif self.parser.get_queuing()['ascii'] is True:
                self.parser.clear_queue(queue_name='ascii')

        def _get_reply(self, terminator: bytes = None, timeout: int = 1) -> [bytes]:

            get_reply = b''

            if self.parser.thread_running is not True:
                get_reply = self.connection.read(terminator=terminator, timeout=timeout)
            elif self.parser.get_queuing()['ascii'] is True:
                init_time = datetime.now()
                while (datetime.now() - init_time).seconds < timeout:
                    ascii_packet = self.parser.read_ascii()
                    if ascii_packet is not None:
                        get_reply += ascii_packet['bytes']

                    if terminator is not None and terminator in get_reply:
                        break


            return get_reply

        def _check_reply(self, data: bytes, command: bytes, terminator: bytes = None):

            if terminator is not None and terminator not in data:
                if b'ERROR' in data:
                    get_error_reply = self.get_error().rstrip(b'OK\r\n')
                    self.messages.write_exception(message='Received ERROR instead of {} after sending {}: {}'.format(terminator, command, get_error_reply))

                else:
                    self.messages.write_warning(message='Did not receive {} after sending {}: {}'.format(terminator, command, data))

            elif terminator is None:
                if b'ERROR' in data:
                    get_error_reply = self.get_error().rstrip(b'OK\r\n')
                    self.messages.write_exception(message='Received ERROR after sending {}: {}'.format(command, get_error_reply))

        def _handle_reply(self, command, terminator: bytes = None, timeout: int = 1, nmea=False) -> [bytes]:

            if nmea:
                terminator = b'$PNOR,' + terminator.rstrip(b'\r\n')
                nmea_checksum = self._nmea_checksum(terminator)
                terminator = terminator + b'*' + nmea_checksum + b'\r\n'

            get_reply = self._get_reply(terminator=terminator, timeout=timeout)
            self._check_reply(data=get_reply, terminator=terminator, command=command)

            reply_list = [i + b'\r\n' for i in get_reply.split(b'\r\n') if i]

            if nmea:
                for reply in reply_list:
                    reply_split = reply.split(b'*')
                    nmea_checksum = self._nmea_checksum(reply_split[0])
                    if nmea_checksum != reply_split[1].rstrip(b'\r\n'):
                        self.messages.write_warning('Reply did not pass nmea checksum: {}\t{}'.format(reply, nmea_checksum))

            return reply_list

        def _nmea_checksum(self, command):

            checksum = 0

            command = command.lstrip(b'$')

            for byte in command:
                checksum ^= byte

            return '{0:02X}'.format(checksum).encode()

        def get_error(self) -> [bytes]:

            self._reset_buffer()

            command = b'GETERROR\r\n'

            self.connection.write(command)

            get_reply = self._get_reply(terminator=b'OK\r\n')

            return get_reply

        def start(self) -> [bytes]:

            self._reset_buffer()

            command = b'START\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=2)

            return get_reply

        def stop(self, timeout=1) -> [bytes]:

            self._reset_buffer()

            command = b'STOP\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=timeout)

            return get_reply

        def fieldcal(self) -> [bytes]:

            self._reset_buffer()

            fieldcal_command = b'FIELDCAL\r\n'

            self.connection.write(fieldcal_command)

            get_reply = self._handle_reply(command=fieldcal_command, terminator=b'OK\r\n')

            return get_reply

        def start_spectrum(self, timeout=1) -> [bytes]:

            self._reset_buffer()

            command = b'STARTSPECTRUM\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=timeout)

            return get_reply

        def reset(self) -> [bytes]:

            self._reset_buffer()

            command = b'RESET\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

            return get_reply

        def get_id(self, sn=False, str=False) -> [bytes]:

            self._reset_buffer()

            command = b'ID'

            if sn is True:
                command += b',SN'

            if str is True:
                command += b',STR'

            command += b'\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

            return get_reply

        def get_fw(self, str=False, major=False, minor=False, patch=False, extra=False, build=False, hash=False, dvlfw=False, dvlminor=False, dvlboot=False, dvlfpga=False, _nmea=False) -> [bytes]:

            self._reset_buffer()

            command = b''

            if _nmea is True:
                command += b'$PNOR,'

            command += b'GETFW'

            if str is True:
                command += b',STR'

            if major is True:
                command += b',MAJOR'

            if minor is True:
                command += b',MINOR'

            if patch is True:
                command += b',PATCH'

            if extra is True:
                command += b',EXTRA'

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

            if _nmea is True:
                nmea_checksum = self._nmea_checksum(command)
                command += b'*'
                command += nmea_checksum

            command += b'\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', nmea=_nmea)

            return get_reply

        def get_hw(self, digital=False, analog=False) -> [bytes]:

            self._reset_buffer()

            command = b'GETHW'

            if digital is True:
                command += b',DIGITAL'

            if analog is True:
                command += b',ANALOG'

            command += b'\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

            return get_reply

        def get_all(self):

            self._reset_buffer()

            command = b'GETALL\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

            return get_reply

        def set_eth(self, ipmethod=None, ip=None, netmask=None, gateway=None):

            self._reset_buffer()

            command = b'SETETH'

            if ipmethod is not None:
                if ipmethod.upper() in self.DHCP_STATIC:
                    command += b',IPMETHOD="' + ipmethod.upper().encode() + b'"'
                else:
                    self.messages.write_warning('Invalid value for IPMETHOD in SETETH command')

            if ip is not None:
                if match(self.ADDRESS_PATTERN, ip):
                    command += b',IP="' + ip.encode() + b'"'
                else:
                    self.messages.write_warning('Invalid value for IP in SETETH command')

            if netmask is not None:
                if match(self.ADDRESS_PATTERN, netmask):
                    command += b',NETMASK="' + netmask.encode() + b'"'
                else:
                    self.messages.write_warning('Invalid value for NETMASK in SETETH command')

            if gateway is not None:
                if match(self.ADDRESS_PATTERN, gateway):
                    command += b',GATEWAY="' + gateway.encode() + b'"'
                else:
                    self.messages.write_warning('Invalid value for GATEWAY in SETETH command')

            command += b'\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

            return get_reply

        def get_eth(self, ipmethod=False, ip=False, netmask=False, gateway=False):

            self._reset_buffer()

            command = b'GETETH'

            if ipmethod is True:
                command += b',IPMETHOD'

            if ip is True:
                command += b',IP'

            if netmask is True:
                command += b',NETMASK'

            if gateway is True:
                command += b',GATEWAY'

            command += b'\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

            return get_reply

        def read_ip(self, ip=False, netmask=False, gateway=False, leasetime=False):

            self._reset_buffer()

            command = b'READIP'

            if ip is True:
                command += b',IP'

            if netmask is True:
                command += b',NETMASK'

            if gateway is True:
                command += b',GATEWAY'

            if leasetime is True:
                command += b',LEASETIME'

            command += b'\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

            return get_reply

        def get_eth_lim(self, ipmethod=False, ip=False, netmask=False, gateway=False):

            self._reset_buffer()

            command = b'GETETHLIM'

            if ipmethod is True:
                command += b',IPMETHOD'

            if ip is True:
                command += b',IP'

            if netmask is True:
                command += b',NETMASK'

            if gateway is True:
                command += b',GATEWAY'

            command += b'\r\n'

            self.connection.write(command)

            get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

            return get_reply

    class Logging:

        def __init__(self, messages, connection):

            self.messages = messages
            self.connection = connection
            self.commands = None

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

            self.cp_nc = None

        def _get_field_names_packet(self):
            """function to return the fieldnames for the UNS logging"""
            '''
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
            '''
            header_fields = ['sizeHeader', 'id', 'family', 'sizeData', 'size', 'dataCheckSum', 'headerCheckSum']

            common_fields = ['version', 'offsetOfData', 'timeStamp', 'microSeconds']

            driver_fields = ['timestampPython']

            is_valid_field = ['isValid']

            ahrs_fields = ['serialNumber', 'operationMode',
                           'ahrsData.roll', 'ahrsData.pitch', 'ahrsData.heading',
                           'ahrsData.quaternionW', 'ahrsData.quaternionX', 'ahrsData.quaternionY', 'ahrsData.quaternionZ',
                           'ahrsData.rotationMatrix_0', 'ahrsData.rotationMatrix_1', 'ahrsData.rotationMatrix_2',
                           'ahrsData.rotationMatrix_3', 'ahrsData.rotationMatrix_4', 'ahrsData.rotationMatrix_5',
                           'ahrsData.rotationMatrix_6', 'ahrsData.rotationMatrix_7', 'ahrsData.rotationMatrix_8',
                           'declination', 'depth', 'fomAhrs', 'fomFc1']

            ins_fields = ['deltaQuaternionW', 'deltaQuaternionX', 'deltaQuaternionY', 'deltaQuaternionZ',
                          'courseOverGround', 'temperature', 'pressure',
                          'altitude', 'latitude', 'longitude', 'height',
                          'positionFrameX', 'positionFrameY', 'positionFrameZ',
                          'deltaPositionFrameX', 'deltaPositionFrameY', 'deltaPositionFrameZ',
                          'deltaPositionNucleusX', 'deltaPositionNucleusY', 'deltaPositionNucleusZ',
                          'velocityNedX', 'velocityNedY', 'velocityNedZ',
                          'velocityNucleusX', 'velocityNucleusY', 'velocityNucleusZ',
                          'deltaVelocityNedX', 'deltaVelocityNedY', 'deltaVelocityNedZ',
                          'deltaVelocityNucleusX', 'deltaVelocityNucleusY', 'deltaVelocityNucleusZ',
                          'speedOverGround', 'turnRateX', 'turnRateY', 'turnRateZ']

            imu_fields = ['status.isValid',
                          'status.hasDataPathOverrun', 'status.hasFlashUpdateFailure', 'status.hasSpiComError',
                          'status.hasLowVoltage', 'status.hasSensorFailure', 'status.hasMemoryFailure',
                          'status.hasGyro1Failure', 'status.hasGyro2Failure', 'status.hasAccelerometerFailure',
                          'accelerometer.x', 'accelerometer.y', 'accelerometer.z',
                          'gyro.x', 'gyro.y', 'gyro.z',
                          'temperature']

            mag_fields = ['status.isCompensatedForHardIron', 'status.dvlActive', 'status.dvlAcousticsActive', 'status.dvlTransmitterActive',
                          'magnetometer.x', 'magnetometer.y', 'magnetometer.z']

            dvl_fields = ['status.beam1VelocityValid', 'status.beam2VelocityValid', 'status.beam3VelocityValid',
                          'status.beam1DistanceValid', 'status.beam2DistanceValid', 'status.beam3DistanceValid',
                          'status.beam1FomValid', 'status.beam2FomValid', 'status.beam3FomValid',
                          'status.xVelocityValid', 'status.yVelocityValid', 'status.zVelocityValid',
                          'status.xFomValid', 'status.yFomValid', 'status.zFomValid',
                          'serialNumber', 'soundSpeed', 'temperature', 'pressure',
                          'velocityBeam1', 'velocityBeam2', 'velocityBeam3',
                          'distanceBeam1', 'distanceBeam2', 'distanceBeam3',
                          'fomBeam1', 'fomBeam2', 'fomBeam3', 'dtBeam1', 'dtBeam2', 'dtBeam3',
                          'timeVelBeam1', 'timeVelBeam2', 'timeVelBeam3',
                          'velocityX', 'velocityY', 'velocityZ', 'fomX', 'fomY', 'fomZ',
                          'dtXYZ', 'timeVelXYZ']

            alti_fields = ['status.altimeterDistanceValid', 'status.altimeterQualityValid', 'status.pressureValid', 'status.temperatureValid',
                           'serialNumber', 'soundSpeed', 'temperature', 'pressure',
                           'altimeterDistance', 'altimeterQuality']

            cp_fields = ['serialNumber', 'soundVelocity', 'temperature', 'pressure',
                         'cellSize', 'blanking', 'numberOfCells', 'ambiguityVelocity']
                         #'velocityData', 'amplitudeData', 'correlationData']  # TODO: Make these fields dynamic?

            if self.cp_nc is not None:
                for index in range(self.cp_nc * 3):
                    cp_fields.append('velocityData_{}'.format(index))

                for index in range(self.cp_nc * 3):
                    cp_fields.append('amplitudeData_{}'.format(index))

                for index in range(self.cp_nc * 3):
                    cp_fields.append('correlationData_{}'.format(index))

            fc_fields = ['status.pointsUsedInEstimation',
                         'hardIron.x', 'hardIron.y', 'hardIron.z',
                         'sAxis_0', 'sAxis_1', 'sAxis_2',
                         'sAxis_3', 'sAxis_4', 'sAxis_5',
                         'sAxis_6', 'sAxis_7', 'sAxis_8',
                         'newPoint.x', 'newPoint.y', 'newPoint.z',
                         'fomFieldCalibration', 'coverage']

            string_fields = ['string']
            '''
            field_names = ['id', 'className', 'family', 'isValid', 'size', 'sizeData', 'sizeHeader',  'headerCheckSum', 'dataCheckSum',
                           'string', 'version', 'timeStamp',   'microSeconds',  'status.altimeterDistanceValid',
                           'accelerometer.x', 'accelerometer.y', 'accelerometer.z',
                           'ahrsData.roll', 'ahrsData.pitch', 'ahrsData.heading',
                           'ahrsData.quaternionW', 'ahrsData.quaternionX', 'ahrsData.quaternionY', 'ahrsData.quaternionZ',
                           'ahrsData.rotationMatrix_0', 'ahrsData.rotationMatrix_1', 'ahrsData.rotationMatrix_2',
                           'ahrsData.rotationMatrix_3', 'ahrsData.rotationMatrix_4', 'ahrsData.rotationMatrix_5',
                           'ahrsData.rotationMatrix_6', 'ahrsData.rotationMatrix_7', 'ahrsData.rotationMatrix_8',
                           'altimeterDistance', 'altimeterQuality', 'declination', 'depth',
                            'distanceBeam1',
                            'distanceBeam2',
                            'distanceBeam3',
                            'dtBeam1',
                            'dtBeam2',
                            'dtBeam3',
                            'dtXYZ',
                            'fomAhrs',
                            'fomBeam1',
                            'fomBeam2',
                            'fomBeam3',
                            'fomFc1',
                            'fomX',
                            'fomY',
                            'fomZ',
                            'gyro.x',
                            'gyro.y',
                            'gyro.z',
                            'magnetometer.x',
                            'magnetometer.y',
                            'magnetometer.z',
                            'serial',
                            'operationMode',
                            'operationModeString',
                            'pressure',
                            'serialNumber',
                            'soundSpeed',
                            'status.altimeterQualityValid',
                            'status.beam1DistanceValid',
                            'status.beam1FomValid',
                            'status.beam1VelocityValid',
                            'status.beam2DistanceValid',
                            'status.beam2FomValid',
                            'status.beam2VelocityValid',
                            'status.beam3DistanceValid',
                            'status.beam3FomValid',
                            'status.beam3VelocityValid',
                            'status.dvlAcousticsActive',
                            'status.dvlActive',
                            'status.dvlTransmitterActive',
                            'status.hasAccelerometerFailure',
                            'status.hasDataPathOverrun',
                            'status.hasDiagnosticsData',
                            'status.hasFlashUpdateFailure',
                            'status.hasGyro1Failure',
                            'status.hasGyro2Failure',
                            'status.hasLowVoltage',
                            'status.hasMemoryFailure',
                            'status.hasSensorFailure',
                            'status.hasSpiComError',
                            'status.isCompensatedForHardIron',
                            'status.isValid',
                            'status.pressureValid',
                            'status.temperatureValid',
                            'status.xFomValid',
                            'status.xVelocityValid',
                            'status.yFomValid',
                            'status.yVelocityValid',
                            'status.zFomValid',
                            'status.zVelocityValid',
                            'temperature',
                            'timeVelBeam1',
                            'timeVelBeam2',
                            'timeVelBeam3',
                            'timeVelXYZ',
                            'velocityBeam1',
                            'velocityBeam2',
                            'velocityBeam3',
                            'velocityX',
                            'velocityY',
                            'velocityZ']
            '''

            data_fields = [header_fields,
                           common_fields,
                           driver_fields,
                           is_valid_field,  # TODO: This is a temporary fix where this value needs to be in the log file, but nothing is logged to this column
                           ahrs_fields,
                           ins_fields,
                           imu_fields,
                           mag_fields,
                           dvl_fields,
                           alti_fields,
                           cp_fields,
                           fc_fields,
                           string_fields]

            field_names = list()
            for fields in data_fields:
                for field in fields:
                    if field not in field_names:
                        field_names.append(field)

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

        def get_cp_nc(self):

            reply = self.commands.get_cur_prof(profile_range=True, cs=True, bd=True, ds=True)

            if len(reply) != 2:
                self.messages.write_warning('Unable to obtain current profile configuration')

            else:
                try:
                    cp_data = reply[0].split(b',')
                    cp_range = float(cp_data[0].decode())
                    cp_cs = float(cp_data[1].decode())
                    cp_bd = float(cp_data[2].decode())
                    cp_ds = cp_data[3].rstrip(b'\r\n').decode()

                    if cp_ds == '"OFF"':
                        self.cp_nc = 0

                    else:
                        nc = ceil((cp_range - cp_bd) / cp_cs)
                        if nc > 0:
                            self.cp_nc = nc
                        else:
                            self.cp_nc = None

                except Exception as e:
                    self.messages.write_warning('Error occured when trying to extract current profile data: {}'.format(e))

        def set_path(self, path: str):

            self._path = path.rstrip('/')

        def start(self) -> str:

            folder = self._path + '/' + datetime.now().strftime('%y%m%d_%H%M%S')

            Path(folder).mkdir(parents=True, exist_ok=True)

            self.messages.write_message('Logging started. Path: {}'.format(folder))

            self.packet_file = open(folder + '/nucleus_log.csv', 'w', newline='')
            self.condition_file = open(folder + '/condition_log.csv', 'w', newline='')
            self.ascii_file = open(folder + '/ascii_log.csv', 'w', newline='')

            self.connection.get_info()

            if self.connection.get_all is not None:
                with open(folder + '/get_all.txt', 'w') as file:
                    file.writelines(self.connection.get_all)

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

        def __init__(self, **kwargs):

            self.connection = kwargs.get('connection')
            self.messages = kwargs.get('messages')
            self.parser = kwargs.get('parser')
            self.logging = kwargs.get('logging')
            self.commands = kwargs.get('commands')

            self.main_running = True
            self.logging_fieldcal = False

        def _print_dashes(self):

            self.messages.write_message('\n------------------------------------------------------------------------------------------------------------------------------------------------------')

        def _print_space(self, line, space_end=80):

            space = ''

            for _ in range(space_end - len(line)):
                space += ' '

            return space

        def print_commands(self, command_dict):

            self.messages.write_message('\nAvailable commands are:\n')

            max_key_length = 0
            max_short_key_length = 0
            max_description_lenght = 0
            for key in command_dict:
                max_key_length = max(max_key_length, len(key))
                max_short_key_length = max(max_short_key_length, len(command_dict[key][0]))
                max_description_lenght = max(max_description_lenght, len(command_dict[key][1]))

            max_key_header = max(max_key_length + max_short_key_length + 2, len('Command name'))

            command_name_dashes = ''
            command_description_dashes = ''

            for _ in range(max_key_header):

                command_name_dashes += '-'

            for _ in range(max_description_lenght):

                command_description_dashes += '-'

            self.messages.write_message('{}{}    {}'.format('Command name', self._print_space('Command name', space_end=max_key_header), 'Command description'))
            self.messages.write_message('{}    {}'.format(command_name_dashes, command_description_dashes))

            for key in command_dict:
                command_name = '{},{} {}'.format(key, self._print_space(key, space_end=max_key_length), command_dict[key][0])

                self.messages.write_message('{}{}    {}'.format(command_name, self._print_space(command_name, space_end=max_key_header), command_dict[key][1]))

        def main(self):

            COMMAND_DICT = {
                'status': ['i', 'Returns the status of the connection, information about nucleus and logging status'],
                'terminal': ['t', 'Send commands directly to device'],
                'logging': ['l', 'Logs data from the Nucleus to files'],
                'help': ['?', 'Lists these commands'],
                'quit': ['q', 'Ends this script']}

            def print_title():

                driver_title = [r"  _   _            _                  _____       _                ",
                                r" | \ | |          | |                |  __ \     (_)               ",
                                r" |  \| |_   _  ___| | ___ _   _ ___  | |  | |_ __ ___   _____ _ __ ",
                                r" | . ` | | | |/ __| |/ _ \ | | / __| | |  | | '__| \ \ / / _ \ '__|",
                                r" | |\  | |_| | (__| |  __/ |_| \__ \ | |__| | |  | |\ V /  __/ |   ",
                                r" |_| \_|\__,_|\___|_|\___|\__,_|___/ |_____/|_|  |_| \_/ \___|_|   "]

                for line in driver_title:
                    self.messages.write_message(line)

                self.messages.write_message('')

            def print_status():

                self.messages.write_message('\n######################################################################################################################################################')

                connection_status = self.connection.get_connection_status()

                if connection_status is True:
                    connection_type = self.connection.get_connection_type()
                    self.messages.write_message('\nNucleus connected: {}'.format(connection_type))
                    if connection_type == 'serial':
                        self.messages.write_message(
                            'port: {}'.format(self.connection.serial_configuration.port))
                        self.messages.write_message(
                            'baudrate: {}'.format(self.connection.serial_configuration.baudrate))
                    if connection_type == 'tcp':
                        self.messages.write_message('hostname: {}'.format(self.connection.tcp_configuration.host))
                        self.messages.write_message('port: {}'.format(self.connection.tcp_configuration.port))
                    if connection_type == 'udp':
                        self.messages.write_message(
                            'port: {}'.format(self.connection.udp_configuration.port))

                    self.messages.write_message('\nNucleus ID: {}'.format(self.connection.nucleus_id))
                    self.messages.write_message(
                        'Firmware version: {}'.format(self.connection.firmware_version))

                else:
                    self.messages.write_message('\nNucleus connected: {}'.format(connection_status))

                logging = self.logging._logging
                self.messages.write_message('\nLogging: {}'.format(logging))
                if logging:
                    self.messages.write_message('Path: {}'.format(self.logging._path))

                self.messages.write_message('\n######################################################################################################################################################')

            print_title()

            self.run_connection()

            print_status()

            while self.main_running:

                self.print_commands(COMMAND_DICT)
                reply = input('\nmain: ')

                if reply in ['status', 'i']:

                    print_status()

                elif reply in ['terminal', 't']:

                    self.run_commands()

                elif reply in ['logging', 'l']:

                    self.run_logging()

                elif reply in ['help', '?']:

                    self.print_commands(COMMAND_DICT)

                elif reply in ['quit', 'q']:

                    self.main_running = False

                else:

                    self.messages.write_message('Unknown command')

        def run_connection(self):

            def serial_configuration() -> bool:

                if self.connection.serial_configuration.port is None:

                    port = self.connection.select_serial_port()

                    if port is None:
                        return False

                    self.connection.set_serial_configuration(port=port)

                if self.connection.serial_configuration.baudrate is None:

                    baudrate = input('\nserial - baudrate (input nothing for 115200): ')

                    if baudrate == '':
                        baudrate = 115200
                    else:
                        try:
                            baudrate = int(baudrate)
                        except ValueError:
                            self.messages.write_warning('baudrate is not integer, port value will not be set')
                            baudrate = None

                    if baudrate is None:
                        return False

                    self.connection.set_serial_configuration(baudrate=baudrate)

                return True

            def tcp_configuration() -> bool:

                if self.connection.tcp_configuration.host is None:

                    self.messages.write_message('\nConnect through TCP with host(hostname/ip) or serial number: ')
                    self.messages.write_message('[0] Host')
                    self.messages.write_message('[1] Serial_number')
                    reply = input('Input integer value in range [0:1]: ')

                    if reply == '0':
                        host = input('\ntcp - host: ')

                        if host == '':
                            return False

                        self.connection.set_tcp_configuration(host=host)

                    elif reply == '1':
                        serial_number = input('\ntcp - serial number: ')

                        try:
                            int(serial_number)
                            condition = self.connection.set_tcp_hostname_from_serial_number(serial_number=serial_number)
                        except ValueError:
                            self.messages.write_warning('serial number is not integer, host name will not be set')
                            condition = False

                        if not condition:
                            return condition

                    else:
                        self.messages.write_message('Invalid selection')
                        return False

                if self.connection.tcp_configuration.port is None:

                    port = input('\ntcp - port (input nothing for 9000): ')

                    if port == '':
                        port = 9000
                    else:
                        try:
                            port = int(port)
                        except ValueError:
                            self.messages.write_warning('port is not integer, port value will not be set')
                            port = None

                    if port is None:
                        return False

                    self.connection.set_tcp_configuration(port=port)

                return True

            def udp_configuration() -> bool:

                if self.connection.udp_configuration.port is None:

                    port = input('\nudp - port: ')

                    if port != '':
                        try:
                            port = int(port)
                        except ValueError:
                            self.messages.write_warning('port is not integer, port value will not be set')
                            port = ''

                    if port == '':
                        return False

                    self.connection.set_udp_configuration(port=port)

                return True

            if self.parser.thread.is_alive():
                self.messages.write_message('Can not handle connection while logging thread is running')
                return

            running = True

            while running:

                self.messages.write_message('\nSelect connection type: ')
                self.messages.write_message('[0] Serial')
                self.messages.write_message('[1] TCP')
                self.messages.write_message('[2] UDP')
                self.messages.write_message('[3] Skip connection')

                reply = input('Input integer value in range [0:3]: ')

                if reply == '0':
                    if serial_configuration():
                        self.connection.connect(connection_type='serial')

                elif reply == '1':
                    if tcp_configuration():
                        self.connection.connect(connection_type='tcp')

                elif reply == '2':
                    if udp_configuration():
                        self.connection.connect(connection_type='udp')

                elif reply == '3':
                    break

                else:
                    self.messages.write_message('Invalid selection')

                if self.connection.get_connection_status():
                    running = False
                else:
                    self.messages.write_message('Failed to connect to Nucleus')

        def run_commands(self):

            COMMAND_DICT = {
                'help': ['?', 'Lists these commands'],
                'return': ['[blank]', 'Leaves terminal and returns to main'],
                'quit': ['q', 'Ends this script']}

            if self.connection.get_connection_status() is not True:
                self.messages.write_message('Nucleus is not connected!')
                return

            self._print_dashes()
            self.print_commands(COMMAND_DICT)

            running = True

            while running:

                command = input('\nterminal: ')

                if command == '':
                    running = False

                elif command.upper() == 'START':
                    self.messages.write_message('start command is only supported through logging')

                elif command.upper() == 'FIELDCAL':

                    self.messages.write_message('fieldcal command is only supported through logging')

                elif command.upper() == 'STARTSPECTRUM':

                    self.messages.write_message('startspectrum command is only supported through logging')

                elif command.upper() == 'STOP':

                    self.messages.write_message('stop command is only supported through logging')

                elif command in ['help', '?']:

                    self.print_commands(COMMAND_DICT)

                elif command in ['return', '']:

                    running = False

                elif command in ['quit', 'q']:

                    running = False
                    self.main_running = False

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
                                self.messages.write_message('Could not decode reply: {}'.format(command_reply))
                                self.messages.write_message('Received error: {}'.format(e))

        def run_logging(self):

            COMMAND_DICT = {
                'start': ['r', 'Starts logging'],
                'fieldcal': ['f', 'Starts fieldcal'],
                'spectrum_analyzer': ['sa', 'Starts spectrum analyzer'],
                'stop': ['s', 'Stops logging'],
                'set_log_path': ['p', 'Specify path for where log files should be saved. If not specified a default path will be used'],
                'status': ['i', 'Returns status of logging'],
                'help': ['?', 'Lists these commands'],
                'return': ['[blank]', 'Leaves logging and returns to main'],
                'quit': ['q', 'Ends this script']}

            if self.connection.get_connection_status() is not True:
                self.messages.write_message('Nucleus is not connected!')
                return

            self._print_dashes()
            self.print_commands(COMMAND_DICT)

            running = True

            while running:

                reply = input('\nlogging: ')

                if reply in ['start', 'r']:

                    self.logging.get_cp_nc()
                    self.parser.start()
                    self.logging.start()
                    self.commands.start()

                elif reply in ['spectrum_analyzer', 'sa']:

                    self.parser.start()
                    self.logging.start()
                    self.commands.start_spectrum()

                elif reply in ['fieldcal', 'f']:

                    self.parser.start()
                    self.logging.start()
                    self.commands.fieldcal()

                    self.logging_fieldcal = True

                elif reply in ['stop', 's']:

                    self.logging.stop()
                    self.parser.stop()
                    if self.logging_fieldcal:
                        time.sleep(0.5)
                    self.commands.stop()

                    self.logging_fieldcal = False

                elif reply in ['set_log_path', 'p']:

                    path = input('logging - set_log_path: ')
                    self.logging.set_path(path=path)

                elif reply in ['status', 'i']:

                    self.messages.write_message('logging running: {}'.format(self.logging._logging))

                elif reply in ['help', '?']:

                    self.print_commands(COMMAND_DICT)

                elif reply in ['return', '']:

                    running = False

                elif reply in ['quit', 'q']:

                    running = False
                    self.main_running = False

                else:

                    self.messages.write_message('Unknown command')


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

    nucleus_driver = NucleusDriver()

    if args.connection_serial_port is not None:
        nucleus_driver.connection.set_serial_configuration(port=args.connection_serial_port)

    if args.connection_serial_baudrate is not None:
        nucleus_driver.connection.set_serial_configuration(baudrate=int(args.connection_serial_baudrate))

    if args.connection_tcp_host is not None:
        nucleus_driver.connection.set_tcp_configuration(host=args.connection_tcp_host)

    if args.connection_tcp_port is not None:
        nucleus_driver.connection.set_tcp_configuration(port=int(args.connection_tcp_port))

    if args.connection_udp_port is not None:
        nucleus_driver.connection.set_udp_configuration(port=int(args.connection_udp_port))

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

        elif args.connection_type == 'udp':

            if nucleus_driver.connection.udp_configuration.port is None:
                port = input('UDP port: ')

                nucleus_driver.connection.set_udp_configuration(port=int(port))

        nucleus_driver.connection.connect(connection_type=args.connection_type)

    if args.logging_set_path is not None:
        nucleus_driver.logging.set_path(path=args.logging_set_path)

    nucleus_driver.parser.set_queuing(ascii=True)

    nucleus_driver.run_classes.main()
