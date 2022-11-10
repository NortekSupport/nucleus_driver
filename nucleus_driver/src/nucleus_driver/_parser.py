from queue import Queue, Empty
from struct import unpack
from itertools import zip_longest
from threading import Thread
from datetime import datetime
import time


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

    def __init__(self, **kwargs):

        self.messages = kwargs.get('messages')
        self.logger = kwargs.get('logger')
        self.connection = kwargs.get('connection')

        self.packet_queue = Queue(maxsize=10000)
        self.ascii_queue = Queue(maxsize=10000)
        self.condition_queue = Queue(maxsize=10000)

        self.reading_packet = False
        self.binary_packet = bytearray()
        self.ascii_packet = list()

        self._queuing = {'packet': True,
                         'ascii': True,
                         'condition': True}

        self.thread = Thread()
        self.thread_running = False

    def init_packet_queue(self, maxsize=0):

        if not self.thread_running:
            self.packet_queue = Queue(maxsize=maxsize)
        else:
            self.messages.write_warning('can not initiate queue when parser is running')

    def init_condition_queue(self, maxsize=0):

        if not self.thread_running:
            self.condition_queue = Queue(maxsize=maxsize)
        else:
            self.messages.write_warning('can not initiate queue when parser is running')

    def init_ascii_queue(self, maxsize=0):

        if not self.thread_running:
            self.ascii_queue = Queue(maxsize=maxsize)
        else:
            self.messages.write_warning('can not initiate queue when parser is running')

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

        if self.logger._logging is True:

            self.logger._writing_packet = True

            try:
                self.logger.packet_writer.writerow(packet)
            except ValueError as exception:
                self.messages.write_warning('Failed to write package to csv file: {}'.format(exception))

            self.logger._writing_packet = False

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

        if self.logger._logging is True:
            ascii_message = ''.join(chr(i) for i in packet if 0 <= i <= 0x7e).rstrip('\r\n')
            ascii_packet = {'timestamp_python': datetime.now().timestamp(),
                            'message': ascii_message}

            self.logger._writing_ascii = True

            try:
                self.logger.ascii_writer.writerow(ascii_packet)
            except ValueError as exception:
                self.messages.write_warning('Failed to write ascii message to csv file: {}'.format(exception))

            self.logger._writing_ascii = False

    def read_ascii(self, timeout=None):

        packet = None

        try:
            if timeout is not None:
                packet = self.ascii_queue.get(timeout=timeout)

            else:
                packet = self.ascii_queue.get(timeout=self.connection.timeout)

        except Empty:
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

        if self.logger._logging is True:

            self.logger._writing_condition = True

            try:
                self.logger.condition_writer.writerow(failed_packet)
            except ValueError as exception:
                self.messages.write_warning('Failed to write condition to csv file: {}'.format(exception))

            self.logger._writing_condition = False

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
                    # sensor['velocityData'] = unpack(velocity_data_format, data[velocity_data_offset: velocity_data_offset + velocity_data_size])[:velocity_data_size]
                    velocity_data = unpack(velocity_data_format, data[velocity_data_offset: velocity_data_offset + velocity_data_size])[:velocity_data_size]
                    for index, velocity_cell in enumerate(velocity_data):
                        sensor['velocityData_{}'.format(index)] = velocity_cell

                    amplitude_data_format = '<' + ''.ljust(3 * sensor['numberOfCells'], 'B')
                    amplitude_data_offset = common_data['offsetOfData'] + 6 * sensor['numberOfCells']
                    amplitude_data_size = 1 * 3 * sensor['numberOfCells']
                    # sensor['amplitudeData'] = unpack(amplitude_data_format, data[amplitude_data_offset: amplitude_data_offset + amplitude_data_size])[:amplitude_data_size]
                    amplitude_data = unpack(amplitude_data_format, data[amplitude_data_offset: amplitude_data_offset + amplitude_data_size])[:amplitude_data_size]
                    for index, amplitude_cell in enumerate(amplitude_data):
                        sensor['amplitudeData_{}'.format(index)] = amplitude_cell

                    correlation_data_format = '<' + ''.ljust(3 * sensor['numberOfCells'], 'B')
                    correlation_data_offset = common_data['offsetOfData'] + 9 * sensor['numberOfCells']
                    correlation_data_size = 1 * 3 * sensor['numberOfCells']
                    # sensor['correlationData'] = unpack(correlation_data_format, data[correlation_data_offset: correlation_data_offset + correlation_data_size])[:correlation_data_size]
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
