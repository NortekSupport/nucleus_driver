from math import ceil
from datetime import datetime
import csv
from pathlib import Path
import io
import time


class Logger:

    def __init__(self, **kwargs):

        self.messages = kwargs.get('messages')
        self.connection = kwargs.get('connection')
        self.commands = None

        self._logging = False
        self._logging_folder = None
        self._current_profile_logging = False

        self._writing_packet = False
        self._writing_ascii = False
        self._writing_condition = False

        self._path = str(Path.cwd()) + '/logs'
        self.packet_file = None
        self.current_profile_file = None
        self.ascii_file = None
        self.condition_file = None

        self.packet_writer = None
        self.current_profile_writer = None
        self.ascii_writer = None
        self.condition_writer = None

        self.cp_nc = None

    def _get_field_names_packet(self):
        """function to return the fieldnames for the UNS logging"""

        header_fields = ['sizeHeader', 'id', 'family', 'sizeData', 'size', 'dataCheckSum', 'headerCheckSum']

        common_fields = ['version', 'offsetOfData', 'flags.posixTime', 'timeStamp', 'microSeconds']

        driver_fields = ['timestampPython']

        is_valid_field = ['isValid']

        ahrs_fields = ['serialNumber', 'operationMode',
                       'ahrsData.roll', 'ahrsData.pitch', 'ahrsData.heading',
                       'ahrsData.quaternionW', 'ahrsData.quaternionX', 'ahrsData.quaternionY', 'ahrsData.quaternionZ',
                       'ahrsData.rotationMatrix_0', 'ahrsData.rotationMatrix_1', 'ahrsData.rotationMatrix_2',
                       'ahrsData.rotationMatrix_3', 'ahrsData.rotationMatrix_4', 'ahrsData.rotationMatrix_5',
                       'ahrsData.rotationMatrix_6', 'ahrsData.rotationMatrix_7', 'ahrsData.rotationMatrix_8',
                       'declination', 'depth', 'fomAhrs', 'fomFc1']

        ins_fields = ['fomIns', 'statusIns.latLonIsValid',
                      'courseOverGround', 'temperature', 'pressure',
                      'altitude', 'latitude', 'longitude',
                      'positionFrameX', 'positionFrameY', 'positionFrameZ',
                      'velocityNedX', 'velocityNedY', 'velocityNedZ',
                      'velocityNucleusX', 'velocityNucleusY', 'velocityNucleusZ',
                      'speedOverGround', 'turnRateX', 'turnRateY', 'turnRateZ']

        imu_fields = ['status.isValid',
                      'status.hasChecksumError', 'status.hasDataPathOverrun', 'status.hasFlashUpdateFailure', 'status.hasSpiComError',
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
        '''
        cp_fields = ['serialNumber', 'soundVelocity', 'temperature', 'pressure',
                     'cellSize', 'blanking', 'numberOfCells', 'ambiguityVelocity']

        if self.cp_nc is not None:
            for index in range(self.cp_nc * 3):
                cp_fields.append('velocityData_{}'.format(index))

            for index in range(self.cp_nc * 3):
                cp_fields.append('amplitudeData_{}'.format(index))

            for index in range(self.cp_nc * 3):
                cp_fields.append('correlationData_{}'.format(index))
        '''
        fc_fields = ['status.pointsUsedInEstimation',
                     'hardIron.x', 'hardIron.y', 'hardIron.z',
                     'sAxis_0', 'sAxis_1', 'sAxis_2',
                     'sAxis_3', 'sAxis_4', 'sAxis_5',
                     'sAxis_6', 'sAxis_7', 'sAxis_8',
                     'newPoint.x', 'newPoint.y', 'newPoint.z',
                     'fomFieldCalibration', 'coverage']

        string_fields = ['string']

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
                       #cp_fields,
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

    def get_logging_status(self):

        return self._logging

    def get_current_profile_logging_status(self):

        return self._current_profile_logging
    
    def set_path(self, path: str):

        self._path = path.rstrip('/')

    def open_current_profile_writer(self, number_of_cells: int) -> bool:
        
        def _get_field_names():

            header_fields = ['sizeHeader', 'id', 'family', 'sizeData', 'size', 'dataCheckSum', 'headerCheckSum']

            common_fields = ['version', 'offsetOfData', 'flags.posixTime', 'timeStamp', 'microSeconds']

            driver_fields = ['timestampPython']

            cp_fields = ['serialNumber', 'curProfConfig.bit0', 'curProfConfig.bit1', 'soundVelocity', 'temperature',
                         'pressure', 'cellSize', 'blanking', 'numberOfCells', 'ambiguityVelocity']

            for index in range(number_of_cells * 3):
                cp_fields.append('velocityData_{}'.format(index))

            for index in range(number_of_cells * 3):
                cp_fields.append('amplitudeData_{}'.format(index))

            for index in range(number_of_cells * 3):
                cp_fields.append('correlationData_{}'.format(index))

            data_fields = [header_fields,
                           common_fields,
                           driver_fields,
                           cp_fields]

            field_names = list()
            for fields in data_fields:
                for field in fields:
                    if field not in field_names:
                        field_names.append(field)

            return field_names

        if self._logging_folder is None:
            return False
        
        self.current_profile_file = open(self._logging_folder + '/current_profile_log.csv', 'w', newline='')

        self.current_profile_writer = csv.DictWriter(self.current_profile_file, fieldnames=_get_field_names())
        
        self.current_profile_writer.writeheader()

        self._current_profile_logging = True

        return True

    def start(self, _converting=False) -> str:

        def get_all_package(get_all: str) -> str:

            get_all_package = b''.join(entry.split(b'$PNOR,')[1].split(b'*')[0] + b'\r\n' for entry in get_all).split(b'OK')[0]   

            sync_byte = b'\xa5'
            header_size = b'\x0A'
            packet_id = b'\xA0'
            family_id = b'\x20'
            data_size = len(get_all_package).to_bytes(2, byteorder='little')

            checksum = self.parser.checksum(get_all_package).to_bytes(2, byteorder='little')

            header = sync_byte + header_size + packet_id + family_id + data_size +  checksum

            header_checksum = self.parser.checksum(header).to_bytes(2, byteorder='little')

            header = header + header_checksum

            return header + get_all_package
    
        folder = self._path + '/' + datetime.now().strftime('%y%m%d_%H%M%S')
        self._logging_folder = folder

        Path(folder).mkdir(parents=True, exist_ok=True)

        if not _converting:
            self.messages.write_message('Logging started. Path: {}'.format(folder))
        else:
            self.messages.write_message('Converting started. Path: {}'.format(folder))

        self.packet_file = open(folder + '/nucleus_log.csv', 'w', newline='')
        self.condition_file = open(folder + '/condition_log.csv', 'w', newline='')
        self.ascii_file = open(folder + '/ascii_log.csv', 'w', newline='')

        if self.connection.get_all is not None and _converting is False:
            with open(folder + '/get_all.txt', 'w') as file:
                file.writelines(self.connection.get_all)

        self.packet_writer = csv.DictWriter(self.packet_file, fieldnames=self._get_field_names_packet())
        self.condition_writer = csv.DictWriter(self.condition_file, fieldnames=self._get_field_names_condition())
        self.ascii_writer = csv.DictWriter(self.ascii_file, fieldnames=self._get_field_names_ascii())

        self.packet_writer.writeheader()
        self.condition_writer.writeheader()
        self.ascii_writer.writeheader()

        self._logging = True

        if self.connection.get_all_nmea is not None:
            get_all = get_all_package(self.connection.get_all_nmea)
            self.parser.add_data(get_all)

        return folder

    def stop(self):

        self._logging = False
        self._logging_folder = None
        self._current_profile_logging = False

        # give the writers up to 0.1 sec to complete writing
        for _ in range(10):

            if not self._writing_packet and not self._writing_ascii and not self._writing_condition:
                break

            time.sleep(0.01)

        if isinstance(self.packet_file, io.TextIOWrapper):
            self.packet_file.close()

        if isinstance(self.current_profile_file, io.TextIOWrapper):
            self.current_profile_file.close()

        if isinstance(self.condition_file, io.TextIOWrapper):
            self.condition_file.close()

        if isinstance(self.ascii_file, io.TextIOWrapper):
            self.ascii_file.close()
