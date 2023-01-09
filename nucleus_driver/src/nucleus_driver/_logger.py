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

        ins_fields = ['courseOverGround', 'temperature', 'pressure',
                      'altitude',
                      'positionFrameX', 'positionFrameY', 'positionFrameZ',
                      'velocityNedX', 'velocityNedY', 'velocityNedZ',
                      'velocityNucleusX', 'velocityNucleusY', 'velocityNucleusZ',
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

    def start(self, _converting=False) -> str:

        folder = self._path + '/' + datetime.now().strftime('%y%m%d_%H%M%S')

        Path(folder).mkdir(parents=True, exist_ok=True)

        if not _converting:
            self.messages.write_message('Logging started. Path: {}'.format(folder))
        else:
            self.messages.write_message('Converting started. Path: {}'.format(folder))

        self.packet_file = open(folder + '/nucleus_log.csv', 'w', newline='')
        self.condition_file = open(folder + '/condition_log.csv', 'w', newline='')
        self.ascii_file = open(folder + '/ascii_log.csv', 'w', newline='')

        if self.connection.get_connection_status():
            self.connection.get_info()

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
