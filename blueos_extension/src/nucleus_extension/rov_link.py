import time
import logging
from threading import Thread
from queue import Queue
import json
import os
from datetime import datetime
import requests
from requests.adapters import HTTPAdapter, Retry


#HOST_URL = "http://127.0.0.1"
HOST_URL = "http://host.docker.internal"


class RovLink:

    TIMEOUT = 1

    EXPECTED_CONFIG_PARAMETERS = {
        'AHRS_EKF_TYPE': 3,
        'EK2_ENABLE': 0,
        'EK3_ENABLE': 1,
        'VISO_TYPE': 1,
        'GPS_TYPE': 0,
        'EK3_SRC1_POSXY': 6,
        'EK3_SRC1_VELXY': 6,
        'EK3_SRC1_POSZ': 1,
        'SERIAL0_PROTOCOL': 2
    }

    PID_PARAMETERS = [
        'PSC_POSXY_P',
        'PSC_POSZ_P',
        'PSC_VELXY_P',
        'PSC_VELXY_I',
        'PSC_VELXY_D',
        'PSC_VELZ_P'
    ]

    def __init__(self, driver):

        self.nucleus_driver = driver

        self.settings_path = os.path.join(os.path.expanduser("~"), ".config", "nucleus", "settings.json")

        self.main_thread = Thread()
        self.thread_running = None

        self.check_cable_guy_thread = Thread()
        self.connect_nucleus_thread = Thread()
        self.check_heartbeat_thread = Thread()
        self.setup_nucleus_thread = Thread()
        self.read_config_parameters_startup_thread = Thread()
        self.read_pid_parameters_thread = Thread()
        self.stop_nucleus_thread = Thread()
        self.start_nucleus_thread = Thread()

        self.timestamp_previous = None
        self.orientation_current = None
        self.orientation_previous = None

        self.hostname = None
        self.nucleus_id = None
        self.nucleus_firmware = None

        self._enable_nucleus_input = True

        self.init_time = datetime.now()

        self.status = {
            'cable_guy': '---',
            'nucleus_connected': '---',
            'nucleus_configured': '---',
            'heartbeat': '---',
            'controller_parameters': '---'
        }

        self._connect_button_text = 'Connect'

        self._cable_guy = False
        self._nucleus_connected = False
        self._nucleus_configured = False
        self._heartbeat = False
        self._config = False

        self._nucleus_running = None

        self._log_path = None

        self.config_parameters = {
            'AHRS_EKF_TYPE': '---',
            'EK2_ENABLE': '---',
            'EK3_ENABLE': '---',
            'VISO_TYPE': '---',
            'GPS_TYPE': '---',
            'EK3_SRC1_POSXY': '---',
            'EK3_SRC1_VELXY': '---',
            'EK3_SRC1_POSZ': '---',
            'SERIAL0_PROTOCOL': '---'
        }

        self.pid_parameters = {
            'PSC_POSXY_P': '---',
            'PSC_POSZ_P': '---',
            'PSC_VELXY_P': '---',
            'PSC_VELXY_I': '---',
            'PSC_VELXY_D': '---',
            'PSC_VELZ_P': '---'
        }
        
        self.vision_position_delta_packet_counter = {
            'packets_sent': 0,
            'packets_failed': 0,
            'packets_skipped': 0,
        }

    def d2r(self, deg):

        return deg * 3.14159 / 360

    def r2d(self, rad):

        return rad * 360 / 3.14159

    def set_enable_nucleus_input(self, enable: bool):

        self._enable_nucleus_input = enable

    def get_enable_nucleus_input(self):

        return self._enable_nucleus_input
    
    def load_settings(self) -> None:
        """
        Load settings from .config/nucleus/settings.json
        """

        try:
            with open(self.settings_path) as settings:
                data = json.load(settings)
                self.hostname = data["hostname"]
                logging.debug(f"{self.timestamp()} Loaded settings: {data}")
        except FileNotFoundError:
            logging.warning(f"{self.timestamp()} Settings file not found, using default.")
        except ValueError:
            logging.warning(f"{self.timestamp()} File corrupted, using default settings.")
        except KeyError as error:
            logging.warning(f"{self.timestamp()} key not found: {error}")
            logging.warning(f"{self.timestamp()} using default instead")

    def save_settings(self) -> None:
        """
        Load settings from .config/dvl/settings.json
        """

        def ensure_dir(file_path) -> None:
            """
            Helper to guarantee that the file path exists
            """
            directory = os.path.dirname(file_path)
            if not os.path.exists(directory):
                os.makedirs(directory)

        try:
            ensure_dir(self.settings_path)
            with open(self.settings_path, "w") as settings:
                settings.write(
                    json.dumps(
                        {
                            "hostname": self.hostname
                        }
                    )
                )

            logging.info(f'{self.timestamp()} Wrote settings to file: {self.settings_path}')
        except Exception as e:
            logging.warning(f'{self.timestamp()} Failed to write settings to file: {e}')
    
    def set_hostname(self, hostname):

        self.hostname = hostname

        #self.save_settings()

    def timestamp(self) -> str:

        time_delta = datetime.now() - self.init_time

        days = time_delta.days
        hours, reminder = divmod(time_delta.seconds, 3600)
        minutes, seconds = divmod(reminder, 60)

        if days >= 1:
            timestamp = f'[{days} - {hours:02}:{minutes:02}:{seconds:02}]'
        else:
            timestamp = f'[{hours:02}:{minutes:02}:{seconds:02}]'

        return timestamp
    
    def _get_param_value(self):

        url = f'{HOST_URL}/mavlink2rest/mavlink/vehicles/1/components/1/messages/PARAM_VALUE'

        return self._get(url=url)

    def _get(self, url, retries=3, backoff_factor=1, status_forcelist=[502]):

        session = requests.Session()
        retry = Retry(total=retries, backoff_factor=backoff_factor, status_forcelist=status_forcelist, raise_on_status=False)
        adapter = HTTPAdapter(max_retries=retry)
        session.mount('http://', adapter)
        
        param_value = session.get(url=url)

        return param_value

    def _post_param_request_read(self, parameter_id):

        data = {
            'header': {
                'system_id': 255,
                'component_id': 0,
                'sequence': 0},
            'message': {
                'type': "PARAM_REQUEST_READ",
                'param_index': -1,
                'target_system': 1,
                'target_component': 1,
                'param_id': ["\u0000" for _ in range(16)]
            }
        }

        for index, char in enumerate(parameter_id):
            data['message']['param_id'][index] = char

        return self._post(url=f'{HOST_URL}/mavlink2rest/mavlink', data=data)

    def _post_param_set(self, parameter_id, parameter_value, parameter_type):

        data = {
            'header': {
                'system_id': 255,
                'component_id': 0,
                'sequence': 0},
            'message': {
                'type': "PARAM_SET",
                'param_value': parameter_value,
                'target_system': 0,
                'target_component': 0,
                'param_id': ["\u0000" for _ in range(16)],
                'param_type': {'type': parameter_type}
            }
        }

        for index, char in enumerate(parameter_id):
            data["message"]["param_id"][index] = char

        return self._post(url=f'{HOST_URL}/mavlink2rest/mavlink', data=data)

    def _post(self, url, data, retries=3, backoff_factor=1, status_forcelist=[502]):

        session = requests.Session()
        retry = Retry(total=retries, backoff_factor=backoff_factor, status_forcelist=status_forcelist, raise_on_status=False)
        adapter = HTTPAdapter(max_retries=retry)
        session.mount('http://', adapter)
        
        param_set = session.post(url=url, json=data)

        return param_set
 
    def set_parameter(self, parameter_id, parameter_value, parameter_type):

        param_value_pre = self._get_param_value()

        if not str(param_value_pre.status_code).startswith('2'):
            logging.warning(f'{self.timestamp()} Unable to obtain PARAM_VALUE before PARAM_SET for parameter "{parameter_id}". status_code: {param_value_pre.status_code}')
            return param_value_pre

        param_set = self._post_param_set(parameter_id=parameter_id, parameter_value=parameter_value, parameter_type=parameter_type)

        if not str(param_set.status_code).startswith('2'):
            logging.warning(f'{self.timestamp()} Unable to obtain PARAM_SET for parameter "{parameter_id}": status_code: {param_set.status_code}')
            return param_set

        for _ in range(3):
            time.sleep(0.05)  # it typically takes this amount of time for PARAM_VALUE to update
            param_value = self._get_param_value()

            if not str(param_value.status_code).startswith('2'):
                logging.warning(f'{self.timestamp()} Unable to obtain PARAM_VALUE after PARAM_SET for parameter "{parameter_id}". status_code: {param_value.status_code}')
                return param_value
            
            if str(param_value.status_code).startswith('2') and param_value_pre.json()["status"]["time"]["last_update"] != param_value.json()["status"]["time"]["last_update"]:
                break

        else:
            logging.warning(f'{self.timestamp()} Same timestamp for PARAM_VALUE before and after PARAM_SET for parameter "{parameter_id}, indicating that the value was not updated')
            param_value.status_code = 418
            return param_value

        response_parameter_id = ''
        for char in param_value.json()['message']['param_id']:
            if char == '\u0000':
                break

            response_parameter_id += char

        if response_parameter_id != parameter_id:
            logging.warning(f'{self.timestamp()} PARAM_VALUE responded with parameter_id "{response_parameter_id}" instead of the expected "{parameter_id}"')
            param_value.status_code = 419
            return param_value

        return param_value

    def get_parameter(self, parameter_id):
        
        param_value_pre = self._get_param_value()

        if not str(param_value_pre.status_code).startswith('2'):
            logging.warning(f'{self.timestamp()} Unable to obtain PARAM_VALUE before PARAM_REQUEST_READ for parameter "{parameter_id}". status_code: {param_value_pre.status_code}')
            return param_value_pre
            
        param_request_read = self._post_param_request_read(parameter_id=parameter_id)

        if not str(param_request_read.status_code).startswith('2'):
            logging.warning(f'{self.timestamp()} Unable to obtain PARAM_REQUEST_READ for parameter "{parameter_id}". status_code: {param_request_read.status_code}')
            return param_request_read
        
        for _ in range(3):
            time.sleep(0.05)  # it typically takes this amount of time for PARAM_VALUE to update
            param_value = self._get_param_value()

            if not str(param_value.status_code).startswith('2'):
                logging.warning(f'{self.timestamp()} Unable to obtain PARAM_VALUE after PARAM_REQUEST_READ for parameter "{parameter_id}". status_code: {param_value.status_code}')
                return param_value
            
            if str(param_value.status_code).startswith('2') and param_value_pre.json()["status"]["time"]["last_update"] != param_value.json()["status"]["time"]["last_update"]:
                break

        else:
            logging.warning(f'{self.timestamp()} Same timestamp for PARAM_VALUE before and after PARAM_REQUEST_READ for parameter "{parameter_id}, indicating that the value was not updated')
            param_value.status_code = 418
            return param_value

        response_parameter_id = ''
        for char in param_value.json()['message']['param_id']:
            if char == '\u0000':
                break

            response_parameter_id += char

        if response_parameter_id != parameter_id:
            logging.warning(f'{self.timestamp()} PARAM_VALUE responded with parameter_id "{response_parameter_id}" instead of the expected "{parameter_id}"')
            param_value.status_code = 419
            return param_value

        return param_value

    def check_cable_guy(self) -> bool:

        logging.info(f'{self.timestamp()} Discovering cable guy')

        self.status['cable_guy'] = 'Discovering...'

        url = f'{HOST_URL}/cable-guy/v1.0/ethernet'

        response = self._get(url=url)

        if str(response.status_code).startswith('2') and response.json()[0]['info']['connected'] is True:
            logging.info(f'{self.timestamp()} Cable guy online!')
            self.status['cable_guy'] = 'OK'

            self._cable_guy = True

        else:
            logging.warning(f'{self.timestamp()} Failed to discover cable guy!')
            self.status['cable_guy'] = 'Failed'

            self._cable_guy = False

        return self._cable_guy

    def check_heartbeat(self):

        vehicle_path = f"vehicles/1/components/1/messages"

        logging.info(f'{self.timestamp()} Checking vehicle heartbeat...')

        self.status['heartbeat'] = 'Listening...'

        url = f'{HOST_URL}/mavlink2rest/mavlink/{vehicle_path}/HEARTBEAT'

        response = self._get(url=url)

        if str(response.status_code).startswith('2') and response.json()["message"]["type"] == "HEARTBEAT":
            logging.info(f'{self.timestamp()} Heartbeat detected')
            self.status['heartbeat'] = 'OK'
            self._heartbeat = True

        else:
            logging.warning(f'{self.timestamp()} Failed to detect heartbeat')
            self.status['heartbeat'] = 'Failed'
            self._heartbeat = False

        return self._heartbeat

    def connect_nucleus(self):

        if self.hostname is None:
            self.status['nucleus_connected'] = 'No hostname'

            return self._nucleus_connected
        
        self.status['nucleus_connected'] = 'Connecting...'

        if self.nucleus_driver.connection.get_connection_status() and self.nucleus_driver.get_connection_type == 'serial':
            self.nucleus_driver.disconnect()

        if self.nucleus_driver.connection.get_connection_status() and self.nucleus_driver.get_connection_type == 'tcp':
            logging.warning(f'{self.timestamp()} Nucleus already connected')

        else:
            self.nucleus_driver.set_tcp_configuration(host=self.hostname)
            if not self.nucleus_driver.connect(connection_type='tcp'):
                logging.warning(f'{self.timestamp()} Failed to connect to Nucleus')
                self.status['nucleus_connected'] = 'Failed'

                self._nucleus_connected = False

                return self._nucleus_connected

        self.nucleus_id = self.nucleus_driver.connection.nucleus_id
        self.nucleus_firmware = self.nucleus_driver.connection.firmware_version

        logging.info(f'{self.timestamp()} Nucleus connected: {self.nucleus_driver.connection.get_connection_status()}')
        logging.info(f'{self.timestamp()} Nucleus ID:        {self.nucleus_id}')
        logging.info(f'{self.timestamp()} Nucleus firmware:  {self.nucleus_firmware}')

        self.status['nucleus_connected'] = 'OK'
        
        self._connect_button_text = 'Disconnect'
        self._nucleus_connected = True
        self._nucleus_running = None
        self._nucleus_configured = False

        return self._nucleus_connected

    def disconnect_nucleus(self):

        self.status['nucleus_connected'] = 'Disconnecting...'

        if not self.nucleus_driver.connection.get_connection_status():
            logging.warning(f'{self.timestamp()} Nucleus already disconnected')
            self.status['nucleus_connected'] = 'Failed'
            return False

        if not self.nucleus_driver.disconnect():
            logging.warning(f'{self.timestamp()} Failed to disconnect from Nucleus')
            self.status['nucleus_connected'] = 'Failed'
            return False
        
        self.status['nucleus_connected'] = 'Disconnected'
        self.status['nucleus_configured'] = '---'

        self._connect_button_text = 'Connect'
        self._nucleus_connected = False
        self._nucleus_running = None
        self._nucleus_configured = False

        return True

    def setup_nucleus(self):

        logging.info(f'{self.timestamp()} setting up nucleus')

        self.status['nucleus_configured'] = 'enabling...'

        reply_ahrs = self.nucleus_driver.commands.set_ahrs(ds="ON")
        if b'OK\r\n' not in reply_ahrs:
            logging.warning(f'{self.timestamp()} Did not receive OK when sending SETAHRS,DS="OFF": {reply}')

        reply_dvl = self.nucleus_driver.commands.set_bt(wt="ON", ds="ON")  # TODO: wt="OFF"
        if b'OK\r\n' not in reply_dvl:
            logging.warning(f'{self.timestamp()} Did not receive OK when sending SETBT,WT="OFF",DS="ON": {reply}')

        if b'OK\r\n' in reply_ahrs and b'OK\r\n' in reply_dvl:
            self.status['nucleus_configured'] = 'OK'
            self._nucleus_configured = True
        
        else:
            self.status['nucleus_configured'] = 'Failed'
            self._nucleus_configured = False

    def read_pid_parameters(self):

        status = True 

        for parameter in self.PID_PARAMETERS:

            response = self.get_parameter(parameter)
            status_code = response.status_code

            if not str(status_code).startswith('2'):
                logging.warning(f'{self.timestamp()} get_parameter did not respond with status code 2xx. Actual status code: {status_code}')
                status = False
                continue

            param_value = response.json()['message']['param_value']

            self.pid_parameters[parameter] = round(float(param_value), 3)

        return status
    
    def read_config_parameters(self):

        correct_values = True
        
        for parameter in self.EXPECTED_CONFIG_PARAMETERS.keys():

            response = self.get_parameter(parameter)
            status_code = response.status_code

            if not str(status_code).startswith('2'):
                logging.warning(f'{self.timestamp()} get_parameter did not respond with status code 2xx. Actual status code: {status_code}')
                correct_values = False
                continue

            param_value = response.json()['message']['param_value']

            self.config_parameters[parameter] = param_value

            if not self.EXPECTED_CONFIG_PARAMETERS[parameter] - 0.1 <= param_value <= self.EXPECTED_CONFIG_PARAMETERS[parameter] + 0.1:
                logging.warning(f'{self.timestamp()} Incorrect value for parameter {parameter}. Expected value: {self.EXPECTED_CONFIG_PARAMETERS[parameter]}. Received value: {response.json()["message"]["param_value"]}.')
                correct_values = False

        return correct_values

    def read_config_parameters_startup(self):

        self.status['controller_parameters'] = 'Reading...'

        correct_values = self.read_config_parameters()

        if correct_values:
             self.status['controller_parameters'] = 'OK'
        else:
             self.status['controller_parameters'] = 'Failed'

        self._config = correct_values

        return self._config
        
    def start_nucleus(self):

        logging.info(f'{self.timestamp()} Starting Nucleus')

        if b'OK\r\n' in self.nucleus_driver.start_measurement():
            logging.info(f'{self.timestamp()} Nucleus successfully started!')
            self._nucleus_running = True
            
        else:
            logging.warning(f'{self.timestamp()} Failed to start Nucleus!')

    def stop_nucleus(self):

        stop_reply = self.nucleus_driver.stop()

        if b'OK\r\n' in stop_reply:
            logging.info(f'{self.timestamp()} Nucleus stopped!')

            self._nucleus_running = False

        elif b'ERROR\r\n' in stop_reply:
            logging.info(f'{self.timestamp()} Nucleus was already stopped!')

            self._nucleus_running = False

        else:
            logging.warning(f'{self.timestamp()} Failed to stop Nucleus!')

    def restart_nucleus(self):

        self.stop_nucleus()

        if self._nucleus_running is not False:
            return
        
        self.setup_nucleus()

        if self._nucleus_configured is not True:
            return
        
        self.start_nucleus()

    def send_vision_position_delta(self, position_delta, angle_delta, confidence, dt):

        try:
            vision_position_delta = {
                "header": {
                    "system_id": 255,
                    "component_id": 0,
                    "sequence": 0},
                "message": {
                    "type": "VISION_POSITION_DELTA",
                    "time_usec": 0,
                    "time_delta_usec": dt,
                    "angle_delta": [angle_delta[0], angle_delta[1], angle_delta[2]],
                    "position_delta": [position_delta[0], position_delta[1], position_delta[2]],
                    "confidence": confidence
                }
            }

        except IndexError:

            logging.warning(f'{self.timestamp()} Failed to create VISION_POSITION_DELTA packet')
            return

        response = requests.post(f'{HOST_URL}/mavlink2rest/mavlink', json=vision_position_delta)

        if str(response.status_code).startswith('2'):
            self.vision_position_delta_packet_counter['packets_sent'] += 1
            logging.debug(f'{self.timestamp()} VISION_POSITION_DELTA\r\nangle_delta: {angle_delta}\r\nposition_delta: {position_delta}\r\nconfidence: {confidence}\r\ndt: {dt}')
        else:
            self.vision_position_delta_packet_counter['packets_failed'] += 1
            logging.warning(f'{self.timestamp()} VISION_POSITION_DELTA packet did not respond with 200: {response.status_code} - {response.text}')
    
    def check_requirements(self):

        if not self._cable_guy and not self.check_cable_guy_thread.is_alive():
            self.check_cable_guy_thread = Thread(target=self.check_cable_guy)
            self.check_cable_guy_thread.start()

        if not self._heartbeat and not self.check_heartbeat_thread.is_alive():
            self.check_heartbeat_thread = Thread(target=self.check_heartbeat)
            self.check_heartbeat_thread.start()

        if (self._nucleus_running is not True or self._nucleus_configured is not True) and self._nucleus_connected and not self.start_nucleus_thread.is_alive():
            self.start_nucleus_thread = Thread(target=self.restart_nucleus)
            self.start_nucleus_thread.start()
        
    def check_requirements_startup(self):

        def read_parameters():
            
            while not self._heartbeat:
                time.sleep(0.1)

            self.read_config_parameters_startup()
            self.read_pid_parameters()

        self.read_parameters_thread = Thread(target=read_parameters)
        self.read_parameters_thread.start()

    def start_main_thread(self):

        if not self.main_thread.is_alive():

            self.main_thread = Thread(target=self.run)
            self.main_thread.start()

        else:

            logging.warning(f'{self.timestamp()} Main thread is already alive. Main thread was not started')

    def stop_main_thread(self):

        if self.main_thread.is_alive():

            self.thread_running = False
            self.main_thread.join(5)

        else:

            logging.warning(f'{self.timestamp()} Main thread was not running. Main thread was not stopped')

    def handle_packet(self):

        packet = self.nucleus_driver.read_packet()

        if packet is None:
            time.sleep(0.005)
            return

        if packet['id'] == 0xb4:

            fom_x = packet['fomX']
            fom_y = packet['fomY']
            fom_z = packet['fomZ']

            fom = max(fom_x, fom_y, fom_z)

            bad_fom = 3
            confidence = (3 - min(fom, bad_fom)) * 100 / bad_fom  # TODO: optimize   operate from 0-3

            velocity_x = packet['velocityX']
            velocity_y = packet['velocityY']
            velocity_z = packet['velocityZ']

            timestamp = (packet['timeStamp'] + packet['microSeconds'] * 1e-6)

            if self.timestamp_previous is None:
                dt = 0
            else:
                dt = timestamp - self.timestamp_previous

            dx = velocity_x * dt
            dy = velocity_y * dt
            dz = velocity_z * dt

            delta_orientation = list()

            if self.orientation_current is None:
                delta_orientation = [0, 0, 0]

            elif self.orientation_previous is None or self.timestamp_previous is None:
                delta_orientation = [0, 0, 0]
                self.orientation_previous = self.orientation_current

            else:
                for (angle_current, angle_previous) in zip(self.orientation_current, self.orientation_previous):
                    delta_orientation.append(angle_current - angle_previous)

                self.orientation_previous = self.orientation_current

            self.timestamp_previous = timestamp

            if self._enable_nucleus_input:
                self.send_vision_position_delta(position_delta=[dx, dy, dz], angle_delta=delta_orientation, confidence=int(confidence), dt=int(dt * 1e6))
            else:
                self.vision_position_delta_packet_counter['packets_skipped'] += 1

        elif packet['id'] == 0xd2:

            orientation = list()
            orientation.append(self.d2r(packet['ahrsData.roll']))
            orientation.append(self.d2r(packet['ahrsData.pitch']))
            orientation.append(self.d2r(packet['ahrsData.heading']))

            self.orientation_current = orientation

    def run(self):

        #self.load_settings()

        self.check_requirements_startup()

        self.thread_running = True

        while self.thread_running:
            
            self.check_requirements()

            if self._nucleus_running is True:
                self.handle_packet()
            else:
                time.sleep(0.05)

        self.stop_nucleus()
