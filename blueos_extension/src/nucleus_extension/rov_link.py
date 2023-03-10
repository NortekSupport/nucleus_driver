import time
import logging
from threading import Thread
from queue import Queue
import socket
from datetime import datetime
import requests
from requests.adapters import HTTPAdapter, Retry

class RovLink(Thread):

    TIMEOUT = 1

    CONFIG_PARAMETERS = {
        'AHRS_EKF_TYPE': {'value': 3, 'type': "MAV_PARAM_TYPE_UINT8"},
        'EK2_ENABLE': {'value': 0, 'type': "MAV_PARAM_TYPE_UINT8"},
        'EK3_ENABLE': {'value': 1, 'type': "MAV_PARAM_TYPE_UINT8"},
        'VISO_TYPE': {'value': 1, 'type': "MAV_PARAM_TYPE_UINT8"},
        #'EK3_GPS_TYPE': {'value': 3, 'type': "MAV_PARAM_TYPE_UINT8"},
        'GPS_TYPE': {'value': 0, 'type': "MAV_PARAM_TYPE_INT8"},
        'EK3_SRC1_POSXY': {'value': 6, 'type': "MAV_PARAM_TYPE_UINT8"},
        'EK3_SRC1_VELXY': {'value': 6, 'type': "MAV_PARAM_TYPE_UINT8"},
        'EK3_SRC1_POSZ': {'value': 1, 'type': "MAV_PARAM_TYPE_UINT8"},
        'SERIAL0_PROTOCOL': {'value': 2, 'type': "MAV_PARAM_TYPE_INT8"}
    }

    PID_PARAMETERS = {
        'PSC_POSXY_P': {'value': 2, 'type': "MAV_PARAM_TYPE_REAL32"},
        'PSC_POSZ_P': {'value': 1.0, 'type': "MAV_PARAM_TYPE_REAL32"},
        'PSC_VELXY_P': {'value': 5.0, 'type': "MAV_PARAM_TYPE_REAL32"},
        'PSC_VELXY_I': {'value': 0.5, 'type': "MAV_PARAM_TYPE_REAL32"},
        'PSC_VELXY_D': {'value': 0.8, 'type': "MAV_PARAM_TYPE_REAL32"},
        'PSC_VELZ_P': {'value': 5.0, 'type': "MAV_PARAM_TYPE_REAL32"},
    }

    def __init__(self, driver):

        Thread.__init__(self)

        self.nucleus_driver = driver

        self.thread = Thread()
        self.thread_running = True

        self.timestamp_previous = None
        self.orientation_current = None
        self.orientation_previous = None

        self.hostname = None
        self.nucleus_id = None
        self.nucleus_firmware = None

        self._enable_nucleus_input = True

        self.packet_queue = Queue(maxsize=1000)

        self.init_time = datetime.now()

        self.status = {
            'cable_guy': 'Not OK',
            'nucleus_available': 'Not OK',
            'nucleus_connected': 'Not OK',
            'dvl_enabled': 'Not OK',
            'heartbeat': 'Not OK',
            'controller_parameters': 'Not OK'
        }

        self._cable_guy = False
        self._nucleus_available = False
        self._nucleus_connected = False
        self._dvl_enabled = False
        self._heartbeat = False
        self._config = False

        self._nucleus_running = False

        self.config_parameters = {
            'AHRS_EKF_TYPE': None,
            'EK2_ENABLE': None,
            'EK3_ENABLE': None,
            'VISO_TYPE': None,
            #'EK3_GPS_TYPE': {'value': 3, 'type': "MAV_PARAM_TYPE_UINT8"},
            'GPS_TYPE': None,
            'EK3_SRC1_POSXY': None,
            'EK3_SRC1_VELXY': None,
            'EK3_SRC1_POSZ': None,
            'SERIAL0_PROTOCOL': None
        }

    def d2r(self, deg):

        return deg * 3.14159 / 360

    def r2d(self, rad):

        return rad * 360 / 3.14159

    def set_enable_nucleus_input(self, enable: bool):

        self._enable_nucleus_input = enable

    def get_enable_nucleus_input(self):

        return self._enable_nucleus_input

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

    def write_packet(self, packet):

        if self.packet_queue.full():
            self.packet_queue.get_nowait()

        self.packet_queue.put_nowait(packet)

    def read_packet(self):

        packet = None

        if not self.packet_queue.empty():
            packet = self.packet_queue.get_nowait()

        return packet

    def set_parameter(self, parameter_id, parameter_value, parameter_type):

        def get_param_value_timestamp():

            param_value_pre_timestamp = None
            try:
                param_value_pre = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")
                param_value_pre_timestamp = param_value_pre.json()["status"]["time"]["last_update"]

            except Exception as e:
                logging.warning(f'Unable to obtain PARAM_VALUE before PARAM_REQUEST_READ: {e}')

            return param_value_pre_timestamp

        def set_through_mavlink(param_value_pre_timestamp):

            def post():

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

                return requests.post(MAVLINK2REST_URL + "/mavlink", json=data)

            def get():

                for _ in range(3):

                    try:
                        time.sleep(0.02)  # It typically takes this amount of time for PARAM_VALUE to change

                        param_value = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")

                        param_value_timestamp = param_value.json()["status"]["time"]["last_update"]

                        if param_value_pre_timestamp is None or param_value_pre_timestamp != param_value_timestamp:
                            break

                    except Exception as e:
                        logging.warning(f'Failed to obtain PARAM_VALUE for parameter {parameter_id}: {e}')
                        continue

                return param_value

            post_response = post()

            if post_response.status_code != 200:
                logging.warning(f'PARAM_SET command did not respond with 200: {post_response.status_code}')
                return post_response

            get_response = get()

            if get_response.status_code != 200:
                logging.warning(f'PARAM_VALUE command did not respond with 200: {get_response.status_code}')

            return get_response

        def check_parameter(param_value):

            def check_parameter_id():

                # Extract PARAM_VALUE id (name)
                response_parameter_id = ''
                for char in param_value.json()['message']['param_id']:
                    if char == '\u0000':
                        break

                    response_parameter_id += char

                # Check if obtained PARAM_ID is the same as requested
                if response_parameter_id != parameter_id:
                    return False

                return True

            def check_parameter_value():

                if int(param_value.json()['message']['param_value']) != int(parameter_value):
                    return False

                return True

            parameter_id_status = check_parameter_id()

            parameter_value_status = check_parameter_value()

            if not parameter_id_status or not parameter_value_status:
                param_value.status_code = 210

            return param_value

        timestamp = get_param_value_timestamp()

        parameter = set_through_mavlink(param_value_pre_timestamp=timestamp)

        if parameter.status_code != 200:
            return parameter

        parameter = check_parameter(parameter)

        return parameter

    def get_parameter(self, parameter_id):

        def get_param_value_timestamp():

            param_value_pre_timestamp = None
            try:
                param_value_pre = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")
                param_value_pre_timestamp = param_value_pre.json()["status"]["time"]["last_update"]

            except Exception as e:
                logging.warning(f'Unable to obtain PARAM_VALUE before PARAM_REQUEST_READ: {e}')

            return param_value_pre_timestamp

        def get_from_mavlink(param_value_pre_timestamp):

            def post():

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

                return requests.post(MAVLINK2REST_URL + "/mavlink", json=data)

            def get():

                for _ in range(3):

                    try:
                        time.sleep(0.02)  # It typically takes this amount of time for PARAM_VALUE to change

                        param_value = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")

                        param_value_timestamp = param_value.json()["status"]["time"]["last_update"]

                        if param_value_pre_timestamp is None or param_value_pre_timestamp != param_value_timestamp:
                            break

                    except Exception as e:
                        logging.warning(f'Failed to obtain PARAM_VALUE for parameter {parameter_id}: {e}')
                        continue

                return param_value

            post_response = post()

            if post_response.status_code != 200:
                logging.warning(f'PARAM_SREQUEST_READ command did not respond with 200: {post_response.status_code}')
                return post_response

            get_response = get()

            if get_response.status_code != 200:
                logging.warning(f'PARAM_VALUE command did not respond with 200: {get_response.status_code}')

            return get_response

        def check_parameter(param_value):

            def check_parameter_id():

                # Extract PARAM_VALUE id (name)
                response_parameter_id = ''
                for char in param_value.json()['message']['param_id']:
                    if char == '\u0000':
                        break

                    response_parameter_id += char

                # Check if obtained PARAM_ID is the same as requested
                if response_parameter_id != parameter_id:
                    return False

                return True

            parameter_id_status = check_parameter_id()

            if not parameter_id_status:
                param_value.status_code = 210

            return param_value

        timestamp = get_param_value_timestamp()

        parameter = get_from_mavlink(param_value_pre_timestamp=timestamp)

        if parameter.status_code != 200:
            return parameter

        parameter = check_parameter(parameter)

        return parameter

    '''Legacy
    def load_settings(self):

        self.hostname = HOSTNAME
    '''
    
    def wait_for_cableguy(self):
        '''
        def get_cableguy_status():

            session = requests.Session()
            retry = Retry(connect=20, backoff_factor=1, status_forcelist=[502])
            adapter = HTTPAdapter(max_retries=retry)
            session.mount('http://', adapter)

            response = session.get('http://127.0.0.1/cable-guy/v1.0/ethernet')

            if response.status_code == 200 and response.json()[0]['info']['connected'] is True:
                return True
            else:
                return False

        logging.info(f'{self.timestamp()} Checking Cable-guy...')

        for _ in range(21):
            if get_cableguy_status():
                break

            logging.info(f"{self.timestamp()} waiting for cable-guy to come online...")
            time.sleep(1)
        else:
            logging.warning(f'{self.timestamp()} Failed to find cable-guy')
            return False

        logging.info(f'{self.timestamp()} Cable-guy online')
        '''

        logging.info(f'{self.timestamp()} waiting for cable-guy to come online...')

        self.status['cable_guy'] = 'Discovering...'

        session = requests.Session()
        retry = Retry(connect=20, backoff_factor=1, status_forcelist=[502])
        adapter = HTTPAdapter(max_retries=retry)
        session.mount('http://', adapter)

        response = session.get('http://127.0.0.1/cable-guy/v1.0/ethernet')

        if response.status_code == 200 and response.json()[0]['info']['connected'] is True:
            logging.info(f'{self.timestamp()} Cable guy online')
            self.status['cable_guy'] = 'Online'

            self._cable_guy = True

        else:
            logging.warning(f'{self.timestamp()} Failed to discover!')
            self.status['cable_guy'] = 'Failed'

            self._cable_guy = False

        return self._cable_guy

    def discover_nucleus(self):

        logging.info(f'{self.timestamp()} Discovering Nucleus...')
        self.status['nucleus_available'] = 'Discovering...'

        for _ in range(21):
            try:
                socket.getaddrinfo(self.hostname, 9000)  # 5 sec timeout
                break
            except socket.gaierror:
                continue
        else:
            logging.warning(f'{self.timestamp()} Failed to discover Nucleus on the network')
            self.status['nucleus_available'] = 'Failed to discover!'
            
            self._nucleus_available = False

            return self._nucleus_available

        logging.info(f'{self.timestamp()} Discovered Nucleus on network')
        self.status['nucleus_available'] = 'OK'

        self._nucleus_available = True

        return self._nucleus_available

    def connect_nucleus(self):

        self.nucleus_driver.set_tcp_configuration(host=self.hostname)
        
        self.status['nucleus_connected'] = 'Connecting...'

        if not self.nucleus_driver.connect(connection_type='tcp'):
            logging.warning('Failed to connect to Nucleus')
            self.status['nucleus_connected'] = 'Connecting Failed!'

            self._nucleus_connected = False

            return self._nucleus_connected

        self.nucleus_id = self.nucleus_driver.connection.nucleus_id
        self.nucleus_firmware = self.nucleus_driver.connection.firmware_version

        logging.info(f'{self.timestamp()} Nucleus connected: {self.nucleus_driver.connection.get_connection_status()}')
        logging.info(f'{self.timestamp()} Nucleus ID:        {self.nucleus_id}')
        logging.info(f'{self.timestamp()} Nucleus firmware:  {self.nucleus_firmware}')

        self.status['nucleus_connected'] = 'OK'

        self._nucleus_connected = True

        return self._nucleus_connected

    def setup_nucleus(self):

        logging.info(f'{self.timestamp()} setting up nucleus')
        

        reply = self.nucleus_driver.commands.set_default_config()
        if b'OK\r\n' not in reply:
            logging.warning(f'{self.timestamp()} Did not receive OK when sending SETDEFAULT,CONFIG: {reply}')

        reply = self.nucleus_driver.commands.set_default_mission()
        if b'OK\r\n' not in reply:
            logging.warning(f'{self.timestamp()} Did not receive OK when sending SETDEFAULT,MISSION: {reply}')

        reply = self.nucleus_driver.commands.set_default_magcal()
        if b'OK\r\n' not in reply:
            logging.warning(f'{self.timestamp()} Did not receive OK when sending SETDEFAULT,MAGCAL: {reply}')

        reply = self.nucleus_driver.commands.set_ahrs(ds="ON")
        if b'OK\r\n' not in reply:
            logging.warning(f'{self.timestamp()} Did not receive OK when sending SETAHRS,DS="OFF": {reply}')

        self.setup['dvl_enabled'] = 'enabling...'
        reply = self.nucleus_driver.commands.set_bt(wt="ON", ds="ON")  # TODO: wt="OFF"
        if b'OK\r\n' not in reply:
            logging.warning(f'{self.timestamp()} Did not receive OK when sending SETBT,WT="OFF",DS="ON": {reply}')

            self.status['dvl_enabled'] = 'Failed to enable'
            self._dvl_enabled = False

        else:
            self.status['dvl_enabled'] = 'OK'
            self._dvl_enabled = True

        reply = self.nucleus_driver.commands.set_alti(ds="ON")  # TODO: OFF
        if b'OK\r\n' not in reply:
            logging.warning(f'{self.timestamp()} Did not receive OK when sending SETALTI,DS="OFF": {reply}')

        reply = self.nucleus_driver.commands.set_cur_prof(ds="OFF")  # TODO: OFF
        if b'OK\r\n' not in reply:
            logging.warning(f'{self.timestamp()} Did not receive OK when sending SETCURPROF,DS="OFF": {reply}')


    def wait_for_heartbeat(self):
        """
        Waits for a valid heartbeat to Mavlink2Rest
        """

        def get_heartbeat():

            response = requests.get(MAVLINK2REST_URL + "/mavlink" + vehicle_path + '/HEARTBEAT')

            if response.status_code != 200:
                logging.warning(f'{self.timestamp()} HEARTBEAT request did not respond with 200: {response.status_code}')
                return False

            status = False
            try:
                if response.json()["message"]["type"] == "HEARTBEAT":
                    status = True
            except Exception as e:
                logging.warning(f'{self.timestamp()} Unable to extract data from HEARTBEAT request: {e}')
                logging.warning(f'{response}')

            return status

        vehicle = 1
        component = 1

        vehicle_path = f"/vehicles/{vehicle}/components/{component}/messages"

        logging.info(f'{self.timestamp()} waiting for vehicle heartbeat...')
        self.status['heartbeat'] = 'Discovering...'

        for _ in range(21):
            if get_heartbeat():
                break

            logging.info(f"{self.timestamp()} waiting for heartbeat...")
            self.status['heartbeat'] = 'Waiting...'
            time.sleep(1)
        else:
            logging.warning(f'{self.timestamp()} Failed to detect heartbeat')
            self.status['heartbeat'] = 'Failed to detect!'

            self._heartbeat = False

            return self._heartbeat

        logging.info(f'{self.timestamp()} Heartbeat detected')
        self.status['heartbeat'] = 'OK'

        self._heartbeat = True

        return self._heartbeat
    
    ''' Legacy
    def handle_config_parameters(self):

        parameter_change = False

        for parameter in self.CONFIG_PARAMETERS.keys():

            response = self.get_parameter(parameter)

            if response.status_code == 200 and self.CONFIG_PARAMETERS[parameter]['value'] - 0.1 <= response.json()['message']['param_value'] <= self.CONFIG_PARAMETERS[parameter]['value'] + 0.1:
                continue

            parameter_change = True

            response = self.set_parameter(parameter, self.CONFIG_PARAMETERS[parameter]['value'], self.CONFIG_PARAMETERS[parameter]['type'])

            if response.status_code == 200:
                logging.warning(f'{self.timestamp()} {parameter} set to {self.CONFIG_PARAMETERS[parameter]["value"]}')
            else:
                logging.warning(f'{self.timestamp()} Failed to set {parameter} to {self.CONFIG_PARAMETERS[parameter]["value"]}')

        return parameter_change
    '''

    def read_config_parameters(self):

        correct_values = True
        
        self.status['controller_parameters'] = 'Reading parameters...'

        for parameter in self.CONFIG_PARAMETERS.keys():

            response = self.get_parameter(parameter)

            if response.status_code == 200:
                self.config_parameters[parameter] = response.json()['message']['param_value']

            if not response.status_code == 200 or self.CONFIG_PARAMETERS[parameter]['value'] - 0.1 <= response.json()['message']['param_value'] <= self.CONFIG_PARAMETERS[parameter]['value'] + 0.1:
                correct_values = False

        if correct_values:
             self.status['controller_parameters'] = 'OK'
        else:
             self.status['controller_parameters'] = 'Incorrect'

        self._config = correct_values

        return correct_values

    ''' Legacy
    def set_pid_parameters(self):

        for parameter in self.PID_PARAMETERS.keys():

            response = self.set_parameter(parameter, self.PID_PARAMETERS[parameter]['value'], self.PID_PARAMETERS[parameter]['type'])

            if response.status_code == 200:
                logging.warning(f'{self.timestamp()} {parameter} set to {self.PID_PARAMETERS[parameter]["value"]}')
            else:
                logging.warning(f'{self.timestamp()} Failed to set {parameter} to {self.PID_PARAMETERS[parameter]["value"]}')
    '''

    def start_nucleus(self):

        if b'OK\r\n' not in self.nucleus_driver.start_measurement():
            logging.warning(f'{self.timestamp()} Failed to start Nucleus')
        
        else:
            self._nucleus_running = True

    def stop_nucleus(self):

        if b'OK\r\n' in self.nucleus_driver.stop():
            logging.warning(f'{self.timestamp()} Nucleus was already running. Nucleus is now stopped')
            self._nucleus_running = False

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

        response = requests.post(MAVLINK2REST_URL + "/mavlink", json=vision_position_delta)

        if response.status_code == 200:
            logging.debug(f'{self.timestamp()} VISION_POSITION_DELTA\r\nangle_delta: {angle_delta}\r\nposition_delta: {position_delta}\r\nconfidence: {confidence}\r\ndt: {dt}')
        else:
            logging.warning(f'{self.timestamp()} VISION_POSITION_DELTA packet did not respond with 200: {response.status_code} - {response.text}')
    
    
    
    def run(self):

        self.load_settings()

        self.wait_for_cableguy()

        if self._cable_guy:
            self.discover_nucleus()
        
        if self._nucleus_available:
            self.connect_nucleus()

        if self._nucleus_connected:
            self.stop_nucleus()
            self.setup_nucleus()

        if self._cable_guy:
            self.wait_for_heartbeat()

        if self._heartbeat:
            return

        if self._nucleus_connected:
            self.set_pid_parameters()

        time.sleep(1)

        self.start_nucleus()

        logging.info(f"{self.timestamp()} Nucleus driver running")

        while self.thread_running:

            if self._cable_guy and self._nucleus_available and self._nucleus_connected and self._dvl_enabled and self._heartbeat and self._config:

                if not self._cable_guy:
                    logging.warning(f"{self.timestamp()} Cable guy not available")

                if not self._nucleus_available:
                    logging.warning(f"{self.timestamp()} Nucleus is not available on the network")

                if not self._nucleus_connected:
                    logging.warning(f"{self.timestamp()} Nucleus is not connected")

                if not self._dvl_enabled:
                    logging.warning(f"{self.timestamp()} DVL is not enabled on Nucleus")

                if not self._heartbeat:
                    logging.warning(f"{self.timestamp()} Can't detect heartbeat")

                if not self._config:
                    logging.warning(f"{self.timestamp()} ROV has incorrect configuration for velocity data input")

                time.sleep(1)

                continue
            
            if not self._nucleus_running:
                qsize = self.nucleus_driver._parser.packet_queue.qsize

                if qsize == 0:
                    self.start_nucleus()

                else:
                    time.sleep(1)
                    if qsize != self.nucleus_driver._parser.packet_queue.qsize:
                        self.start_nucleus()

            packet = self.nucleus_driver.read_packet()

            if packet is None:
                time.sleep(0.005)
                continue

            self.write_packet(packet=packet)

            if not self._enable_nucleus_input:
                time.sleep(0.005)
                continue

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

                if self.enable_nucleus_input:
                    self.send_vision_position_delta(position_delta=[dx, dy, dz], angle_delta=delta_orientation, confidence=int(confidence), dt=int(dt * 1e6))

            if packet['id'] == 0xd2:

                orientation = list()
                orientation.append(self.d2r(packet['ahrsData.roll']))
                orientation.append(self.d2r(packet['ahrsData.pitch']))
                orientation.append(self.d2r(packet['ahrsData.heading']))

                self.orientation_current = orientation
    