import time

from flask import Flask, json, request, jsonify
from flask_restful import Api, reqparse
from threading import Thread
import requests
import json
import logging
import socket


from nucleus_driver import NucleusDriver


#DOCKER_HOST = '0.0.0.0'
#DOCKER_PORT = 5000

#NUCLEUS_HOST = 'Nucleus-300004.local'

logging.basicConfig(level=logging.DEBUG)

#HOSTNAME = 'NORTEK-300004.local'
HOSTNAME = '192.168.2.201'
PORT = 5000  # TODO: Is this the port?

MAVLINK2REST_URL = "http://127.0.0.1/mavlink2rest"

app = Flask(__name__)
api = Api(app)


@app.route("/nucleus_driver/connect_serial", methods=['GET'])
def connect_serial():

    serial_port = request.args.get('serial_port')

    nucleus_driver.set_serial_configuration(port=serial_port)

    # TODO: Test what happens if serial port is not given
    # TODO: Test what happens if serial port is wrong

    if not nucleus_driver.connect(connection_type='serial'):
        response = jsonify({'status': 'Failed to connect through serial'})
        response.status_code = 400
        return response

    nucleus_id = nucleus_driver.connection.nucleus_id
    nucleus_firmware = nucleus_driver.connection.firmware_version

    return jsonify({'status': 'Connected through serial',
                    'ID': nucleus_id,
                    'Firmware': nucleus_firmware})


@app.route("/nucleus_driver/connect_tcp", methods=['GET'])
def connect_tcp():
    host = request.args.get('host')

    logging.info(f'connecting to host: {host}')

    nucleus_driver.set_tcp_configuration(host=host)

    # TODO: Test what happens if serial port is not given
    # TODO: Test what happens if serial port is wrong

    if not nucleus_driver.connect(connection_type='tcp'):
        response = jsonify({'status': 'Failed to connect through TCP'})
        response.status_code = 400
        return response

    nucleus_id = nucleus_driver.connection.nucleus_id
    nucleus_firmware = nucleus_driver.connection.firmware_version

    return jsonify({'status': 'Connected through TCP',
                    'ID': nucleus_id,
                    'Firmware': nucleus_firmware})


@app.route("/nucleus_driver/disconnect", methods=['GET'])
def disconnect():

    # TODO: Test what happens if serial port is not given
    # TODO: Test what happens if serial port is wrong

    if not nucleus_driver.disconnect():
        response = jsonify({'status': 'Failed to disconnect from Nucleus device'})
        response.status_code = 400
        return response

    return jsonify({'status': 'Disconnected from Nucleus'})


@app.route("/nucleus_driver/start", methods=['GET'])
def start():

    if not nucleus_driver.connection.get_connection_status():
        return jsonify({'reply': 'Nucleus not connected!'})

    reply = nucleus_driver.start_measurement()

    try:
        reply = reply[0].decode()
    except IndexError:
        reply = 'index error'

    return jsonify({'reply': reply})


@app.route("/nucleus_driver/stop", methods=['GET'])
def stop():

    if not nucleus_driver.connection.get_connection_status():
        return jsonify({'reply': 'Nucleus not connected!'})

    reply = nucleus_driver.stop()

    try:
        stop_reply = reply[0][-4:].decode()
    except IndexError:
        stop_reply = 'index error'
    except UnicodeDecodeError:
        stop_reply = 'decode error'

    return jsonify({'reply': stop_reply})


@app.route("/nucleus_driver/get_packet", methods=['GET'])
def get_packet():

    if not nucleus_driver.connection.get_connection_status():
        return jsonify({'reply': 'Nucleus not connected!'})

    size = request.args.get('size')

    packet_list = list()

    if size is not None:

        for _ in range(int(size)):

            reply = nucleus_driver.read_packet()

            if reply is None:
                break

            packet_list.append(reply)

    else:
        reply = nucleus_driver.read_packet()

        if reply is not None:
            packet_list.append(reply)

    return jsonify({'packets': packet_list})


@app.route("/nucleus_driver/get_all", methods=['GET'])
def nucleus_driver_get_all():

    if not nucleus_driver.connection.get_connection_status():
        return jsonify({'reply': 'Nucleus not connected!'})

    reply = nucleus_driver.commands.get_all()

    reply_list = list()

    try:
        _ = reply[0]  # Test if list is not empty

        for value in reply:
            try:
                if value == b'OK\r\n':
                    break

                decoded_value = value.decode()
                reply_list.append(decoded_value)

            except UnicodeDecodeError:
                reply_list.append('decode error')

    except IndexError:
        reply_list.append('index error')

    return jsonify({'get_all': reply_list})


@app.route('/mavlink/get_parameter', methods=['GET'])
def mavlink_get_parameter():

    # Check if a parameter_id is given
    parameter_id = request.args.get('parameter_id')

    if parameter_id is None:
        response = jsonify({'message': 'parameter_id must be specified'})
        response.status_code = 400
        return response

    parameter_id = parameter_id.upper()

    return get_parameter(parameter_id)


@app.route('/mavlink/set_parameter', methods=['GET'])
def mavlink_set_parameter():

    PARAMETER_TYPES = ["MAV_PARAM_TYPE_UINT8"]

    # Check if a parameter_id is given
    parameter_id = request.args.get('parameter_id')
    parameter_value = request.args.get('parameter_value')
    parameter_type = request.args.get('parameter_type')

    if parameter_id is None:
        response = jsonify({'message': 'parameter_id must be specified'})
        response.status_code = 400
        return response

    if parameter_value is None:
        response = jsonify({'message': 'parameter_value must be specified'})
        response.status_code = 400
        return response

    if parameter_type is None:
        response = jsonify({'message': 'parameter_type must be specified'})
        response.status_code = 400
        return response

    parameter_id = parameter_id.upper()

    if parameter_type not in PARAMETER_TYPES:
        response = jsonify({'message': f'invalid parameter_type, must be in {PARAMETER_TYPES}'})
        response.status_code = 400
        return response

    try:
        if parameter_type in ["MAV_PARAM_TYPE_UINT8"]:
            parameter_value = int(parameter_value)
    except ValueError:
        response = jsonify({'message': 'parameter_value must be a number'})
        response.status_code = 400
        return response

    return set_parameter(parameter_id, parameter_value, parameter_type)


@app.route('/mavlink/set_default_parameters', methods=['GET'])
def mavlink_set_default_parameters():
    AHRS_EKF_TYPE = 3.0
    EK2_ENABLE = 0.0
    EK3_ENABLE = 1.0
    VISO_TYPE = None
    EK3_GPS_TYPE = 3.0
    EK3_SRC1_POSXY = None
    EK3_SRC1_VELXY = None
    EK3_SRC1_POSZ = None

    return


def _get_param_value_timestamp():

    param_value_pre_timestamp = None
    try:
        param_value_pre = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")
        param_value_pre_timestamp = param_value_pre.json()["status"]["time"]["last_update"]

    except Exception as e:
        logging.warning(f'Unable to obtain PARAM_VALUE before PARAM_REQUEST_READ: {e}')

    return param_value_pre_timestamp


def set_parameter(parameter_id, parameter_value, parameter_type):

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

            for _ in range(10):

                time.sleep(0.01)  # It typically takes this amount of time for PARAM_VALUE to change

                param_value = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")

                param_value_timestamp = param_value.json()["status"]["time"]["last_update"]

                if param_value_pre_timestamp is None or param_value_pre_timestamp != param_value_timestamp:
                    break

            return param_value

        post_response = post()

        if post_response.status_code != 200:
            param_value = jsonify({'message': f'PARAM_SET command did not respond with 200: {post_response.status_code}'})
            param_value.status_code = post_response.status_code
            return param_value

        get_response = get()

        if get_response.status_code != 200:
            param_value = jsonify({'message': f'PARAM_VALUE command did not respond with 200: {get_response.status_code}'})
            param_value.status_code = get_response.status_code
            return param_value

        return get_response

    def check_parameter(param_value):

        def check_parameter_id():

            # Extract PARAM_VALUE id (name)
            response_parameter_id = ''
            for char in param_value['message']['param_id']:
                if char == '\u0000':
                    break

                response_parameter_id += char

            # Check if obtained PARAM_ID is the same as requested
            if response_parameter_id != parameter_id:
                if 'WARNING' not in param_value.keys():
                    param_value.update({'WARNING': {}})

                param_value['WARNING'].update({'param_id': {'message': 'The obtained parameter is not the same as the specified parameter',
                                                            'specified_parameter': parameter_id,
                                                            'obtained_parameter': response_parameter_id
                                                            }})
                return False

            return True

        def check_parameter_value():

            if int(param_value['message']['param_value']) != int(parameter_value):
                if 'WARNING' not in param_value.keys():
                    param_value.update({'WARNING': {}})

                param_value['WARNING'].update({'param_value': {'message': 'The obtained parameter value is not the same as the specified value',
                                                               'specified_parameter': parameter_value,
                                                               'obtained_parameter': int(param_value['message']['param_value'])
                                                               }})
                return False

            return True

        parameter_id_status = check_parameter_id()

        parameter_value_status = check_parameter_value()

        param_value = jsonify(param_value)
        if parameter_id_status and parameter_value_status:
            param_value.status_code = 200
        else:
            param_value.status_code = 210

        return param_value

    logging.info('set_param')

    timestamp = _get_param_value_timestamp()

    logging.info('timestamp received')

    parameter = set_through_mavlink(param_value_pre_timestamp=timestamp)

    logging.info('parameter set')

    if parameter.status_code != 200:
        return parameter

    parameter = check_parameter(parameter.json())

    logging.info('parameter checked')

    return parameter


def get_parameter(parameter_id):

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

            for _ in range(10):

                time.sleep(0.01)  # It typically takes this amount of time for PARAM_VALUE to change

                param_value = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")

                param_value_timestamp = param_value.json()["status"]["time"]["last_update"]

                if param_value_pre_timestamp is None or param_value_pre_timestamp != param_value_timestamp:
                    break

            return param_value

        post_response = post()

        if post_response.status_code != 200:
            param_value = jsonify({'message': f'PARAM_SREQUEST_READ command did not respond with 200: {post_response.status_code}'})
            param_value.status_code = post_response.status_code
            return param_value

        get_response = get()

        if get_response.status_code != 200:
            param_value = jsonify({'message': f'PARAM_VALUE command did not respond with 200: {get_response.status_code}'})
            param_value.status_code = get_response.status_code
            return param_value

        return get_response

    def check_parameter(param_value):

        def check_parameter_id():

            # Extract PARAM_VALUE id (name)
            response_parameter_id = ''
            for char in param_value['message']['param_id']:
                if char == '\u0000':
                    break

                response_parameter_id += char

            # Check if obtained PARAM_ID is the same as requested
            if response_parameter_id != parameter_id:
                if 'WARNING' not in param_value.keys():
                    param_value.update({'WARNING': {}})

                param_value['WARNING'].update({'param_id': {'message': 'The obtained parameter is not the same as the specified parameter',
                                                            'specified_parameter': parameter_id,
                                                            'obtained_parameter': response_parameter_id
                                                            }})
                return False

            return True

        parameter_id_status = check_parameter_id()

        param_value = jsonify(param_value)
        if parameter_id_status:
            param_value.status_code = 200
        else:
            param_value.status_code = 210

        return param_value

    timestamp = _get_param_value_timestamp()

    parameter = get_from_mavlink(param_value_pre_timestamp=timestamp)

    if parameter.status_code != 200:
        return parameter

    parameter = check_parameter(parameter.json())

    return parameter


class RovLink(Thread):

    TIMEOUT = 1

    PARAMETERS = {'AHRS_EKF_TYPE': {'value': 3, 'type': "MAV_PARAM_TYPE_UINT8"},
                  'EK2_ENABLE': {'value': 0, 'type': "MAV_PARAM_TYPE_UINT8"},
                  'EK3_ENABLE': {'value': 1, 'type': "MAV_PARAM_TYPE_UINT8"},
                  'VISO_TYPE': {'value': 1, 'type': "MAV_PARAM_TYPE_UINT8"},
                  'EK3_GPS_TYPE': {'value': 3, 'type': "MAV_PARAM_TYPE_UINT8"},
                  'EK3_SRC1_POSXY': {'value': 6, 'type': "MAV_PARAM_TYPE_UINT8"},
                  'EK3_SRC1_VELXY': {'value': 6, 'type': "MAV_PARAM_TYPE_UINT8"},
                  'EK3_SRC1_POSZ': {'value': 1, 'type': "MAV_PARAM_TYPE_UINT8"}
                  }

    def __init__(self, driver):

        Thread.__init__(self)

        self.nucleus_driver = driver

        self.thread = Thread()
        self.thread_running = False

        self.timestamp_previous = None
        self.orientation_current = None
        self.orientation_previous = None

        self.hostname = None
        self.nucleus_id = None
        self.nucleus_firmware = None

        self.packet_received_timestamp = None
    '''
    def reconnect(self):

        # TODO: Add reconnect funtionality

        pass
    '''
    def load_settings(self):

        # TODO: How to implement settings? .config/dvl/settings.json?

        logging.info('Loading settings')

        self.hostname = HOSTNAME

    def wait_for_cableguy(self):

        logging.info('waiting for cableguy')

        def get_cableguy_status():
            response = requests.get("http://127.0.0.1/cable-guy/v1.0/ethernet")

            logging.info(f'\r\n\r\nCABLEGUY:\r\n{response.json()}\r\n\r\n{response.status_code}\r\n\r\n')

            return False

        while not get_cableguy_status():
            logging.info("waiting for cable-guy to come online...")
            time.sleep(1)

    def discover_nucleus(self):

        # TODO: Waterlinked implementation uses Nmap. Necessary?

        logging.info('discovering nucleus')

        for _ in range(21):
            try:
                socket.getaddrinfo(self.hostname, 9000)  # 5 sec timeout
                break
            except socket.gaierror:
                continue
        else:
            logging.warning('Failed to discover Nucleus on the network')
            return False

        logging.info('Discovered Nucleus on network')
        return True

    def connect_nucleus(self):

        logging.info('####################  TRIGGERDED connect_nucleus() function  ##################')

        self.nucleus_driver.set_tcp_configuration(host=self.hostname)

        if not self.nucleus_driver.connect(connection_type='tcp'):
            return False

        self.nucleus_id = self.nucleus_driver.connection.nucleus_id
        self.nucleus_firmware = self.nucleus_driver.connection.firmware_version

        logging.info(f'Nucleus connected: {self.nucleus_driver.connection.get_connection_status()}')
        logging.info(f'Nucleus ID: -+-    {self.nucleus_id}')
        logging.info(f'Nucleus firmware:  {self.nucleus_firmware}')

        return True

    def setup_nucleus(self):

        logging.info('setting up nucleus')

        reply = self.nucleus_driver.commands.set_default_config()
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETDEFAULT,CONFIG: {reply}')

        reply = self.nucleus_driver.commands.set_default_mission()
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETDEFAULT,MISSION: {reply}')

        reply = self.nucleus_driver.commands.set_default_magcal()
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETDEFAULT,MAGCAL: {reply}')

        reply = self.nucleus_driver.commands.set_ahrs(ds="ON")
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETAHRS,DS="ON": {reply}')

        reply = self.nucleus_driver.commands.set_bt(wt="OFF", ds="ON")
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETBT,WT="OFF",DS="ON": {reply}')

        reply = self.nucleus_driver.commands.set_alti(ds="OFF")
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETALTI,DS="OFF": {reply}')

        reply = self.nucleus_driver.commands.set_cur_prof(ds="OFF")
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETCURPROF,DS="OFF": {reply}')

    def wait_for_heartbeat(self):
        """
        Waits for a valid heartbeat to Mavlink2Rest
        """

        def get_heartbeat():

            response = requests.get(MAVLINK2REST_URL + "/mavlink" + vehicle_path + '/HEARTBEAT')

            if response.status_code != 200:
                logging.warning(f'HEARTBEAT request did not respond with 200')
                return False

            status = False
            try:
                if response.json()["message"]["type"] == "HEARTBEAT":
                    status = True
            except Exception as e:
                logging.warning(f'Unable to extract data from HEARTBEAT request: {e}')

            return status

        vehicle = 1
        component = 1

        vehicle_path = f"/vehicles/{vehicle}/components/{component}/messages"

        logging.info('waiting for vehicle heartbeat...')

        while not get_heartbeat():

            time.sleep(1)

    def setup_parameters(self):

        for parameter in self.PARAMETERS.keys():

            logging.info(f"{parameter}, {self.PARAMETERS[parameter]['value']}, {self.PARAMETERS[parameter]['type']}")

            response = set_parameter(parameter, self.PARAMETERS[parameter]['value'], self.PARAMETERS[parameter]['type'])

            logging.info(f'parameter set response: {response}')

            if response.status_code != 200:
                logging.warning(f'Failed to set {parameter} to {self.PARAMETERS[parameter]["value"]}')

        '''
        response = set_parameter("AHRS_EKF_TYPE", 3, "MAV_PARAM_TYPE_UINT8")
        logging.info(f'AHRS_EKF_TYPE: {response}')

        response = set_parameter("EK2_ENABLE", 0, "MAV_PARAM_TYPE_UINT8")
        logging.info(f'EK2_ENABLE: {response}')

        response = set_parameter("EK3_ENABLE", 1, "MAV_PARAM_TYPE_UINT8")
        logging.info(f'EK3_ENABLE: {response}')

        response = set_parameter("VISO_TYPE", 1, "MAV_PARAM_TYPE_UINT8")
        logging.info(f'VISO_TYPE: {response}')

        response = set_parameter("EK3_GPS_TYPE", 3, "MAV_PARAM_TYPE_UINT8")
        logging.info(f'EK3_GPS_TYPE: {response}')

        response = set_parameter("EK3_SRC1_POSXY", 6, "MAV_PARAM_TYPE_UINT8")  # EXTNAV
        logging.info(f'EK3_SRC1_POSXY: {response}')

        response = set_parameter("EK3_SRC1_VELXY", 6, "MAV_PARAM_TYPE_UINT8")  # EXTNAV
        logging.info(f'EK3_SRC1_VELXY: {response}')

        response = set_parameter("EK3_SRC1_POSZ", 1, "MAV_PARAM_TYPE_UINT8")
        logging.info(f'EK3_SRC1_POSZ: {response}')
        '''

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

            logging.warning('Failed to create VISION_POSITION_DELTA packet')
            return

        response = requests.post(MAVLINK2REST_URL + "/mavlink", data=vision_position_delta)

        logging.info(f'\r\nVISION_POSITION_DELTA response: \r\n{response}\r\n\r\nstatus_code:\r\n{response.status_code}\r\n\r\n')

    def run(self):

        logging.info('Waiting 10 seconds...')
        #time.sleep(10)
        logging.info('continuing')

        logging.info('RUN STARTED')

        self.load_settings()
        #self.wait_for_cableguy()
        self.discover_nucleus()

        #return

        if not self.connect_nucleus():
            logging.warning('Failed to connect to Nucleus')

        #return

        self.setup_nucleus()
        self.wait_for_heartbeat()
        #self.setup_mavlink()  # TODO
        self.setup_parameters()
        time.sleep(1)  # TODO
        logging.debug("Running")  # TODO
        #self.packet_received_timestamp = time.time()  # TODO

        #self.connected = True  # TODO

        while self.thread_running:

            packet = self.nucleus_driver.read_packet()

            if packet is None:
                if time.time() - self.packet_received_timestamp > self.TIMEOUT:
                    logging.warning('Timeout, restarting')
                    self.reconnect()  # TODO
                time.sleep(0.005)
                continue

            #self.packet_received_timestamp = time.time()

            if packet['id'] == 0xb4:

                logging.info('received BT package')

                fom_x = packet['fomX']
                fom_y = packet['fomY']
                fom_z = packet['fomZ']

                fom = max(fom_x, fom_y, fom_z)

                confidence = (10 - fom) * 10

                velocity_x = packet['velocityX']
                velocity_y = packet['velocityY']
                velocity_z = packet['velocityZ']

                timestamp = (packet['timestamp'] + packet['microseconds'] * 1e-6) / 1000

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

                self.send_vision_position_delta(position_delta=[dx, dy, dz], angle_delta=delta_orientation, confidence=confidence, dt=dt)
                logging.info(f'VISION_POSITION_DELTA - POS_DELTA={[dx, dy, dz]} - ANG_DELTA={delta_orientation} - FOM={confidence} - dt={dt}')

            if packet['id'] == 0xd2:

                roll = packet['ahrsData.roll']
                pitch = packet['ahrsData.pitch']
                heading = packet['ahrsData.heading']

                self.orientation_current = [roll, pitch, heading]


if __name__ == "flask_app":

    nucleus_driver = NucleusDriver()
    #nucleus_driver.connection.set_tcp_configuration(host=NUCLEUS_HOST)
    #nucleus_driver.connection.connect(connection_type='tcp')

    packet_args = reqparse.RequestParser()
    packet_args.add_argument('size', type=int, help="Number of packets returned by packets")

    #app.run(debug=True, host=DOCKER_HOST, port=DOCKER_PORT)

    logging.info('initiating RovLink')
    rov_link = RovLink(driver=nucleus_driver)
    rov_link.start()
    logging.info('RovLink running')

    print('INITIATED DRIVER')
