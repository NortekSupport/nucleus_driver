import time

from flask import Flask, json, request, jsonify
from flask_restful import Api, reqparse
from threading import Thread
import requests
import logging
import socket


from nucleus_driver import NucleusDriver


logging.basicConfig(level=logging.DEBUG)

#HOSTNAME = 'NORTEK-300004.local'
HOSTNAME = '192.168.2.201'
PORT = 5000  # TODO: Is this the port?

MAVLINK2REST_URL = "http://127.0.0.1/mavlink2rest"  # TODO: Fix


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

        self.enable_nucleus_input = True
        self.enable_vision_position_delta = True
        self.enable_vision_speed_estimate = False
        self.enable_global_vision_position_estimate = False

    def d2r(self, deg):

        return deg * 3.14159 / 360

    def r2d(self, rad):

        return rad * 360 / 3.14159

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

                for _ in range(10):

                    try:
                        time.sleep(0.01)  # It typically takes this amount of time for PARAM_VALUE to change

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

                for _ in range(10):

                    try:
                        time.sleep(0.01)  # It typically takes this amount of time for PARAM_VALUE to change

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

    def load_settings(self):

        self.hostname = HOSTNAME

    def wait_for_cableguy(self):

        def get_cableguy_status():
            response = requests.get("http://127.0.0.1/cable-guy/v1.0/ethernet")

            if response.status_code == 200 and response.json()[0]['info']['connected'] is True:
                return True
            else:
                return False

        while not get_cableguy_status():
            logging.info("waiting for cable-guy to come online...")
            time.sleep(1)

        logging.info('Cable-guy online')

    def discover_nucleus(self):

        logging.info('Discovering Nucleus...')

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

        self.nucleus_driver.set_tcp_configuration(host=self.hostname)

        if not self.nucleus_driver.connect(connection_type='tcp'):
            logging.warning('Failed to connect to Nucleus')
            return False

        self.nucleus_id = self.nucleus_driver.connection.nucleus_id
        self.nucleus_firmware = self.nucleus_driver.connection.firmware_version

        logging.info(f'Nucleus connected: {self.nucleus_driver.connection.get_connection_status()}')
        logging.info(f'Nucleus ID:        {self.nucleus_id}')
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

        reply = self.nucleus_driver.commands.set_ahrs(ds="OFF")
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETAHRS,DS="OFF": {reply}')

        reply = self.nucleus_driver.commands.set_bt(wt="OFF", ds="ON")
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETBT,WT="OFF",DS="ON": {reply}')

        reply = self.nucleus_driver.commands.set_alti(ds="OFF")
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETALTI,DS="OFF": {reply}')

        reply = self.nucleus_driver.commands.set_cur_prof(ds="OFF")
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETCURPROF,DS="OFF": {reply}')

        reply = self.nucleus_driver.commands.set_cur_prof(ds="OFF")
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETCURPROF,DS="OFF": {reply}')

        command = b'SETINST,TYPE="NAV"\r\n'
        self.nucleus_driver.connection.write(command)
        reply = self.nucleus_driver.commands._handle_reply(command=command, terminator=b'OK\r\n')
        if b'OK\r\n' not in reply:
            logging.warning(f'Did not receive OK when sending SETINST,TYPE="NAV": {reply}')

    def wait_for_heartbeat(self):
        """
        Waits for a valid heartbeat to Mavlink2Rest
        """

        def get_heartbeat():

            response = requests.get(MAVLINK2REST_URL + "/mavlink" + vehicle_path + '/HEARTBEAT')

            if response.status_code != 200:
                logging.warning(f'HEARTBEAT request did not respond with 200: {response.status_code}')
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

        for _ in range(21):
            if get_heartbeat():
                break
        else:
            logging.warning('Failed to detect heartbeat')
            return False

        logging.info('Heartbeat detected')
        return True

    def handle_config_parameters(self):

        parameter_change = False

        for parameter in self.CONFIG_PARAMETERS.keys():

            response = self.get_parameter(parameter)

            if response.status_code == 200 and self.CONFIG_PARAMETERS[parameter]['value'] - 0.1 <= response.json()['message']['param_value'] <= self.CONFIG_PARAMETERS[parameter]['value'] + 0.1:
                continue

            parameter_change = True

            response = self.set_parameter(parameter, self.CONFIG_PARAMETERS[parameter]['value'], self.CONFIG_PARAMETERS[parameter]['type'])

            if response.status_code == 200:
                logging.warning(f'{parameter} set to {self.CONFIG_PARAMETERS[parameter]["value"]}')
            else:
                logging.warning(f'Failed to set {parameter} to {self.CONFIG_PARAMETERS[parameter]["value"]}')

        return parameter_change

    def set_pid_parameters(self):

        for parameter in self.PID_PARAMETERS.keys():

            response = self.set_parameter(parameter, self.PID_PARAMETERS[parameter]['value'], self.PID_PARAMETERS[parameter]['type'])

            if response.status_code == 200:
                logging.warning(f'{parameter} set to {self.PID_PARAMETERS[parameter]["value"]}')
            else:
                logging.warning(f'Failed to set {parameter} to {self.PID_PARAMETERS[parameter]["value"]}')

    def start_nucleus(self):

        if b'OK\r\n' not in self.nucleus_driver.start_measurement():
            logging.warning('Failed to start Nucleus')

    def stop_nucleus(self):

        if b'OK\r\n' in self.nucleus_driver.stop():
            logging.warning('Nucleus was already running. Nucleus is now stopped')

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

        response = requests.post(MAVLINK2REST_URL + "/mavlink", json=vision_position_delta)

        if response.status_code == 200:
            logging.debug(f'VISION_POSITION_DELTA\r\nangle_delta: {angle_delta}\r\nposition_delta: {position_delta}\r\nconfidence: {confidence}\r\ndt: {dt}')
        else:
            logging.warning(f'VISION_POSITION_DELTA packet did not respond with 200: {response.status_code} - {response.text}')

    def send_vision_speed_estimate(self, velocity, timestamp):

        try:
            vision_speed_estimate = {
                "header": {
                    "system_id": 255,
                    "component_id": 0,
                    "sequence": 0},
                "message": {
                    "type": "VISION_SPEED_ESTIMATE",
                    "usec": timestamp,
                    "x": velocity[0],
                    "y": velocity[1],
                    "z": velocity[2],
                    "covariance": [0.0 for _ in range(9)],
                    "reset_counter": 0
                }
            }

        except IndexError:

            logging.warning('Failed to create VISION_POSITION_DELTA packet')
            return

        response = requests.post(MAVLINK2REST_URL + "/mavlink", json=vision_speed_estimate)

        if response.status_code == 200:
            logging.debug(f'VISION_SPEED_ESTIMATE\r\nvelocity: {velocity}\r\ntimestamp: {timestamp}\r\n')
        else:
            logging.info(f'VISION_SPEED_ESTIMATE packet did not respond with 200: {response.status_code} - {response.text}')

    def send_global_vision_position_estimate(self, position, orientation, timestamp):

        try:
            global_vision_position_estimate = {
                "header": {
                    "system_id": 255,
                    "component_id": 0,
                    "sequence": 0},
                "message": {
                    "type": "GLOBAL_VISION_POSITION_ESTIMATE",
                    "usec": timestamp,
                    "x": position[0],
                    "y": position[1],
                    "z": position[2],
                    "roll": orientation[0],
                    "pitch": orientation[1],
                    "yaw": orientation[2],
                    "covariance": [0.0 for _ in range(21)],
                    "reset_counter": 0
                }
            }

        except IndexError:

            logging.warning('Failed to create VISION_POSITION_DELTA packet')
            return

        response = requests.post(MAVLINK2REST_URL + "/mavlink", json=global_vision_position_estimate)

        if response.status_code == 200:
            logging.debug(f'GLOBAL_VISION_POSITION_ESTIMATE\r\nposition: {position}\r\norientation: {orientation}\r\ntimestamp: {timestamp}\r\n')
        else:
            logging.info(f'GLOBAL_VISION_POSITION_ESTIMATE packet did not respond with 200: {response.status_code} - {response.text}')

    def run(self):

        self.load_settings()
        self.wait_for_cableguy()

        if not self.discover_nucleus():
            return

        if not self.connect_nucleus():
            return

        self.stop_nucleus()
        self.setup_nucleus()

        if not self.wait_for_heartbeat():
            return

        if self.handle_config_parameters():
            logging.warning('Parameters had to be set to enable Nucleus integration into ROV. Power cycle the ROV for parameters to take effect!')
            return

        self.set_pid_parameters()

        time.sleep(1)  # TODO
        self.start_nucleus()

        logging.info("Nucleus driver running")

        while self.thread_running:

            packet = self.nucleus_driver.read_packet()

            if packet is None:
                time.sleep(0.005)
                continue

            if not self.enable_nucleus_input:
                time.sleep(0.005)
                continue

            if packet['id'] == 0xb4:

                fom_x = packet['fomX']
                fom_y = packet['fomY']
                fom_z = packet['fomZ']

                fom = max(fom_x, fom_y, fom_z)

                confidence = (10 - fom) * 10
                #confidence = 100  # TODO: REMOVE

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

                if self.enable_vision_position_delta:
                    self.send_vision_position_delta(position_delta=[dx, dy, dz], angle_delta=delta_orientation, confidence=int(confidence), dt=int(dt * 1e6))

                elif self.enable_vision_speed_estimate:
                    self.send_vision_speed_estimate(velocity=[velocity_x, velocity_y, velocity_z], timestamp=int(timestamp))

            if packet['id'] == 0xdc:

                orientation = list()
                orientation.append(self.d2r(packet['ahrsData.roll']))
                orientation.append(self.d2r(packet['ahrsData.pitch']))
                orientation.append(self.d2r(packet['ahrsData.heading']))

                position = list()
                position.append(packet['positionFrameX'])
                position.append(packet['positionFrameY'])
                position.append(packet['positionFrameZ'])

                timestamp = (packet['timeStamp'] + packet['microSeconds'] * 1e-6) * 1e6

                self.orientation_current = orientation

                if self.enable_global_vision_position_estimate:
                    self.send_global_vision_position_estimate(position=position, orientation=orientation, timestamp=int(timestamp))


if __name__ == "flask_app":

    nucleus_driver = NucleusDriver()

    packet_args = reqparse.RequestParser()
    packet_args.add_argument('size', type=int, help="Number of packets returned by packets")

    rov_link = RovLink(driver=nucleus_driver)
    rov_link.start()

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

        nucleus_driver.set_tcp_configuration(host=host)

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
            if parameter_type in ["MAV_PARAM_TYPE_UINT8", "MAV_PARAM_TYPE_INT8"]:
                parameter_value = int(parameter_value)
            elif parameter_type in ["MAV_PARAM_TYPE_REAL32"]:
                parameter_value = float(parameter_value)
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
        VISO_TYPE = 1.0
        EK3_GPS_TYPE = 3.0  # outdated?
        GPS_TYPE = 1.0  # replacing EK3_GPS_TYPE?
        EK3_SRC1_POSXY = 3.0
        EK3_SRC1_VELXY = 3.0
        EK3_SRC1_POSZ = 1.0

        SERIAL0_PROTOCOL = 2
        PSC_POSXY_P = 1
        PSC_POSZ_P = 3
        PSC_VELXY_P = 1
        PSC_VELXY_I = 0.5
        PSC_VELXY_D = 0
        PSC_VELZ_P = 8

        return

    @app.route('/mavlink/handle_nucleus_inputs')
    def mavlink_enable_nucleus_input():

        enable_nucleus_input = request.args.get('enable_nucleus_input')
        enable_vision_position_delta = request.args.get('enable_vision_position_delta')
        enable_vision_speed_estimate = request.args.get('enable_vision_speed_estimate')
        enable_global_vision_position_estimate = request.args.get('enable_global_vision_position_estimate')

        status = 200

        response_dict = dict()

        if enable_nucleus_input is not None:
            if enable_nucleus_input.lower() in ["true", '1']:
                rov_link.enable_nucleus_input = True
                response_dict.update({'enable_nucleus_input': True})
            elif enable_nucleus_input.lower() in ["false", '0']:
                rov_link.enable_nucleus_input = False
                response_dict.update({'enable_nucleus_input': False})
            else:
                status = 210
                response_dict.update({'enable_nucleus_input': 'Failed to set value'})
                logging.warning('Failed to parse value for "enable_nucleus_input"')

        if enable_vision_position_delta is not None:
            if enable_vision_position_delta.lower() in ["true", '1']:
                rov_link.enable_vision_position_delta = True
                response_dict.update({'enable_vision_position_delta': True})
            elif enable_vision_position_delta.lower() in ["false", '0']:
                rov_link.enable_vision_position_delta = False
                response_dict.update({'enable_vision_position_delta': False})
            else:
                status = 210
                response_dict.update({'enable_vision_position_delta': 'Failed to set value'})
                logging.warning('Failed to parse value for "enable_vision_position_delta"')

        if enable_vision_speed_estimate is not None:
            if enable_vision_speed_estimate.lower() in ["true", '1']:
                rov_link.enable_vision_speed_estimate = True
                response_dict.update({'enable_vision_speed_estimate': True})
            elif enable_vision_speed_estimate.lower() in ["false", '0']:
                rov_link.enable_vision_speed_estimate = False
                response_dict.update({'enable_vision_speed_estimate': False})
            else:
                status = 210
                response_dict.update({'enable_vision_speed_estimate': 'Failed to set value'})
                logging.warning('Failed to parse value for "enable_vision_speed_estimate"')

        if enable_global_vision_position_estimate is not None:
            if enable_global_vision_position_estimate.lower() in ["true", '1']:
                rov_link.enable_global_vision_position_estimate = True
                response_dict.update({'enable_global_vision_position_estimate': True})
            elif enable_global_vision_position_estimate.lower() in ["false", '0']:
                rov_link.enable_global_vision_position_estimate = False
                response_dict.update({'enable_global_vision_position_estimate': False})
            else:
                status = 210
                response_dict.update({'enable_global_vision_position_estimate': 'Failed to set value'})
                logging.warning('Failed to parse value for "enable_global_vision_position_estimate"')

        response = jsonify(response_dict)
        response.status_code = status

        return response

    def set_parameter(parameter_id, parameter_value, parameter_type):

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

                for _ in range(10):

                    try:
                        time.sleep(0.01)  # It typically takes this amount of time for PARAM_VALUE to change

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

        timestamp = get_param_value_timestamp()

        parameter = set_through_mavlink(param_value_pre_timestamp=timestamp)

        if parameter.status_code != 200:
            return parameter

        parameter = check_parameter(parameter.json())

        return parameter

    def get_parameter(parameter_id):

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

                for _ in range(10):

                    try:
                        time.sleep(0.01)  # It typically takes this amount of time for PARAM_VALUE to change

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

            #param_value = json.dumps(param_value)
            param_value = jsonify(param_value)
            if parameter_id_status:
                param_value.status_code = 200
            else:
                param_value.status_code = 210

            return param_value

        timestamp = get_param_value_timestamp()

        parameter = get_from_mavlink(param_value_pre_timestamp=timestamp)

        if parameter.status_code != 200:
            return parameter

        parameter = check_parameter(parameter.json())

        return parameter
