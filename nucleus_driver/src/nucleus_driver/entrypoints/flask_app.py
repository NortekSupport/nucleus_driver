import time

from flask import Flask, json, request, jsonify
from flask_restful import Api, reqparse
from threading import Thread
import requests
import json
import logging


from nucleus_driver import NucleusDriver


#DOCKER_HOST = '0.0.0.0'
#DOCKER_PORT = 5000

#NUCLEUS_HOST = 'Nucleus-300004.local'

logging.basicConfig(level=logging.DEBUG)

#HOSTNAME = 'NORTEK-300004.local'
HOSTNAME = '192.168.2.201'

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


@app.route('/mavlink/get_param', methods=['GET'])
def mavlink_get_param():

    param_get = requests.get(MAVLINK2REST_URL + "/helper/mavlink?name=PARAM_SET")

    param_get_text = param_get.text

    param_json_laods = json.loads(param_get_text)

    logging.info(f'param_get: {param_get}\r\n')
    logging.info(f'param_get_text: {param_get_text}\r\n')
    logging.info(f'param_json_loads: {param_json_laods}\r\n')

    return param_get


@app.route('/mavlink/get_parameter', methods=['GET'])
def mavlink_get_parameter():

    # Check if a parameter_id is given
    parameter_id = request.args.get('parameter_id')

    if parameter_id is None:
        response = jsonify({'message': 'parameter_id must be specified'})
        response.status_code = 400
        return response

    parameter_id = parameter_id.upper()

    # Extract timestamp of current PARAM_VALUE
    param_value_pre_timestamp = None
    try:
        param_value_pre = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")
        param_value_pre_timestamp = param_value_pre.json()["status"]["time"]["last_update"]

    except Exception as e:
        logging.warning(f'Unable to obtain PARAM_VALUE before PARAM_REQUEST_READ: {e}')

    # Get PARAM_VALUE
    param_value = jsonify({'message': 'no PARAM_VALUE obtained'})
    try:
        data = json.loads(requests.get(MAVLINK2REST_URL + "/helper/mavlink?name=PARAM_REQUEST_READ").text)

        for index, char in enumerate(parameter_id):
            data['message']['param_id'][index] = char

        data['message']['param_index'] = -1
        data['message']['target_system'] = 1
        data['message']['target_component'] = 1

        post_result = requests.post(MAVLINK2REST_URL + "/mavlink", json=data)

        # Check if PARAM_REQUEST_READ responded with 200
        if post_result.status_code != 200:
            param_value = jsonify({'message': f'PARAM_REQUEST_READ command did not respond with 200: {post_result.status_code}'})
            param_value.status_code = 400
            return param_value

        # Read PARAM_VALUE until it has been updated
        for _ in range(10):

            time.sleep(0.01)  # It typically takes this amount of time for PARAM_VALUE to change

            param_value = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")

            logging.info(f'\r\n\r\n{param_value.json()}\r\n')

            param_value_timestamp = param_value.json()["status"]["time"]["last_update"]

            if param_value_pre_timestamp is None or param_value_pre_timestamp != param_value_timestamp:
                break

        param_value = param_value.json()

    except Exception as error:
        param_value = jsonify({'message': f'Failed to obtain parameter {parameter_id}: {error}'})
        param_value.status_code = 400
        return param_value

    # Extract PARAM_VALUE id (name)
    response_parameter_id = ''
    for char in param_value['message']['param_id']:
        if char == '\u0000':
            break

        response_parameter_id += char

    # Check if obtained PARAM_VALUE is the same as requested
    if response_parameter_id != parameter_id:
        param_value['WARNING'] = {'PARAMETER_NAME': 'The obtained parameter is not the same as the requested parameter',
                                  'requested_parameter': parameter_id,
                                  'obtained_parameter': response_parameter_id
                                  }
        param_value = jsonify(param_value)
        param_value.status_code = 210

    try:
        logging.info(f'PARAM_VALUE: {response_parameter_id} = {param_value["message"]["param_value"]}')
    except TypeError:
        logging.warning('Failed to display PARAM_VALUE')

    return param_value


@app.route('/mavlink/set_parameter', methods=['GET'])
def mavlink_set_parameter():

    PARAMETER_TYPES = ["MAV_PARAM_TYPE_UINT8"]

    AHRS_EKF_TYPE = 3.0
    EK2_ENABLE = 0.0
    EK3_ENABLE = 1.0
    VISO_TYPE = None
    EK3_GPS_TYPE = 3.0
    EK3_SRC1_POSXY = None
    EK3_SRC1_VELXY = None
    EK3_SRC1_POSZ = None

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

    # Extract timestamp of current PARAM_VALUE
    param_value_pre_timestamp = None
    try:
        param_value_pre = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")
        param_value_pre_timestamp = param_value_pre.json()["status"]["time"]["last_update"]

    except Exception as e:
        logging.warning(f'Unable to obtain PARAM_VALUE before PARAM_REQUEST_READ: {e}')

    try:
        data = json.loads(requests.get(MAVLINK2REST_URL + "/helper/mavlink?name=PARAM_SET").text)

        for index, char in enumerate(parameter_id):
            data["message"]["param_id"][index] = char

        data["message"]["param_type"] = {"type": parameter_type}
        data["message"]["param_value"] = parameter_value

        post_result = requests.post(MAVLINK2REST_URL + "/mavlink", json=data)

        # Check if PARAM_REQUEST_READ responded with 200
        if post_result.status_code != 200:
            param_value = jsonify({'message': f'PARAM_REQUEST_READ command did not respond with 200: {post_result.status_code}'})
            param_value.status_code = 400
            return param_value

        for _ in range(10):

            time.sleep(0.01)  # It typically takes this amount of time for PARAM_VALUE to change

            param_value = requests.get(MAVLINK2REST_URL + "/mavlink/vehicles/1/components/1/messages/PARAM_VALUE")

            logging.info(f'\r\nRECEIVED_PARAM:\r\n{param_value.json()}\r\n')

            param_value_timestamp = param_value.json()["status"]["time"]["last_update"]

            if param_value_pre_timestamp is None or param_value_pre_timestamp != param_value_timestamp:
                break

        logging.info(f'\r\nPARAM_VALUE.json()\r\n')  # TODO: Remove
        param_value = param_value.json()
        logging.info(f'\t{param_value}\r\n')  # TODO: Remove

    except Exception as error:
        param_value = jsonify({'message': f'Failed to set parameter {parameter_id}: {error}'})
        param_value.status_code = 400
        return param_value

    logging.info(f'\r\nLOOPING THROUGH PARAM_VALUE[MESSAGE][PARAM_ID]\r\n')  # TODO: Remove
    # Extract PARAM_VALUE id (name)
    response_parameter_id = ''
    for char in param_value['message']['param_id']:
        logging.info(f'\tlooping: {char}\r\n')  # TODO: Remove
        if char == '\u0000':
            break

        response_parameter_id += char

    logging.info(f'\r\nCOMPARING PARAM ID\r\n')  # TODO: Remove

    valid_response = True

    # Check if obtained PARAM_ID is the same as requested
    if response_parameter_id != parameter_id:
        logging.info(f'\tPARAM_ID MATCHES\r\n')  # TODO: Remove
        param_value['WARNING'] = {'param_id': 'The obtained parameter is not the same as the requested parameter',
                                  'requested_parameter': parameter_id,
                                  'obtained_parameter': response_parameter_id
                                  }
        valid_response = False

    logging.info(f'\r\nCHECKING PÅARAM VALUE: {param_value["message"]}\r\n')  # TODO: Remove
    logging.info(f'\r\nCHECKING PÅARAM VALUE: {param_value["message"]["param_value"]}\r\n')  # TODO: Remove
    # Check if obtained PARAM_VALUE is the same as set
    try:
        if int(param_value['message']['param_value']) != int(parameter_value):
            param_value['WARNING'] = {'param_value': 'The obtained parameter is not the same as the requested parameter',
                                      'requested_parameter': parameter_id,
                                      'obtained_parameter': response_parameter_id
                                      }
            valid_response = False

    except Exception as e:
        logging.warning(f'\r\n\r\n{param_value}\r\n\r\n')

    try:
        logging.info(f'PARAM_VALUE: {response_parameter_id} = {param_value["message"]["param_value"]}')
    except TypeError:
        logging.warning('Failed to display PARAM_VALUE')

    param_value = jsonify(param_value)
    if valid_response:
        param_value.status_code = 200
    else:
        param_value.status_code = 210

    return param_value


class RovLink:

    def __init__(self):

        self.thread = Thread()
        self.thread_running = False

        self.orientation_current = None
        self.orientation_previous = None

        self.hostname = None
        self.nucleus_id = None
        self.nucleus_firmware = None

        self.timeout = 1

    def reconnect(self):

        # TODO: Add reconnect funtionality

        pass

    def load_settings(self):

        # TODO: How to implement settings? .config/dvl/settings.json?

        self.hostname = HOSTNAME

    def discover_nucleus(self):

        # TODO: Implement funcitonality for discovering the Nucleus?

        pass

    def connect_nucleus(self):

        # TODO: implement an active search algorithm before connecting?

        nucleus_driver.set_tcp_configuration(host=self.hostname)

        if not nucleus_driver.connect(connection_type='TCP'):
            return False

        self.nucleus_id = nucleus_driver.connection.nucleus_id
        self.nucleus_firmware = nucleus_driver.connection.firmware_version

        logging.info(f'Nucleus connected: {nucleus_driver.connect.get_connection_status()}')
        logging.info(f'Nucleus ID: {self.nucleus_id}')
        logging.info(f'Nucleus firmware: {self.nucleus_firmware}')

        return True

    def wait_for_vehicle(self):
        """
        Waits for a valid heartbeat to Mavlink2Rest
        """
        vehicle = 1
        component = 1

        vehicle_path = f"/vehicles/{vehicle}/components/{component}/messages"
        while not requests.get(MAVLINK2REST_URL + "/mavlink" + vehicle_path + '/HEARTBEAT'):
            logging.info('waiting for vehicle heartbeat')
            time.sleep(1)

    def setup_parameters(self):

        def set_parameter(parameter_id, parameter_type, parameter_value):

            try:

                data = json.loads(requests.get(MAVLINK2REST_URL + "/helper/mavlink?name=PARAM_SET").text)

                for index, char in enumerate(parameter_id):

                    data['message']['param_id'][index] = char

                data['message']['param_type'] = {'type': parameter_type}
                data['message']['param_value'] = parameter_value

                result = requests.post(MAVLINK2REST_URL + "/mavlink", json=data)

                logging.info(result)

                return result.status_code == 200

            except Exception as error:
                logging.warning(f"Error setting parameter '{parameter_id}': {error}")
                return False

        response = set_parameter("AHRS_EKF_TYPE", "MAV_PARAM_TYPE_UINT8", 3)
        logging.info(f'AHRS_EKF_TYPE: {response}')

        response = set_parameter("EK2_ENABLE", "MAV_PARAM_TYPE_UINT8", 0)
        logging.info(f'EK2_ENABLE: {response}')

        response = set_parameter("EK3_ENABLE", "MAV_PARAM_TYPE_UINT8", 1)
        logging.info(f'EK3_ENABLE: {response}')

        response = set_parameter("VISO_TYPE", "MAV_PARAM_TYPE_UINT8", 1)
        logging.info(f'VISO_TYPE: {response}')

        response = set_parameter("EK3_GPS_TYPE", "MAV_PARAM_TYPE_UINT8", 3)
        logging.info(f'EK3_GPS_TYPE: {response}')

        response = set_parameter("EK3_SRC1_POSXY", "MAV_PARAM_TYPE_UINT8", 6)  # EXTNAV
        logging.info(f'EK3_SRC1_POSXY: {response}')

        response = set_parameter("EK3_SRC1_VELXY", "MAV_PARAM_TYPE_UINT8", 6)  # EXTNAV
        logging.info(f'EK3_SRC1_VELXY: {response}')

        response = set_parameter("EK3_SRC1_POSZ", "MAV_PARAM_TYPE_UINT8", 1)
        logging.info(f'EK3_SRC1_POSZ: {response}')

    def send_vision_position_delta(self, position_delta, angle_delta, confidence, dt):

        try:
            vision_position_delta = {
                "header": {
                    "system_id": 255,
                    "component_id": 0,
                    "sequence": 0
                },
                "message": {
                    "type": "VISION_POSITION_DELTA",
                    "time_usec": 0,
                    "time_delta_usec": dt,
                    "angle_delta": [
                        angle_delta[0],
                        angle_delta[1],
                        angle_delta[2]
                    ],
                    "position_delta": [
                        position_delta[0],
                        position_delta[1],
                        position_delta[2]
                    ],
                    "confidence": confidence
                }
            }

        except IndexError:

            print('Failed to send VISION_POSITION_DELTA packet')
            return

        response = requests.post(MAVLINK2REST_URL + "/mavlink", data=vision_position_delta)

    def run(self):

        self.load_settings()
        self.discover_nucleus()  # TODO
        self.connect_nucleus()
        self.wait_for_vehicle()
        #self.setup_mavlink()  # TODO
        self.setup_parameters()
        time.sleep(1)  # TODO
        logging.debug("Running")  # TODO
        self.packet_received_timestamp = time.time()  # TODO

        connected = True  # TODO


        while self.thread_running:

            packet = nucleus_driver.read_packet()

            if packet is None:
                if time.time() - self.packet_received_timestamp > self.timeout:
                    logging.warning('Timeout, restarting')
                    self.reconnect()
                time.sleep(0.005)
                continue

            self.packet_received_timestamp = time.time()

            if packet['id'] == 0xb4:

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
                    dt = timestamp - self.timestamp_prevous
                    self.timestamp_previous = timestamp

                dx = velocity_x * dt
                dy = velocity_y * dt
                dz = velocity_z * dt

                delta_orientation = list()

                if self.orientation_current is None:
                    delta_orientation = [0, 0, 0]

                elif self.timestamp_previous is None:
                    delta_orientation = [0, 0, 0]
                    self.orientation_previous = self.orientation_current

                else:
                    for (angle_current, angle_previous) in zip(self.orientation_current - self.orientation_previous):
                        delta_orientation.append(angle_current - angle_previous)

                    self.orientation_previous = self.orientation_current

                self.send_vision_position_delta(position_delta=[dx, dy, dz], angle_delta=delta_orientation, confidence=confidence, dt=dt)

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

    print('INITIATED DRIVER')