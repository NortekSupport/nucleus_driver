import time
from flask import Flask, json, request, jsonify, render_template
from flask_restful import Api, reqparse
#from threading import Thread
import requests
import logging
#import socket
#from queue import Queue
#from datetime import datetime
from rov_link import RovLink

from nucleus_driver import NucleusDriver

logging.basicConfig(level=logging.DEBUG)

#HOSTNAME = 'NORTEK-300046.local'
HOSTNAME = '192.168.2.201'
#PORT = 5000  # TODO: Is this the port?

MAVLINK2REST_URL = "http://127.0.0.1/mavlink2rest"  # TODO: Fix


if __name__ == "flask_app":

    nucleus_driver = NucleusDriver()

    rov_link = RovLink(driver=nucleus_driver)
    rov_link.start()

    app = Flask(__name__)
    api = Api(app)

    @app.route("/", defaults={"js": "home"})
    @app.route("/<any(home, pid_parameters, controller_parameters):js>")
    def index(js):
        print(f'js: {js}')
        return render_template(f"{js}.html", js=js)
    
    @app.route("/write_pid_parameters", methods=["POST"])
    def write_pid_parameters():
        psc = dict()
        psc.update({'PSC_POSXY_P': request.form.get("PSC_POSXY_P", None, type=float)})
        psc.update({'PSC_POSZ_P': request.form.get("PSC_POSZ_P", None, type=float)})
        psc.update({'PSC_VELXY_P': request.form.get("PSC_VELXY_P", None, type=float)})
        psc.update({'PSC_VELXY_I': request.form.get("PSC_VELXY_I", None, type=float)})
        psc.update({'PSC_VELXY_D': request.form.get("PSC_VELXY_D", None, type=float)})
        psc.update({'PSC_VELZ_P': request.form.get("PSC_VELZ_P", None, type=float)})
        
        # TODO: Uncomment to actually send values to mavlink
        
        for parameter in psc.keys():

            if psc[parameter] is None:
                logging.warning(f'SKIPPING {parameter} since it is NONE')
                continue


            response = rov_link.set_parameter(parameter_id=parameter, parameter_value=psc[parameter], parameter_type="MAV_PARAM_TYPE_REAL32")

            if response.status_code != 200:
                logging.warning(f'Failed to set parameter value for {parameter}')
        
        return jsonify(result='Parameters set')

    @app.route("/write_controller_parameters", methods=["POST"])
    def write_controller_parameters():
        controller = dict()
        controller.update({'AHRS_EKF_TYPE': request.form.get("AHRS_EKF_TYPE", None, type=float)})
        controller.update({'EK2_ENABLE': request.form.get("EK2_ENABLE", None, type=float)})
        controller.update({'EK3_ENABLE': request.form.get("EK3_ENABLE", None, type=float)})
        controller.update({'VISO_TYPE': request.form.get("VISO_TYPE", None, type=float)})
        controller.update({'GPS_TYPE': request.form.get("GPS_TYPE", None, type=float)})
        controller.update({'EK3_SRC1_POSXY': request.form.get("EK3_SRC1_POSXY", None, type=float)})
        controller.update({'EK3_SRC1_VELXY': request.form.get("EK3_SRC1_VELXY", None, type=float)})
        controller.update({'EK3_SRC1_POSZ': request.form.get("EK3_SRC1_POSZ", None, type=float)})
        controller.update({'SERIAL0_PROTOCOL': request.form.get("SERIAL0_PROTOCOL", None, type=float)})
        
        # TODO: Uncomment to actually send values to mavlink
        
        for parameter in controller.keys():

            if controller[parameter] is None:
                logging.warning(f'SKIPPING {parameter} since it is NONE')
                continue

            response = rov_link.set_parameter(parameter_id=parameter, parameter_value=controller[parameter], parameter_type="MAV_PARAM_TYPE_REAL32")

            if response.status_code != 200:
                logging.warning(f'Failed to set parameter value for {parameter}')
        
        correct_values = rov_link.read_config_parameters()


        if correct_values:
            rov_link.status['controller_parameters'] = 'Restart ROV'
            status = 'Correct parameters set. Restart ROV'
        else:
            rov_link.status['controller_parameters'] = 'Incorrect'
            status = 'Incorrect parameters set. Restart ROV'

        return jsonify(result=status)

    @app.route("/toggle_driver", methods=["POST"])
    def toggle_driver():

        enable = request.form.get("toggle_driver", None, type=str)

        if enable == 'enable':
            rov_link.set_enable_nucleus_input(enable=True)
        else:
            rov_link.set_enable_nucleus_input(enable=False)

        enabled = rov_link.get_enable_nucleus_input()

        print(f'enable driver: {enabled}')
        return jsonify(result=enabled)

    @app.route("/get_status", methods=['GET'])
    def get_status():
        
        status = rov_link.status
        status.update({'enable_nucleus_input': rov_link._enable_nucleus_input})

        print('message triggered from RovLink')
        print(status)
        return jsonify(status)

    @app.route("/nucleus_driver/start", methods=['GET'])
    def nucleus_driver_start():

        if not nucleus_driver.connection.get_connection_status():
            return jsonify({'reply': 'Nucleus not connected!'})

        reply = nucleus_driver.start_measurement()

        try:
            reply = reply[0].decode()
        except IndexError:
            reply = 'index error'

        return jsonify({'reply': reply})

    @app.route("/nucleus_driver/stop", methods=['GET'])
    def nucleus_driver_stop():

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
    def nucleus_driver_get_packet():

        if not nucleus_driver.connection.get_connection_status():
            return jsonify({'reply': 'Nucleus not connected!'})

        size = request.args.get('size')

        packet_list = list()

        if size is not None:

            for _ in range(int(size)):

                reply = rov_link.read_packet()

                if reply is None:
                    break

                packet_list.append(reply)

        else:
            reply = rov_link.read_packet()

            if reply is not None:
                packet_list.append(reply)

        return jsonify({'packets': packet_list})

    ''' Legacy
    @app.route("/nucleus_driver/get_all", methods=['GET'])
    def nucleus_driver_get_all():

        if not nucleus_driver.connection.get_connection_status():
            return jsonify({'reply': 'Nucleus not connected!'})

        if nucleus_driver.parser.thread_running:
            return jsonify({'reply': 'Nucleus is running, stop before sending get_all'})

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
    '''

    ''' Legacy
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
    '''

    ''' Legacy
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
    '''

    ''' Legacy
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
    '''

    ''' Legacy
    @app.route('/mavlink/enable_input')
    def mavlink_enable_nucleus_input():

        enable_nucleus_input = request.args.get('enable')

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

        response = jsonify(response_dict)
        response.status_code = status

        return response
    '''

    ''' Legacy
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
    '''
    
    ''' Legacy
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
    '''