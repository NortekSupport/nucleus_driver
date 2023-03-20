from flask import Flask, request, jsonify, render_template, send_from_directory
from flask_restful import Api
import logging
from rov_link import RovLink
import os

from nucleus_driver import NucleusDriver

logging.basicConfig(level=logging.DEBUG)

NUCLEUS_IP = os.environ["NUCLEUS_IP"]

MAVLINK2REST_URL = "http://127.0.0.1/mavlink2rest"  # TODO: Fix

if __name__ == "flask_app":

    nucleus_driver = NucleusDriver()
    nucleus_driver.set_tcp_configuration(host=NUCLEUS_IP)

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
        
        for parameter in psc.keys():

            if psc[parameter] is None:
                logging.warning(f'SKIPPING {parameter} since it is NONE')
                continue

            response = rov_link.set_parameter(parameter_id=parameter, parameter_value=psc[parameter], parameter_type="MAV_PARAM_TYPE_REAL32")

            if response.status_code != 200:
                logging.warning(f'Failed to set parameter value for {parameter}')
        
        rov_link.read_pid_parameters()

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
        status.update({'logging': nucleus_driver.logger._logging})
        status.update(rov_link.config_parameters)
        status.update(rov_link.pid_parameters)
        status.update(rov_link.vision_position_delta_packet_counter)

        print('message triggered from RovLink')
        print(status)
        return jsonify(status)

    @app.route("/handle_logging", methods=['POST'])
    def handle_logging():
        
        logging = request.form.get("logging", None, type=str)

        if logging == 'start':
            status = rov_link.start_logging()
        else:
            status = rov_link.stop_logging()

        return jsonify(status)

    @app.route("/download_log_file", methods=['GET'])
    def download_log_file():
        
        if nucleus_driver.logger._logging:
            logging.warning('Can not download file while logging')
            return

        path = rov_link.get_download_path()
        log_file = rov_link.get_download_file_name()

        return

        if path is None:
            logging.warning('Could not find path to download file')
            return

        return send_from_directory(directory=path, filename=log_file)


    ''' Future support
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
    '''

    ''' Future support
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
    '''

    ''' Future support
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
    '''

    ''' Future support
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
