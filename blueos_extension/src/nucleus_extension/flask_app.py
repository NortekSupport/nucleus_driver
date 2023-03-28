from flask import Flask, request, jsonify, render_template
from flask_restful import Api
import logging
from rov_link import RovLink
import os

from nucleus_driver import NucleusDriver

logging.basicConfig(level=logging.DEBUG)

#NUCLEUS_IP = os.environ["NUCLEUS_IP"]

if __name__ == "flask_app":

    nucleus_driver = NucleusDriver()
    #nucleus_driver.set_tcp_configuration(host=NUCLEUS_IP)

    rov_link = RovLink(driver=nucleus_driver)
    rov_link.start()

    app = Flask(__name__)
    api = Api(app)

    @app.route("/", defaults={"js": "home"})
    @app.route("/<any(home, pid_parameters, controller_parameters):js>")
    def index(js):
        
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

        return jsonify(result=enabled)
    
    @app.route("/get_status", methods=['GET'])
    def get_status():
        
        status = rov_link.status
        status.update({'enable_nucleus_input': rov_link._enable_nucleus_input})
        status.update({'logging': nucleus_driver.logger._logging})
        status.update(rov_link.config_parameters)
        status.update(rov_link.pid_parameters)
        status.update(rov_link.vision_position_delta_packet_counter)
        status.update({'hostname': rov_link.hostname})

        return jsonify(status)

    @app.route("/set_hostname", methods=['POST'])
    def set_hostname():

        hostname = request.form.get("HOSTNAME", None, type=str)

        print(f'HOSTNAME: {hostname}')

        if hostname is None:
            logging.warning(f'Hostname can not be None')
            jsonify(status='Hostname can not be None')
                            
        #rov_link.hostname = hostname

        rov_link.set_hostname(hostname)

        jsonify(result=f'Hostname set to {hostname}')

    #@app.route("/get_hostname", methods=['GET'])
    #def get_hostname():
    #
    #    jsonify(hostname=rov_link.hostname)


