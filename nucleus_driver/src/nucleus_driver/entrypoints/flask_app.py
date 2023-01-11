from flask import Flask, json, request, jsonify
from flask_restful import Api, reqparse

from nucleus_driver import NucleusDriver


DOCKER_HOST = '0.0.0.0'
DOCKER_PORT = 5000

NUCLEUS_HOST = 'Nucleus-300004.local'

app = Flask(__name__)
api = Api(app)

print('NAME:')
print(__name__)

@app.route("/nucleus_driver/start", methods=['GET'])
def start():

    nucleus_driver.thread.start()

    reply = nucleus_driver.commands.start()
    try:
        reply = reply[0].decode()
    except IndexError:
        reply = 'index error'

    return jsonify({'reply': reply})


@app.route("/nucleus_driver/stop", methods=['GET'])
def stop():

    nucleus_driver.thread.stop()

    reply = nucleus_driver.commands.stop()

    try:
        stop_reply = reply[0][-4:].decode()
    except IndexError:
        stop_reply = 'index error'
    except UnicodeDecodeError:
        stop_reply = 'decode error'

    return jsonify({'reply': stop_reply})


@app.route("/nucleus_driver/get_packet", methods=['GET'])
def get_packet():

    size = request.args.get('size')

    packet_list = list()

    if size is not None:

        for _ in range(int(size)):

            reply = nucleus_driver.parser.read_packet()

            if reply is None:
                break

            packet_list.append(reply)

    else:
        reply = nucleus_driver.parser.read_packet()

        if reply is not None:  # will it become None?
            packet_list.append(reply)

    return jsonify({'packets': packet_list})


@app.route("/nucleus_driver/get_all", methods=['GET'])
def nucleus_driver_get_all():

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


if __name__ == "__main__":

    nucleus_driver = NucleusDriver()
    nucleus_driver.connection.set_tcp_configuration(host=NUCLEUS_HOST)
    nucleus_driver.connection.connect(connection_type='tcp')

    packet_args = reqparse.RequestParser()
    packet_args.add_argument('size', type=int, help="Number of packets returned by packets")

    app.run(debug=True, host=DOCKER_HOST, port=DOCKER_PORT)