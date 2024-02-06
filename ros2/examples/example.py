from threading import Thread
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import logging
import time
from queue import Queue
import sys
from argparse import ArgumentParser

logging.basicConfig(level=logging.INFO)

from nucleus_clients.command import ClientCommand
from nucleus_clients.connect_serial import ClientConnectSerial
from nucleus_clients.connect_tcp import ClientConnectTcp
from nucleus_clients.disconnect import ClientDisconnect
from nucleus_clients.start import ClientStart
from nucleus_clients.field_calibration import ClientFieldCalibration
from nucleus_clients.stop import ClientStop

from nucleus_subscribers.ahrs_packets import SubscriberAhrsPackets
from nucleus_subscribers.altimeter_packets import SubscriberAltimeterPackets
from nucleus_subscribers.bottom_track_packets import SubscriberBottomTrackPackets
from nucleus_subscribers.current_profile_packets import SubscriberCurrentProfilePackets
from nucleus_subscribers.field_calibration_packets import SubscriberFieldCalibrationPackets
from nucleus_subscribers.imu_packets import SubscriberImuPackets
from nucleus_subscribers.ins_packets import SubscriberInsPackets
from nucleus_subscribers.magnetometer_packets import SubscriberMagnetometerPackets
from nucleus_subscribers.water_track_packets import SubscriberWaterTrackPackets


class NucleusCommunication:

    def __init__(self) -> None:

        self.ahrs_queue = Queue()
        self.altimeter_queue = Queue()
        self.bottom_track_queue = Queue()
        self.current_profile_queue = Queue()
        self.field_calibration_queue = Queue()
        self.imu_queue = Queue()
        self.ins_queue = Queue()
        self.mag_queue = Queue()
        self.water_track_queue = Queue()

        self.ahrs_subscriber = SubscriberAhrsPackets(callback_function=self.ahrs_queue.put)
        self.altimeter_subscriber = SubscriberAltimeterPackets(callback_function=self.altimeter_queue.put)
        self.bottom_track_subscriber = SubscriberBottomTrackPackets(callback_function=self.bottom_track_queue.put)
        self.current_profile_subscriber = SubscriberCurrentProfilePackets(callback_function=self.current_profile_queue.put)
        self.field_calibration_subscriber = SubscriberFieldCalibrationPackets(callback_function=self.field_calibration_queue.put)
        self.imu_subscriber = SubscriberImuPackets(callback_function=self.imu_queue.put)
        self.ins_subscriber = SubscriberInsPackets(callback_function=self.ins_queue.put)
        self.mag_subscriber = SubscriberMagnetometerPackets(callback_function=self.mag_queue.put)
        self.water_track_subscriber = SubscriberWaterTrackPackets(callback_function=self.water_track_queue.put)

        self.subscriber_executor = MultiThreadedExecutor()
        self.subscriber_thread = Thread()

    def start_subscribers(self):

        self.subscriber_executor.add_node(self.ahrs_subscriber)
        self.subscriber_executor.add_node(self.altimeter_subscriber)
        self.subscriber_executor.add_node(self.bottom_track_subscriber)
        self.subscriber_executor.add_node(self.current_profile_subscriber)
        self.subscriber_executor.add_node(self.field_calibration_subscriber)
        self.subscriber_executor.add_node(self.imu_subscriber)
        self.subscriber_executor.add_node(self.ins_subscriber)
        self.subscriber_executor.add_node(self.ahrs_subscriber)
        self.subscriber_executor.add_node(self.mag_subscriber)
        self.subscriber_executor.add_node(self.water_track_subscriber)

        self.subscriber_thread = Thread(target=self.subscriber_executor.spin)
        self.subscriber_thread.start()

    def stop_subscribers(self):

        self.subscriber_executor.shutdown()

        self.ahrs_subscriber.destroy_node()
        self.altimeter_subscriber.destroy_node()
        self.bottom_track_subscriber.destroy_node()
        self.current_profile_subscriber.destroy_node()
        self.field_calibration_subscriber.destroy_node()
        self.imu_subscriber.destroy_node()
        self.ins_subscriber.destroy_node()
        self.mag_subscriber.destroy_node()
        self.water_track_subscriber.destroy_node()

        self.subscriber_thread.join(1)

    def command(self, command: str):

        client = ClientCommand()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(command=command, timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        return response.reply
    
    def start(self):

        client = ClientStart()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        return response.reply
    
    def field_calibration(self):

        client = ClientFieldCalibration()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        return response.reply
    
    def stop(self):

        client = ClientStop()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        return response.reply
    
    def connect_serial(self, serial_port: str):

        client = ClientConnectSerial()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(serial_port=serial_port, timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        return response.status
    
    def connect_tcp(self, hostname: str, password: str):

        client = ClientConnectTcp()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(host=hostname, password=password, timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        return response.status
    
    def disconnect(self):

        client = ClientDisconnect()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        return response.status


def main():

    parser = ArgumentParser()
    parser.add_argument('-s', '--serial_port', help='serial_port for serial connection')
    parser.add_argument('-n', '--hostname', help='Hostname fo TCP connection')
    parser.add_argument('-p', '--password', help='Password for tcp connection')

    args = parser.parse_args()

    if args.serial_port is None and args.hostname is None:
        logging.exception('Specify argument for either serial_port or hostname')
        return

    if args.serial_port is None and args.hostname is not None and args.password is None:
        logging.exception('Password argument must be specified for TCP connection')
        return

    rclpy.init()

    nucleus_communication = NucleusCommunication()

    nucleus_communication.start_subscribers()

    if args.serial_port is not None:
        status = nucleus_communication.connect_serial(serial_port=args.serial_port)
        logging.info(f'serial connection status: {status}')

    elif args.hostname is not None and args.password is not None:
        status = nucleus_communication.connect_tcp(hostname=args.hostname, password=args.password)
        logging.info(f'TCP connection status: {status}')
    
    else:
        logging.exception(f'Failed to satisfy argument requirements for connection')
        return

    reply = nucleus_communication.command(command='setahrs,ds="ON"')
    logging.info(f'command reply: {reply}')

    reply = nucleus_communication.start()
    logging.info(f'start reply: {reply}')

    time.sleep(2)

    reply = nucleus_communication.stop()
    logging.info(f'stop reply: {reply}')

    status = nucleus_communication.disconnect()
    logging.info(f'disconnect status: {status}\r\n')

    nucleus_communication.stop_subscribers()

    logging.info('AHRS data')
    while not nucleus_communication.ahrs_queue.empty():

        ahrs_data = nucleus_communication.ahrs_queue.get()
        logging.info(f'roll: {ahrs_data.roll} | pitch: {ahrs_data.pitch} | heading: {ahrs_data.heading}')

    rclpy.shutdown()


if __name__ == '__main__':

    main()