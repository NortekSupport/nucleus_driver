import pytest
from threading import Thread
import rclpy
from rclpy.executors import SingleThreadedExecutor
import logging
import time
from queue import Queue

from nucleus_node import NucleusNode
from clients.command import ClientCommand
from clients.connect_serial import ClientConnectSerial
from clients.connect_tcp import ClientConnectTcp
from clients.disconnect import ClientDisconnect
from clients.start import ClientStart
from clients.stop import ClientStop
from subscribers.ahrs_packets import SubscriberAhrsPackets


class TestNucleusNode:

    @pytest.fixture(scope='module', autouse=True)
    def init_rclpy(self):

        rclpy.init()

        yield

        rclpy.shutdown()

    @pytest.fixture(scope='module', autouse=True)
    def run_nucleus_node(self, init_rclpy):

        nucleus_node = NucleusNode()

        nucleus_node_executor = SingleThreadedExecutor()
        nucleus_node_executor.add_node(nucleus_node)
        
        nucleus_node_thread = Thread(target=nucleus_node_executor.spin)
        nucleus_node_thread.start()

        yield

        nucleus_node_executor.shutdown()
        nucleus_node_thread.join(timeout=1.0)

    @pytest.mark.dependency(name='connect_serial')
    def test_client_connect_serial(self, run_nucleus_node):

        client = ClientConnectSerial()
        
        executor = SingleThreadedExecutor()
        executor.add_node(client)
        
        response = client.send_request(serial_port='/dev/ttyUSB0', timeout_sec=1)

        executor.shutdown()
        client.destroy_node()

        assert response.status is True

    @pytest.mark.dependency(name='command', depends=['connect_serial'])
    def test_client_command(self, run_nucleus_node):

        client = ClientCommand()
        
        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(command='ID', timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        assert 'Nucleus1000' in response.reply

    @pytest.mark.dependency(name='disconnect_serial', depends=['connect_serial'])
    def test_client_disconnect_serial(self, run_nucleus_node):

        client = ClientDisconnect()
        
        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        assert response.status is True

    
    @pytest.mark.dependency(name='connect_tcp')
    def test_client_connect_tcp(self, run_nucleus_node):

        client = ClientConnectTcp()
        
        executor = SingleThreadedExecutor()
        executor.add_node(client)
        
        response = client.send_request(host='NORTEK-300004.local', password='nortek', timeout_sec=1)

        executor.shutdown()
        client.destroy_node()

        assert response.status is True

    @pytest.mark.dependency(name='start', depends=['connect_tcp'])
    def test_client_start(self, run_nucleus_node):

        client = ClientStart()
        
        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        assert 'OK' in response.reply

    @pytest.mark.dependency(name='stop', depends=['connect_tcp', 'start'])
    def test_subscriber_ahrs(self, run_nucleus_node):
        
        ahrs_packets = Queue()

        def callback_function(data):
            
            ahrs_packets.put(data)

        subscriber = SubscriberAhrsPackets(callback_function=callback_function)

        executor = SingleThreadedExecutor()
        executor.add_node(subscriber)

        subscriber_thread = Thread(target=subscriber.subscribe)
        subscriber_thread.start()

        time.sleep(1)

        executor.shutdown()
        subscriber_thread.join(timeout=1.0)

        assert not ahrs_packets.empty()

        while not ahrs_packets.empty():
             
            data = ahrs_packets.get()

            assert isinstance(data.roll, float)
            assert isinstance(data.pitch, float)
            assert isinstance(data.heading, float)


    @pytest.mark.dependency(name='stop', depends=['connect_tcp', 'start'])
    def test_client_stop(self, run_nucleus_node):

        client = ClientStop()
        
        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)
        
        executor.shutdown()
        client.destroy_node()

        assert 'OK' in response.reply