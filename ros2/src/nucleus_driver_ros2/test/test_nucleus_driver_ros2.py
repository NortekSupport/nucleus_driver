import pytest
from threading import Thread
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import logging
import time
from queue import Queue
from array import array

from nucleus_node.nucleus_node import NucleusNode

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


class TestNucleusNode:

    ahrs_queue = Queue()
    altimeter_queue = Queue()
    bottom_track_queue = Queue()
    current_profile_queue = Queue()
    field_calibration_queue = Queue()
    imu_queue = Queue()
    ins_queue = Queue()
    mag_queue = Queue()
    water_track_queue = Queue()

    @pytest.fixture(scope="module", autouse=True)
    def init_rclpy(self):

        rclpy.init()

        yield

        rclpy.shutdown()

    @pytest.fixture(scope="module", autouse=True)
    def run_nucleus_node(self, init_rclpy, nucleus_driver_connection):

        nucleus_node = NucleusNode()

        nucleus_node_executor = SingleThreadedExecutor()
        nucleus_node_executor.add_node(nucleus_node)

        nucleus_node_thread = Thread(target=nucleus_node_executor.spin)
        nucleus_node_thread.start()

        yield

        nucleus_node_executor.shutdown()

        nucleus_node.destroy_node()

        nucleus_node_thread.join(timeout=1.0)

    @pytest.fixture(scope="module", autouse=True)
    def run_clients(self, init_rclpy, run_nucleus_node):

        ahrs_subscriber = SubscriberAhrsPackets(callback_function=self.ahrs_queue.put)
        altimeter_subscriber = SubscriberAltimeterPackets(callback_function=self.altimeter_queue.put)
        bottom_track_subscriber = SubscriberBottomTrackPackets(callback_function=self.bottom_track_queue.put)
        current_profile_subscriber = SubscriberCurrentProfilePackets(callback_function=self.current_profile_queue.put)
        field_calibration_subscriber = SubscriberFieldCalibrationPackets(callback_function=self.field_calibration_queue.put)
        imu_subscriber = SubscriberImuPackets(callback_function=self.imu_queue.put)
        ins_subscriber = SubscriberInsPackets(callback_function=self.ins_queue.put)
        mag_subscriber = SubscriberMagnetometerPackets(callback_function=self.mag_queue.put)
        water_track_subscriber = SubscriberWaterTrackPackets(callback_function=self.water_track_queue.put)

        executor = MultiThreadedExecutor()
        executor.add_node(ahrs_subscriber)
        executor.add_node(altimeter_subscriber)
        executor.add_node(bottom_track_subscriber)
        executor.add_node(current_profile_subscriber)
        executor.add_node(field_calibration_subscriber)
        executor.add_node(imu_subscriber)
        executor.add_node(ins_subscriber)
        executor.add_node(ahrs_subscriber)
        executor.add_node(mag_subscriber)
        executor.add_node(water_track_subscriber)

        subscriber_thread = Thread(target=executor.spin)
        subscriber_thread.start()

        yield

        executor.shutdown()

        ahrs_subscriber.destroy_node()
        altimeter_subscriber.destroy_node()
        bottom_track_subscriber.destroy_node()
        current_profile_subscriber.destroy_node()
        field_calibration_subscriber.destroy_node()
        imu_subscriber.destroy_node()
        mag_subscriber.destroy_node()
        water_track_subscriber.destroy_node()

        subscriber_thread.join(1)

    @pytest.mark.dependency(name="connect")
    def test_client_connect(self, run_clients, nucleus_driver_connection):

        if nucleus_driver_connection == " tcp ":
            client = ClientConnectTcp()
        elif nucleus_driver_connection == " serial ":
            client = ClientConnectSerial()
        else:
            raise Exception("Unknown connection type")

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        password = "nortek"
        if pytest.password != "":
            password = pytest.password

        if nucleus_driver_connection == " tcp ":
            response = client.send_request(host=pytest.hostname, password=password, timeout_sec=1)

        elif nucleus_driver_connection == " serial ":
            response = client.send_request(serial_port=pytest.serial_port, timeout_sec=1)

        executor.shutdown()
        client.destroy_node()

        assert response.status is True

    @pytest.mark.dependency(name="test_command_setup_measurement", depends=["connect"])
    def test_command_setup_measurement(self, run_clients):

        client = ClientCommand()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response_setnav = client.send_request(command='SETNAV,FREQ=10,DS="ON"', timeout_sec=1)
        response_setahrs = client.send_request(command='SETAHRS,FREQ=10,DS="ON"', timeout_sec=1)
        response_setbt = client.send_request(command='SETBT,WT="ON",DS="ON"', timeout_sec=1)
        response_setalti = client.send_request(command='SETALTI,DS="ON"s', timeout_sec=1)
        response_setcurprof = client.send_request(command='SETCURPROF,DS="ON"', timeout_sec=1)
        response_settrig = client.send_request(command='SETTRIG,SRC="INTERNAL",FREQ=2,ALTI=2,CP=2', timeout_sec=1)

        response_setimu = client.send_request(command='SETIMU,DS="OFF"', timeout_sec=1)
        response_setmag = client.send_request(command='SETMAG,DS="OFF"', timeout_sec=1)

        executor.shutdown()
        client.destroy_node()

        assert "OK" in response_setnav.reply
        assert "OK" in response_setahrs.reply
        assert "OK" in response_setbt.reply
        assert "OK" in response_setalti.reply
        assert "OK" in response_setcurprof.reply
        assert "OK" in response_settrig.reply
        assert "OK" in response_setimu.reply
        assert "OK" in response_setmag.reply

    @pytest.mark.dependency(name="start_measurement", depends=["connect", "test_command_setup_measurement"])
    def test_client_start_measurement(self, run_clients):

        client = ClientStart()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)

        executor.shutdown()
        client.destroy_node()

        assert "OK" in response.reply

    @pytest.mark.dependency(name="stop_measurement", depends=["connect", "test_command_setup_measurement", "start_measurement"],)
    def test_client_stop_measurement(self, run_clients):

        time.sleep(3)  # allow for 1 second of measurement. Move to fixture?

        client = ClientStop()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)

        executor.shutdown()
        client.destroy_node()

        assert "OK" in response.reply

    @pytest.mark.dependency(name="test_command_setup_field_calibration", depends=["connect"])
    def test_command_setup_field_calibration(self, run_clients):

        client = ClientCommand()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response_setnav = client.send_request(command='SETNAV,FREQ=10,DS="OFF"', timeout_sec=1)
        response_setahrs = client.send_request(command='SETAHRS,FREQ=10,DS="OFF"', timeout_sec=1)
        response_setbt = client.send_request(command='SETBT,WT="ON",DS="OFF"', timeout_sec=1)
        response_setalti = client.send_request(command='SETALTI,DS="OFF"', timeout_sec=1)
        response_setcurprof = client.send_request(command='SETCURPROF,DS="OFF"', timeout_sec=1)
        response_settrig = client.send_request(command='SETTRIG,SRC="INTERNAL",FREQ=2,ALTI=2,CP=2', timeout_sec=1)

        response_setimu = client.send_request(command='SETIMU,DS="ON"', timeout_sec=1)
        response_setmag = client.send_request(command='SETMAG,DS="ON"', timeout_sec=1)

        executor.shutdown()
        client.destroy_node()

        assert "OK" in response_setnav.reply
        assert "OK" in response_setahrs.reply
        assert "OK" in response_setbt.reply
        assert "OK" in response_setalti.reply
        assert "OK" in response_setcurprof.reply
        assert "OK" in response_settrig.reply
        assert "OK" in response_setimu.reply
        assert "OK" in response_setmag.reply

    @pytest.mark.dependency(name="start_field_calibration", depends=["connect", "test_command_setup_field_calibration"])
    def test_client_start_field_calibration(self, run_clients):

        client = ClientFieldCalibration()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=2)

        executor.shutdown()
        client.destroy_node()

        assert "OK" in response.reply

    @pytest.mark.dependency(name="stop_field_calibration", depends=["connect", "test_command_setup_field_calibration", "start_field_calibration"])
    def test_client_stop_field_calibration(self, run_clients):

        time.sleep(3)  # allow for 1 second of measurement. Move to fixture?

        client = ClientStop()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)

        executor.shutdown()
        client.destroy_node()

        assert "OK" in response.reply

    @pytest.mark.dependency(name="disconnect", depends=["connect", "test_command_setup_field_calibration", "start_field_calibration", "stop_field_calibration"])
    def test_client_disconnect(self, run_clients):

        client = ClientDisconnect()

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        response = client.send_request(timeout_sec=1)

        executor.shutdown()
        client.destroy_node()

        assert response.status is True

    @pytest.mark.dependency(name="ahrs_subscriber", depends=[ "connect", "test_command_setup_measurement", "start_measurement", "stop_measurement", "disconnect"])
    def test_ahrs_subscriber(self, run_clients):

        assert not self.ahrs_queue.empty()

        while not self.ahrs_queue.empty():

            data = self.ahrs_queue.get()

            assert isinstance(data.posix_time, bool)
            assert isinstance(data.timestamp, int)
            assert isinstance(data.microseconds, int)

            assert isinstance(data.serial_number, int)
            assert isinstance(data.operation_mode, int)
            assert isinstance(data.fom_ahrs, float)
            assert isinstance(data.fom_fc1, float)
            assert isinstance(data.roll, float)
            assert isinstance(data.pitch, float)
            assert isinstance(data.heading, float)
            assert isinstance(data.quaternion_w, float)
            assert isinstance(data.quaternion_x, float)
            assert isinstance(data.quaternion_y, float)
            assert isinstance(data.quaternion_z, float)
            assert isinstance(data.rotation_matrix_0, float)
            assert isinstance(data.rotation_matrix_1, float)
            assert isinstance(data.rotation_matrix_2, float)
            assert isinstance(data.rotation_matrix_3, float)
            assert isinstance(data.rotation_matrix_4, float)
            assert isinstance(data.rotation_matrix_5, float)
            assert isinstance(data.rotation_matrix_6, float)
            assert isinstance(data.rotation_matrix_7, float)
            assert isinstance(data.rotation_matrix_8, float)
            assert isinstance(data.declination, float)
            assert isinstance(data.depth, float)

    @pytest.mark.dependency(name="ins_subscriber", depends=[ "connect", "test_command_setup_measurement", "start_measurement", "stop_measurement", "disconnect"])
    def test_ins_subscriber(self, run_clients):

        assert not self.ins_queue.empty()

        while not self.ins_queue.empty():

            data = self.ins_queue.get()

            assert isinstance(data.posix_time, bool)
            assert isinstance(data.timestamp, int)
            assert isinstance(data.microseconds, int)

            assert isinstance(data.serial_number, int)
            assert isinstance(data.operation_mode, int)
            assert isinstance(data.fom_ahrs, float)
            assert isinstance(data.fom_fc1, float)
            assert isinstance(data.roll, float)
            assert isinstance(data.pitch, float)
            assert isinstance(data.heading, float)
            assert isinstance(data.quaternion_w, float)
            assert isinstance(data.quaternion_x, float)
            assert isinstance(data.quaternion_y, float)
            assert isinstance(data.quaternion_z, float)
            assert isinstance(data.rotation_matrix_0, float)
            assert isinstance(data.rotation_matrix_1, float)
            assert isinstance(data.rotation_matrix_2, float)
            assert isinstance(data.rotation_matrix_3, float)
            assert isinstance(data.rotation_matrix_4, float)
            assert isinstance(data.rotation_matrix_5, float)
            assert isinstance(data.rotation_matrix_6, float)
            assert isinstance(data.rotation_matrix_7, float)
            assert isinstance(data.rotation_matrix_8, float)
            assert isinstance(data.declination, float)
            assert isinstance(data.depth, float)

            assert isinstance(data.fom_ins, float)
            assert isinstance(data.lat_long_is_valid, bool)
            assert isinstance(data.course_over_ground, float)
            assert isinstance(data.temperature, float)
            assert isinstance(data.pressure, float)
            assert isinstance(data.altitude, float)
            assert isinstance(data.latitude, float)
            assert isinstance(data.longitude, float)
            assert isinstance(data.position_frame_x, float)
            assert isinstance(data.position_frame_y, float)
            assert isinstance(data.position_frame_z, float)
            assert isinstance(data.velocity_ned_x, float)
            assert isinstance(data.velocity_ned_y, float)
            assert isinstance(data.velocity_ned_z, float)
            assert isinstance(data.velocity_nucleus_x, float)
            assert isinstance(data.velocity_nucleus_y, float)
            assert isinstance(data.velocity_nucleus_z, float)
            assert isinstance(data.speed_over_ground, float)
            assert isinstance(data.turn_rate_x, float)
            assert isinstance(data.turn_rate_y, float)
            assert isinstance(data.turn_rate_z, float)

    @pytest.mark.dependency(
        name="altimeter_subscriber",
        depends=[
            "connect",
            "test_command_setup_measurement",
            "start_measurement",
            "stop_measurement",
            "disconnect",
        ],
    )
    def test_altimeter_subscriber(self, run_clients):

        assert not self.altimeter_queue.empty()

        while not self.altimeter_queue.empty():

            data = self.altimeter_queue.get()

            assert isinstance(data.posix_time, bool)
            assert isinstance(data.timestamp, int)
            assert isinstance(data.microseconds, int)

            assert isinstance(data.altimeter_distance_valid, bool)
            assert isinstance(data.altimeter_quality_valid, bool)
            assert isinstance(data.pressure_valid, bool)
            assert isinstance(data.temperature_valid, bool)

            assert isinstance(data.serial_number, int)
            assert isinstance(data.sound_speed, float)
            assert isinstance(data.temperature, float)
            assert isinstance(data.pressure, float)
            assert isinstance(data.altimeter_distance, float)
            assert isinstance(data.altimeter_quality, int)

    @pytest.mark.dependency(
        name="bottom_track_subscriber",
        depends=[
            "connect",
            "test_command_setup_measurement",
            "start_measurement",
            "stop_measurement",
            "disconnect",
        ],
    )
    def test_bottom_track_subscriber(self, run_clients):

        assert not self.bottom_track_queue.empty()

        while not self.bottom_track_queue.empty():

            data = self.bottom_track_queue.get()

            assert isinstance(data.posix_time, bool)
            assert isinstance(data.timestamp, int)
            assert isinstance(data.microseconds, int)

            assert isinstance(data.beam_1_velocity_valid, bool)
            assert isinstance(data.beam_2_velocity_valid, bool)
            assert isinstance(data.beam_3_velocity_valid, bool)
            assert isinstance(data.beam_1_distance_valid, bool)
            assert isinstance(data.beam_2_distance_valid, bool)
            assert isinstance(data.beam_3_distance_valid, bool)
            assert isinstance(data.beam_1_fom_valid, bool)
            assert isinstance(data.beam_2_fom_valid, bool)
            assert isinstance(data.beam_3_fom_valid, bool)
            assert isinstance(data.x_velocity_valid, bool)
            assert isinstance(data.y_velocity_valid, bool)
            assert isinstance(data.z_velocity_valid, bool)
            assert isinstance(data.x_fom_valid, bool)
            assert isinstance(data.y_fom_valid, bool)
            assert isinstance(data.z_fom_valid, bool)

            assert isinstance(data.serial_number, int)
            assert isinstance(data.sound_speed, float)
            assert isinstance(data.temperature, float)
            assert isinstance(data.pressure, float)
            assert isinstance(data.velocity_beam_1, float)
            assert isinstance(data.velocity_beam_2, float)
            assert isinstance(data.velocity_beam_3, float)
            assert isinstance(data.distance_beam_1, float)
            assert isinstance(data.distance_beam_2, float)
            assert isinstance(data.distance_beam_3, float)
            assert isinstance(data.fom_beam_1, float)
            assert isinstance(data.fom_beam_2, float)
            assert isinstance(data.fom_beam_3, float)
            assert isinstance(data.dt_beam_1, float)
            assert isinstance(data.dt_beam_2, float)
            assert isinstance(data.dt_beam_3, float)
            assert isinstance(data.time_vel_beam_1, float)
            assert isinstance(data.time_vel_beam_2, float)
            assert isinstance(data.time_vel_beam_3, float)
            assert isinstance(data.velocity_x, float)
            assert isinstance(data.velocity_y, float)
            assert isinstance(data.velocity_z, float)
            assert isinstance(data.fom_x, float)
            assert isinstance(data.fom_y, float)
            assert isinstance(data.fom_z, float)
            assert isinstance(data.dt_xyz, float)
            assert isinstance(data.time_vel_xyz, float)

    @pytest.mark.dependency(
        name="water_track_subscriber",
        depends=[
            "connect",
            "test_command_setup_measurement",
            "start_measurement",
            "stop_measurement",
            "disconnect",
        ],
    )
    def test_water_track_subscriber(self, run_clients):

        assert not self.water_track_queue.empty()

        while not self.water_track_queue.empty():

            data = self.water_track_queue.get()

            assert isinstance(data.posix_time, bool)
            assert isinstance(data.timestamp, int)
            assert isinstance(data.microseconds, int)

            assert isinstance(data.beam_1_velocity_valid, bool)
            assert isinstance(data.beam_2_velocity_valid, bool)
            assert isinstance(data.beam_3_velocity_valid, bool)
            assert isinstance(data.beam_1_distance_valid, bool)
            assert isinstance(data.beam_2_distance_valid, bool)
            assert isinstance(data.beam_3_distance_valid, bool)
            assert isinstance(data.beam_1_fom_valid, bool)
            assert isinstance(data.beam_2_fom_valid, bool)
            assert isinstance(data.beam_3_fom_valid, bool)
            assert isinstance(data.x_velocity_valid, bool)
            assert isinstance(data.y_velocity_valid, bool)
            assert isinstance(data.z_velocity_valid, bool)
            assert isinstance(data.x_fom_valid, bool)
            assert isinstance(data.y_fom_valid, bool)
            assert isinstance(data.z_fom_valid, bool)

            assert isinstance(data.serial_number, int)
            assert isinstance(data.sound_speed, float)
            assert isinstance(data.temperature, float)
            assert isinstance(data.pressure, float)
            assert isinstance(data.velocity_beam_1, float)
            assert isinstance(data.velocity_beam_2, float)
            assert isinstance(data.velocity_beam_3, float)
            assert isinstance(data.distance_beam_1, float)
            assert isinstance(data.distance_beam_2, float)
            assert isinstance(data.distance_beam_3, float)
            assert isinstance(data.fom_beam_1, float)
            assert isinstance(data.fom_beam_2, float)
            assert isinstance(data.fom_beam_3, float)
            assert isinstance(data.dt_beam_1, float)
            assert isinstance(data.dt_beam_2, float)
            assert isinstance(data.dt_beam_3, float)
            assert isinstance(data.time_vel_beam_1, float)
            assert isinstance(data.time_vel_beam_2, float)
            assert isinstance(data.time_vel_beam_3, float)
            assert isinstance(data.velocity_x, float)
            assert isinstance(data.velocity_y, float)
            assert isinstance(data.velocity_z, float)
            assert isinstance(data.fom_x, float)
            assert isinstance(data.fom_y, float)
            assert isinstance(data.fom_z, float)
            assert isinstance(data.dt_xyz, float)
            assert isinstance(data.time_vel_xyz, float)

    @pytest.mark.dependency(
        name="current_profile_subscriber",
        depends=[
            "connect",
            "test_command_setup_measurement",
            "start_measurement",
            "stop_measurement",
            "disconnect",
        ],
    )
    def test_current_profile_subscriber(self, run_clients):

        assert not self.current_profile_queue.empty()

        while not self.current_profile_queue.empty():

            data = self.current_profile_queue.get()

            assert isinstance(data.posix_time, bool)
            assert isinstance(data.timestamp, int)
            assert isinstance(data.microseconds, int)

            assert isinstance(data.serial_number, int)
            assert isinstance(data.sound_velocity, float)
            assert isinstance(data.temperature, float)
            assert isinstance(data.pressure, float)
            assert isinstance(data.cell_size, float)
            assert isinstance(data.blanking, float)
            assert isinstance(data.number_of_cells, int)
            assert isinstance(data.ambiguity_velocity, int)

            assert isinstance(data.velocity_data, array)
            assert isinstance(data.amplitude_data, array)
            assert isinstance(data.correlation_data, array)

    @pytest.mark.dependency(
        name="imu_subscriber",
        depends=[
            "connect",
            "test_command_setup_field_calibration",
            "start_field_calibration",
            "stop_field_calibration",
            "disconnect",
        ],
    )
    def test_imu_subscriber(self, run_clients):

        assert not self.imu_queue.empty()

        while not self.imu_queue.empty():

            data = self.imu_queue.get()

            assert isinstance(data.posix_time, bool)
            assert isinstance(data.timestamp, int)
            assert isinstance(data.microseconds, int)

            assert isinstance(data.is_valid, bool)
            assert isinstance(data.has_data_path_overrun, bool)
            assert isinstance(data.has_flash_update_failure, bool)
            assert isinstance(data.has_spi_com_error, bool)
            assert isinstance(data.has_low_voltage, bool)
            assert isinstance(data.has_sensor_failure, bool)
            assert isinstance(data.has_memory_failure, bool)
            assert isinstance(data.has_gyro_1_failure, bool)
            assert isinstance(data.has_gyro_2_failure, bool)
            assert isinstance(data.has_accelerometer_failure, bool)

            assert isinstance(data.accelerometer_x, float)
            assert isinstance(data.accelerometer_y, float)
            assert isinstance(data.accelerometer_z, float)
            assert isinstance(data.gyro_x, float)
            assert isinstance(data.gyro_y, float)
            assert isinstance(data.gyro_z, float)
            assert isinstance(data.temperature, float)

    @pytest.mark.dependency(
        name="mag_subscriber",
        depends=[
            "connect",
            "test_command_setup_field_calibration",
            "start_field_calibration",
            "stop_field_calibration",
            "disconnect",
        ],
    )
    def test_mag_subscriber(self, run_clients):

        assert not self.mag_queue.empty()

        while not self.mag_queue.empty():

            data = self.mag_queue.get()

            assert isinstance(data.posix_time, bool)
            assert isinstance(data.timestamp, int)
            assert isinstance(data.microseconds, int)

            assert isinstance(data.is_compensated_for_hard_iron, bool)
            assert isinstance(data.dvl_active, bool)
            assert isinstance(data.dvl_acoustics_active, bool)
            assert isinstance(data.dvl_transmitter_active, bool)

            assert isinstance(data.magnetometer_x, float)
            assert isinstance(data.magnetometer_y, float)
            assert isinstance(data.magnetometer_z, float)

    @pytest.mark.dependency(
        name="field_calibration_subscriber",
        depends=[
            "connect",
            "test_command_setup_field_calibration",
            "start_field_calibration",
            "stop_field_calibration",
            "disconnect",
        ],
    )
    def test_field_calibration_subscriber(self, run_clients):

        assert not self.field_calibration_queue.empty()

        while not self.field_calibration_queue.empty():

            data = self.field_calibration_queue.get()

            assert isinstance(data.posix_time, bool)
            assert isinstance(data.timestamp, int)
            assert isinstance(data.microseconds, int)

            assert isinstance(data.points_used_in_estimation, bool)
            assert isinstance(data.hard_iron_x, float)
            assert isinstance(data.hard_iron_y, float)
            assert isinstance(data.hard_iron_z, float)
            assert isinstance(data.s_axis_0, float)
            assert isinstance(data.s_axis_1, float)
            assert isinstance(data.s_axis_2, float)
            assert isinstance(data.s_axis_3, float)
            assert isinstance(data.s_axis_4, float)
            assert isinstance(data.s_axis_5, float)
            assert isinstance(data.s_axis_6, float)
            assert isinstance(data.s_axis_7, float)
            assert isinstance(data.s_axis_8, float)
            assert isinstance(data.new_point_x, float)
            assert isinstance(data.new_point_y, float)
            assert isinstance(data.new_point_z, float)
            assert isinstance(data.fom_field_calibration, float)
            assert isinstance(data.coverage, float)
