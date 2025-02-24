import rclpy
from rclpy.node import Node
from threading import Thread
from rclpy._rclpy_pybind11 import RCLError

from interfaces.srv import (
    ConnectTcp,
    ConnectSerial,
    Disconnect,
    Start,
    Stop,
    StartFieldCalibration,
    Command,
)
from interfaces.msg import (
    AHRS,
    INS,
    Altimeter,
    BottomTrack,
    WaterTrack,
    CurrentProfile,
    FieldCalibration,
    IMU,
    Magnetometer,
)

from nucleus_driver import NucleusDriver


class NucleusNode(Node):

    def __init__(self):
        super().__init__("nucleus_node")

        self.nucleus_driver = NucleusDriver()

        self.connect_tcp_service = self.create_service(ConnectTcp, "nucleus_node/connect_tcp", self.connect_tcp_callback)
        self.connect_serial_service = self.create_service(ConnectSerial, "nucleus_node/connect_serial", self.connect_serial_callback)
        self.disconnect_service = self.create_service(Disconnect, "nucleus_node/disconnect", self.disconnect_callback)
        self.start_service = self.create_service(Start, "nucleus_node/start", self.start_callback)
        self.start_service = self.create_service(StartFieldCalibration, "nucleus_node/field_calibration", self.start_field_calibration_callback)
        self.stop_service = self.create_service(Stop, "nucleus_node/stop", self.stop_callback)
        self.command_service = self.create_service(Command, "nucleus_node/command", self.command_callback)

        self.ahrs_publisher = self.create_publisher(AHRS, "nucleus_node/ahrs_packets", 100)
        self.altimeter_publisher = self.create_publisher(Altimeter, "nucleus_node/altimeter_packets", 100)
        self.bottom_track_publisher = self.create_publisher(BottomTrack, "nucleus_node/bottom_track_packets", 100)
        self.water_track_publisher = self.create_publisher(WaterTrack, "nucleus_node/water_track_packets", 100)
        self.current_profile_publisher = self.create_publisher(CurrentProfile, "nucleus_node/current_profile_packets", 100)
        self.field_calibration_publisher = self.create_publisher(FieldCalibration, "nucleus_node/field_calibration_packets", 100)
        self.imu_publisher = self.create_publisher(IMU, "nucleus_node/imu_packets", 100)
        self.ins_publisher = self.create_publisher(INS, "nucleus_node/ins_packets", 100)
        self.mag_publisher = self.create_publisher(Magnetometer, "nucleus_node/magnetometer_packets", 100)

        self.start_thread_timer = self.create_timer(0.1, self.start_thread) # This ensures that the thread is started after the node is initialized and rcplpy.spin() is called

        self._thread_running = False
        self.packet_thread = Thread(target=self.packet_handling)

        self.get_logger().info("Nucleus Node initiated")

    def start_thread(self):
        if not self._thread_running:
            self.start()
            self.start_thread_timer.cancel()

    def start(self):
        self._thread_running = True
        self.packet_thread.start()
        self.get_logger().info("Nucleus Node started")

    def stop(self):
        self._thread_running = False
        self.packet_thread.join(timeout=2)  # 1 second more than the timeout of the read_packet method

    def connect_tcp_callback(self, request, response):

        self.nucleus_driver.set_tcp_configuration(host=request.host)
        status = self.nucleus_driver.connect(connection_type="tcp", password=request.password)

        if status:
            self.get_logger().info(f"Connected through TCP with host: {request.host}")
        else:
            self.get_logger().info(f"Failed to connect with host: {request.host}")

        response.status = status

        return response

    def connect_serial_callback(self, request, response):

        self.nucleus_driver.set_serial_configuration(port=request.serial_port)
        status = self.nucleus_driver.connect(connection_type="serial")

        if status:
            self.get_logger().info(f"connected through serial with serial port: {request.serial_port}")
        else:
            self.get_logger().info(f"failed to connect through serial port: {request.serial_port}")

        response.status = status

        return response

    def disconnect_callback(self, request, response):

        status = self.nucleus_driver.disconnect()

        response.status = status

        if status:
            self.get_logger().info("Successfully disconnected from Nucleus")
        else:
            self.get_logger().info("Failed to disconnect from Nucleus")

        return response

    def start_callback(self, request, response):

        if not self.nucleus_driver.connection.get_connection_status():
            self.get_logger().info("Nucleus is not connected")
            response.reply = "Nucleus is not connected"

        else:
            reply = self.nucleus_driver.start_measurement()

            try:
                response.reply = reply[0].decode()
                self.get_logger().info(f"start reply: {response.reply.strip()}")
            except Exception as e:
                response.reply = f"Failed to decode response from start command: {e}"

        return response

    def start_field_calibration_callback(self, request, response):

        if not self.nucleus_driver.connection.get_connection_status():
            self.get_logger().info("Nucleus is not connected")
            response.reply = "Nucleus is not connected"

        else:
            reply = self.nucleus_driver.start_fieldcal()

            try:
                response.reply = reply[0].decode()
                self.get_logger().info(f"start field calibration reply: {response.reply.strip()}")
            except Exception as e:
                response.reply = "Failed to decode response from start field calibration command: {e}"

        return response

    def stop_callback(self, request, response):

        if not self.nucleus_driver.connection.get_connection_status():
            self.get_logger().info("Nucleus is not connected")
            response.reply = "Nucleus is not connected"

        else:
            reply = self.nucleus_driver.stop()

            try:
                response.reply = reply[0].decode()
                self.get_logger().info(f"stop reply: {response.reply.strip()}")
            except Exception as e:
                response.reply = f"Failed to decode response from start command: {e}"

        return response

    def command_callback(self, request, response):

        if not self.nucleus_driver.connection.get_connection_status():
            self.get_logger().info("Nucleus is not connected")
            response.reply = "Nucleus is not connected"

        else:
            command = request.command

            reply = self.nucleus_driver.send_command(command=command)

            try:
                for entry in reply:
                    response.reply += entry.decode()

                self.get_logger().info(f"command reply: {response.reply.strip()}")
            except Exception as e:
                response.reply = f"Failed to decode response from command: {e}"

        return response

    
    def packet_handling(self):

        while self._thread_running and rclpy.ok():

            packet = self.nucleus_driver.read_packet(timeout=1, _suppress_warning=True)

            if packet is None:
                continue

            system_timestamp = self.get_clock().now()

            if packet["id"] == 0xD2:

                ahrs_packet = AHRS()

                ahrs_packet.system_timestamp = system_timestamp.to_msg()

                ahrs_packet.posix_time = packet["flags.posixTime"]
                ahrs_packet.timestamp = packet["timeStamp"]
                ahrs_packet.microseconds = packet["microSeconds"]

                ahrs_packet.serial_number = packet["serialNumber"]
                ahrs_packet.operation_mode = packet["operationMode"]

                ahrs_packet.fom_ahrs = packet["fomAhrs"]
                ahrs_packet.fom_fc1 = packet["fomFc1"]

                ahrs_packet.roll = packet["ahrsData.roll"]
                ahrs_packet.pitch = packet["ahrsData.pitch"]
                ahrs_packet.heading = packet["ahrsData.heading"]

                ahrs_packet.quaternion_w = packet["ahrsData.quaternionW"]
                ahrs_packet.quaternion_x = packet["ahrsData.quaternionX"]
                ahrs_packet.quaternion_y = packet["ahrsData.quaternionY"]
                ahrs_packet.quaternion_z = packet["ahrsData.quaternionZ"]

                ahrs_packet.rotation_matrix_0 = packet["ahrsData.rotationMatrix_0"]
                ahrs_packet.rotation_matrix_1 = packet["ahrsData.rotationMatrix_1"]
                ahrs_packet.rotation_matrix_2 = packet["ahrsData.rotationMatrix_2"]
                ahrs_packet.rotation_matrix_3 = packet["ahrsData.rotationMatrix_3"]
                ahrs_packet.rotation_matrix_4 = packet["ahrsData.rotationMatrix_4"]
                ahrs_packet.rotation_matrix_5 = packet["ahrsData.rotationMatrix_5"]
                ahrs_packet.rotation_matrix_6 = packet["ahrsData.rotationMatrix_6"]
                ahrs_packet.rotation_matrix_7 = packet["ahrsData.rotationMatrix_7"]
                ahrs_packet.rotation_matrix_8 = packet["ahrsData.rotationMatrix_8"]

                ahrs_packet.declination = packet["declination"]
                ahrs_packet.depth = packet["depth"]

                try:
                    self.ahrs_publisher.publish(ahrs_packet)
                except RCLError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"Failed to publish AHRS packet: {e}")

            elif packet["id"] == 0xDC:

                ins_packet = INS()

                ins_packet.system_timestamp = system_timestamp.to_msg()

                ins_packet.posix_time = packet["flags.posixTime"]
                ins_packet.timestamp = packet["timeStamp"]
                ins_packet.microseconds = packet["microSeconds"]

                ins_packet.serial_number = packet["serialNumber"]
                ins_packet.operation_mode = packet["operationMode"]

                ins_packet.fom_ahrs = packet["fomAhrs"]
                ins_packet.fom_fc1 = packet["fomFc1"]

                ins_packet.roll = packet["ahrsData.roll"]
                ins_packet.pitch = packet["ahrsData.pitch"]
                ins_packet.heading = packet["ahrsData.heading"]

                ins_packet.quaternion_w = packet["ahrsData.quaternionW"]
                ins_packet.quaternion_x = packet["ahrsData.quaternionX"]
                ins_packet.quaternion_y = packet["ahrsData.quaternionY"]
                ins_packet.quaternion_z = packet["ahrsData.quaternionZ"]

                ins_packet.rotation_matrix_0 = packet["ahrsData.rotationMatrix_0"]
                ins_packet.rotation_matrix_1 = packet["ahrsData.rotationMatrix_1"]
                ins_packet.rotation_matrix_2 = packet["ahrsData.rotationMatrix_2"]
                ins_packet.rotation_matrix_3 = packet["ahrsData.rotationMatrix_3"]
                ins_packet.rotation_matrix_4 = packet["ahrsData.rotationMatrix_4"]
                ins_packet.rotation_matrix_5 = packet["ahrsData.rotationMatrix_5"]
                ins_packet.rotation_matrix_6 = packet["ahrsData.rotationMatrix_6"]
                ins_packet.rotation_matrix_7 = packet["ahrsData.rotationMatrix_7"]
                ins_packet.rotation_matrix_8 = packet["ahrsData.rotationMatrix_8"]

                ins_packet.declination = packet["declination"]
                ins_packet.depth = packet["depth"]

                ins_packet.fom_ins = packet["fomIns"]
                ins_packet.lat_long_is_valid = packet["statusIns.latLonIsValid"]
                ins_packet.course_over_ground = packet["courseOverGround"]
                ins_packet.temperature = packet["temperature"]
                ins_packet.pressure = packet["pressure"]
                ins_packet.altitude = packet["altitude"]
                ins_packet.latitude = packet["latitude"]
                ins_packet.longitude = packet["longitude"]
                ins_packet.position_frame_x = packet["positionFrameX"]
                ins_packet.position_frame_y = packet["positionFrameY"]
                ins_packet.position_frame_z = packet["positionFrameZ"]
                ins_packet.velocity_ned_x = packet["velocityNedX"]
                ins_packet.velocity_ned_y = packet["velocityNedY"]
                ins_packet.velocity_ned_z = packet["velocityNedZ"]
                ins_packet.velocity_nucleus_x = packet["velocityNucleusX"]
                ins_packet.velocity_nucleus_y = packet["velocityNucleusY"]
                ins_packet.velocity_nucleus_z = packet["velocityNucleusZ"]
                ins_packet.speed_over_ground = packet["speedOverGround"]
                ins_packet.turn_rate_x = packet["turnRateX"]
                ins_packet.turn_rate_y = packet["turnRateY"]
                ins_packet.turn_rate_z = packet["turnRateZ"]

                try:
                    self.ins_publisher.publish(ins_packet)
                except RCLError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"Failed to publish INS packet: {e}")

            elif packet["id"] == 0x82:

                imu_packet = IMU()

                imu_packet.system_timestamp = system_timestamp.to_msg()

                imu_packet.posix_time = packet["flags.posixTime"]
                imu_packet.timestamp = packet["timeStamp"]
                imu_packet.microseconds = packet["microSeconds"]

                imu_packet.is_valid = packet["status.isValid"]
                imu_packet.has_data_path_overrun = packet["status.hasDataPathOverrun"]
                imu_packet.has_flash_update_failure = packet["status.hasFlashUpdateFailure"]
                imu_packet.has_spi_com_error = packet["status.hasSpiComError"]
                imu_packet.has_low_voltage = packet["status.hasLowVoltage"]
                imu_packet.has_sensor_failure = packet["status.hasSensorFailure"]
                imu_packet.has_memory_failure = packet["status.hasMemoryFailure"]
                imu_packet.has_gyro_1_failure = packet["status.hasGyro1Failure"]
                imu_packet.has_gyro_2_failure = packet["status.hasGyro2Failure"]
                imu_packet.has_accelerometer_failure = packet["status.hasAccelerometerFailure"]

                imu_packet.accelerometer_x = packet["accelerometer.x"]
                imu_packet.accelerometer_y = packet["accelerometer.y"]
                imu_packet.accelerometer_z = packet["accelerometer.z"]
                imu_packet.gyro_x = packet["gyro.x"]
                imu_packet.gyro_y = packet["gyro.y"]
                imu_packet.gyro_z = packet["gyro.z"]
                imu_packet.temperature = packet["temperature"]

                try:
                    self.imu_publisher.publish(imu_packet)
                except RCLError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"Failed to publish IMU packet: {e}")

            elif packet["id"] == 0x87:

                mag_packet = Magnetometer()

                mag_packet.system_timestamp = system_timestamp.to_msg()

                mag_packet.posix_time = packet["flags.posixTime"]
                mag_packet.timestamp = packet["timeStamp"]
                mag_packet.microseconds = packet["microSeconds"]

                mag_packet.is_compensated_for_hard_iron = packet["status.isCompensatedForHardIron"]
                mag_packet.dvl_active = packet["status.dvlActive"]
                mag_packet.dvl_acoustics_active = packet["status.dvlAcousticsActive"]
                mag_packet.dvl_transmitter_active = packet["status.dvlTransmitterActive"]

                mag_packet.magnetometer_x = packet["magnetometer.x"]
                mag_packet.magnetometer_y = packet["magnetometer.y"]
                mag_packet.magnetometer_z = packet["magnetometer.z"]

                try:
                    self.mag_publisher.publish(mag_packet)
                except RCLError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"Failed to publish Magnetometer packet: {e}")

            elif packet["id"] == 0xB4:

                bottom_track_packet = BottomTrack()

                bottom_track_packet.system_timestamp = system_timestamp.to_msg()

                bottom_track_packet.posix_time = packet["flags.posixTime"]
                bottom_track_packet.timestamp = packet["timeStamp"]
                bottom_track_packet.microseconds = packet["microSeconds"]

                bottom_track_packet.beam_1_velocity_valid = packet["status.beam1VelocityValid"]
                bottom_track_packet.beam_2_velocity_valid = packet["status.beam2VelocityValid"]
                bottom_track_packet.beam_3_velocity_valid = packet["status.beam3VelocityValid"]
                bottom_track_packet.beam_1_distance_valid = packet["status.beam1DistanceValid"]
                bottom_track_packet.beam_2_distance_valid = packet["status.beam2DistanceValid"]
                bottom_track_packet.beam_3_distance_valid = packet["status.beam3DistanceValid"]
                bottom_track_packet.beam_1_fom_valid = packet["status.beam1FomValid"]
                bottom_track_packet.beam_2_fom_valid = packet["status.beam2FomValid"]
                bottom_track_packet.beam_3_fom_valid = packet["status.beam3FomValid"]
                bottom_track_packet.x_velocity_valid = packet["status.xVelocityValid"]
                bottom_track_packet.y_velocity_valid = packet["status.yVelocityValid"]
                bottom_track_packet.z_velocity_valid = packet["status.zVelocityValid"]
                bottom_track_packet.x_fom_valid = packet["status.xFomValid"]
                bottom_track_packet.y_fom_valid = packet["status.yFomValid"]
                bottom_track_packet.z_fom_valid = packet["status.zFomValid"]

                bottom_track_packet.serial_number = packet["serialNumber"]
                bottom_track_packet.sound_speed = packet["soundSpeed"]
                bottom_track_packet.temperature = packet["temperature"]
                bottom_track_packet.pressure = packet["pressure"]
                bottom_track_packet.velocity_beam_1 = packet["velocityBeam1"]
                bottom_track_packet.velocity_beam_2 = packet["velocityBeam2"]
                bottom_track_packet.velocity_beam_3 = packet["velocityBeam3"]
                bottom_track_packet.distance_beam_1 = packet["distanceBeam1"]
                bottom_track_packet.distance_beam_2 = packet["distanceBeam2"]
                bottom_track_packet.distance_beam_3 = packet["distanceBeam3"]
                bottom_track_packet.fom_beam_1 = packet["fomBeam1"]
                bottom_track_packet.fom_beam_2 = packet["fomBeam2"]
                bottom_track_packet.fom_beam_3 = packet["fomBeam3"]
                bottom_track_packet.dt_beam_1 = packet["dtBeam1"]
                bottom_track_packet.dt_beam_2 = packet["dtBeam2"]
                bottom_track_packet.dt_beam_3 = packet["dtBeam3"]
                bottom_track_packet.time_vel_beam_1 = packet["timeVelBeam1"]
                bottom_track_packet.time_vel_beam_2 = packet["timeVelBeam2"]
                bottom_track_packet.time_vel_beam_3 = packet["timeVelBeam3"]
                bottom_track_packet.velocity_x = packet["velocityX"]
                bottom_track_packet.velocity_y = packet["velocityY"]
                bottom_track_packet.velocity_z = packet["velocityZ"]
                bottom_track_packet.fom_x = packet["fomX"]
                bottom_track_packet.fom_y = packet["fomY"]
                bottom_track_packet.fom_z = packet["fomZ"]
                bottom_track_packet.dt_xyz = packet["dtXYZ"]
                bottom_track_packet.time_vel_xyz = packet["timeVelXYZ"]

                try:
                    self.bottom_track_publisher.publish(bottom_track_packet)
                except RCLError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"Failed to publish Bottom Track packet: {e}")

            elif packet["id"] == 0xBE:

                water_track_packet = WaterTrack()

                water_track_packet.system_timestamp = system_timestamp.to_msg()

                water_track_packet.posix_time = packet["flags.posixTime"]
                water_track_packet.timestamp = packet["timeStamp"]
                water_track_packet.microseconds = packet["microSeconds"]

                water_track_packet.beam_1_velocity_valid = packet["status.beam1VelocityValid"]
                water_track_packet.beam_2_velocity_valid = packet["status.beam2VelocityValid"]
                water_track_packet.beam_3_velocity_valid = packet["status.beam3VelocityValid"]
                water_track_packet.beam_1_distance_valid = packet["status.beam1DistanceValid"]
                water_track_packet.beam_2_distance_valid = packet["status.beam2DistanceValid"]
                water_track_packet.beam_3_distance_valid = packet["status.beam3DistanceValid"]
                water_track_packet.beam_1_fom_valid = packet["status.beam1FomValid"]
                water_track_packet.beam_2_fom_valid = packet["status.beam2FomValid"]
                water_track_packet.beam_3_fom_valid = packet["status.beam3FomValid"]
                water_track_packet.x_velocity_valid = packet["status.xVelocityValid"]
                water_track_packet.y_velocity_valid = packet["status.yVelocityValid"]
                water_track_packet.z_velocity_valid = packet["status.zVelocityValid"]
                water_track_packet.x_fom_valid = packet["status.xFomValid"]
                water_track_packet.y_fom_valid = packet["status.yFomValid"]
                water_track_packet.z_fom_valid = packet["status.zFomValid"]

                water_track_packet.serial_number = packet["serialNumber"]
                water_track_packet.sound_speed = packet["soundSpeed"]
                water_track_packet.temperature = packet["temperature"]
                water_track_packet.pressure = packet["pressure"]
                water_track_packet.velocity_beam_1 = packet["velocityBeam1"]
                water_track_packet.velocity_beam_2 = packet["velocityBeam2"]
                water_track_packet.velocity_beam_3 = packet["velocityBeam3"]
                water_track_packet.distance_beam_1 = packet["distanceBeam1"]
                water_track_packet.distance_beam_2 = packet["distanceBeam2"]
                water_track_packet.distance_beam_3 = packet["distanceBeam3"]
                water_track_packet.fom_beam_1 = packet["fomBeam1"]
                water_track_packet.fom_beam_2 = packet["fomBeam2"]
                water_track_packet.fom_beam_3 = packet["fomBeam3"]
                water_track_packet.dt_beam_1 = packet["dtBeam1"]
                water_track_packet.dt_beam_2 = packet["dtBeam2"]
                water_track_packet.dt_beam_3 = packet["dtBeam3"]
                water_track_packet.time_vel_beam_1 = packet["timeVelBeam1"]
                water_track_packet.time_vel_beam_2 = packet["timeVelBeam2"]
                water_track_packet.time_vel_beam_3 = packet["timeVelBeam3"]
                water_track_packet.velocity_x = packet["velocityX"]
                water_track_packet.velocity_y = packet["velocityY"]
                water_track_packet.velocity_z = packet["velocityZ"]
                water_track_packet.fom_x = packet["fomX"]
                water_track_packet.fom_y = packet["fomY"]
                water_track_packet.fom_z = packet["fomZ"]
                water_track_packet.dt_xyz = packet["dtXYZ"]
                water_track_packet.time_vel_xyz = packet["timeVelXYZ"]

                try:
                    self.water_track_publisher.publish(water_track_packet)
                except RCLError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"Failed to publish Water Track packet: {e}")

            elif packet["id"] == 0xAA:

                altimeter_packet = Altimeter()

                altimeter_packet.system_timestamp = system_timestamp.to_msg()

                altimeter_packet.posix_time = packet["flags.posixTime"]
                altimeter_packet.timestamp = packet["timeStamp"]
                altimeter_packet.microseconds = packet["microSeconds"]

                altimeter_packet.altimeter_distance_valid = packet["status.altimeterDistanceValid"]
                altimeter_packet.altimeter_quality_valid = packet["status.altimeterQualityValid"]
                altimeter_packet.pressure_valid = packet["status.pressureValid"]
                altimeter_packet.temperature_valid = packet["status.temperatureValid"]

                altimeter_packet.serial_number = packet["serialNumber"]
                altimeter_packet.sound_speed = packet["soundSpeed"]
                altimeter_packet.temperature = packet["temperature"]
                altimeter_packet.pressure = packet["pressure"]
                altimeter_packet.altimeter_distance = packet["altimeterDistance"]
                altimeter_packet.altimeter_quality = packet["altimeterQuality"]

                try:
                    self.altimeter_publisher.publish(altimeter_packet)
                except RCLError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"Failed to publish Altimeter packet: {e}")

            elif packet["id"] == 0xC0:

                current_profile_packet = CurrentProfile()

                current_profile_packet.system_timestamp = system_timestamp.to_msg()

                current_profile_packet.posix_time = packet["flags.posixTime"]
                current_profile_packet.timestamp = packet["timeStamp"]
                current_profile_packet.microseconds = packet["microSeconds"]

                current_profile_packet.serial_number = packet["serialNumber"]
                current_profile_packet.sound_velocity = packet["soundVelocity"]
                current_profile_packet.temperature = packet["temperature"]
                current_profile_packet.pressure = packet["pressure"]
                current_profile_packet.cell_size = packet["cellSize"]
                current_profile_packet.blanking = packet["blanking"]
                current_profile_packet.number_of_cells = packet["numberOfCells"]
                current_profile_packet.ambiguity_velocity = packet["ambiguityVelocity"]

                velocity_data = list()
                amplitude_data = list()
                correlation_data = list()

                for key in packet.keys():
                    if "velocityData" in key:
                        velocity_data.append(packet[key])

                    elif "amplitudeData" in key:
                        amplitude_data.append(packet[key])

                    elif "correlationData" in key:
                        correlation_data.append(packet[key])

                current_profile_packet.velocity_data = velocity_data
                current_profile_packet.amplitude_data = amplitude_data
                current_profile_packet.correlation_data = correlation_data

                try:
                    self.current_profile_publisher.publish(current_profile_packet)
                except RCLError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"Failed to publish Current Profile packet: {e}")

            elif packet["id"] == 0x8B:

                field_calibration_packet = FieldCalibration()

                field_calibration_packet.system_timestamp = system_timestamp.to_msg()

                field_calibration_packet.posix_time = packet["flags.posixTime"]
                field_calibration_packet.timestamp = packet["timeStamp"]
                field_calibration_packet.microseconds = packet["microSeconds"]

                field_calibration_packet.points_used_in_estimation = packet["status.pointsUsedInEstimation"]

                field_calibration_packet.hard_iron_x = packet["hardIron.x"]
                field_calibration_packet.hard_iron_y = packet["hardIron.y"]
                field_calibration_packet.hard_iron_z = packet["hardIron.z"]
                field_calibration_packet.s_axis_0 = packet["sAxis_0"]
                field_calibration_packet.s_axis_1 = packet["sAxis_1"]
                field_calibration_packet.s_axis_2 = packet["sAxis_2"]
                field_calibration_packet.s_axis_3 = packet["sAxis_3"]
                field_calibration_packet.s_axis_4 = packet["sAxis_4"]
                field_calibration_packet.s_axis_5 = packet["sAxis_5"]
                field_calibration_packet.s_axis_6 = packet["sAxis_6"]
                field_calibration_packet.s_axis_7 = packet["sAxis_7"]
                field_calibration_packet.s_axis_8 = packet["sAxis_8"]
                field_calibration_packet.new_point_x = packet["newPoint.x"]
                field_calibration_packet.new_point_y = packet["newPoint.y"]
                field_calibration_packet.new_point_z = packet["newPoint.z"]
                field_calibration_packet.fom_field_calibration = packet["fomFieldCalibration"]
                field_calibration_packet.coverage = packet["coverage"]

                try:
                    self.field_calibration_publisher.publish(field_calibration_packet)
                except RCLError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"Failed to publish Field Calibration packet: {e}")


def main():

    rclpy.init()

    nucleus_node = NucleusNode()

    try:
        rclpy.spin(nucleus_node)

    except KeyboardInterrupt:
        print('Keyboard Interrupt (SIGINT)')

    except Exception as e:
        nucleus_node.get_logger().error(f'An unexpected error occurred: {e}')

    finally:
        nucleus_node.stop()

        if nucleus_node.nucleus_driver.connection.get_connection_status():
            stop_status = nucleus_node.nucleus_driver.stop()
            print(f'stop reply: {stop_status[0].decode().strip()}')

            disconnect_status = nucleus_node.nucleus_driver.disconnect()
            print(f'Disconnect status {disconnect_status}')

        nucleus_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
