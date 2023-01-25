import time
from binascii import crc32
import re
from zipfile import ZipFile


class Flash:

    def __init__(self, **kwargs):

        self.BIN_FILE_PATTERN = r'(\w+)-v(\d+).(\d+).(\d+)(.*)'
        self.LDR_FILE_PATTERN = r'(\w+)_V(\d+)_(\d+)'

        self.messages = kwargs.get('messages')
        self.connection = kwargs.get('connection')
        self.commands = kwargs.get('commands')

        self.nucleus_firmware_name = None
        self.dvl_firmware_name = None
        self.nucleus_firmware = None
        self.dvl_firmware = None

    def set_flash_files(self, path: str) -> bool:

        status = False

        if path.endswith('.zip'):

            archive = ZipFile(path, 'r')

            for index, file in enumerate(archive.namelist()):
                if file.endswith('.bin'):
                    self.nucleus_firmware_name = archive.namelist()[index]
                if file.endswith('.ldr'):
                    self.dvl_firmware_name = archive.namelist()[index]

            if self.nucleus_firmware_name is not None:
                self.nucleus_firmware = archive.read(self.nucleus_firmware_name)

                if self.nucleus_firmware[:4] != b'\x3d\xb8\xf3\x96':
                    self.messages.write_warning(message='Invalid Nucleus firmware')
                    self.nucleus_firmware = None
                    self.nucleus_firmware_name = None

            if self.dvl_firmware_name is not None:
                self.dvl_firmware = archive.read(self.dvl_firmware_name)

            status = True

        elif path.endswith('.bin'):

            self.nucleus_firmware_name = path.split('/')[-1]

            with open(path, mode='rb') as file:
                self.nucleus_firmware = file.read()

            if self.nucleus_firmware[:4] != b'\x3d\xb8\xf3\x96':
                self.messages.write_warning(message='Invalid Nucleus firmware')
                self.nucleus_firmware = None
                self.nucleus_firmware_name = None

            status = True

        elif path.endswith('.ldr'):

            self.dvl_firmware_name = path.split('/')[-1]

            with open(path, mode='rb') as file:
                self.dvl_firmware = file.read()

            status = True

        else:
            self.messages.write_warning('Invalid extension for flash file. Flash file not set')

        return status

    def reset_flash_files(self):

        self.nucleus_firmware_name = None
        self.dvl_firmware_name = None
        self.nucleus_firmware = None
        self.dvl_firmware = None

    def compare_firmware(self):

        nucleus_rematch = None
        dvl_rematch = None

        nucleus_firmware_match = None
        dvl_firmware_match = None

        firmware_version = dict()
        firmware_name_version = dict()

        for key in ['MAJOR', 'MINOR', 'PATCH', 'EXTRA', 'BUILD', 'DVLFW', 'DVLMINOR']:
            if key == 'EXTRA':
                firmware_name_version[key] = '""'
                firmware_version[key] = '""'
            elif key == 'BUILD':
                firmware_name_version[key] = '0'
                firmware_version[key] = None
            else:
                firmware_name_version[key] = None
                firmware_version[key] = None

        for i in range(5):
            reply = self.commands.get_fw(_nmea=True)

            if len(reply) == 2 and reply[1] == b'$PNOR,OK*2B\r\n':
                break

        else:
            self.messages.write_warning('was not able to retrieve firmware information')
            return False, False, ''

        get_fw = reply[0].lstrip(b'$PNOR,GETFW,').split(b'*')[0]
        get_fw_list = get_fw.decode().split(',')

        for entry in get_fw_list:
            try:
                data = entry.split('=')
                if data[0] in firmware_version.keys():
                    firmware_version[data[0]] = data[1]
            except Exception as e:
                self.messages.write_warning('Encountered an error when parsing get_fw data: {}'.format(e))

        if self.nucleus_firmware_name is not None:
            nucleus_rematch = re.match(self.BIN_FILE_PATTERN, self.nucleus_firmware_name)
            if nucleus_rematch is not None:
                firmware_name_version['MAJOR'] = nucleus_rematch[2]
                firmware_name_version['MINOR'] = nucleus_rematch[3]
                firmware_name_version['PATCH'] = nucleus_rematch[4]

                extra_build = nucleus_rematch[5].split('.')[0].split('-')

                for element in extra_build:
                    if 'internal' in element:
                        break

                    if element == '':
                        continue

                    try:
                        int(element)
                        firmware_name_version['BUILD'] = element
                        break
                    except ValueError:
                        firmware_name_version['EXTRA'] = '"{}"'.format(element)

        if self.dvl_firmware_name is not None:
            dvl_rematch = re.match(self.LDR_FILE_PATTERN, self.dvl_firmware_name)
            firmware_name_version['DVLFW'] = dvl_rematch[2]
            firmware_name_version['DVLMINOR'] = dvl_rematch[3]

        if self.nucleus_firmware_name is not None and not nucleus_rematch:
            self.messages.write_warning(message='Nucleus firmware does not contain version number in its file name')

        elif self.nucleus_firmware_name is not None:

            for key in ['MAJOR', 'MINOR', 'PATCH', 'EXTRA', 'BUILD']:
                if firmware_name_version[key] == firmware_version[key]:
                    nucleus_firmware_match = True
                else:
                    nucleus_firmware_match = False
                    break

        if self.dvl_firmware_name is not None and not dvl_rematch:
            self.messages.write_warning(message='DVL firmware does not contain version number in its file name')

        elif self.dvl_firmware_name is not None:

            for key in ['DVLFW', 'DVLMINOR']:
                if firmware_name_version[key] == firmware_version[key]:
                    dvl_firmware_match = True
                else:
                    dvl_firmware_match = False
                    break

        return nucleus_firmware_match, dvl_firmware_match, get_fw.decode()

    def pack_firmware(self, firmware):

        data_length = len(firmware)
        crc = crc32(firmware)
        firmware_package = bytearray(data_length.to_bytes(4, 'little'))
        firmware_package.extend(firmware)
        firmware_package.extend(crc.to_bytes(4, 'little'))

        return firmware_package

    def flash_firmware(self, password=None) -> int:

        if self.nucleus_firmware is None and self.dvl_firmware is None:
            self.messages.write_message(message='No firmware has been selected for flashing')
            return -1

        condition = 0

        nucleus_firmware_match, dvl_firmware_match = self.compare_firmware()[:2]

        if self.nucleus_firmware is not None and condition == 0 and nucleus_firmware_match is not True:
            condition = self.flash_nucleus_firmware(password=password)
        elif nucleus_firmware_match is True:
            self.messages.write_message(message='New Nucleus firmware matches old firmware. Nucleus flashing will not be executed')

        if self.dvl_firmware is not None and condition == 0 and dvl_firmware_match is not True:
            condition = self.flash_dvl_firmware()
        elif dvl_firmware_match is True:
            self.messages.write_message(message='New DVL firmware matches old firmware. DVL flashing will not be executed')

        if condition == 0:
            condition = self.confirm_firmware()

        if condition == 0:
            condition = self.set_default_values()

        return condition

    def flash_nucleus_firmware(self, password=None) -> int:

        if not self.connection.get_connection_status():
            self.messages.write_message(message='Nucleus not connected')
            return -101

        firmware_package = self.pack_firmware(self.nucleus_firmware)

        self.messages.write_message(message='Uploading Nucleus firmware...')

        reply = self.commands._upload(package=firmware_package)
        if reply is None:
            return -102
        elif b'ACK\r\n' not in reply:
            return -103
        elif b'OK\r\n' not in reply:
            return -104

        self.messages.write_message(message='Nucleus firmware uploaded')
        self.messages.write_message(message='Updating Nucleus firmware...')

        reply = self.commands._fw_update()
        if reply is None:
            return -105
        elif b'ACK\r\n' not in reply:
            return -106
        elif b'OK\r\n' not in reply:
            return -107

        self.messages.write_message(message='Nucleus firmware updated')
        self.messages.write_message(message='Restarting system, this could take up to 80 seconds...')

        if self.connection.get_connection_type() == 'serial':
            start_up_message = self.commands.read_start_up(timeout=80)
            if b'Nortek Fusion DVL1000\r\n' not in start_up_message[0] and b'Nortek Nucleus1000\r\n' not in start_up_message[0]:
                self.messages.write_message(message='System did not start up properly: {}'.format(start_up_message))
                return -108

            self.connection.get_info()

        elif self.connection.get_connection_type() == 'tcp':

            self.connection.disconnect()

            time.sleep(80)

            serial_number = self.connection.get_serial_number_from_tcp_hostname()

            if 'Fusion' in self.nucleus_firmware_name:
                condition = self.connection.set_tcp_hostname_from_serial_number(serial_number=serial_number, name='NortekFusion')
            else:
                condition = self.connection.set_tcp_hostname_from_serial_number(serial_number=serial_number, name='NORTEK')

            if condition is not True:
                self.messages.write_warning(message='Failed to retrieve hostname. Attempting to reconnect with old hostname')

            self.connection.connect(connection_type='tcp', password=password)

            if not self.connection.get_connection_status():
                self.messages.write_warning(message='Failed to connect through TCP with specified password. Trying default password...')
                self.connection.connect(connection_type='tcp', password='nortek')

            if not self.connection.get_connection_status():
                self.messages.write_warning(message='Failed to reconnect to TCP after Nucleus flashing')
                return -109

        else:
            self.messages.write_warning('Unrecognized connection type')
            return -110

        self.messages.write_message(message='System started up properly')

        return 0

    def flash_dvl_firmware(self) -> int:

        # TODO: Figure out why DVL flashing is violently unstable

        if not self.connection.get_connection_status():
            self.messages.write_message(message='Nucleus not connected')
            return -201

        firmware_package = self.pack_firmware(self.dvl_firmware)

        self.messages.write_message(message='Uploading DVL firmware...')

        reply = self.commands._upload(package=firmware_package)

        if reply is None:
            return -202
        elif b'ACK\r\n' not in reply:
            return -203
        elif b'OK\r\n' not in reply:
            return -204

        self.messages.write_message(message='DVL firmware uploaded')

        reply = self.commands._dvl_update()
        if reply is None:
            return -205
        elif b'ACK\r\n' not in reply:
            return -206

        update_in_progress = True
        while update_in_progress:

            response = self.connection.readline(timeout=5)

            if b'OK\r\n' in response:
                self.messages.write_message(message='DVL firmware updated')
                break

            elif b'FAIL -1\r\n' in response:
                self.messages.write_exception(message='Received FAIL -1. CRC failure')
                return -207

            elif b'DVLUPDATE,PROGRESS=' in response:
                progress = response.rstrip(b'\r\n').split(b'=')[-1].decode()
                try:
                    if int(progress) % 10 == 0:
                        self.messages.write_message(message='{}%'.format(progress))
                except Exception as exception:
                    self.messages.write_warning(message='Failed to output DVL flashing progress: {}'.format(exception))

            else:
                self.messages.write_warning(message='received a message that was not "PROGRESS" or "OK": {}'.format(response))
                return -208

        return 0

    def set_default_values(self) -> int:

        return 0  # TODO: This prevents setting of default values after flash

        if not self.connection.get_connection_status():
            self.messages.write_message(message='Nucleus not connected')
            return -301

        if b'OK\r\n' not in self.commands.set_default_config():
            self.messages.write_message(message='Did not receive OK when setting default config')
            return -302
        else:
            self.messages.write_message(message='default config values set')

        if b'OK\r\n' not in self.commands.save_config():
            self.messages.write_message(message='Did not receive OK when saving default config')
            return -303
        else:
            self.messages.write_message(message='default config values saved')

        if b'OK\r\n' not in self.commands.set_default_mission():
            self.messages.write_message(message='Did not receive OK when setting default mission')
            return -304
        else:
            self.messages.write_message(message='default mission values set')

        if b'OK\r\n' not in self.commands.save_mission():
            self.messages.write_message(message='Did not receive OK when saving default mission')
            return -305
        else:
            self.messages.write_message(message='default mission values saved')

        if b'OK\r\n' not in self.commands.set_default_magcal():
            self.messages.write_message(message='Did not receive OK when setting default magcal')
            return -306
        else:
            self.messages.write_message(message='default magcal values set')

        if b'OK\r\n' not in self.commands.save_magcal():
            self.messages.write_message(message='Did not receive OK when saving default magcal')
            return -307
        else:
            self.messages.write_message(message='default magcal values saved')

        return 0

    def confirm_firmware(self, password=None) -> int:

        if self.nucleus_firmware_name is None and self.dvl_firmware_name is None:
            return -401

        nucleus_firmware_match, dvl_firmware_match, version = self.compare_firmware()

        self.messages.write_message(message='New firmware version: {}'.format(version))

        if nucleus_firmware_match is not False and dvl_firmware_match is not False:
            reply = self.commands._fw_confirm()
            if b'OK\r\n' not in reply:
                return -402

            self.messages.write_message(message='New firmware confirmed')

        else:
            self.messages.write_warning(message='Firmware update failed. Nucleus did not start up properly')
            self.messages.write_warning(message='Reverting back to old firmware...')

            self.commands.reboot()

            if self.connection.get_connection_type() == 'serial':

                start_up_message = self.commands.read_start_up(timeout=100)

                if b'Nortek Fusion DVL1000\r\n' not in start_up_message[0] and b'Nortek Nucleus1000\r\n' not in start_up_message[0]:
                    self.messages.write_message(message='System did not start up properly')
                    return -403

            elif self.connection.get_connection_type == 'tcp':

                self.connection.disconnect()

                time.sleep(20)

                self.connection.connect(connection_type='tcp', password=password)

                if not self.connection.get_connection_status():
                    self.messages.write_warning(message='Failed to reconnect to TCP after Nucleus flashing')
                    return -404

            return -405

        self.messages.write_message(message='Firmware update complete')

        return 0
