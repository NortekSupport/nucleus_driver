from re import match
from datetime import datetime


class Commands:
    DHCP_STATIC = ['DHCP', 'STATIC']
    ADDRESS_PATTERN = r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$'

    def __init__(self, **kwargs):

        self.connection = kwargs.get('connection')
        self.messages = kwargs.get('messages')
        self.parser = kwargs.get('parser')

    def _reset_buffer(self):

        if self.parser.thread_running is not True:
            self.connection.reset_buffers()
        elif self.parser.get_queuing()['ascii'] is True:
            self.parser.clear_queue(queue_name='ascii')

    def _get_reply(self, terminator: bytes = None, timeout: int = 1) -> [bytes]:

        get_reply = b''

        if self.parser.thread_running is not True:
            get_reply = self.connection.read(terminator=terminator, timeout=timeout)
        elif self.parser.get_queuing()['ascii'] is True:
            init_time = datetime.now()
            while (datetime.now() - init_time).seconds < timeout:
                ascii_packet = self.parser.read_ascii()
                if ascii_packet is not None:
                    get_reply += ascii_packet['bytes']

                if terminator is not None and terminator in get_reply:
                    break

        return get_reply

    def _check_reply(self, data: bytes, command: bytes, terminator: bytes = None):

        if terminator is not None and terminator not in data:
            if b'ERROR' in data:
                get_error_reply = self.get_error().rstrip(b'OK\r\n')
                self.messages.write_exception(message='Received ERROR instead of {} after sending {}: {}'.format(terminator, command, get_error_reply))

            else:
                self.messages.write_warning(message='Did not receive {} after sending {}: {}'.format(terminator, command, data))

        elif terminator is None:
            if b'ERROR' in data:
                get_error_reply = self.get_error().rstrip(b'OK\r\n')
                self.messages.write_exception(message='Received ERROR after sending {}: {}'.format(command, get_error_reply))

    def _handle_reply(self, command, terminator: bytes = None, timeout: int = 1, nmea=False) -> [bytes]:

        if nmea:
            terminator = b'$PNOR,' + terminator.rstrip(b'\r\n')
            nmea_checksum = self._nmea_checksum(terminator)
            terminator = terminator + b'*' + nmea_checksum + b'\r\n'

        get_reply = self._get_reply(terminator=terminator, timeout=timeout)
        self._check_reply(data=get_reply, terminator=terminator, command=command)

        reply_list = [i + b'\r\n' for i in get_reply.split(b'\r\n') if i]

        if nmea:
            for reply in reply_list:
                reply_split = reply.split(b'*')
                nmea_checksum = self._nmea_checksum(reply_split[0])
                if nmea_checksum != reply_split[1].rstrip(b'\r\n'):
                    self.messages.write_warning('Reply did not pass nmea checksum: {}\t{}'.format(reply, nmea_checksum))

        return reply_list

    def _nmea_checksum(self, command):

        checksum = 0

        command = command.lstrip(b'$')

        for byte in command:
            checksum ^= byte

        return '{0:02X}'.format(checksum).encode()

    def get_error(self) -> [bytes]:

        self._reset_buffer()

        command = b'GETERROR\r\n'

        self.connection.write(command)

        get_reply = self._get_reply(terminator=b'OK\r\n')

        return get_reply

    def start(self) -> [bytes]:

        self._reset_buffer()

        command = b'START\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=2)

        return get_reply

    def stop(self, timeout=1) -> [bytes]:

        self._reset_buffer()

        command = b'STOP\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=timeout)

        return get_reply

    def fieldcal(self) -> [bytes]:

        self._reset_buffer()

        fieldcal_command = b'FIELDCAL\r\n'

        self.connection.write(fieldcal_command)

        get_reply = self._handle_reply(command=fieldcal_command, terminator=b'OK\r\n')

        return get_reply

    def start_spectrum(self, timeout=1) -> [bytes]:

        self._reset_buffer()

        command = b'STARTSPECTRUM\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=timeout)

        return get_reply

    def reset(self) -> [bytes]:

        self._reset_buffer()

        command = b'RESET\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_id(self, sn=False, str=False) -> [bytes]:

        self._reset_buffer()

        command = b'ID'

        if sn is True:
            command += b',SN'

        if str is True:
            command += b',STR'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_fw(self, str=False, major=False, minor=False, patch=False, extra=False, build=False, hash=False, dvlfw=False, dvlminor=False, dvlboot=False, dvlfpga=False, _nmea=False) -> [bytes]:

        self._reset_buffer()

        command = b''

        if _nmea is True:
            command += b'$PNOR,'

        command += b'GETFW'

        if str is True:
            command += b',STR'

        if major is True:
            command += b',MAJOR'

        if minor is True:
            command += b',MINOR'

        if patch is True:
            command += b',PATCH'

        if extra is True:
            command += b',EXTRA'

        if build is True:
            command += b',BUILD'

        if hash is True:
            command += b',HASH'

        if dvlfw is True:
            command += b',DVLFW'

        if dvlminor is True:
            command += b',DVLMINOR'

        if dvlboot is True:
            command += b',DVLBOOT'

        if dvlfpga is True:
            command += b',DVLFPGA'

        if _nmea is True:
            nmea_checksum = self._nmea_checksum(command)
            command += b'*'
            command += nmea_checksum

        #if _nmea is True:
        #    command = _append_nmea()

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', nmea=_nmea)

        return get_reply

    def get_hw(self, digital=False, analog=False) -> [bytes]:

        self._reset_buffer()

        command = b'GETHW'

        if digital is True:
            command += b',DIGITAL'

        if analog is True:
            command += b',ANALOG'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_all(self):

        self._reset_buffer()

        command = b'GETALL\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def set_eth(self, ipmethod=None, ip=None, netmask=None, gateway=None):

        self._reset_buffer()

        command = b'SETETH'

        if ipmethod is not None:
            if ipmethod.upper() in self.DHCP_STATIC:
                command += b',IPMETHOD="' + ipmethod.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for IPMETHOD in SETETH command')

        if ip is not None:
            if match(self.ADDRESS_PATTERN, ip):
                command += b',IP="' + ip.encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for IP in SETETH command')

        if netmask is not None:
            if match(self.ADDRESS_PATTERN, netmask):
                command += b',NETMASK="' + netmask.encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for NETMASK in SETETH command')

        if gateway is not None:
            if match(self.ADDRESS_PATTERN, gateway):
                command += b',GATEWAY="' + gateway.encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for GATEWAY in SETETH command')

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_eth(self, ipmethod=False, ip=False, netmask=False, gateway=False):

        self._reset_buffer()

        command = b'GETETH'

        if ipmethod is True:
            command += b',IPMETHOD'

        if ip is True:
            command += b',IP'

        if netmask is True:
            command += b',NETMASK'

        if gateway is True:
            command += b',GATEWAY'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def read_ip(self, ip=False, netmask=False, gateway=False, leasetime=False):

        self._reset_buffer()

        command = b'READIP'

        if ip is True:
            command += b',IP'

        if netmask is True:
            command += b',NETMASK'

        if gateway is True:
            command += b',GATEWAY'

        if leasetime is True:
            command += b',LEASETIME'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_eth_lim(self, ipmethod=False, ip=False, netmask=False, gateway=False):

        self._reset_buffer()

        command = b'GETETHLIM'

        if ipmethod is True:
            command += b',IPMETHOD'

        if ip is True:
            command += b',IP'

        if netmask is True:
            command += b',NETMASK'

        if gateway is True:
            command += b',GATEWAY'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def set_cur_prof(self, profile_range=None, cs=None, bd=None, ds=None, df=None) -> [bytes]:

        self._reset_buffer()

        command = b'SETCURPROF'

        if profile_range is not None:
            if isinstance(profile_range, int) or isinstance(profile_range, float):
                command += b',RANGE=' + str(profile_range).encode()
            else:
                self.messages.write_warning('Invalid value for RANGE in SETCURPROF command')

        if cs is not None:
            if isinstance(cs, int) or isinstance(cs, float):
                command += b',CS=' + str(cs).encode()
            else:
                self.messages.write_warning('Invalid value for CS in SETCURPROF command')

        if bd is not None:
            if isinstance(bd, int) or isinstance(bd, float):
                command += b',BD=' + str(bd).encode()
            else:
                self.messages.write_warning('Invalid value for BD in SETCURPROF command')

        if ds is not None:
            if ds.upper() in self.ON_OFF:
                command += b',DS="' + ds.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for DS in SETCURPROF command')

        if df is not None:
            if isinstance(df, int):
                command += b',DF=' + str(df).encode()
            else:
                self.messages.write_warning('Invalid value for DF in SETCURPROF command')

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_cur_prof(self, profile_range=False, cs=False, bd=False, ds=False, df=False):

        self._reset_buffer()

        command = b'GETCURPROF'

        if profile_range is True:
            command += b',RANGE'

        if cs is True:
            command += b',CS'

        if bd is True:
            command += b',BD'

        if ds is True:
            command += b',DS'

        if df is True:
            command += b',DF'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_cur_prof_lim(self, profile_range=False, cs=False, bd=False, ds=False, df=False):

        self._reset_buffer()

        command = b'GETCURPROFLIM'

        if profile_range is True:
            command += b',RANGE'

        if cs is True:
            command += b',CS'

        if bd is True:
            command += b',BD'

        if ds is True:
            command += b',DS'

        if df is True:
            command += b',DF'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def fw_update(self, timeout=38) -> [bytes]:

        self._reset_buffer()

        command = b'FWUPDATE\r\n'

        self.connection.write(command)
        get_ack_reply = self._handle_reply(command=command, terminator=b'ACK\r\n')

        if b'ACK\r\n' not in get_ack_reply:
            return get_ack_reply

        # The following timeout is defined based on the maximum time each of the following operations may take.
        # The timeouts are given by the following calculations: https://dev.azure.com/NortekGroup/9f822050-97e3-47d7-9697-3773608ceeff/_apis/git/repositories/4dea5944-f474-4ef9-b642-a82f0e306b66/items?path=/doc/flash.ods&versionDescriptor%5BversionOptions%5D=0&versionDescriptor%5BversionType%5D=0&versionDescriptor%5Bversion%5D=master&resolveLfs=true&%24format=octetStream&api-version=5.0&download=true
        get_ok_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=timeout)

        get_reply = list()
        get_reply.extend(get_ack_reply)
        get_reply.extend(get_ok_reply)

        return get_reply

    def fw_confirm(self):

        self._reset_buffer()

        command = b'FWCONFIRM\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def dvl_update(self) -> [bytes]:

        self._reset_buffer()

        command = b'DVLUPDATE\r\n'

        self.connection.write(command)
        get_reply = self._handle_reply(command=command, terminator=b'ACK\r\n')

        return get_reply

    def upload(self, package) -> [bytes]:

        self._reset_buffer()

        # command = b'UPLOAD\r\n'
        command = b'UPLOAD\r'  # TODO: add b'\n' back into command when bug related to serial buffer is fixed

        self.connection.write(command)
        get_ack_reply = self._handle_reply(command=command, terminator=b'ACK\r\n')

        if b'ACK\r\n' not in get_ack_reply:
            return get_ack_reply

        self.connection.write(package)
        get_ok_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        get_reply = list()
        get_reply.extend(get_ack_reply)
        get_reply.extend(get_ok_reply)

        return get_reply

    def save_all(self) -> [bytes]:

        self._reset_buffer()

        command = b'SAVE,ALL\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def save_config(self) -> [bytes]:

        self._reset_buffer()

        save_config_command = b'SAVE,CONFIG\r\n'

        self.connection.write(save_config_command)

        get_reply = self._handle_reply(command=save_config_command, terminator=b'OK\r\n')

        return get_reply

    def save_mission(self) -> [bytes]:

        self._reset_buffer()

        save_mission_command = b'SAVE,MISSION\r\n'

        self.connection.write(save_mission_command)

        get_reply = self._handle_reply(command=save_mission_command, terminator=b'OK\r\n')

        return get_reply

    def save_magcal(self) -> [bytes]:

        self._reset_buffer()

        save_magcal_command = b'SAVE,MAGCAL\r\n'

        self.connection.write(save_magcal_command)

        get_reply = self._handle_reply(command=save_magcal_command, terminator=b'OK\r\n')

        return get_reply

    def save_comm(self) -> [bytes]:

        self._reset_buffer()

        command = b'SAVE,COMM\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def set_default_all(self) -> [bytes]:

        self._reset_buffer()

        command = b'SETDEFAULT,ALL\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def set_default_config(self) -> [bytes]:

        self._reset_buffer()

        set_default_config_command = b'SETDEFAULT,CONFIG\r\n'

        self.connection.write(set_default_config_command)

        get_reply = self._handle_reply(command=set_default_config_command, terminator=b'OK\r\n')

        return get_reply

    def set_default_mission(self) -> [bytes]:

        self._reset_buffer()

        set_default_mission_command = b'SETDEFAULT,MISSION\r\n'

        self.connection.write(set_default_mission_command)

        get_reply = self._handle_reply(command=set_default_mission_command, terminator=b'OK\r\n')

        return get_reply

    def set_default_magcal(self) -> [bytes]:

        self._reset_buffer()

        set_default_magcal_command = b'SETDEFAULT,MAGCAL\r\n'

        self.connection.write(set_default_magcal_command)

        get_reply = self._handle_reply(command=set_default_magcal_command, terminator=b'OK\r\n')

        return get_reply

    def set_default_comm(self) -> [bytes]:

        self._reset_buffer()

        command = b'SETDEFAULT,COMM\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def read_start_up(self, timeout: int = 1) -> [bytes]:

        reply = self._get_reply(terminator=b'OK\r\n', timeout=timeout)
        # print(reply)
        # for i in range(5):
        #    reply += self._get_reply()
        #    print(reply)
        #    if b'OK\r\n' in reply:
        #        break

        # TODO: Add _handle_reply() ?

        start_up_message = [reply]

        return start_up_message
