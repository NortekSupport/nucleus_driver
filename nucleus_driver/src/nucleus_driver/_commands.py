from re import match
from datetime import datetime


class Commands:

    DHCP_STATIC = ['DHCP', 'STATIC']
    ON_OFF = ['ON', 'OFF']
    MODES = ['NORMAL', 'AUTO']
    PL_MODES = ['MAX', 'USER']
    TRIGGER_SOURCES = ['INTERNAL', 'EXTRISE', 'EXTFALL', 'EXTEDGES', 'COMMAND']
    ALTI_RANGE = [0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    CP_RANGE = [0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    MAG_METHODS = ['AUTO', 'OFF', 'WMM']

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

    def _start(self) -> [bytes]:

        self._reset_buffer()

        command = b'START\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=2)

        return get_reply

    def _stop(self, timeout=1) -> [bytes]:

        self._reset_buffer()

        command = b'STOP\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=timeout)

        return get_reply

    def trig(self) -> [bytes]:

        self._reset_buffer()

        trig_command = b'TRIG\r\n'

        self.connection.write(trig_command)

        get_reply = self._handle_reply(command=trig_command, terminator=b'OK\r\n')

        return get_reply

    def _fieldcal(self) -> [bytes]:

        self._reset_buffer()

        fieldcal_command = b'FIELDCAL\r\n'

        self.connection.write(fieldcal_command)

        get_reply = self._handle_reply(command=fieldcal_command, terminator=b'OK\r\n')

        return get_reply

    def reboot(self) -> [bytes]:

        self._reset_buffer()

        command = b'REBOOT\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def _fw_update(self, timeout=38) -> [bytes]:

        self._reset_buffer()

        command = b'FWUPDATE\r\n'

        self.connection.write(command)
        get_ack_reply = self._handle_reply(command=command, terminator=b'ACK\r\n')

        if b'ACK\r\n' not in get_ack_reply:
            return get_ack_reply

        get_ok_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=timeout)

        get_reply = list()
        get_reply.extend(get_ack_reply)
        get_reply.extend(get_ok_reply)

        return get_reply

    def _fw_confirm(self):

        self._reset_buffer()

        command = b'FWCONFIRM\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def _dvl_update(self) -> [bytes]:

        self._reset_buffer()

        command = b'DVLUPDATE\r\n'

        self.connection.write(command)
        get_reply = self._handle_reply(command=command, terminator=b'ACK\r\n')

        return get_reply

    def _upload(self, package) -> [bytes]:

        self._reset_buffer()

        command = b'UPLOAD\r\n'

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

    def restore_all(self) -> [bytes]:

        self._reset_buffer()

        command = b'RESTORE,ALL\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def restore_config(self) -> [bytes]:

        self._reset_buffer()

        command = b'RESTORE,CONFIG\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def restore_mission(self) -> [bytes]:

        self._reset_buffer()

        command = b'RESTORE,MISSION\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def restore_magcal(self) -> [bytes]:

        self._reset_buffer()

        command = b'RESTORE,MAGCAL\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def restore_comm(self) -> [bytes]:

        self._reset_buffer()

        command = b'RESTORE,COMM\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def set_mission(self, poff=None, long=None, lat=None, decl=None, range=None, bd=None, sv=None,
                    sa=None) -> [bytes]:

        self._reset_buffer()

        set_mission_command = b'SETMISSION'

        if poff is not None:
            if isinstance(poff, float) or isinstance(poff, int):
                set_mission_command += b',POFF=' + str(poff).encode()
            else:
                self.messages.write_warning('Invalid value for POFF in SETMISSION command')

        if long is not None:
            if isinstance(long, float) or isinstance(long, int):
                set_mission_command += b',LONG=' + str(long).encode()
            else:
                self.messages.write_warning('Invalid value for LONG in SETMISSION command')

        if lat is not None:
            if isinstance(lat, float) or isinstance(lat, int):
                set_mission_command += b',LAT=' + str(lat).encode()
            else:
                self.messages.write_warning('Invalid value for LAT in SETMISSION command')

        if decl is not None:
            if isinstance(decl, float) or isinstance(decl, int):
                set_mission_command += b',DECL=' + str(decl).encode()
            else:
                self.messages.write_warning('Invalid value for DECL in SETMISSION command')

        if range is not None:
            if isinstance(range, float) or isinstance(range, int):
                set_mission_command += b',RANGE=' + str(range).encode()
            else:
                self.messages.write_warning('Invalid value for RANGE in SETMISSION command')

        if bd is not None:
            if isinstance(bd, float) or isinstance(bd, int):
                set_mission_command += b',BD=' + str(bd).encode()
            else:
                self.messages.write_warning('Invalid value for BD in SETMISSION command')

        if sv is not None:
            if isinstance(sv, float) or isinstance(sv, int):
                set_mission_command += b',SV=' + str(sv).encode()
            else:
                self.messages.write_warning('Invalid value for SV in SETMISSION command')

        if sa is not None:
            if isinstance(sa, float) or isinstance(sa, int):
                set_mission_command += b',SA=' + str(sa).encode()
            else:
                self.messages.write_warning('Invalid value for SA in SETMISSION command')

        set_mission_command += b'\r\n'

        self.connection.write(set_mission_command)

        get_reply = self._handle_reply(command=set_mission_command, terminator=b'OK\r\n')

        return get_reply

    def get_mission(self, poff=False, long=False, lat=False, decl=False, range=False, bd=False, sv=False,
                    sa=False) -> [bytes]:

        self._reset_buffer()

        get_mission_command = b'GETMISSION'

        if poff is True:
            get_mission_command += b',POFF'

        if long is True:
            get_mission_command += b',LONG'

        if lat is True:
            get_mission_command += b',LAT'

        if decl is True:
            get_mission_command += b',DECL'

        if range is True:
            get_mission_command += b',RANGE'

        if bd is True:
            get_mission_command += b',BD'

        if sv is True:
            get_mission_command += b',SV'

        if sa is True:
            get_mission_command += b',SA'

        get_mission_command += b'\r\n'

        self.connection.write(get_mission_command)

        get_reply = self._handle_reply(command=get_mission_command, terminator=b'OK\r\n')

        return get_reply

    def get_mission_lim(self, poff=False, long=False, lat=False, decl=False, range=False, bd=False, sv=False,
                        sa=False) -> [bytes]:

        self._reset_buffer()

        get_mission_lim_command = b'GETMISSIONLIM'

        if poff is True:
            get_mission_lim_command += b',POFF'

        if long is True:
            get_mission_lim_command += b',LONG'

        if lat is True:
            get_mission_lim_command += b',LAT'

        if decl is True:
            get_mission_lim_command += b',DECL'

        if range is True:
            get_mission_lim_command += b',RANGE'

        if bd is True:
            get_mission_lim_command += b',BD'

        if sv is True:
            get_mission_lim_command += b',SV'

        if sa is True:
            get_mission_lim_command += b',SA'

        get_mission_lim_command += b'\r\n'

        self.connection.write(get_mission_lim_command)

        get_reply = self._handle_reply(command=get_mission_lim_command, terminator=b'OK\r\n')

        return get_reply

    def set_inst(self, rotxy=None, rotyz=None, rotxz=None, led=None) -> [bytes]:

        self._reset_buffer()

        set_inst_command = b'SETINST'

        if rotxy is not None:
            if isinstance(rotxy, float) or isinstance(rotxy, int):
                set_inst_command += b',ROTXY=' + str(rotxy).encode()
            else:
                self.messages.write_warning('Invalid value for ROTXY in SETINST command')

        if rotyz is not None:
            if isinstance(rotyz, float) or isinstance(rotyz, int):
                set_inst_command += b',ROTYZ=' + str(rotyz).encode()
            else:
                self.messages.write_warning('Invalid value for ROTYZ in SETINST command')

        if rotxz is not None:
            if isinstance(rotxz, float) or isinstance(rotxz, int):
                set_inst_command += b',ROTXZ=' + str(rotxz).encode()
            else:
                self.messages.write_warning('Invalid value for ROTXZ in SETINST command')

        if led is not None:
            if led.upper() in self.ON_OFF:
                set_inst_command += b',LED="' + led.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for LED in SETINST command')

        set_inst_command += b'\r\n'

        self.connection.write(set_inst_command)

        get_reply = self._handle_reply(command=set_inst_command, terminator=b'OK\r\n')

        return get_reply

    def get_inst(self, type=False, rotxy=False, rotyz=False, rotxz=False) -> [bytes]:

        self._reset_buffer()

        get_inst_command = b'GETINST'

        if type is True:
            get_inst_command += b',TYPE'

        if rotxy is True:
            get_inst_command += b',ROTXY'

        if rotyz is True:
            get_inst_command += b',ROTYZ'

        if rotxz is True:
            get_inst_command += b',ROTXZ'

        get_inst_command += b'\r\n'

        self.connection.write(get_inst_command)

        get_reply = self._handle_reply(command=get_inst_command, terminator=b'OK\r\n')

        return get_reply

    def get_inst_lim(self, type=False, rotxy=False, rotyz=False, rotxz=False) -> [bytes]:

        self._reset_buffer()

        get_inst_lim_command = b'GETINSTLIM'

        if type is True:
            get_inst_lim_command += b',TYPE'

        if rotxy is True:
            get_inst_lim_command += b',ROTXY'

        if rotyz is True:
            get_inst_lim_command += b',ROTYZ'

        if rotxz is True:
            get_inst_lim_command += b',ROTXZ'

        get_inst_lim_command += b'\r\n'

        self.connection.write(get_inst_lim_command)

        get_reply = self._handle_reply(command=get_inst_lim_command, terminator=b'OK\r\n')

        return get_reply

    def set_ahrs(self, freq=None, mode=None, ds=None, df=None) -> [bytes]:

        self._reset_buffer()

        set_ahrs_command = b'SETAHRS'

        if freq is not None:
            if isinstance(freq, int):
                set_ahrs_command += b',FREQ=' + str(freq).encode()
            else:
                self.messages.write_warning('Invalid value for FREQ in SETAHRS command')

        if mode is not None:
            if isinstance(mode, int):
                set_ahrs_command += b',MODE=' + str(mode).encode()
            else:
                self.messages.write_warning('Invalid value for MODE in SETAHRS command')

        if ds is not None:
            if ds.upper() in self.ON_OFF:
                set_ahrs_command += b',DS="' + ds.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for DS in SETAHRS command')

        if df is not None:
            if isinstance(df, int):
                set_ahrs_command += b',DF=' + str(df).encode()
            else:
                self.messages.write_warning('Invalid value for DF in SETAHRS command')

        set_ahrs_command += b'\r\n'

        self.connection.write(set_ahrs_command)

        get_reply = self._handle_reply(command=set_ahrs_command, terminator=b'OK\r\n')

        return get_reply

    def get_ahrs(self, freq=False, mode=False, ds=False, df=False) -> [bytes]:

        self._reset_buffer()

        get_ahrs_command = b'GETAHRS'

        if freq is True:
            get_ahrs_command += b',FREQ'

        if mode is True:
            get_ahrs_command += b',MODE'

        if ds is True:
            get_ahrs_command += b',DS'

        if df is True:
            get_ahrs_command += b',DF'

        get_ahrs_command += b'\r\n'

        self.connection.write(get_ahrs_command)

        get_reply = self._handle_reply(command=get_ahrs_command, terminator=b'OK\r\n')

        return get_reply

    def get_ahrs_lim(self, freq=False, mode=False, ds=False, df=False) -> [bytes]:

        self._reset_buffer()

        get_ahrs_lim_command = b'GETAHRSLIM'

        if freq is True:
            get_ahrs_lim_command += b',FREQ'

        if mode is True:
            get_ahrs_lim_command += b',MODE'

        if ds is True:
            get_ahrs_lim_command += b',DS'

        if df is True:
            get_ahrs_lim_command += b',DF'

        get_ahrs_lim_command += b'\r\n'

        self.connection.write(get_ahrs_lim_command)

        get_reply = self._handle_reply(command=get_ahrs_lim_command, terminator=b'OK\r\n')

        return get_reply

    def set_fieldcal(self, mode=None) -> [bytes]:

        self._reset_buffer()

        command = b'SETFIELDCAL'

        if mode is not None:
            if isinstance(mode, int) and mode in [1, 2]:
                command += b',MODE=' + str(mode).encode()
            else:
                self.messages.write_warning('Invalid value for MODE in SETFIELDCAL command')

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_fieldcal(self, mode=False) -> [bytes]:

        self._reset_buffer()

        command = b'GETFIELDCAL'

        if mode is True:
            command += b',MODE'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_fieldcal_lim(self, mode=False) -> [bytes]:

        self._reset_buffer()

        command = b'GETFIELDCALLIM'

        if mode is True:
            command += b',MODE'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def set_bt(self, mode=None, vr=None, wt=None, pl=None, pl_mode=None, ds=None, df=None) -> [bytes]:

        self._reset_buffer()

        set_bt_command = b'SETBT'

        if mode is not None:
            if isinstance(mode, str) and mode.upper() in self.MODES:
                set_bt_command += b',MODE="' + mode.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for MODE in SETBT command')

        if vr is not None:
            if isinstance(vr, int) or isinstance(vr, float):
                set_bt_command += b',VR=' + str(vr).encode()
            else:
                self.messages.write_warning('Invalid value for VR in SETBT command')

        if wt is not None:
            if wt.upper() in self.ON_OFF:
                set_bt_command += b',WT="' + wt.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for WT in SETBT command')

        if pl is not None:
            if isinstance(pl, int) or isinstance(pl, float):
                set_bt_command += b',PL=' + str(pl).encode()
            else:
                self.messages.write_warning('Invalid value for PL in SETBT command')

        if pl_mode is not None:
            if isinstance(pl_mode, str) and pl_mode.upper() in self.PL_MODES:
                set_bt_command += b',PLMODE="' + pl_mode.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for PLMODE in SETBT command')

        if ds is not None:
            if ds.upper() in self.ON_OFF:
                set_bt_command += b',DS="' + ds.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for DS in SETBT command')

        if df is not None:
            if isinstance(df, int):
                set_bt_command += b',DF=' + str(df).encode()
            else:
                self.messages.write_warning('Invalid value for DF in SETBT command')

        set_bt_command += b'\r\n'

        self.connection.write(set_bt_command)

        get_reply = self._handle_reply(command=set_bt_command, terminator=b'OK\r\n')

        return get_reply

    def get_bt(self, vr=False, wt=False, pl=False, pl_mode=False, ds=False, df=False) -> [bytes]:

        self._reset_buffer()

        get_bt_command = b'GETBT'

        if vr is True:
            get_bt_command += b',VR'

        if wt is True:
            get_bt_command += b',WT'

        if pl is True:
            get_bt_command += b',PL'

        if pl_mode is True:
            get_bt_command += b',PLMODE'

        if ds is True:
            get_bt_command += b',DS'

        if df is True:
            get_bt_command += b',DF'

        get_bt_command += b'\r\n'

        self.connection.write(get_bt_command)

        get_reply = self._handle_reply(command=get_bt_command, terminator=b'OK\r\n')

        return get_reply

    def get_bt_lim(self, vr=False, wt=False, pl=False, pl_mode=False, ds=False, df=False) -> [bytes]:

        self._reset_buffer()

        get_bt_lim_command = b'GETBTLIM'

        if vr is True:
            get_bt_lim_command += b',VR'

        if wt is True:
            get_bt_lim_command += b',WT'

        if pl is True:
            get_bt_lim_command += b',PL'

        if pl_mode is True:
            get_bt_lim_command += b',PLMODE'

        if ds is True:
            get_bt_lim_command += b',DS'

        if df is True:
            get_bt_lim_command += b',DF'

        get_bt_lim_command += b'\r\n'

        self.connection.write(get_bt_lim_command)

        get_reply = self._handle_reply(command=get_bt_lim_command, terminator=b'OK\r\n')

        return get_reply

    def set_alti(self, pl=None, ds=None, df=None) -> [bytes]:

        self._reset_buffer()

        set_alti_command = b'SETALTI'

        if pl is not None:
            if isinstance(pl, int) or isinstance(pl, float):
                set_alti_command += b',PL=' + str(pl).encode()
            else:
                self.messages.write_warning('Invalid value for PL in SETALTI command')

        if ds is not None:
            if ds.upper() in self.ON_OFF:
                set_alti_command += b',DS="' + ds.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for DS in SETALTI command')

        if df is not None:
            if isinstance(df, int):
                set_alti_command += b',DF=' + str(df).encode()
            else:
                self.messages.write_warning('Invalid value for DF in SETALTI command')

        set_alti_command += b'\r\n'

        self.connection.write(set_alti_command)

        get_reply = self._handle_reply(command=set_alti_command, terminator=b'OK\r\n')

        return get_reply

    def get_alti(self, pl=False, ds=False, df=False) -> [bytes]:

        self._reset_buffer()

        get_alti_command = b'GETALTI'

        if pl is True:
            get_alti_command += b',PL'

        if ds is True:
            get_alti_command += b',DS'

        if df is True:
            get_alti_command += b',DF'

        get_alti_command += b'\r\n'

        self.connection.write(get_alti_command)

        get_reply = self._handle_reply(command=get_alti_command, terminator=b'OK\r\n')

        return get_reply

    def get_alti_lim(self, pl=False, ds=False, df=False) -> [bytes]:

        self._reset_buffer()

        get_alti_lim_command = b'GETALTILIM'

        if pl is True:
            get_alti_lim_command += b',PL'

        if ds is True:
            get_alti_lim_command += b',DS'

        if df is True:
            get_alti_lim_command += b',DF'

        get_alti_lim_command += b'\r\n'

        self.connection.write(get_alti_lim_command)

        get_reply = self._handle_reply(command=get_alti_lim_command, terminator=b'OK\r\n')

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

    def set_trig(self, src=None, freq=None, alti=None, cp=None) -> [bytes]:

        self._reset_buffer()

        set_trig_command = b'SETTRIG'

        if src is not None:
            if src.upper() in self.TRIGGER_SOURCES:
                set_trig_command += b',SRC="' + src.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for SRC in SETTRIG command')

        if freq is not None:
            if isinstance(freq, int):
                set_trig_command += b',FREQ=' + str(freq).encode()
            else:
                self.messages.write_warning('Invalid value for FREQ in SETTRIG command')

        if alti is not None:
            if isinstance(alti, int) and alti in self.ALTI_RANGE:
                set_trig_command += b',ALTI=' + str(alti).encode()
            else:
                self.messages.write_warning('Invalid value for ALTI in SETTRIG command')

        if cp is not None:
            if isinstance(cp, int) and cp in self.CP_RANGE:
                set_trig_command += b',CP=' + str(cp).encode()
            else:
                self.messages.write_warning('Invalid value for CP in SETTRIG command')

        set_trig_command += b'\r\n'

        self.connection.write(set_trig_command)

        get_reply = self._handle_reply(command=set_trig_command, terminator=b'OK\r\n')

        return get_reply

    def get_trig(self, src=False, freq=False, alti=False, cp=False) -> [bytes]:

        self._reset_buffer()

        command = b'GETTRIG'

        if src is True:
            command += b',SRC'

        if freq is True:
            command += b',FREQ'

        if alti is True:
            command += b',ALTI'

        if cp is True:
            command += b',CP'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_trig_lim(self, src=False, freq=False, alti=False, cp=False) -> [bytes]:

        self._reset_buffer()

        command = b'GETTRIGLIM'

        if src is True:
            command += b',SRC'

        if freq is True:
            command += b',FREQ'

        if alti is True:
            command += b',ALTI'

        if cp is True:
            command += b',CP'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def set_mag(self, method=None) -> [bytes]:

        self._reset_buffer()

        command = b'SETMAG'

        if method is not None:
            if isinstance(method, str) and method.upper() in self.MAG_METHODS:
                command += b',METHOD="' + method.upper().encode() + b'"'
            else:
                self.messages.write_warning('Invalid value for METHOD in SETMAG command')

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_mag(self, method=False) -> [bytes]:

        self._reset_buffer()

        command = b'GETMAG'

        if method is True:
            command += b',METHOD'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_mag_lim(self, method=False) -> [bytes]:

        self._reset_buffer()

        command = b'GETMAGLIM'

        if method is True:
            command += b',METHOD'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def set_mag_cal(self, hx=None, hy=None, hz=None, m11=None, m12=None, m13=None, m21=None, m22=None, m23=None,
                    m31=None, m32=None, m33=None) -> [bytes]:

        self._reset_buffer()

        set_mag_cal_command = b'SETMAGCAL'

        if hx is not None:
            if isinstance(hx, int) or isinstance(hx, float):
                set_mag_cal_command += b',HX=' + str(hx).encode()
            else:
                self.messages.write_warning('Invalid value for HX in SETMAGCAL command')

        if hy is not None:
            if isinstance(hy, int) or isinstance(hy, float):
                set_mag_cal_command += b',HY=' + str(hy).encode()
            else:
                self.messages.write_warning('Invalid value for HY in SETMAGCAL command')

        if hz is not None:
            if isinstance(hz, int) or isinstance(hz, float):
                set_mag_cal_command += b',HZ=' + str(hz).encode()
            else:
                self.messages.write_warning('Invalid value for HZ in SETMAGCAL command')

        if m11 is not None:
            if isinstance(m11, int) or isinstance(m11, float):
                set_mag_cal_command += b',M11=' + str(m11).encode()
            else:
                self.messages.write_warning('Invalid value for M11 in SETMAGCAL command')

        if m12 is not None:
            if isinstance(m12, int) or isinstance(m12, float):
                set_mag_cal_command += b',M12=' + str(m12).encode()
            else:
                self.messages.write_warning('Invalid value for M12 in SETMAGCAL command')

        if m13 is not None:
            if isinstance(m13, int) or isinstance(m13, float):
                set_mag_cal_command += b',M13=' + str(m13).encode()
            else:
                self.messages.write_warning('Invalid value for M13 in SETMAGCAL command')

        if m21 is not None:
            if isinstance(m21, int) or isinstance(m21, float):
                set_mag_cal_command += b',M21=' + str(m21).encode()
            else:
                self.messages.write_warning('Invalid value for M21 in SETMAGCAL command')

        if m22 is not None:
            if isinstance(m22, int) or isinstance(m22, float):
                set_mag_cal_command += b',M22=' + str(m22).encode()
            else:
                self.messages.write_warning('Invalid value for M22 in SETMAGCAL command')

        if m23 is not None:
            if isinstance(m23, int) or isinstance(m23, float):
                set_mag_cal_command += b',M23=' + str(m23).encode()
            else:
                self.messages.write_warning('Invalid value for M23 in SETMAGCAL command')

        if m31 is not None:
            if isinstance(m31, int) or isinstance(m31, float):
                set_mag_cal_command += b',M31=' + str(m31).encode()
            else:
                self.messages.write_warning('Invalid value for M31 in SETMAGCAL command')

        if m32 is not None:
            if isinstance(m32, int) or isinstance(m32, float):
                set_mag_cal_command += b',M32=' + str(m32).encode()
            else:
                self.messages.write_warning('Invalid value for M32 in SETMAGCAL command')

        if m33 is not None:
            if isinstance(m33, int) or isinstance(m33, float):
                set_mag_cal_command += b',M33=' + str(m33).encode()
            else:
                self.messages.write_warning('Invalid value for M33 in SETMAGCAL command')

        set_mag_cal_command += b'\r\n'

        self.connection.write(set_mag_cal_command)

        get_reply = self._handle_reply(command=set_mag_cal_command, terminator=b'OK\r\n')

        return get_reply

    def get_mag_cal(self, hx=False, hy=False, hz=False, m11=False, m12=False, m13=False, m21=False, m22=False,
                    m23=False, m31=False, m32=False, m33=False) -> [bytes]:

        self._reset_buffer()

        get_mag_cal_command = b'GETMAGCAL'

        if hx is True:
            get_mag_cal_command += b',HX'

        if hy is True:
            get_mag_cal_command += b',HY'

        if hz is True:
            get_mag_cal_command += b',HZ'

        if m11 is True:
            get_mag_cal_command += b',M11'

        if m12 is True:
            get_mag_cal_command += b',M12'

        if m13 is True:
            get_mag_cal_command += b',M13'

        if m21 is True:
            get_mag_cal_command += b',M21'

        if m22 is True:
            get_mag_cal_command += b',M22'

        if m23 is True:
            get_mag_cal_command += b',M23'

        if m31 is True:
            get_mag_cal_command += b',M31'

        if m32 is True:
            get_mag_cal_command += b',M32'

        if m33 is True:
            get_mag_cal_command += b',M33'

        get_mag_cal_command += b'\r\n'

        self.connection.write(get_mag_cal_command)

        get_reply = self._handle_reply(command=get_mag_cal_command, terminator=b'OK\r\n')

        return get_reply

    def get_mag_cal_lim(self, hx=False, hy=False, hz=False, m11=False, m12=False, m13=False, m21=False,
                        m22=False, m23=False, m31=False, m32=False, m33=False) -> [bytes]:

        self._reset_buffer()

        get_mag_cal_lim_command = b'GETMAGCAL'

        if hx is True:
            get_mag_cal_lim_command += b',HX'

        if hy is True:
            get_mag_cal_lim_command += b',HY'

        if hz is True:
            get_mag_cal_lim_command += b',HZ'

        if m11 is True:
            get_mag_cal_lim_command += b',M11'

        if m12 is True:
            get_mag_cal_lim_command += b',M12'

        if m13 is True:
            get_mag_cal_lim_command += b',M13'

        if m21 is True:
            get_mag_cal_lim_command += b',M21'

        if m22 is True:
            get_mag_cal_lim_command += b',M22'

        if m23 is True:
            get_mag_cal_lim_command += b',M23'

        if m31 is True:
            get_mag_cal_lim_command += b',M31'

        if m32 is True:
            get_mag_cal_lim_command += b',M32'

        if m33 is True:
            get_mag_cal_lim_command += b',M33'

        get_mag_cal_lim_command += b'\r\n'

        self.connection.write(get_mag_cal_lim_command)

        get_reply = self._handle_reply(command=get_mag_cal_lim_command, terminator=b'OK\r\n')

        return get_reply

    def set_eth(self, ipmethod=None, ip=None, netmask=None, gateway=None, password=None):

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

        if password is not None:
            command += b',PASSWORD="' + password.encode() + b'"'

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

    def get_id(self, sn=False, str=False, _timeout = 1) -> [bytes]:

        self._reset_buffer()

        command = b'ID'

        if sn is True:
            command += b',SN'

        if str is True:
            command += b',STR'

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=_timeout)

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

        command += b'\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n', nmea=_nmea)

        return get_reply

    def get_all(self):

        self._reset_buffer()

        command = b'GETALL\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def set_clockstring(self, clockstring: str) -> [bytes]:

        self._reset_buffer()

        command = 'SETCLOCKSTR,TIME="{}"\r\n'.format(clockstring).encode()

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def get_clockstring(self) -> [bytes]:

        self._reset_buffer()

        command = b'GETCLOCKSTR\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def list_license(self) -> [bytes]:

        self._reset_buffer()

        command = b'LISTLICENSE\r\n'

        self.connection.write(command)

        get_reply = self._handle_reply(command=command, terminator=b'OK\r\n')

        return get_reply

    def read_assert(self, timeout=1) -> [bytes]:

        self._reset_buffer()

        command = b'READASSERT\r\n'

        self.connection.write(command)

        reply = self._handle_reply(command=command, terminator=b'OK\r\n', timeout=timeout)

        get_reply = b''
        for entry in reply:
            get_reply += entry

        if get_reply[-4:] == b'OK\r\n':
            get_reply_list = [get_reply[:-4], get_reply[-4:]]
        else:
            get_reply_list = [get_reply]

        return get_reply_list

    def clear_assert(self) -> [bytes]:

        self._reset_buffer()

        set_default_mission_command = b'CLEARASSERT\r\n'

        self.connection.write(set_default_mission_command)

        get_reply = self._handle_reply(command=set_default_mission_command, terminator=b'OK\r\n')

        return get_reply

    def read_syslog(self) -> [bytes]:

        self._reset_buffer()

        read_syslog_command = b'READSYSLOG\r\n'

        self.connection.write(read_syslog_command)

        data = bytes()

        while True:

            readout = self.connection.read(terminator=b'OK\r\n', timeout=1)

            if readout == b'':
                break
            else:
                data += readout

        if data[-4:] == b'OK\r\n':
            data_list = [data[:-4], data[-4:]]
        else:
            data_list = [data]

        return data_list

    def clear_syslog(self) -> [bytes]:

        self._reset_buffer()

        clear_syslog_command = b'CLEARSYSLOG\r\n'

        self.connection.write(clear_syslog_command)

        get_reply = self._handle_reply(command=clear_syslog_command, terminator=b'OK\r\n', timeout=5)

        return get_reply

    def list_syslog(self) -> [bytes]:

        self._reset_buffer()

        list_syslog_command = b'LISTSYSLOG\r\n'

        self.connection.write(list_syslog_command)

        get_reply = self._handle_reply(command=list_syslog_command, terminator=b'OK\r\n')

        return get_reply

    def list_files(self, src=None):

        self._reset_buffer()

        listfiles_command = b'LISTFILES'

        if src is not None:
            if isinstance(src, int):
                listfiles_command += b',SRC=' + str(src).encode()
            else:
                self.messages.write_warning('Invalid value for SRC in DOWNLOAD command')

        listfiles_command += b'\r\n'

        self.connection.write(listfiles_command)

        get_reply = self._handle_reply(command=listfiles_command, terminator=b'OK\r\n', timeout=10)

        return get_reply
