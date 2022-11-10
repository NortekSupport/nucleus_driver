from pathlib import Path
from binascii import crc32
from datetime import datetime


class Download:

    def __init__(self, **kwargs):

        self.messages = kwargs.get('messages')
        self.connection = kwargs.get('connection')
        self.commands = kwargs.get('commands')
        self.parser = kwargs.get('parser')
        self.logger = kwargs.get('logger')

        self._path = str(Path.cwd()) + '/download'

        self.dvl_download_statistics = dict()
        self.dvl_download_statistics['successful bytes'] = 0
        self.dvl_download_statistics['failed bytes'] = 0

        self.PACKET_LENGTH = 1024 * 10

    @staticmethod
    def _handle_crc(dvl_data: bytes):

        crc_fail_data = b''

        while True:
            crc_fail_data += dvl_data[:1]
            dvl_data = dvl_data[1:]

            if len(dvl_data) <= 1:
                return dvl_data, crc_fail_data

            elif len(dvl_data) <= 2 and dvl_data[:2] != b'\x00\x00' and int.from_bytes(dvl_data[:2], 'little') <= int.from_bytes(b'\x00\x40', 'little'):
                return dvl_data, crc_fail_data

            elif len(dvl_data) <= 3 and dvl_data[:2] != b'\x00\x00' and dvl_data[2:3] == b'\x00' and int.from_bytes(dvl_data[:2], 'little') <= int.from_bytes(b'\x00\x40', 'little'):
                return dvl_data, crc_fail_data

            elif dvl_data[:2] != b'\x00\x00' and dvl_data[2:4] == b'\x00\x00' and int.from_bytes(dvl_data[:2], 'little') <= int.from_bytes(b'\x00\x40', 'little'):
                return dvl_data, crc_fail_data

    @staticmethod
    def progress_bar(current, total, bar_length=20):
        fraction = current / total

        arrow = int(fraction * bar_length - 1) * '-' + '>'
        padding = int(bar_length - len(arrow)) * ' '

        ending = '\n' if current == total else '\r'

        print(f'Progress: [{arrow}{padding}] {int(fraction * 100)}%', end=ending)

    def set_path(self, path):

        self._path = path.rstrip('/') + '/download'

        return self._path

    def get_download_parameters(self, fid: int, src: int, sa: int, length: int) -> (bool, dict):

        def _get_file_list():

            if src not in [0, 1]:
                self.messages.write_warning('src argument must be 0 or 1')
                return False

            list_files = self.commands.list_files(src=src)

            if len(list_files) == 0 or list_files[-1] != b'OK\r\n':
                self.messages.write_warning('Not able to obtain file list from device')
                return False

            elif len(list_files) == 1 and list_files[0] == b'OK\r\n':
                self.messages.write_warning('No files available on device')
                return False

            # extract fids and lengths from file list
            for entry in list_files:
                if entry == b'OK\r\n':
                    break

                id = None
                for data in entry.split(b','):
                    if b'FID' in data:
                        try:
                            id = int(data.split(b'=')[1].decode())
                        except (ValueError, IndexError, AttributeError):
                            self.messages.write_warning('Failed to extract id from entry. Skipping this entry')
                            break

                    elif b'LEN' in data:
                        if id is not None:
                            try:
                                len_data = data.split(b'=')
                                fid_dict[id] = {'length': int(len_data[1])}
                            except (ValueError, IndexError, AttributeError):
                                self.messages.write_warning('Failed to extract len from entry. Skipping this entry')
                                break
                        else:
                            self.messages.write_warning('Found LEN before FID in download entry. entry not appended til list of available fids')
                            break

            if len(fid_dict.keys()) < 1:
                self.messages.write_warning('Not able to extract any entries from file list')
                return False

            return True

        def _get_download_parameters():

            if fid is None:
                try:
                    download_parameters['fid'] = max(fid_dict.keys())
                except (TypeError, ValueError):
                    self.messages.write_warning('Was not able to obtain latest fid from FID list')
                    return False

            elif fid in fid_dict.keys():
                download_parameters['fid'] = fid

            else:
                self.messages.write_warning('Could not find specified fid on device')
                return False

            if sa is None:
                download_parameters['sa'] = 0

            elif sa < fid_dict[download_parameters['fid']]['length']:
                download_parameters['sa'] = sa

            else:
                self.messages.write_warning('sa argument is larger or equal to length of specified fid')
                return False

            if length is None:
                download_parameters['end'] = fid_dict[download_parameters['fid']]['length']

            elif download_parameters['sa'] + length <= fid_dict[download_parameters['fid']]['length']:
                download_parameters['end'] = download_parameters['sa'] + length

            else:
                self.messages.write_warning('Specified sa and length argument is larger the file length. Setting end parameter to file length')
                download_parameters['end'] = fid_dict[download_parameters['fid']]['length']

            if download_parameters['fid'] is None or download_parameters['sa'] is None or download_parameters['end'] is None:
                self.messages.write_warning('Unable to specify all download parameters')
                return False

            return True

        fid_dict = dict()
        if not _get_file_list():
            return False, dict()

        download_parameters = dict()
        download_parameters['fid'] = None
        download_parameters['sa'] = None
        download_parameters['end'] = None
        if not _get_download_parameters():
            return False, dict()

        return True, download_parameters

    def download_data(self, fid, src, sa, length) -> (bool, bytes):

        expected_data_length = len(str(length)) + 2 + length + 2 + 10 + 4  # length of _len number + length of \r\n + length of _len + length of \r\n + length of crc and \r\n + length of OK\r\n

        command = b'DOWNLOAD,FID=' + str(fid).encode() + b',SRC=' + str(src).encode() + b',SA=' + str(sa).encode() + b',LEN=' + str(length).encode() + b',CRC=1\r\n'
        self.connection.write(command)
        data = self.connection.read(size=expected_data_length, timeout=3)
        self.commands._check_reply(data=data, terminator=b'OK\r\n', command=command)

        if len(data) != expected_data_length:
            self.messages.write_warning('received data from download reply is incorrect length')
            return False, b''

        length_reply = data[:len(str(length)) + 2]
        data_reply = data[len(str(length)) + 2:-14]
        crc_reply = data[-14:-4]
        ok_reply = data[-4:]

        if length_reply[-2:] != b'\r\n':
            self.messages.write_warning('unexpected format of the length reply from download command')
            return False, b''

        if data_reply[-2:] != b'\r\n':
            self.messages.write_warning('unexpected format of the data reply from download command')
            return False, b''

        if crc_reply[-2:] != b'\r\n':
            self.messages.write_warning('unexpected format of the crc reply from download command')
            return False, b''

        if ok_reply[-2:] != b'\r\n':
            self.messages.write_warning('unexpected format of the ok reply from download command')
            return False, b''

        try:
            crc = int(crc_reply[:-2].decode(), 16)
        except ValueError:
            self.messages.write_warning('Unable to convert received crc value to integer')
            return False, b''

        if crc32(data_reply[:-2]) == crc:
            return True, data_reply[:-2]
        else:
            return False, data_reply[:-2]

    def download_get_all_data(self, fid: int = None):

        def _extract_get_all(data):

            b = None
            b_prev = None
            c = data

            get_all = list()
            while True:
                if b is not None:
                    b_prev = b
                a, b, c = c.partition(b'\xa5')
                if b_prev is not None:
                    get_all.append(b_prev + a)
                if len(c) < 2:
                    break

            return get_all

        def _check_nucleus_data():

            if nucleus_data['sync bit'] != 0xa5:
                self.messages.write_warning('Incorrect "sync bit" for Nucleus get_all-data')
                return False

            if nucleus_data['header size'] != len(nucleus_data['header']):
                self.messages.write_warning('Incorrect "header size" for Nucleus get_all-data')
                return False

            if nucleus_data['data series id'] != 0xa0:
                self.messages.write_warning('Incorrect "data series id" for Nucleus get_all-data')
                return False

            if nucleus_data['family id'] != 0x20:
                self.messages.write_warning('Incorrect "family id" for Nucleus get_all-data')
                return False

            if nucleus_data['data size'] != len(nucleus_data['data']):
                self.messages.write_warning('"data size" does not match length of data for Nucleus get_all-data')
                return False

            if nucleus_data['data checksum'] != self.parser.checksum(nucleus_data['data']):
                self.messages.write_warning('"data checksum" does not match calculated crc of data for Nucleus get_all-data')
                return False

            if nucleus_data['header checksum'] != self.parser.checksum(nucleus_data['header'][:-2]):
                self.messages.write_warning('"header checksum" does not match calculated crc of data for Nucleus get_all-data')
                return False

            return True

        def _check_dvl_data():

            if dvl_data['sync bit'] != 0xa5:
                self.messages.write_warning('Incorrect "sync bit" for DVL get_all-data')
                return False

            if dvl_data['header size'] != len(dvl_data['header']):
                self.messages.write_warning('Incorrect "header size" for DVL get_all-data')
                return False

            if dvl_data['data series id'] != 0xa0:
                self.messages.write_warning('Incorrect "data series id" for DVL get_all-data')
                return False

            if dvl_data['family id'] != 0x16:
                self.messages.write_warning('Incorrect "family id" for DVL get_all-data')
                return False

            if dvl_data['data size'] != len(dvl_data['data'][:-2]):
                self.messages.write_warning('"data size" does not match length of data for DVL get_all-data')
                return False

            if dvl_data['data checksum'] != self.parser.checksum(dvl_data['data'][:-2]):
                self.messages.write_warning('"data checksum" does not match calculated crc of data for DVL get_all-data')
                return False

            if dvl_data['header checksum'] != self.parser.checksum(dvl_data['header'][:-2]):
                self.messages.write_warning('"header checksum" does not match calculated crc of data for DVL get_all-data')
                return False

            return True

        if fid is not None:
            try:
                int(fid)
            except ValueError:
                self.messages.write_warning('Invalid type for fid, must be integer')
                return None, None
            else:
                encoded_fid = b'FID=' + str(fid).encode() + b','
        else:
            encoded_fid = b''

        command = b'DOWNLOAD,' + encoded_fid + b'SRC=2\r\n'

        self.connection.write(command)
        get_all_data = self.commands._get_reply(terminator=b'OK\r\n', timeout=3)
        self.commands._check_reply(data=get_all_data, terminator=b'OK\r\n', command=command)

        if len(get_all_data) < 100 or get_all_data[-4:] != b'OK\r\n':
            self.messages.write_warning('Received incomplete data from attempting to download get_all data from device')
            return None, None

        get_all_list = _extract_get_all(get_all_data[:-4])

        if len(get_all_list) != 2:
            self.messages.write_warning('Failed to extract get_all data from download,src=2 reply')

            return None, None

        nucleus_data = dict()
        nucleus_data['sync bit'] = get_all_list[0][0]
        nucleus_data['header size'] = get_all_list[0][1]
        nucleus_data['data series id'] = get_all_list[0][2]
        nucleus_data['family id'] = get_all_list[0][3]
        nucleus_data['data size'] = get_all_list[0][4] | get_all_list[0][5] << 8
        nucleus_data['data checksum'] = get_all_list[0][6] | get_all_list[0][7] << 8
        nucleus_data['header checksum'] = get_all_list[0][8] | get_all_list[0][9] << 8
        nucleus_data['header'] = get_all_list[0][:10]
        nucleus_data['data'] = get_all_list[0][10:]

        dvl_data = dict()
        dvl_data['sync bit'] = get_all_list[1][0]
        dvl_data['header size'] = get_all_list[1][1]
        dvl_data['data series id'] = get_all_list[1][2]
        dvl_data['family id'] = get_all_list[1][3]
        dvl_data['data size'] = get_all_list[1][4] | get_all_list[1][5] << 8
        dvl_data['data checksum'] = get_all_list[1][6] | get_all_list[1][7] << 8
        dvl_data['header checksum'] = get_all_list[1][8] | get_all_list[1][9] << 8
        dvl_data['header'] = get_all_list[1][:10]
        dvl_data['data'] = get_all_list[1][10:]

        if _check_nucleus_data():
            get_all_nucleus = nucleus_data['data']
        else:
            get_all_nucleus = None

        if _check_dvl_data():
            get_all_dvl = get_all_list[1]
        else:
            get_all_dvl = None

        return get_all_nucleus, get_all_dvl

    def download_dvl_data(self, fid=None, sa=None, length=None) -> bool:

        def _check_arguments():

            if fid is not None and (not isinstance(fid, int) or fid < 1):
                self.messages.write_warning('fid argument must be a positive integer larger or equal to 1')
                return False

            if sa is not None and (not isinstance(sa, int) or sa < 0):
                self.messages.write_warning('sa argument must be a non-negative integer')
                return False

            if length is not None and (not isinstance(length, int) or length < 1):
                self.messages.write_warning('length argument must be a positive integer larger or equal to 1')
                return False

            return True

        def _download_data() -> (bool, bytes):

            failed_attempt = False
            for attempt in range(1, 11):
                status, package = self.download_data(fid=download_parameters['fid'], src=1, sa=index, length=min(download_parameters['end'] - index, self.PACKET_LENGTH))

                if status:
                    if failed_attempt:
                        self.messages.write_message('successfully received packet at index {}'.format(index))
                    break
                else:
                    failed_attempt = True
                    self.messages.write_message('Failed to receive packet at index {} on attempt {}. Retrying...'.format(index, attempt))

            else:
                self.messages.write_warning('Failed to receive package from DVL debug data download 10 consecutive attempts. Aborting download!')
                return False, b''

            return True, package

        def _download_get_all() -> (bool, bytes):

            for i in range(1, 11):
                get_all = self.download_get_all_data(download_parameters['fid'])[1]

                if get_all is not None:
                    break

            else:
                return False, b''

            return True, get_all

        def _write_data_to_file(_data: bytes, _file, _fail_file) -> bytes:

            previous_data = None
            while True:

                if previous_data == _data:
                    self.messages.write_exception('Encountered an unexected situation where previous data is equal current data in dvl file writing')
                    return _data
                else:
                    previous_data = _data

                if len(_data) < 4:
                    return _data

                length = int.from_bytes(_data[:4], 'little')

                if length > 10000:
                    _data, failed_data = self._handle_crc(_data)
                    _fail_file.write(failed_data)
                    self.dvl_download_statistics['failed bytes'] += len(failed_data)

                elif len(_data) >= 4 + length + 4:
                    dvl_data = _data[4:4 + length]
                    crc = int.from_bytes(_data[4 + length:4 + length + 4], 'little')

                    if crc == crc32(dvl_data):
                        _file.write(dvl_data)
                        _data = _data[4 + length + 4:]
                        self.dvl_download_statistics['successful bytes'] += len(dvl_data)
                    else:
                        _data, failed_data = self._handle_crc(_data)
                        _fail_file.write(failed_data)
                        self.dvl_download_statistics['failed bytes'] += len(failed_data)

                else:
                    return _data

        if not _check_arguments():
            return False

        status, download_parameters = self.get_download_parameters(src=1, fid=fid, sa=sa, length=length)
        if not status:
            return False

        status, get_all = _download_get_all()

        if status:

            self.dvl_download_statistics['successful bytes'] = 0
            self.dvl_download_statistics['failed bytes'] = 0
            percentage_previous = -1

            file_path = self._path + '/dvl/' + datetime.now().strftime('%y%m%d_%H%M%S')
            self.messages.write_message('Downloading data to: {}'.format(file_path))
            Path(file_path).mkdir(parents=True, exist_ok=True)
            with open(file_path + '/dvl_data.bin', 'wb') as file, open(file_path + '/dvl_crc_fails.bin', 'wb') as fail_file:

                file.write(get_all)

                downloaded_bytes = 0
                data = b''
                for index in range(download_parameters['sa'], download_parameters['end'], self.PACKET_LENGTH):

                    status, package = _download_data()

                    downloaded_bytes += len(package)
                    percentage = downloaded_bytes * 100 / (download_parameters['end'] - download_parameters['sa'])
                    if percentage > percentage_previous + 1:
                        if percentage > 99:
                            percentage = 100
                        self.progress_bar(percentage, 100)
                        percentage_previous = int(percentage)

                    if not status:
                        break

                    data += package

                    data = _write_data_to_file(data, file, fail_file)

                self.messages.write_message('Downloaded and converted {} bytes of data. {} bytes of data failed conversion due to CRC checks'.format(self.dvl_download_statistics['successful bytes'], self.dvl_download_statistics['failed bytes']))

        return status

    def download_nucleus_data(self, fid=None, sa=None, length=None) -> bool:

        def _check_arguments():

            if fid is not None and (not isinstance(fid, int) or fid < 1):
                self.messages.write_warning('fid argument must be a positive integer larger or equal to 1')
                return False

            if sa is not None and (not isinstance(sa, int) or sa < 0):
                self.messages.write_warning('sa argument must be a non-negative integer')
                return False

            if length is not None and (not isinstance(length, int) or length < 1):
                self.messages.write_warning('length argument must be a positive integer larger or equal to 1')
                return False

            return True

        def _download_data() -> (bool, bytes):

            failed_attempt = False
            for attempt in range(1, 11):
                status, package = self.download_data(fid=download_parameters['fid'], src=0, sa=index, length=min(download_parameters['end'] - index, self.PACKET_LENGTH))

                if status:
                    if failed_attempt:
                        self.messages.write_message('successfully received packet at index {}'.format(index))
                    break
                else:
                    failed_attempt = True
                    self.messages.write_message('Failed to receive packet at index {} on attempt {}. Retrying...'.format(index, attempt))

            else:
                self.messages.write_warning('Failed to receive package from DVL debug data download 10 consecutive attempts. Aborting download!')
                return False, b''

            return True, package

        def _download_get_all() -> (bool, bytes):

            for i in range(1, 11):
                get_all = self.download_get_all_data(download_parameters['fid'])[0]

                if get_all is not None:
                    break

            else:
                return False, b''

            return True, get_all

        if not _check_arguments():
            return False

        status, download_parameters = self.get_download_parameters(src=0, fid=fid, sa=sa, length=length)
        if not status:
            return False

        status, get_all = _download_get_all()

        if status:

            file_path = self._path + '/nucleus/' + datetime.now().strftime('%y%m%d_%H%M%S')
            self.messages.write_message('Downloading data to: {}'.format(file_path))
            Path(file_path).mkdir(parents=True, exist_ok=True)

            with open(file_path + '/get_all.txt', 'w') as file:
                file.writelines(get_all.decode())

            with open(file_path + '/nucleus_data.bin', 'wb') as file:
                file.write(get_all)
                percentage_previous = -1
                downloaded_bytes = 0
                for index in range(download_parameters['sa'], download_parameters['end'], self.PACKET_LENGTH):

                    status, package = _download_data()

                    downloaded_bytes += len(package)
                    percentage = downloaded_bytes * 100 / (download_parameters['end'] - download_parameters['sa'])
                    if percentage > percentage_previous + 1:
                        if percentage > 99:
                            percentage = 100
                        self.progress_bar(percentage, 100)
                        percentage_previous = int(percentage)

                    if not status:
                        break

                    file.write(package)

        return status

    def convert_nucleus_data(self, path) -> bool:

        if not Path(path).is_file():
            self.messages.write_warning('selected file path is not a file')
            return False

        path_folder = Path(path).parent

        self.logger.set_path(path=str(path_folder) + '/nucleus_converted')

        self.logger.start(_converting=True)

        with open(path, 'rb') as raw_file:

            raw_file.seek(0, 2)
            file_length = raw_file.tell()
            raw_file.seek(0, 0)

            percentage_previous = -1

            while True:
                data = raw_file.read(1)

                percentage = raw_file.tell() * 100 / file_length
                if percentage > percentage_previous + 1:
                    if percentage > 99:
                        percentage = 100
                    self.progress_bar(percentage, 100)
                    percentage_previous = int(percentage)

                if data == b'':
                    break

                self.parser.add_data(data)

            self.logger.stop()

        return True
