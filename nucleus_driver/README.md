# Nucleus Driver

This is a driver for the Nortek Nucleus device. The driver requires Python with a minimum version of 3.7

## Installation

Run the following to install

```python
pip3 install nucleus_driver
```

## Integration into python scripts

The purpose of the driver is to integrate it into your python code 

### Create driver object
```python
from nucleus_driver import NucleusDriver

driver = NucleusDriver()
``` 

### Establish connection to Nucleus device

The nucleus device supports both serial and TCP connection. For serial connection the baudrate is 115200, 
thus only the serial port is required to establish a connection with the driver

```python
SERIAL_PORT = "dev/ttyUSB0"

driver.set_serial_configuration(port=SERIAL_PORT)
driver.connect(connection_type='serial')
```

For TCP connection the Nucleus device supports both connecting through hostname and through the device's IP address.
The device's IP address is given by a dhcp server by default (if the device is connected to a router), or it can be given a static IP address. 
The hostname of the device will be "NORTEK-######.local" where ###### is the device's serial number, i.e. NORTEK-300001.local.

```python
TCP_HOST = 'NORTEK-######.local'  # Using hostname, replace ###### with device's serial number
TCP_HOST = '192.168.10.10'  # Using IP adress

driver.set_tcp_configuration(host=TCP_HOST)
driver.connect(connection_type='tcp')
```

Disconnecting from the device is the same regardless of connection type

```python
driver.disconnect()
``` 

### Send commands to Nucleus device

The driver supports sending commands directly to the Nucleus device. For an overview of the supported commands refer to the Nucleus manual

```python
command = 'GETALL\r\n'  # Refer to the Nucleus documentation for available commands
respone = driver.send_command(command=command)
```

The driver also comes with several built-in commands. These commands can be accessed through the Command class (refer to src/nucleus_driver/_commands.py for available commands).
The equivalent command from above can be sent through the Command class as follows
```python
respone = driver.commands.get_all()
```

### Start and stop measurement

These commands start and stops the Nucleus device as well as starting the parser in the driver.
The parser extracts packets from the datastream and adds them to the packet queue,
and adds them to a log file if logging is enabled. Parsing and logging is covered in the following sections

Start a normal measurement

```python
driver.start_measurement()
```

Start field calibration

```python
driver.start_fieldcal()
```

Stop any kind of measurement

```python
driver.stop()
```

### Extract parsed data

The parser extracts all the data from the Nucleus data stream and separates it into different packets. 
The packets containing measurement data is added to a packet queue, 
the packets containing ascii data (that is responses from different commands) are added to an ascii queue, 
and failed packets (i.e. data packets that failed the CRC check) will be added to a condition queue.

These packets can be obtained by reading their respective FIFO queues. If a queue is empty, that queue will return None

For measurement data, use read_packet()
```python
packet = driver.read_packet()  # returns the first data packet in the packet queue
```

For ascii data, use read_ascii()
```python
ascii_packet = driver.read_ascii()  # returns the first ascii message from the ascii queue
```

For condition data, use read_condition()
```python
condition_packet = driver.read_condition()  # returns the first packet from the condition queue
```

### Log data

The different packets extracted from the parser will also be logged to file is logging is enabled.
When logging is started the driver creates a folder named according to the current date and time, and this folder contains 4 files:
* get_all.txt: contains the data returned by the "getall" command, this provides information about all the configuration of the device at the time of measurement
* nucleus_log.csv: contains the data packets
* ascii_log.csv: contains any ascii messages that has been received during logging
* condition_log.csv: contains data that failed to be parsed successfully

Specify path for log data, if not specified the driver will create a "logs" folder in the current path
```python
PATH = '/path/to/your/desired/log/file/destination'
driver.set_log_path(path=PATH)
```

Start logging
```python
driver.start_logging()
```

Stop logging
```python
driver.stop_logging()
```

### Flash firmware

The driver supports flashing of a Nucleus device. The device use two different firmwares, one for the Nucleus, and one for the DVL. 
New firmware from Nortek will be distributed in a zip file containing both the Nucleus and DVL firmware, ensuring that these firmwares work well together.

Flash firmware unto the Nucleus device. The file can be a nucleus firmware, a dvl firmware, or a zip file containing both nucleus and dvl firmware. 
It is recommended to only use the zip files provided by Nortek as it ensures that the Nucleus and DVL firmware it contains are compatible.
```python
FILE_PATH = 'path/to/flash/file'  # supported file extentions are .ldr, .bin and .zip
driver.flash_firmware(path=FILE_PATH)
```

### Download data from SD card

The driver supports downloading data from the SD card. The downloaded data can either Nucleus measurement data or DVL diagnostics data.
The downloaded data will be in binary format.

To set the path for the download files use the set_download_path function

```python
PATH = 'path/to/download/folder'
driver.set_download_path(path=PATH)
```
The download functions will create a folder called "download" within the specified path. If no path is specified the current folder is the path.

To see what files are available for download at the SD card the list files function can be used:

```python
driver.list_files(src=None)
```

Here the "src" argument is referring to which datatype that should be listed:
* src=0 lists nucleus measurement files
* src=1 lists dvl diagnostic data files
* src=2 list getall data files. The getall data consist of the configuration of the Nucleus device at the time of measurement of the Nucleus measurement data and DVL configuration data. The FID value in this list corresponds to the FID value of the other data sets.


To download Nucleus measurement data:

```python
driver.download_nucleus_data(fid=None, sa=None, length=None)
```

To download DVL diagnostics data:

```python
driver.download_dvl_data(fid=None, sa=None, length=None)
```

The getall data corresponding with the specified file will also be downloaded as part of these downloads.

In both the Nucleus measurement data and DVL diagnostics data download the arguments "fid", "sa" and "length" are used.
"fid" is the id of the file to be downloaded.
"sa" is the start byte in the file from where you want to download data, with a minimum value of 1.
"length" is the length in bytes from "sa" of data you wish to download.
All of these arguments are optional, omitting "fid" results in downloading the latest file, 
omitting "sa" results in downloading from the start of the file,
and omitting "length" results in download until the end of the file.

The downloaded Nucleus measurement data can be converted to a .csv file with the convert data function

```python
PATH = 'path/to/nucleus/binary/file'
driver.convert_nucleus_data(path=PATH)
```

### Asserts

Any asserts occurring during operation will be stored on the Nucleus device. For debugging purposes these asserts can be downloaded and sent to Nortek during support tasks.

To set the path for assert download
```python
PATH = 'path/to/asserts/folder'
driver.set_assert_path(path=PATH)
```

The driver will create an assert folder within the specified path for the download. If no path is specified the current path is used.

To download the data use the assert_download function

```python
driver.assert_download()
```

To clear the asserts on the nucleus device, use the clear_assert function

```python
driver.clear_assert()
```

### Syslog

The Nucleus device continuously logs internal statistics in its syslog, for debugging purposes this syslog is a crucial tool, and is good to have in support cases.

To set a path for syslog download

```python
PATH = 'path/to/syslog/folder'
driver.set_syslog_path(path=PATH)
```

The driver will create a syslog folder in the specified path. If no path is specified, the current path will be used.

To download the syslog, use the syslog_download function

```python
driver.syslog_download()
```

To clear the syslog on the nucleus device, use the clear_syslog function

```python
driver.clear_syslog()
```

## Running driver as shell script

The driver also comes with a console script that supports most of the functionality of the driver. 
To run the console script, execute the command

```shell
nucleus_driver
```

![Console script](docs/nucleus_driver_shell.png)