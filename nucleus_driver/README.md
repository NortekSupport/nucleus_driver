# Nucleus Driver

The Nuclues driver classes NucleusDriver.py and NucluesDriverNortek.py are both classes that are intended to make it easy to integrate a Nucleus Device. If either class  are executed directly the user is presented with a shell like interface that allows the user to connect to and use a Fusion device, as well as using all the features the classes offers.

The NucleusDriver.py class contains all the basic functionality of the driver and is intended to be a useful resource for anyone wanting to use the device, that includes people outside of Nortek. The NucleusDriverNortek.py is and extension of NucleusDriver.py and therefore contain all of its functionality, but in addition NortekDriverNortek.py offers functionality that is intended for Nortek employees only.

An executable of the Nucleus Driver is also created as part of the continuous integration in the firmware development, the intention of this executable is to make it easy for anyone in Nortek to use a Fusion device and the executable is based on the FusionDriverNortek.py class. 

## Running the Fusion Driver

### Connection

When running FusionDriver.py, FusionDriverNortek.py, or the executable the user is presented with a shell like interface. The user will first be navigated through a connection procedure which let's the user connect to a device through either serial, TCP, or UDP. Only serial or TCP are supported by the Fusion device, UDP connection is a special case connection which is used when a Fusion device is routed through the network of a BlueRobotics ROV. 

#### Serial connection

When connecting through serial the driver will have to specify which serial port the device is located and at what baud rate the device communicates on. Available serial ports will be listed by the driver for the user to select from. The baud rate can be specified, but if nothing is provided the default baud rate of 115200 is used.

#### TCP connection

When connecting through TCP the user can choose between connecting with "host" or "serial number". If "host" is selected the user must provide either the IP address of the device or its host name, which is typically "NORTEK-######.local" where #### is the serial number of the device with leading zeros's. If "serial number" is selected the user types in the serial number of the device and the driver will generate and connect with the expected host name as specified earlier.

#### UDP connectiong

When connecting through UDP the port of the ROV should be specified

### Status

When the connection has been established the user will be presented with the status of the device. This will contain what kind of connection is established as well as connection info. It should contain the ID and firmware version of the device, if this is not the case (the values of these fields are "None") it is likely that something is wrong with the connection. This is typically that wrong connection info is specified or that the device is not powered on. Lastly it provides information on wether the device is currently logging data.

### Help

At every step when navigating through the driver the "help" text will be presented for the user. this text ill list all available commands in the current step as well as a short description of what these commands do. Each command also supports a shortened syntax to allow for faster navigation through the driver. In the case of the "help" command the user can either type in "help" or "?" in order to trigger this command.

### NortekDriver.py functionality

The basic driver supports the following commands:

* status: This outputs the current status of the connection
* terminal: This opens a direct link to the device which let's the user send and receive commands directly to the device
* logging: This let's the user log data from the device

### NortekDriverNortek.py and executable functionality

The extended driver supports the following commands:

* status: This outputs the current status of the connection
* terminal: This opens a direct link to the device which let's the user send and receive commands directly to the device
* logging: This let's the user log data from the device
* flash: Let's the user flash the Nucleus and DVL firmware unto the device
* calibration: This allows for reading and writing calibration data to and from the device and to read and write this data to .csv files
* asserts: Read out asserts from device and save them to file
* syslog: Read out syslog from device and save it to file

## AME specific commands

Certain functionality are designed to optimize the  procedures for AME. These procedures are covered here and it is assumed that the user is running the executable version of the driver.

### Setup

Preparations:
* Have SEGGER J-Link connected between the PC and Fusion device.
* Have JLinkExe installed on the PC: https://www.segger.com/products/debug-probes/j-link/tools/j-link-commander/
* Ensure that mcuboot.hex is located in the setup folder next to the executable
* Ensure that uns_fw.signed.hex is located in the setup folder next to the executable

Procedure:
* Connect to device through serial. If this is the first time the device is being used the status field will NOT containing information about ID and firmware
* Navigate to setup (type "setup" or "s")
* Execute ame_setup command (type "ame_setup" or "a")
* Input serial number, bootloader file, and flash file on the drivers request

What is happening:
* option bytes is written to device through J-Link
* bootloader is flashed unto device through J-Link
* firmware is flashed unto device through J-Link
* Imprints are written to device through serial (This step sets serial number)
* calibration data is written to device (default data is used unless a csv file has been specified)
* option bytes with locking is written to device unless skip_lock has been set (This should NEVER be the case if device is sent to customer)

Completion criteria:
* If setup is successful the calibration data from the driver and device will be printed
* The user MUST compare the calibration data and MAKE SURE they match (Differences due to values being float is ok, that is 0.00139999 is considered equal to 0.0014)

### Calibration

Preparations:
* Place .csv file with calibration data into the calibration folder next to the executable

Procedure:
* Connect to the device
* Ensure that ID and firmware was successfully retrieved during connection in the status field
* Navigate to calibration (type "calibration" or "c")
* Execute ame_calibration command (type "ame_calibration" or "a")
* Select .csv file with calibration data on drivers request

What is happening:
* Driver read calibration data from file
* calibration data is uploaded to device memory
* calibration data is written from device memory to flash
* calibration data is read from device flash into driver

Completion criteria:
* If setup is successful the calibration data from the .csv file  and device will be printed
* The user MUST compare the calibration data and MAKE SURE they match (Differences due to values being float is ok, that is 0.00139999 is considered equal to 0.0014)




 



