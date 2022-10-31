# Nucleus Driver

The Nuclues driver classes NucleusDriver.py and NucluesDriverNortek.py are both classes that are intended to make it easy to integrate a Nucleus Device. If either class  are executed directly the user is presented with a shell like interface that allows the user to connect to and use a Fusion device, as well as using all the features the classes offers.

The NucleusDriver.py class contains all the basic functionality of the driver and is intended to be a useful resource for anyone wanting to use the device, that includes people outside of Nortek. The NucleusDriverNortek.py is and extension of NucleusDriver.py and therefore contain all of its functionality, but in addition NortekDriverNortek.py offers functionality that is intended for Nortek employees only.

An executable of the Nucleus Driver is also created as part of the continuous integration in the firmware development, the intention of this executable is to make it easy for anyone in Nortek to use a Fusion device and the executable is based on the FusionDriverNortek.py class. 

## Running the Nucleus Driver

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
 



