# ros2 BETA

The nucleus_driver_ros2 package contains the node nucleus_node which is based on the nucleus driver. nucleus_node consists of several services and publishers that maps functionality from the nucleus driver to a ros2 interface. nucleus_node should be running on a system where it has access to a Nucleus device either through a serial or TCP connection, and it will allow for communication with the Nucleus through the ros2 environment.

The nucleus_driver_ros2 package also consists of several client and subscriber modules to be used with the nucleus_node and could serve as a useful tools to communicate with the nucleus_node and to ease the integration of the Nucleus into your ros2 environment.

Supported functionality consists of establishing connection to the Nucleus, starting and stopping measurement, sending commands to the Nucleus, and receive packages from the Nucleus during runtime.

The ros2 package is developed for ros2 humble and it is currently in its beta phase

# Prerequisites

It is assumed that ros2 humble is already installed. Installation guide can be found here: https://docs.ros.org/en/humble/Installation.html

# Installation

Clone the Nucleus driver repo to your preferred directory with the following command:

```
git clone git@github.com:nortekgroup/nucleus_driver.git
```

Navigate to the ROS2 workspace folder `ros2`

```
cd path/to/nucleus_driver/ros2
```

**Optional:** Create a virtual environment for installed modules. Refer to troubleshooting section for potential issues.

```
virtualenv venv --system-site-packages --symlinks
touch ./venv/COLCON_IGNORE
source venv/bin/activate
```

Install requirements
```
pip3 install -r requirements
```

Source ros2 base layer

```
source /opt/ros/humble/setup.bash
```

Build ros2 Nucleus driver with colcon. Refer to troubleshooting section for potential issues

```
colcon build
```

# Using the ros2 nodes

In any new terminal that should run a node, first source the base layer

```
source /opt/ros/humble/setup.bash
```

Then, source the installation

```
cd path/to/nucleus_driver/ros2/
source install/setup.bash
```

The main node is the nucleus_node which is a wrapper around the nucleus_driver module. This node handles the communication with the Nucleus device and should have access to the Nucleus through either serial or TCP. This node can be executed with the following command

```
ros2 run nucleus_driver_ros2 nucleus_node
```

The nucleus_node supports a range of different services and topics. The services allows for connecting and disconnect from a device, running measurements, and sending command. The list of services are

```
/nucleus_node/connect_serial [interfaces/srv/ConnectSerial]
/nucleus_node/connect_tcp [interfaces/srv/ConnectTcp]
/nucleus_node/disconnect [interfaces/srv/Disconnect]
/nucleus_node/start [interfaces/srv/Start]
/nucleus_node/field_calibration [interfaces/srv/StartFieldCalibration]
/nucleus_node/stop [interfaces/srv/Stop]
/nucleus_node/command [interfaces/srv/Command]
```

The topics are various packet types that will be sent from the Nucleus during run time. The list of topics are

```
/nucleus_node/ahrs_packets [interfaces/msg/AHRS]
/nucleus_node/altimeter_packets [interfaces/msg/Altimeter]
/nucleus_node/bottom_track_packets [interfaces/msg/BottomTrack]
/nucleus_node/current_profile_packets [interfaces/msg/CurrentProfile]
/nucleus_node/field_calibration_packets [interfaces/msg/FieldCalibration]
/nucleus_node/imu_packets [interfaces/msg/IMU]
/nucleus_node/magnetometer_packets [interfaces/msg/Magnetometer]
/nucleus_node/water_track_packets [interfaces/msg/BottomTrack]
```

## Clients

There are also client modules for every service that the nucleus_node supports. This allows the user to control the Nucleus device through nucleus_node by invoking these clients. 

Assuming nucleus_node is running in another instance, the following commands will connect to the Nucleus device through serial, turn on the AHRS output stream on the device, start and stop the device, and then disconnect from the Nucleus device.

```
ros2 run nucleus_driver_ros2 connect_serial /dev/ttyUSB0
ros2 run nucleus_driver_ros2 command 'SETAHRS,DS="ON"'
ros2 run nucleus_driver_ros2 start
ros2 run nucleus_driver_ros2 stop
ros2 run nucleus_driver_ros2 disconnect
```

**N.B.** Note that if a command requires the use of double quotation marks in it command string, the entire command string should be within single quotation marks as seen with the command `'SETAHRS,DS="ON"'` in the previous example.

## Subscribers

There are also a subscriber module for every topic that nucleus_node supports. These can also be run in the terminal like the clients and the callback function in the subscribers will simply print out some of the received data from the relevant packet. The ahrs packets subscriber can be invoked in the terminal with the following command

```
ros2 run nucleus_driver_ros2 ahrs_packets
```

Assuming the nucleus_node is running and the Nucleus device is both running and configured to send AHRS packets, this subscriber would yield an  output like this

```
[INFO] [1689323189.142425896] [ahrs_packets]: roll: 0.6595 | pitch: -0.128 | heading: 61.314
[INFO] [1689323189.243115454] [ahrs_packets]: roll: 0.5820 | pitch: -0.142 | heading: 61.341
[INFO] [1689323189.374060632] [ahrs_packets]: roll: 0.5014 | pitch: -0.150 | heading: 61.340
[INFO] [1689323189.513965824] [ahrs_packets]: roll: 0.4361 | pitch: -0.162 | heading: 61.328
[INFO] [1689323189.693899847] [ahrs_packets]: roll: 0.3640 | pitch: -0.179 | heading: 61.263

```

# Docker

## docker build

Build the docker image 

```
cd path/to/nucleus_driver/ros2/
sudo docker build . -t nucleus_driver_ros2
```

## docker run

The docker image can run nucleus_node with the following command 

```
docker run --name=Nucleus-Node -it nucleus_driver_ros2 bash -c "ros2 run nucleus_driver_ros2 nucleus_node"
```

### Serial connection

In order for the docker container to have access to a serial connection to the Nucleus device, one of the two following arguments has to be added in the run command. To specifically map the serial port into the container the following argument can be added

```
# this maps a port from the host machine to the container: --device=/from/host:/to/container
--device=/dev/ttyUSB0:/dev/ttyUSB0
```

The previous argument maps only the specified serial port to the container, and is considered safer than the the following alternative

```
--privileged
```

which gives the container access to all the serial ports on the host. This is practical when the serial port is unknown when executing the container or when the serial port is likely to change. 

### TCP connection
If the docker container needs to connect to the Nucleus device through TCP, it is necessary to use the actual IP address of the Nucleus device and not its hostname.


# Troubleshooting

## The command `colcon build` fails

During colcon build, atleast one of the following errors occurs

```
ModuleNotFoundError: No module named 'catkin_pkg'
ModuleNotFoundError: No module named 'em'
ModuleNotFoundError: No module named 'lark'
```

This could be related to the python version on your system being newer than what ros2 humble is built for. To fix this issue, install the required modules with the respective command for the respective error. It is likely that all 3 modules has to be installed.

```
pip3 install catkin_pkg
pip3 install empy
pip3 install lark
```

## The command `ros2 run nucleus_driver_ros2 nucleus_node` fails

During execution of the nucleus node the following error occurs

```
ModuleNotFoundError: No module named 'nucleus_driver'
```

This indicates that the requirements for the installation is not satisfied. This could be related to a known issue where ros2 is not able to find modules in virtual environments. 

The easiest fix is to not use virtual environments and install the modules system wide.

Alternatively, a workaround is to add the virtual environment to PYTHONPATH.

This path can be found by running the following command:

```
pip3 show nucleus-driver | grep Location
```

which should yield something similar to this:

```
Location: /path/to/nucleus_driver/ros2/venv/lib/python3.10/site-packages
```

This path can then be added to PYTHONPATH with the follofing command

```
export PYTHONPATH=$PYTHONPATH:/path/to/nucleus_driver/ros2/venv/lib/python3.10/site-packages
```
