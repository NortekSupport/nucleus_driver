# Introduction

Made for ros2 humble

# Installation

It is assumed that ros2 humble is already installed. Installation guide can be found here: https://docs.ros.org/en/humble/Installation.html

Clone this repo to your preferred directory with the following command:
```
git clone git@github.com:nortekgroup/nucleus_driver.git
```

Navigate to the ROS2 workspace folder ros2_ws

```
cd path/to/nucleus_driver/ros2_ws
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

Build ros2 Nucleus node with colcon. Refer to troubleshooting section for potential issues

```
colcon build
```

# Using the ros2 nodes

In any new terminal that should run a node, source the installation

```
cd path/to/nucleus_driver/ros2_ws/
source install/setup.bash
```

The main node is the nucleus_node which is a wrapper around the nucleus_driver module. This node handles the communication with the Nucleus device and should have access to the Nucleus through either serial or TCP. This node can be executed with the following command

```
ros2 run nucleus_node nucleus_node
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
/nucleus_node/ins_packets [interfaces/msg/INS]
/nucleus_node/magnetometer_packets [interfaces/msg/Magnetometer]
/nucleus_node/water_track_packets [interfaces/msg/BottomTrack]
```

## Clients

There are also client modules for every service that the nucleus_node supports. This allows the user to control the Nucleus device through nucleus_node by invoking these clients. 

Assuming nucleus_node is running in another instance, the following commands will connect to the Nucleus device, turn on the AHRS output stream, start and stop the device, and then disconnect from the Nucleus device.

```
ros2 run nucleus_node connect_serial /dev/ttyUSB0
ros2 run nucleus_node command 'SETAHRS,DS="ON"'
ros2 run nucleus_node start
ros2 run nucleus_node stop
ros2 run nucleus_node disconnect
```

## Subscribers

There are also a subscriber module for every topic that nucleus_node supports. These can also be run in the terminal like the clients and the callback function in the subscribers will simply print out some of the received data from the relevant packet. The ahrs packets subscriber can be invoked in the terminal with the following command

```
ros2 run nucleus_node ahrs_packets
```

Assuming this subscriber was running when during the client command in the previous example, the subscriber would yield the following output

```
[INFO] [1689323189.142425896] [subscriber_ahrs_packets]: roll: 0.6595 | pitch: -0.128 | heading: 61.314 | quat w: 0.8602 | quat x: 0.0055 | quat y: 0.0020 | quat z: 0.5099
[INFO] [1689323189.243115454] [subscriber_ahrs_packets]: roll: 0.5820 | pitch: -0.142 | heading: 61.341 | quat w: 0.8601 | quat x: 0.0050 | quat y: 0.0015 | quat z: 0.5101
[INFO] [1689323189.374060632] [subscriber_ahrs_packets]: roll: 0.5014 | pitch: -0.150 | heading: 61.340 | quat w: 0.8601 | quat x: 0.0044 | quat y: 0.0011 | quat z: 0.5101
[INFO] [1689323189.513965824] [subscriber_ahrs_packets]: roll: 0.4361 | pitch: -0.162 | heading: 61.328 | quat w: 0.8602 | quat x: 0.0040 | quat y: 0.0007 | quat z: 0.5100
[INFO] [1689323189.693899847] [subscriber_ahrs_packets]: roll: 0.3640 | pitch: -0.179 | heading: 61.263 | quat w: 0.8605 | quat x: 0.0035 | quat y: 0.0003 | quat z: 0.5095

```

# Example code

The clients and subscribers are designed to be imported into a python module. The example code in ros2_ws/examples demonstrates how the clients and subscribers could be used for integration.

To use the example script the nucleus_node has to be running in an separate instance, and the script should be executed with either of the following commands

```
python3 example.py -s /dev/ttyUSB0
python3 example.py -n NORTEK-xxxxxx.local -p nortek
```

where -s is the serial port where the Nucleus is connect, -n is the hostname of the Nucleus, and -p is the password required for TCP connection. the values of these arguments must be adapted to the corresponding values of your system. The x'es in hostname refer to the serial number of the Nucleus device.

If both serial and tcp connection is specified through the arguments, a serial connection will be performed.

# PyTest

pytest will configure Nucleus device!

run nucleus_node seperatly? in order to support the node running somewhere else on the system, i.e. in a docker container?


modify test_setup.json file
```
{
    "hostname": "NORTEK-300004.local",
    "serial_port": "/dev/ttyUSB0"
}
```


```
pytest test_nucleus_node.py
```

# Example









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

## The command `ros2 run nucleus_node nucleus_node` fails

During execution of the nucleus node the following error occurs

```
ModuleNotFoundError: No module named 'nucleus_driver'
```

This indicates that the requirements for the installation is not satisfied. This could be related to a known issue where ros2 is not able to find modules in virtual environments. 

A workaround is to add the virtual environment to PYTHONPATH.

This path can be found by running the following command:

```
pip3 show nucleus-driver | grep Location
```

which should yield something similar to this:

```
Location: /path/to/nucleus_driver/ros2_ws/venv/lib/python3.10/site-packages
```

This path can then be added to PYTHONPATH with the follofing command

```
export PYTHONPATH=$PYTHONPATH:/path/to/nucleus_driver/ros2_ws/venv/lib/python3.10/site-packages
```

An alternative is to not use a virtual environment and install the required modules system wide.
