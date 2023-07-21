# nucleus_driver_ros2 example

The script `example.py` demonstrates how the clients and subscribers in this module can be integrated into a python environment in order to communicate with nucleus_node. 

Before running the example script, source the nucleus_driver_ros2 package

```
cd path/to/nucleus_driver/ros2/
source install/setup.bash
```

To use the example script the nucleus_node has to be running in an separate instance, and the script should be executed with either of the following commands

```
python3 example.py -s /dev/ttyUSB0
python3 example.py -n NORTEK-xxxxxx.local -p nortek
```

where -s is the serial port where the Nucleus is connect, -n is the hostname of the Nucleus, and -p is the password required for TCP connection. the values of these arguments must be adapted to the corresponding values of your system. The x'es in hostname refer to the serial number of the Nucleus device.

If both serial and tcp connection is specified through the arguments, a serial connection will be performed.

**N.B.** If nucleus_node is running in a docker container, the hostname must be defined as the Nucleus' IP address instead of its hostname.