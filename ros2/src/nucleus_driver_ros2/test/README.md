# PyTest

The test module will test the functionality of the ros2 nucleus driver. To run the tests, install the requirements.

```
cd path/to/nucleus_driver/ros2/src/nucleus_driver_ros2/test
pip3 install -r requirements.txt
```

Configure the `test_setup.json` file to fit the configuration of your Nucleus device. For the testing to be performed either hostname or serial_port must be defined. If neither are defined, no tests will be executed. If only one is defined, the tests will be performed with that connection. If both are defined, all tests will be performed on both connections.

In `test_setup.json`, password refers to the password needed to perform a TCP connection to you Nucleus device, if not specified, pytest will use the default password which is "nortek".

```
{
    "hostname": "NORTEK-xxxxxx.local",
    "password": "nortek",
    "serial_port": "/dev/ttyUSB0"
}
```

Source nucleus_driver_ros2

```
cd path/to/nucleus_driver/ros2
source install/setup.bash
```

run tests

```
cd path/to/nucleus_driver/ros2/src/nucleus_driver_ros2/test
pytest test_nucleus_driver_ros2.py
```

**N.B.** Pytest will configure the Nucleus device as part of its testing!

