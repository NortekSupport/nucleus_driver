
# BlueOS extension

This is a Nucleus extension for the BlueOS software running on BlueROV. 

It works by wrapping up the Nucleus Driver in a docker image, with a script that feeds velocity data from the Nucleus device into the Ardusub control system through mavlink commands.

This allows the user to utilize the "position hold" functionality in the ROV

## setup

### Nucleus

This extension assumes that the Nucleus is connected to the network of the ROV through an ethernet connection. It is therefore necessary to set a static IP in the Nucleus.

The network prefix of the BlueROV is 192.168.2.0, with a netmask of 255.255.255.0. The Nucleus' static IP must therefore be set to a fitting value, i.e. 192.168.2.201.

### Ardusub

Ardusub needs to be of version 4.1.0 or newer for it to support the VISUAL_POSITION_DELTA packets used to send velocity data to the ROV.


## Adding extension to BlueOS

### BlueOS

The extension should be added through BlueOS' extensions menu.

### Docker

Alternatively, the extension can be added by ssh-ing into the onboard raspberry pi and running the following command.

```
docker run --net=host --name=BlueOS-Nucleus --restart=unless-stopped -e NUCLEUS_IP="192.168.2.201" martinbjnortek/blueos_nucleus:latest
```

N.B. the user interface that comes with the extension depends on being integrated through BlueOS and will not be available if the extension is added in this manner

## Using the extension


















This is an extension for the BlueROV, utilizing a Nucleus device to send velocity data to the ROV. 
This allows for the ROV to use its "position hold" functionality.

For this implementation to work, the ROV needs to run ArduSub version of 4.1.0 or newer.

This extension is available through Dockerhub and can be downloaded and executed through the following command:

```
docker run --net=host dockerhub/nucleus-something
```

# First use
Upon startup the extension will check if necessary parameters are set to the correct values in ArduSub. These parameters are:

* AHRS_EKF_TYPE: 3
* EK2_ENABLE: 0
* EK3_ENABLE: 1
* VISO_TYPE: 1
* GPS_TYPE: 0
* EK3_SRC1_POSXY: 6
* EK3_SRC1_VELXY: 6
* EK3_SRC_POSZ: 1
* SERIAL0_PROTOCOL: 2

If any of these values are incorrect the extension will change the value of the parameter as listed above. 
Any parameter changes will require a power cycle for the changes to take effect, thus the extension will not start sending data from the Nucleus until the device has been restarted.

# General use

# Running the extension

Beyond the first use, and assuming the extension was able to set all the required parameters properly. 
The extension will always run automatically upon vehicle startup and no interaction from the user is required.

To verify that the Nucleus is running, ensure that the led light on the Nucleus is blinking.

## PID parameters
For better performance of the "position hold" functionality, certain PID parameters in the ROV will be changed on startup.
These parameters and their values are:

* PSC_POSXY_P: 2.0
* PSC_POSZ_P: 1.0
* PSC_VELXY_P: 5.0
* PSC_VELXY_I: 0.5
* PSC_VELXY_D: 0.8
* PSC_VELZ_P: 5.0

These parameters will change the behavior of the ROV, so be aware that the ROV will handle differently in general after these parameters has been set.

## Communication

The communication protocol used within ArduSub is Mavlink.
With the parameter changes performed above, ArduSub will accept the VISION_POSITION_DELTA package, a package originally intended for visual odometry.
The VISION_POSITION_DELTA package takes velocity and orientation data as input and feed this data into the extended kalman filter algorithm in ArduSub.

The extension extracts velocity and orientation from the DVL and AHRS packages respectively from the Nucleus and feeds them to ArduSub through the VISION_POSITION_DELTA package.

# User interaction

The extension does not need any interaction from the user in order for it to work,
however, a few commands has been added to the extension which allows the user to change the behavior of the extension.

Be aware, utilizing these commands will change the behavior of the ROV and/or extension.

