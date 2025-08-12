# BlueOS Nucleus Extension

## Intro

This extension integrates the Nortek **Nucleus** into the **BlueROV2** via the **BlueOS** extensions platform.  
It packages the [Nucleus Driver](https://github.com/NortekSupport/nucleus_driver/tree/main/nucleus_driver) inside a Docker image alongside a Node.js-based frontend, which streams velocity, orientation, and position data from the Nucleus into the ArduSub control system via MAVLink commands.  

By providing the ROV with this navigation data, the extension enables advanced control modes such as **Position Hold**, **AUTO**, and **GUIDED**.  
Note: **AUTO** and **GUIDED** modes require the Nucleus to have the INS license.


## Nortek Nucleus integration

To mount the Nucleus unto the BlueROV2 and connect it to its network, refer Nortek's [Nucleus integration guide](https://support.nortekgroup.com/hc/en-us/articles/8246995934748-Nucleus-integration-with-BlueROV2).

## setup

### BlueOS

BlueOS needs to be of version 1.2.6 or newer in order to support the adding of third party extensions.

### ArduSub

ArduSub needs to be of version 4.1.2 or newer for it to support the mavlink packets used to send data from the Nucleus.

The current version of the extension is developed on ArduSub version 4.5.0 and it is recommended to use at least this version of ArduSub.

### Nucleus

The Nucleus must be properly configured in order for it to work with the extension.

It must be configured with a **static IP address** in order for the extension to connect to it over the ROV network. Assuming the network configuration on the BlueROV2 is default, the following example is valid:
```
SETETH,IPMETHOD="STATIC",IP="192.168.2.201"
```

In order for the various mode to be available for use in the ROV's controller, certain messages **must** be sent from the Nucleus. The minimum configurations of the Nucleus are described in the following sections, but it is expected that the user reads the **MANUAL** to properly configure the Nucleus to fit their needs.

#### Bottom track

Bottom track is responsible for sending velocity data to the ROV and it is required for the ROV to use its Position Hold feature. To enable this output, set the following:
```
SETBT,DS="ON"
```

The frequency of which bottom track data is collected is configured through the trigger settings, i.e.:

```
SETTRIG,FREQ=4
```

The trigger settings also handles the altimeter and current profile settings, which influences the trigger frequency. In addition, the frequency of the trigger settings is limited by the bottom track settings. Therefore, read the **MANUAL** and familiarize yourself with these settings before setting up the Nucleus for your use. For the extension, setting this value as high as you can afford for your configuration is advised.

#### AHRS

AHRS data is essential for providing the ROV with an orientation when sending the velocity data to the ROV. The velocity data sent will be limited to the trigger frequency defined for the bottom track data. Ensure that the AHRS frequency is higher than the bottom track data, but there is no benefit from maxing it out. Example config:

```
SETAHRS,DS="ON",FREQ=10
```


#### INS

The INS must be enabled in order to utilize the **AUTO** and **GUIDED** modes in the ROV. Enable the datastream for the INS:

```
SETNAV,DS="ON"
```

And set the instrument settings to navigation mode:

```
SETINST,TYPE="NAV"
```



## PID parameters

**N.B.** Adjusting these parameters **WILL** change the behavior of the ROV

**N.B.** The ROV does **not** store original values of these parameters in case they are changed.

The PID parameters can be changed through the "Autopilot parameters" menu in the blueos.local home page. Good parameterization is necessary for a good performance of the ROV. The correct parameters varies from vehicle to vehicle as the physical attributes of the ROV has an impact on its behavior. However, recommended starting parameters for a standard BlueROV2 with only the Nucleus being the third party installation are as follows:

| Parameter | Value |
| ---| --- |
| PSC_POSXY_P | 1.0 |
| PSC_POSZ_P | 1.0 |
| PSC_VELXY_P | 5.0 |
| PSC_VELXY_I | 0.5 |
| PSC_VELXY_D | 0.8 |
| PSC_VELZ_P | 5.0 |

While these parameters might provide a well performing ROV, they are most likely not optimal. It is recommended to adjust these parameters to get the desired ROV behavior.

## Controller parameters

**N.B.** Adjusting any of these parameters may change the behavior of the ROV

**N.B.** The ROV does **not** store original values of these parameters in case they are changed.

In order for the controller to accept and utilize the velocity and position packets sent from the Nucleus, certain parameters has to be changed. The parameters can be changed in the "Autopilot parameters" menu in the blueos.local home page. 

After these parameters has been changed it is necessary to power cycle the vehicle for these parameters to take effect

The parameters and their required values are the following

| Parameter | Value |
| ---| --- |
| SERIAL0_PROTOCOL | MAVLink2 |
| EK3_ENABLE | Enabled |
| AHRS_EKF_TYPE | Enable EKF3 |
| EK2_ENABLE | Disable |
| VISO_TYPE | MAVLink |
| EK3_SRC1_POSXY | ExternalNav |
| EK3_SRC1_VELXY | ExternalNav |
| EK3_SRC1_YAW | ExternalN |
| EK3_SRC2_YAW | Compass |

Also, EK3 algorithm in the ROV may struggle in **AUTO** mode and its noticed with "EKf3 lane switch" errors. As well as jumps in the estimated position which can be seen in the ROVs position in QGroundControl.

If this is happening, a dirty fix is to adjust the "EKF3 Lane Relative Error Sensitivity Threshold", which is the threshold for how much error there can be in the estimates before the algorithm changes the "lane". By increasing this value the algorithm is less likely to change lanes while running and is likely to yield a better performance as these lane changes causes quite big jump it the estimate positions.

During testing there were good results by setting this value to its maximum, that is:


| Parameter | Value |
| ---| --- |
| EK3_ERR_THRESH | 1.0 |

## GUI

Accessible via the **Nortek Nucleus** entry in the BlueOS menu. It consists of two tabs: **Connection** and **Home**.

### Connection Page

Allows the user to establish a connection to the Nucleus, using the static IP configured on the device. The connection will be established to either port 9000, or 9002 depending on whether "Stream only" is enabled. 

If not enabled, the connection is done to port 9000 which allows the extension to communicate with (send data to) the Nucleus, which is essential for some of the functionality in the GUI such as:

* Starting or stopping the device
* Setting GNSS origin in the Nucleus (**N.B** This is only relevant in order to see the position in the map! This will not affect how the data is sent to the ROV!)

By enabling "Stream only", the user may (and must) issue commands to the Nucleus from another source, such as the Nucleus Software, or other software the user may have developed for the Nucleus. While connected to this port, the extension can **ONLY** receive data from the Nucleus, which is sufficient in order for the ROV to receive the required data. 

The connection also includes options for **Autoconnect** and **Autostart**:  
- **Autoconnect** – Automatically connects to the Nucleus whenever the ROV is powered up.
    - The "autoconnect" will attempt to use the latest IP address and port used to connect to the Nucleus
- **Autostart** – Automatically starts the Nucleus after connecting (requires Autoconnect to be enabled).  
    - If "Stream only" (port 9002) was enabled for the latest connection, autostart will **fail** as a start command cannot be issued when connected to port 9002

#### GUI free usage
If the user wish to use the extension without having to interface the GUI **at all** in consecutive missions, an initial connection can be done in the extension where they connect to the Nucleus selecting the following:

* **Stream only** is **NOT** checked (the extension must issue the start command)
* **Autoconnect** is checked
* **Autostart** is checked

Additionally, the user must send a SET_GPS_GLOBAL_ORIGIN command to the ROV through MavLink in order to provide the ROV with an initial condition before they can use the **AUTO** and **GUIDED** modes of the ROV, as the ROV must know where it is before it can use these features.

By doing this, the extension will start the Nucleus and start sending data to the ROV whenever it starts up, and the operator does not have to use the GUI in order to benefit from the extension.
    
### Home Page
The **Home** page provides tools for monitoring and controlling data flow between the Nucleus and the ROV.  

#### Packet Counters
Two packet counters serve as a quick sanity check to verify that data is being sent to the ROV:  
- **Position Delta** packets  
- **Position Estimate** packets  

Each counter displays:  
- **Sent** – Number of successfully sent packets.  
- **Skipped** – Packets not sent because transmission is disabled.  
- **Failed** – Packets that failed to send due to an error.  
- **Frequency** – The rate (in Hz) at which packets are being sent.  

An **enable toggle** for each packet type determines whether packets are sent to the ROV. Disabling a packet type will cause its skipped counter to increase.

#### GNSS Origin and Map
The **GNSS Origin** panel, combined with the interactive map, allows the user to define the ROV’s reference origin.  
This can be done by:  
- Clicking on the map, or  
- Manually entering latitude and longitude in the GNSS Origin panel.  

The **gnss origin** must be set before that **AUTO** and **GUIDED** modes can be used in the ROV.

**Important:** The GNSS origin can only be set **once** per ROV power cycle. To change it, the ROV must be rebooted.

The map also displays the path traveled by the Nucleus based on its **INS** output.  
An active internet connection is required for the map to display.



