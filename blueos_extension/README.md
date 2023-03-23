
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

It is also possible to run a docker container directly on the onboard computer without going through BlueOS.

**N.B.** It is assumed that the build cammand in this section is executed on the ROVs onboard computer. If not, refer to dockers `buildx` functionality and ensure that that the image is build for the hardware matching the onboard computer.

Navigate to the blueos_extension folder (the folder containing the Dockerfile) and build the docker image with the following command:

```
docker build . -t nucleus_driver
```

The web interface of the extension is by default on port 5000. In the case of a BlueROV system the web interface can be accessed in a browser by navigating to `192.168.2.2:5000` (or `blueos.local:5000`) when the docker container is running.

If another port is preferred for the web interface the image can be build with the preffered port as an argument with the following command

```
docker build . -t nucleus_driver --build-arg PORT=5000
```

with the value following "`PORT=`" being your preferred port.

**N.B.** The docker image built for the BlueOS extension has the web interface running on port 80, allowing BlueOS to handle how the web interface should be accessed. 

The docker container can be executed with the following command

```
docker run --net=host --name=Nucleus-Driver --restart=unless-stopped -e NUCLEUS_IP="192.168.2.201" nucleus_driver
```

`--net=host` allows the container to share the network of the ROV which is necessary for it to communicate with the ROV and make the web interface available

`--name=Nucleus-Driver` is the preferred name of the container.

`--restart=unless-stopped` allows the extension to automatically start when the ROV is powered up

`-e NUCLEUS_IP="192.168.2.201"` is the IP adress of the Nucleus device. Ensure that this value matches the static IP set on the device.

## Using the extension

**N.B.** In order for the extension to work it is necessary to change certain controller parameters. Refer to "Controller parameters" section for more info

**N.B.** In order for the "position hold" algorithm to perform well it might be necessary to change the ROV's PID parameters. Refer to "PID parameters" section for more info

With the extension added, its user interface (UI) can be found by navigating to [blueos.local/nucleus](blueos.local/nucleus).

The UI presents the user with a home page and two pages for paramterization. These pages can be navigated inbetween using the navigation banner at the top of the UI

### Home

The home screen presents the user with a status field which displays the results of various checks performed during the startup of the ROV. It is necessarry for all of these checks to pass in order for extension to work. Some easy troubleshooting is presented in the home screen in case any of these checks were to fail

It is also a field which allows the user to decide whether the driver is enabled. The driver must be enabled for it to feed velocity data to the ROV. If it is not enabled the driver is still runnning and extracting data from the Nucleus, but the velocity is not sent to the ROV.

The packet counter field displays how many velocity data packets has been handled by the extension. Sent packages refers to packages that has been sent to the ROV, Failed packets refers to packets that for some reason failed to be sent, and Skipped packets are packets that were in good condition and ready to be sent, but weren't due to the driver not being enabled.

### PID parameters

**N.B.** Adjusting any of these parameters **WILL** change the bahavior of the ROV

**N.B.** Neither the extension nor the ROV will remember the original values of these parameters in case they are changed. It is therefore the users responsibility to remember the origial values in case they wish to revert back to the original parameterization.

The PID parameters page gives the user opportuniy to modify selected PID parameters in the controller. Good parametirzation is necessary for a good performance of the ROV. The correct parameters varies from vehicle to vehicle as the physical attributes of the ROV has an impact on its behavior. However, recommended parameters for a standard BlueROV2 with only the Nucleus being the third party installation is presented on this page.

### Controller parameters

**N.B.** Adjusting any of these parameters may change the behaviour of the ROV

**N.B.** Neither the extension nor the ROV will remember the original values of these parameters in case they are changed. It is therefore the users responsibility to remember the origial values in case they wish to revert back to the original parameterization.

In order for the controller to accept and utilize the velocity packets sent from the Nucleus, certain parameters has to be changed. The parameters and their required values are presented on this page. 

After these parameters has been changed it is necessary to power cycle the vehicle for these parameters to take effect

