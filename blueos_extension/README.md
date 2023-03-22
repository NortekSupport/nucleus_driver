
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

Here "NUCLEUS_IP" has to be the static IP configured on the Nucleus device.

N.B. the user interface that comes with the extension depends on being integrated through BlueOS and will not be available if the extension is added in this manner

## Using the extension (Is this covering only the UI?)

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

