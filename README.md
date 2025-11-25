
# Marvelmind Indoor GPS for ROS 2

![image](https://github.com/user-attachments/assets/331ce079-19c9-46e2-9972-cdfedc97bbee)

## Introduction

[Marvelmind Indoor Navigation System](https://marvelmind.com/) is an indoor navigation system that
provides precise (±2cm) location data to vehicles. It can track moving objects via mobile beacons attached to them.
The system can be connected to the ArduPilot.

This repository provides a Docker container to facilitate ```Dashboard Application``` the necessary software for setting up and tuning the system. 

The repository also provides the ROS 2 package, which communicates with the modem to obtain the position of the ```hedgehog```, the moving part of the system located on the vehicle.

Note: Building and running are necessary steps to configure the system. Omit if the system is already configured.

## Build

```bash
git clone https://github.com/markusbuchholz/indoor_gps_ros2_interface.git

cd indoor_gps_ros2_interface/docker

sudo ./build.sh

```

## Run
Note: Adjust the path in the ```run.sh```
```bash 


sudo ./run.sh

```
## Run application (on Host - not in Docker)

```basg
cd indoor_gps_ros2_interface/src/x86/

# Copy libdashapi.so to /usr/local/lib
sudo cp libdashapi.so /usr/local/lib

./dashboard_x86

```
![image](https://github.com/user-attachments/assets/71b39b09-0f13-4b75-b6cb-0d02a47e6a9c)



## ROS 2 Interface

Upload the ```ros2``` folder of this repository to your workspace (```name_ws/src```)

```bash
colcon build

source install/source.bash

ros2 launch marvelmind_ros2 marvelmind_ros2.launch.py
```

## Launch filter

The filter smooths data from the topic /hedgehog_pos and map to /hedgehog_pos_fileter.

```bash
cd marvelmind_extras

python3 ros2_boat_pos_yaw_filered.py
```


ROS 2 topics:

```bash
/beacon_raw_distance
/beacons_pos_addressed
/hedgehog_imu_fusion
/hedgehog_imu_raw
/hedgehog_pos
/hedgehog_pos_addressed
/hedgehog_pos_ang
/hedgehog_quality
/hedgehog_telemetry
/marvelmind_user_data
/marvelmind_waypoint
```

## Python Interface

It is possible to communicate with the system using the Python3 interface. It is easy to capture the position of the moving beacon ```hedgehog``` using the following terminal commands.

```bash
cd marvelmind_python/src

python3 example.py

# expected output:
# Hedge 6: X: 15.489 m, Y: -9.779 m, Z: -2.073 m, Angle: 0 at time T: 2024-07-15 10:37:50-009
```
## Operation

Please take note of the following instructions:
- Ensure that the beacons are charged with POWER DIP switch OFF.
- Prior to operating, launch the ```Dashboard``` program with the modem connected to the PC. Verify that the system is operational, and then close the program.
- Review the example Python script to confirm the position readings of the ```Hedge mobile beacon```, and terminate the program.
- Run the ROS 2 interface.

## Communication troubleshooting

First, verify the working system using the ```dashboard``` program. Next, run the Python script. If this works, the ROS 2 pkg can be launched.

The USB port can be stuck.

Perform the following commands.

```bash
lsusb
# output
Bus 009 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 010 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 008 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 007 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 006 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 005 Device 002: ID 0d62:dabc Darfon Electronics Corp. Keyboard
Bus 005 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 003: ID 187c:0550 Alienware Corporation LED controller
Bus 003 Device 099: ID 0483:5740 STMicroelectronics Virtual COM Port
Bus 003 Device 094: ID 0461:4141 Primax Electronics, Ltd HP 320M USB Optical Mouse
Bus 003 Device 093: ID 046d:c21d Logitech, Inc. F310 Gamepad [XInput Mode]
Bus 003 Device 092: ID 05e3:0608 Genesys Logic, Inc. Hub
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 003: ID 0bda:554b Realtek Semiconductor Corp. Integrated_Webcam_HD
Bus 001 Device 002: ID 0489:e0c8 Foxconn / Hon Hai Wireless_Device
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub


# look for STMicroelectronics  - here the Bus is 03 device 099

lsusb -t

###
/:  Bus 03.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/3p, 480M
    |__ Port 1: Dev 92, If 0, Class=Hub, Driver=hub/4p, 480M
        |__ Port 2: Dev 93, If 0, Class=Vendor Specific Class, Driver=xpad, 12M
        |__ Port 3: Dev 94, If 0, Class=Human Interface Device, Driver=usbhid, 1.5M
    |__ Port 2: Dev 99, If 0, Class=Communications, Driver=cdc_acm, 12M
    |__ Port 2: Dev 99, If 1, Class=CDC Data, Driver=cdc_acm, 12M
    |__ Port 3: Dev 3, If 0, Class=Human Interface Device, Driver=usbhid, 12M

# Our device is connected to port 2

echo '3-2' | sudo tee /sys/bus/usb/drivers/usb/unbind

echo '3-2' | sudo tee /sys/bus/usb/drivers/usb/bind

```

## References

- [User manual](https://marvelmind.com/pics/marvelmind_navigation_system_manual.pdf)
- [Unpacking Starter Set Super-MP](https://www.youtube.com/watch?v=Uj2_BGS1AjI)
- [Marvelmind API](https://marvelmind.com/download/#api)
- [Marvelmind for Non-GPS navigation](https://ardupilot.org/rover/docs/common-marvelmind.html)
- [PixHawk_Marvelmind_Integration_Manual](https://marvelmind.com/pics/PixHawk_Marvelmind_Integration_Manual.pdf)
- [marvelmind_px4_integration](https://marvelmind.com/pics/marvelmind_px4_integration.pdf)
- [Hardware interfaces](https://marvelmind.com/pics/marvelmind_interfaces.pdf)
