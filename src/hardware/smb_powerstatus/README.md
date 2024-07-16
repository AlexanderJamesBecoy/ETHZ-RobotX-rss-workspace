# smb_powerstatus
[![Build Test](https://github.com/ETHZ-RobotX/smb_powerstatus/actions/workflows/smb_power_package.yaml/badge.svg)](https://github.com/ETHZ-RobotX/smb_powerstatus/actions/workflows/smb_power_package.yaml)

Software to display the power source status of the SuperMegaBot.

The repository contains the launchfile for the SuperMegaBot, the custom message definition for publishing the power source status and the firmware for the power control board.

## Install
### Install dependecies
``` bash
# Install dependencies
sudo apt install ros-noetic-rosserial ros-noetic-rosserial-arduino
``` 
### Clone and Build package
``` bash
# Enter your workspace
cd <directory_to_ws>/<catkin_ws_name>/src
# Clone the repository
git clone git@github.com:ETHZ-RobotX/smb_powerstatus.git
# Build package
catkin build smb_powerstatus
# Source the package
source ../devel/setup.bash
```
### Build Arduino firmware
To build the firmware you have to install the [Arduino IDE](https://www.arduino.cc/en/software). In order to build please install the following libraries from the library manager:
- arduino-timer **(v. 2.3.0)**
- Adafruit ADS1X15 **(v. 2.1.1)**

The power control board is using an **ATSAMD21** MCU, please install **Arduino SAMD Boards** from the board manager and select **Arduino MKRZERO** as board.

Now link the libraries of the repository to the Arduino library folder and add the ROS libraries too.

``` bash
# Create a symbolic link in the Arduino library folder pointing to the smb_powerstatus library
ln -s /path/to/repo/firmware/libraries/smb_libraries $HOME/Arduino/libraries

# Add ros libraries. DON'T FORGET TO SOURCE
cd $HOME/Arduino/libraries
rosrun rosserial make_libraries.py .
```

Build the firmware with the Arduino IDE and upload it to the MCU

### Setup udev rule
Add user to ```dialout``` group and copy the rule
``` bash
sudo adduser $USER dialout
cd /path/to/repository
sudo cp firmware/98-smb-power.rules /etc/udev/rules.d/98-smb-power.rules
```

Reload the rules
``` bash
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo ldconfig
```
Note: check if the device is avaible with ```ls /dev/smb-power```. If there is an error reboot the computer.

## Launch smb_powerstatus
To start publish the status launch rosserial with the provided launchfile
``` bash
roslaunch smb_powerstatus smb_powerstatus.launch
```
