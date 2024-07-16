[![Build](https://github.com/ETHZ-RobotX/smb_rviz_plugins/workflows/build/badge.svg)](https://github.com/ETHZ-RobotX/smb_rviz_plugins/actions/workflows/build.yml)
# SMB RVIZ PLUGINS
SMB rviz plugins

## Build
```bash
# Build the package
catkin build smb_rviz_plugins

# Source the package
source devel/setup.bash
```

## Message type **SMBPower**
This repository contains a custom message type that contains the data of the batteries of the payload

### SMBPower
```bash
# This message holds the informations about the batteries 
# and the power supply od the SuperMegaBot

Header header

# Power supply status
bool power_supply_present
float32 power_supply_voltage

# Batteries status
sensor_msgs/BatteryState battery_1
sensor_msgs/BatteryState battery_2
```
In order to use the command ```rostopic echo``` you have to source the package first.