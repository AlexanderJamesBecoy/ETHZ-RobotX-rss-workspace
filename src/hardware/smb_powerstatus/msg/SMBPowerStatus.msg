# This message holds the informations about the batteries 
# and the power supply od the SuperMegaBot

Header header

# Power supply status
bool power_supply_present
float32 power_supply_voltage

# measured current
float32 output_current

# Batteries status
sensor_msgs/BatteryState battery_1
sensor_msgs/BatteryState battery_2