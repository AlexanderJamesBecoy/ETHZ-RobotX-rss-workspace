#!/usr/bin/env python

import rospy
import paho.mqtt.client as mqtt
import json
from sensor_msgs.msg import BatteryState

class SMBBaseBattery:
    def __init__(self):
        rospy.init_node('mqtt_ros_bridge')
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_broker_address = rospy.get_param('~mqtt_broker_address', 'localhost')
        print(self.mqtt_broker_address)
        self.mqtt_topic = rospy.get_param('~mqtt_topic', 'smb_base_battery/cell_info')
        self.ros_topic = rospy.get_param('~ros_topic', 'ros_topic')
        self.ros_publisher = rospy.Publisher(self.ros_topic, BatteryState, queue_size=10)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        rospy.loginfo('Connected to MQTT broker with result code: %d', rc)
        self.mqtt_client.subscribe(self.mqtt_topic)

    def on_mqtt_message(self, client, userdata, msg):
        m_decode=str(msg.payload.decode("utf-8", "ignore"))
        m_in=json.loads(m_decode)
        ros_message = BatteryState()
        ros_message.header.stamp = rospy.Time.now()
        ros_message.voltage = m_in["total_voltage"]
        ros_message.current = m_in["current"]
        ros_message.capacity = m_in["capacity_remain"]
        ros_message.design_capacity = m_in["capacity_nominal"]
        ros_message.percentage = m_in['battery_soc'] / 100
        ros_message.present = True
        ros_message.power_supply_status = 3
        ros_message.cell_voltage = m_in["voltages"]
        self.ros_publisher.publish(ros_message)

    def run(self):
        self.mqtt_client.connect(self.mqtt_broker_address)
        self.mqtt_client.loop_start()
        rospy.spin()

if __name__ == '__main__':
    baseBattery = SMBBaseBattery()
    baseBattery.run()
