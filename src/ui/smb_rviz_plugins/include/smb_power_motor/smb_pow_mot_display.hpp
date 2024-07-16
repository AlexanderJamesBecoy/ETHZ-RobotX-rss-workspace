#ifndef SMB_BATTERY_MOTOR_DISPLAY_HPP__
#define SMB_BATTERY_MOTOR_DISPLAY_HPP__

#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <sensor_msgs/BatteryState.h>

#include "smb_power_motor/smb_batt_mot_panel.hpp"

namespace smb_rviz_plugins {

class SMBPowerMotDisplay : public rviz::Display {
    Q_OBJECT
    public:

        SMBPowerMotDisplay();

        void setTopic( const QString& topic, const QString& datatype ) override;
    
    protected:

        void onInitialize() override;
        void onEnable() override;
        void onDisable() override;

    private Q_SLOTS:
        void updateTopic();        

    private:
        void subscribe();
        void unsubscribe();
        void batteryMsgCallback(const sensor_msgs::BatteryStateConstPtr &msg);

        rviz::RosTopicProperty* battery_topic_;
        
        ros::Subscriber battery_subscriber_;
        
        BatteryMotPanel* battery_panel_{nullptr};
};


}

#endif //SMB_BATTERY_MOTOR_DISPLAY_HPP__