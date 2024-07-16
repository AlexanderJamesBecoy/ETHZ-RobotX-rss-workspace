#include "smb_power_motor/smb_pow_mot_display.hpp"
#include <pluginlib/class_list_macros.h>
#include <QHBoxLayout>

namespace smb_rviz_plugins {
using namespace rviz;

SMBPowerMotDisplay::SMBPowerMotDisplay(){

    battery_topic_ = new rviz::RosTopicProperty("Topic Power Motor", "",
                                            QString::fromStdString(ros::message_traits::datatype<sensor_msgs::BatteryState>()),
                                            "sensor_msgs::BatteryState topic to subscribe to.",
                                            this, SLOT(updateTopic()));
}

void SMBPowerMotDisplay::setTopic(const QString& topic, const QString& datatype){
    battery_topic_->setString(topic);
}

void SMBPowerMotDisplay::onInitialize(){
    // Create widget to add to rviz
    QWidget * parent = new QWidget;
    auto layout = new QHBoxLayout;
    
    // Add 3 battery widgets for the 3 batteries
    battery_panel_ = new BatteryMotPanel("Motor Battery");
    
    // Add to the layout
    layout->addWidget(battery_panel_);
    parent->setLayout(layout);
    
    setAssociatedWidget(parent);

}

void SMBPowerMotDisplay::onEnable(){
    subscribe();
}

void SMBPowerMotDisplay::onDisable(){
    unsubscribe();
}

void SMBPowerMotDisplay::updateTopic(){
    unsubscribe();
    reset();
    subscribe();
}

void SMBPowerMotDisplay::subscribe(){
    if(!isEnabled())
        return;
    
    std::string topic_name = battery_topic_->getTopicStd();
    if (topic_name.empty())
    {
        setStatus(StatusProperty::Error, "Output Topic", "No topic set");
        return;
    }

    std::string error;
    if (!ros::names::validate(topic_name, error))
    {
        setStatus(StatusProperty::Error, "Output Topic", QString(error.c_str()));
        return;
    }

    try
    {
        battery_subscriber_ = update_nh_.subscribe(topic_name, 1, &SMBPowerMotDisplay::batteryMsgCallback, this);
        setStatus(StatusProperty::Ok, "Battery", "OK");
    }
    catch (ros::Exception& e)
    {
        setStatus(StatusProperty::Error, "Battery", QString("Error subscribing: ") + e.what());
    }
}

void SMBPowerMotDisplay::unsubscribe(){
    battery_subscriber_.shutdown();
}

void SMBPowerMotDisplay::batteryMsgCallback(const sensor_msgs::BatteryStateConstPtr &msg){
    battery_panel_->setPercentage(msg->percentage);
    battery_panel_->setVoltage(msg->voltage);
    if(!msg->present){
        battery_panel_->setBatteryStatus(BatteryMotPanel::BatteryStatus::Missing);
    }else{
        switch (msg->power_supply_status)
        {
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
                battery_panel_->setBatteryStatus(
                    BatteryMotPanel::BatteryStatus::Charging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL:
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
                battery_panel_->setBatteryStatus(
                    BatteryMotPanel::BatteryStatus::Discharging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN:
                battery_panel_->setBatteryStatus(
                    BatteryMotPanel::BatteryStatus::Unknown);
                break;
        }
    }

}


}

PLUGINLIB_EXPORT_CLASS(smb_rviz_plugins::SMBPowerMotDisplay, rviz::Display)