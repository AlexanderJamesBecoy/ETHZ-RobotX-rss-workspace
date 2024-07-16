#include "smb_power_payload/smb_pow_pay_display.hpp"
#include <pluginlib/class_list_macros.h>
#include <QHBoxLayout>

namespace smb_rviz_plugins {
using namespace rviz;

SMBPowerPayDisplay::SMBPowerPayDisplay(){

    visual_category_  = new rviz::Property("Show/Hide", QVariant(), "", this);

    battery_topic_ = new rviz::RosTopicProperty("Topic Power Payload", "",
                                            QString::fromStdString(ros::message_traits::datatype<smb_powerstatus::SMBPowerStatus>()),
                                            "smb_powerstatus::SMBPowerStatus topic to subscribe to.",
                                            this, SLOT(updateTopic()));

    visual_battery_1_ = new rviz::BoolProperty("Show Battery 1", true, "Show/Hide Battery 1",
                                            visual_category_, SLOT(changeVisibility()),this);

    visual_battery_2_ = new rviz::BoolProperty("Show Battery 2", true, "Show/Hide Battery 2",
                                            visual_category_, SLOT(changeVisibility()),this);

    visual_plug_ = new rviz::BoolProperty("Show Plug", true, "Show/Hide Plug",
                                            visual_category_, SLOT(changeVisibility()),this);                                    

}

void SMBPowerPayDisplay::setTopic(const QString& topic, const QString& datatype){
    battery_topic_->setString(topic);
}

void SMBPowerPayDisplay::onInitialize(){
    // Create widget to add to rviz
    QWidget * parent = new QWidget;
    auto layout = new QHBoxLayout;
    
    // Add 3 battery widgets for the 3 batteries
    battery_1_panel_ = new BatteryPayPanel("Battery 1");
    battery_2_panel_ = new BatteryPayPanel("Battery 2");
    plug_panel_ = new PlugPanel("Plug");
    
    // Add to the layout
    layout->addWidget(battery_1_panel_);
    layout->addWidget(battery_2_panel_);
    layout->addWidget(plug_panel_);
    parent->setLayout(layout);
    
    setAssociatedWidget(parent);

}

void SMBPowerPayDisplay::onEnable(){
    subscribe();
}

void SMBPowerPayDisplay::onDisable(){
    unsubscribe();
}

void SMBPowerPayDisplay::changeVisibility(){

    battery_1_panel_->setVisible(visual_battery_1_->getBool());
    battery_2_panel_->setVisible(visual_battery_2_->getBool());
    plug_panel_->setVisible(visual_plug_->getBool());

}

void SMBPowerPayDisplay::updateTopic(){
    unsubscribe();
    reset();
    subscribe();
}

void SMBPowerPayDisplay::subscribe(){
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
        battery_subscriber_ = update_nh_.subscribe(topic_name, 1, &SMBPowerPayDisplay::batteryMsgCallback, this);
        setStatus(StatusProperty::Ok, "Battery", "OK");
    }
    catch (ros::Exception& e)
    {
        setStatus(StatusProperty::Error, "Battery", QString("Error subscribing: ") + e.what());
    }
}

void SMBPowerPayDisplay::unsubscribe(){
    battery_subscriber_.shutdown();
}

void SMBPowerPayDisplay::batteryMsgCallback(const smb_powerstatus::SMBPowerStatusConstPtr &msg){

    battery_1_panel_->setEnabled(msg->battery_1.present);
    if(!msg->battery_1.present){
        battery_1_panel_->setBatteryStatus(BatteryPayPanel::BatteryStatus::Missing);
    }else{
        battery_1_panel_->setPercentage(msg->battery_1.percentage);
        battery_1_panel_->setVoltage(msg->battery_1.voltage);
        switch (msg->battery_1.power_supply_status)
        {
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
                battery_1_panel_->setBatteryStatus(
                    BatteryPayPanel::BatteryStatus::Charging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
                battery_1_panel_->setBatteryStatus(
                    BatteryPayPanel::BatteryStatus::Discharging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL:
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
                battery_1_panel_->setBatteryStatus(
                    BatteryPayPanel::BatteryStatus::NotCharging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN:
                battery_1_panel_->setBatteryStatus(
                    BatteryPayPanel::BatteryStatus::Unknown);
                break;
        }
    }

    battery_2_panel_->setEnabled(msg->battery_2.present);
    if(!msg->battery_2.present){
        battery_2_panel_->setBatteryStatus(BatteryPayPanel::BatteryStatus::Missing);
    }else{
        battery_2_panel_->setPercentage(msg->battery_2.percentage);
        battery_2_panel_->setVoltage(msg->battery_2.voltage);
        switch (msg->battery_2.power_supply_status)
        {
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
                battery_2_panel_->setBatteryStatus(
                    BatteryPayPanel::BatteryStatus::Charging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
                battery_2_panel_->setBatteryStatus(
                    BatteryPayPanel::BatteryStatus::Discharging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL:
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
                battery_2_panel_->setBatteryStatus(
                    BatteryPayPanel::BatteryStatus::NotCharging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN:
                battery_2_panel_->setBatteryStatus(
                    BatteryPayPanel::BatteryStatus::Unknown);
                break;
        }
    }

    plug_panel_->setEnabled(msg->power_supply_present);
    if(!msg->power_supply_present){
        plug_panel_->setVoltage(0);
        plug_panel_->setInUse(false);
    }else{
        plug_panel_->setVoltage(msg->power_supply_voltage);
        plug_panel_->setInUse(true);
    }

}


}

PLUGINLIB_EXPORT_CLASS(smb_rviz_plugins::SMBPowerPayDisplay, rviz::Display)