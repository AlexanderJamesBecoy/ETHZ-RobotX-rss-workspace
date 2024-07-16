#include "smb_power_motor/smb_batt_mot_panel.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPainter>
#include <QPen>
#include <iostream>

namespace smb_rviz_plugins{

BatteryMotPanel::BatteryMotPanel(QString name, QWidget *parent)
    :QWidget(parent){

    QVBoxLayout* layout = new QVBoxLayout;
    battery_name_= new QLabel(name);
    battery_name_->setStyleSheet("font-weight: bold");
    layout->addWidget(battery_name_);

    QHBoxLayout* layout_indicator = new QHBoxLayout;

    battery_icon_ = new QLabel();
    setIcon(":/battery/battery_warning.svg");
    layout_indicator->addWidget(battery_icon_);

    battery_text_ = new QLabel("No Data");
    layout_indicator->addWidget(battery_text_);

    layout_indicator->addStretch();

    layout->addLayout(layout_indicator);

    setLayout(layout);
}

void BatteryMotPanel::setBatteryStatus(BatteryMotPanel::BatteryStatus status){
    battery_status_ = status;
    updateWidgets();
}

void BatteryMotPanel::setPercentage(double percentage){
    percentage_ = percentage;
    updateWidgets();
}

void BatteryMotPanel::setVoltage(double voltage){
    voltage_ = voltage;
    updateWidgets();
}

void BatteryMotPanel::setIcon(const QString &path){
    QPixmap pixmap(path);
    const auto& font_metrics = battery_icon_->fontMetrics();
    auto icon_width = font_metrics.averageCharWidth() * 6;
    auto icon_height = font_metrics.height()*2;
    battery_icon_->setPixmap(pixmap.scaled(icon_width, icon_height, Qt::KeepAspectRatio));
}

void BatteryMotPanel::updateWidgets(){
    double percentage = percentage_ * 100;
    battery_text_->setText(QString("%1% (%2 V)").arg(QString::number(percentage, 'f', 2)).arg(QString::number(voltage_, 'f', 2)));
    switch(battery_status_)
    {
        case BatteryStatus::Unknown:
            setIcon(":/battery/battery_warning.svg");
            break;
        case BatteryStatus::Charging:
            setIcon(":/battery/battery_charge.svg");
            break;
        case BatteryStatus::Discharging:
        case BatteryStatus::NotCharging:
            if(percentage_ < 0.1)
                setIcon(":/battery/battery_0.svg");
            else if(percentage_ < 0.2)
                setIcon(":/battery/battery_1.svg");
            else if(percentage_ < 0.3)
                setIcon(":/battery/battery_2.svg");
            else if(percentage_ < 0.4)
                setIcon(":/battery/battery_3.svg");
            else if(percentage_ < 0.5)
                setIcon(":/battery/battery_4.svg");
            else if(percentage_ < 0.6)
                setIcon(":/battery/battery_5.svg");
            else if(percentage_ < 0.7)
                setIcon(":/battery/battery_6.svg");
            else if(percentage_ < 0.8)
                setIcon(":/battery/battery_7.svg");
            else if(percentage_ < 0.9)
                setIcon(":/battery/battery_8.svg");
            else
                setIcon(":/battery/battery_full.svg");
            break;
        case BatteryStatus::Missing:
            setIcon(":/battery/battery_warning.svg");
            break;
    }
}


}