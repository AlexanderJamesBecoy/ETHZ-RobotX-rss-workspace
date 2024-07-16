#include <pluginlib/class_list_macros.h>
#include <QHBoxLayout>

#include "smb_control/smb_control_panel.hpp"

namespace smb_rviz_plugins
{

SMBControlPanel::SMBControlPanel(QWidget* parent){
    QHBoxLayout* layout = new QHBoxLayout;
    stop_button_ = new QPushButton();
    stop_button_->setText("STOP");
    layout->addWidget(stop_button_);
    setLayout(layout);
}

}
PLUGINLIB_EXPORT_CLASS(smb_rviz_plugins::SMBControlPanel,rviz::Panel )