#ifndef SMB_CONTROL_PANEL_HPP__
#define SMB_CONTROL_PANEL_HPP__

#include <ros/ros.h>
#include <QPushButton>

#include <rviz/panel.h>

namespace smb_rviz_plugins
{

class SMBControlPanel : public rviz::Panel{
    Q_OBJECT

    public:
        SMBControlPanel(QWidget* parent = 0);


    protected:
        QPushButton* stop_button_;
};

}

#endif