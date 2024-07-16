#ifndef SMB_PLUG_PAYLOAD_PANEL_HPP__
#define SMB_PLUG_PAYLOAD_PANEL_HPP__

#include <QWidget>
#include <QLabel>

namespace smb_rviz_plugins{

class PlugPanel : public QWidget{
    Q_OBJECT

    public:
        PlugPanel(QString name, QWidget *parent = nullptr);
        void setInUse(bool in_use);
        void setVoltage(double voltage);

    private:
        void setIcon(const QString &path);
        void updateWidgets();

        double voltage_{0.0};

        QLabel* plug_name_;
        QLabel* plug_icon_;
        QLabel* plug_text_;

};

}

#endif