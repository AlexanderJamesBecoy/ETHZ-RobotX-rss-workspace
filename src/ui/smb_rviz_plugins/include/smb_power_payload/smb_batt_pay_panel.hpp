#ifndef SMB_BATTERY_PAYLOAD_PANEL_HPP__
#define SMB_BATTERY_PAYLOAD_PANEL_HPP__

#include <QWidget>
#include <QLabel>

namespace smb_rviz_plugins {

class BatteryPayPanel : public QWidget{
    Q_OBJECT

    public:
        BatteryPayPanel(QString name, QWidget *parent = nullptr);
        

        enum class BatteryStatus {
            Unknown,
            Charging,
            Discharging,
            Missing,
            NotCharging
        };
        Q_ENUM(BatteryStatus);

        enum class BatteryUsage {
            inUse,
            notInUse,
            warning
        };
        Q_ENUM(BatteryUsage);

        void setBatteryStatus(BatteryStatus status);
        void setPercentage(double percentage);
        void setVoltage(double voltage);
        void setInUse(BatteryUsage usage);

    private:
        void setIcon(const QString &path);
        void updateWidgets();

        BatteryStatus battery_status_{BatteryStatus::Unknown};
        double percentage_{0.0};
        double voltage_{0.0};

        QLabel* battery_name_;
        QLabel* battery_icon_;
        QLabel* battery_text_;
};

}
#endif //SMB_BATTERY_PANEL_HPP__