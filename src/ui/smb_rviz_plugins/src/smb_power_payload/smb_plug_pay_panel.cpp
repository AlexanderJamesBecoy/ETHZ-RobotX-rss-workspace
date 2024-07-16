#include <smb_power_payload/smb_plug_pay_panel.hpp>
#include <QVBoxLayout>
#include <QPixmap>
#include <QPen>
#include <QPainter>

namespace smb_rviz_plugins{
    PlugPanel::PlugPanel(QString name, QWidget * parent)
    :QWidget(parent){
        QVBoxLayout* layout = new QVBoxLayout;
        plug_name_ = new QLabel(name);
        plug_name_->setStyleSheet("font-weight: bold");
        layout->addWidget(plug_name_);

        QHBoxLayout* layout_indicator = new QHBoxLayout;

        plug_icon_ = new QLabel();
        setIcon(":/battery/connector.svg");
        layout_indicator->addWidget(plug_icon_);

        plug_text_ = new QLabel("No Data");
        layout_indicator->addWidget(plug_text_);

        layout_indicator->addStretch();

        layout->addLayout(layout_indicator);

        setLayout(layout);
    }

    void PlugPanel::setVoltage(double voltage){
        voltage_ = voltage;
        plug_text_->setText(QString("%1 V").arg(QString::number(voltage_, 'f', 2)));
    }

    void PlugPanel::setIcon(const QString &path){
        QPixmap pixmap(path);
        const auto& font_metrics = plug_icon_->fontMetrics();
        auto icon_width = font_metrics.averageCharWidth() * 6;
        auto icon_height = font_metrics.height()*2;
        plug_icon_->setPixmap(pixmap.scaled(icon_width, icon_height, Qt::KeepAspectRatio));
    }

    void PlugPanel::setInUse(bool in_use){
        auto pixmap = plug_icon_->pixmap()->toImage();
        QPainter painter(&pixmap);
        QPen pen;
        if(in_use){
            pen.setColor(Qt::green);
        }else{
            pen.setColor(Qt::red);
        }
        pen.setWidth(15);
        painter.setPen(pen);
        painter.drawPoint(5,5);
        painter.end();
        plug_icon_->setPixmap(QPixmap::fromImage(pixmap));
    }
}