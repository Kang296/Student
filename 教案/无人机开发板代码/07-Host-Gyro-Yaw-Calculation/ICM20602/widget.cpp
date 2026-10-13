#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <QByteArray>

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    s = new SerialPortManager;
    s->openSerialPort("COM6", 115200);

    parser = new GyroParser(this);
    connect(parser, &GyroParser::attitudeChanged, this, [this](double roll, double pitch, double yaw){
        ui->label->setText(QStringLiteral("横滚: %1°  俯仰: %2°  偏航: %3°")
                           .arg(roll, 0, 'f', 1)
                           .arg(pitch, 0, 'f', 1)
                           .arg(yaw, 0, 'f', 1));
    });
    connect(parser, &GyroParser::calibrationDone, this, [](double gx, double gy, double gz){
        qDebug() << QStringLiteral("标定完成，零漂 gx=%1 gy=%2 gz=%3 dps")
                    .arg(gx, 0, 'f', 2).arg(gy, 0, 'f', 2).arg(gz, 0, 'f', 2);
    });

    connect(s, &SerialPortManager::dataReceived, this, [this](const QByteArray& data){
        const QString line = QString::fromUtf8(data);
        qDebug() << line;
        parser->processLine(line);
    });
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_pushButton_clicked()
{
    QString text = ui->lineEdit->text();
    if (text.isEmpty())
    {
        qDebug() << "输入为空";
        return;
    }

    s->writeData(text.toUtf8());

    ui->lineEdit->clear();
}
