#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <QByteArray>
#include <QPushButton>

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    s = new SerialPortManager;
    s->openSerialPort("COM6", 115200);

    parser = new FlowParser(this);
    connect(parser, &FlowParser::sampleReady, this,
            [this](double xMm, double yMm, double heightMm,
                   double vxMmS, double vyMmS, int squal){
        ui->label->setText(QStringLiteral("相对位移  X: %1 mm   Y: %2 mm")
                           .arg(xMm, 0, 'f', 1)
                           .arg(yMm, 0, 'f', 1));
        ui->labelHeight->setText(QStringLiteral("高度: %1 mm").arg(heightMm, 0, 'f', 0));
        ui->labelVel->setText(QStringLiteral("速度  Vx: %1 mm/s   Vy: %2 mm/s   squal: %3")
                              .arg(vxMmS, 0, 'f', 1)
                              .arg(vyMmS, 0, 'f', 1)
                              .arg(squal));
    });

    connect(ui->resetButton, &QPushButton::clicked, this, [this](){
        parser->reset();
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
