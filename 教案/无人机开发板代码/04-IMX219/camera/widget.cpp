#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <QHostAddress>
#include <QByteArray>
#include <opencv2/opencv.hpp>
#include <QImage>
#include <QPixmap>
#include <QDebug>

typedef struct {
    uint16_t header;    // 2字节 0xAA55
    uint32_t length;    // 4字节 图像长度
} FrameProto;  // 一共8字节

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    socket = new QUdpSocket;
    socket->bind(QHostAddress::Any, 9000);
    connect(socket, &QUdpSocket::readyRead, this, &Widget::handleMsg);
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

    socket->writeDatagram(text.toUtf8(), QHostAddress("192.168.137.1"), 9999);

    ui->lineEdit->clear();
}

void Widget::handleMsg()
{
    while (socket->hasPendingDatagrams())
    {
        QByteArray data;
        data.resize(socket->pendingDatagramSize());
        socket->readDatagram(data.data(), data.size());

        // 数据包至少要有协议头长度
        if(data.size() < sizeof(FrameProto))
        {
            qDebug() << "数据包过小，丢弃";
            continue;
        }

        // 解析帧头
        FrameProto *proto = (FrameProto*)data.data();
        if(proto->header != 0xAA55)
        {
            qDebug() << "帧头校验错误";
            continue;
        }

        // 取出真正的JPEG数据（跳过头部8字节）
        QByteArray jpegData = data.mid(sizeof(FrameProto));

        // 校验长度匹配
        if((uint32_t)jpegData.size() != proto->length)
        {
            qDebug() << "图像长度不匹配";
            continue;
        }

        // OpenCV解码JPEG
        cv::Mat buffer(1, jpegData.size(), CV_8UC1, (void *)jpegData.data());
        cv::Mat img = cv::imdecode(buffer, cv::IMREAD_COLOR);
        buffer.release();

        if (img.empty())
        {
            qDebug() << "JPEG解码失败";
            continue;
        }

        QImage image = QImage((const uchar*)img.data,
                              img.cols,
                              img.rows,
                              img.step,
                              QImage::Format_RGB888).rgbSwapped().copy();
        if (!image.isNull())
        {
            ui->label->setPixmap(QPixmap::fromImage(image).scaled(ui->label->size(), Qt::KeepAspectRatio));
        }
    }
}
