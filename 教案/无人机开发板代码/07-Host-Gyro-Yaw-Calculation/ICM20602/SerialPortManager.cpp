#include "SerialPortManager.h"
#include <QDebug>

SerialPortManager::SerialPortManager(QObject* parent)
	: QObject(parent), serialPort(new QSerialPort(this)) {
	connect(serialPort, &QSerialPort::readyRead, this, &SerialPortManager::handleReadyRead);
}

SerialPortManager::~SerialPortManager() {
	closeSerialPort();
}

bool SerialPortManager::openSerialPort(const QString& portName, qint32 baudRate) {
	serialPort->setPortName(portName);
	serialPort->setBaudRate(baudRate);
	serialPort->setDataBits(QSerialPort::Data8);
	serialPort->setParity(QSerialPort::NoParity);
	serialPort->setStopBits(QSerialPort::OneStop);
	serialPort->setFlowControl(QSerialPort::NoFlowControl);

	if (serialPort->open(QIODevice::ReadWrite)) {
		qDebug() << "Serial port opened successfully.";
		return true;
	}
	else {
		qDebug() << "Failed to open serial port:" << serialPort->errorString();
		return false;
	}
}

void SerialPortManager::closeSerialPort() {
	if (serialPort->isOpen()) {
		serialPort->close();
		qDebug() << "Serial port closed.";
	}
}

void SerialPortManager::writeData(const QByteArray& data) {
	if (serialPort->isOpen()) {
		serialPort->write(data);
	}
}

QStringList SerialPortManager::getAvailablePorts() const {
	QStringList portList;
	foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts()) {
		portList << info.portName();
	}
	return portList;
}

void SerialPortManager::handleReadyRead() {
    m_recvBuf.append(serialPort->readAll()); // 追加到缓冲区

    // 循环查找换行符 \n，分割完整行
    while(true)
    {
        int idx = m_recvBuf.indexOf('\n');
        if(idx < 0) break; // 没有完整一行，退出，等待下次数据

        // 取出一行（包含 \r）
        QByteArray line = m_recvBuf.left(idx+1);
        m_recvBuf = m_recvBuf.mid(idx+1); // 缓冲区剩余数据保留

        line = line.trimmed(); // 去掉 \r \n 回车换行
        if(!line.isEmpty())
        {
            emit dataReceived(line); // 发出**完整一行**
        }
    }
}
