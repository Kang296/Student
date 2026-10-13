#ifndef SERIALPORTMANAGER_H
#define SERIALPORTMANAGER_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QStringList>

class SerialPortManager : public QObject {
	Q_OBJECT

public:
	explicit SerialPortManager(QObject* parent = nullptr);
	~SerialPortManager();

	bool openSerialPort(const QString& portName, qint32 baudRate = QSerialPort::Baud9600);
	void closeSerialPort();
	void writeData(const QByteArray& data);

	QStringList getAvailablePorts() const; // 获取当前可用串口

signals:
	void dataReceived(const QByteArray& data);
private slots:
	void handleReadyRead();

private:
	QSerialPort* serialPort;
    QByteArray m_recvBuf;
};

#endif // SERIALPORTMANAGER_H
