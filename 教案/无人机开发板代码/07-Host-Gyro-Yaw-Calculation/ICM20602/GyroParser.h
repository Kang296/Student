#ifndef GYROPARSER_H
#define GYROPARSER_H

#include <QObject>
#include <QVector>

class GyroParser : public QObject
{
    Q_OBJECT

public:
    explicit GyroParser(QObject *parent = nullptr);

    void processLine(const QString &line);

    double roll()  const { return m_roll; }
    double pitch() const { return m_pitch; }
    double yaw()   const { return m_yaw; }

    void setGyroBias(double gxDps, double gyDps, double gzDps);

    void setTau(double seconds) { m_tau = seconds > 0 ? seconds : 1.0; }

    void startCalibration(int sampleCount = 50);
    bool isCalibrating() const { return m_calibrating; }
    void reset();

signals:
    void attitudeChanged(double rollDeg, double pitchDeg, double yawDeg);

    void calibrationDone(double gxBias, double gyBias, double gzBias);

private:
    void finishCalibration();

    double m_roll = 0.0;   // 横滚角（度）
    double m_pitch = 0.0;  // 俯仰角（度）
    double m_yaw = 0.0;    // 偏航角（度）

    double m_biasX = 0.0;  // gx 零漂（dps）
    double m_biasY = 0.0;  // gy 零漂（dps）
    double m_biasZ = 0.0;  // gz 零漂（dps）

    double m_tau = 1.0;    // 互补滤波时间常数（秒）

    qint64 m_lastTs = -1;

    bool m_calibrating = true;
    int  m_calibTarget = 50;
    QVector<double> m_calibX, m_calibY, m_calibZ;
};

#endif // GYROPARSER_H
