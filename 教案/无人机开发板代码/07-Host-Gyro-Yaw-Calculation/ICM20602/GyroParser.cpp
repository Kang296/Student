#include "GyroParser.h"
#include <QRegularExpression>
#include <QDebug>
#include <QtMath>

static double wrap180(double a)
{
    while (a > 180.0)
        a -= 360.0;
    while (a < -180.0)
        a += 360.0;
    return a;
}

static double wrap360(double a)
{
    while (a >= 360.0)
        a -= 360.0;
    while (a < 0.0)
        a += 360.0;
    return a;
}

GyroParser::GyroParser(QObject *parent)
    : QObject(parent)
{
}

void GyroParser::processLine(const QString &line)
{
    if (!line.contains(QStringLiteral("PHY")))
        return;

    static const QRegularExpression tsRe(QStringLiteral(R"(\(\s*(\d+)\s*\))"));

    static const QRegularExpression axRe(QStringLiteral(R"(ax\s*=\s*([-+]?\d+(?:\.\d+)?))"));
    static const QRegularExpression ayRe(QStringLiteral(R"(ay\s*=\s*([-+]?\d+(?:\.\d+)?))"));
    static const QRegularExpression azRe(QStringLiteral(R"(az\s*=\s*([-+]?\d+(?:\.\d+)?))"));

    static const QRegularExpression gxRe(QStringLiteral(R"(gx\s*=\s*([-+]?\d+(?:\.\d+)?))"));
    static const QRegularExpression gyRe(QStringLiteral(R"(gy\s*=\s*([-+]?\d+(?:\.\d+)?))"));
    static const QRegularExpression gzRe(QStringLiteral(R"(gz\s*=\s*([-+]?\d+(?:\.\d+)?))"));

    const auto tsM = tsRe.match(line);
    const auto axM = axRe.match(line);
    const auto ayM = ayRe.match(line);
    const auto azM = azRe.match(line);
    const auto gxM = gxRe.match(line);
    const auto gyM = gyRe.match(line);
    const auto gzM = gzRe.match(line);

    if (!tsM.hasMatch() || !axM.hasMatch() || !ayM.hasMatch() || !azM.hasMatch() ||
        !gxM.hasMatch() || !gyM.hasMatch() || !gzM.hasMatch())
        return;

    const qint64 ts = tsM.captured(1).toLongLong();
    const double ax = axM.captured(1).toDouble();
    const double ay = ayM.captured(1).toDouble();
    const double az = azM.captured(1).toDouble();
    const double gx = gxM.captured(1).toDouble();
    const double gy = gyM.captured(1).toDouble();
    const double gz = gzM.captured(1).toDouble();

    // 标定阶段
    if (m_calibrating)
    {
        m_calibX.append(gx);
        m_calibY.append(gy);
        m_calibZ.append(gz);
        if (m_calibZ.size() >= m_calibTarget)
            finishCalibration();
        return;
    }

    if (m_lastTs < 0)
    {
        m_lastTs = ts;
        return;
    }

    double dt = (ts - m_lastTs) / 1000.0; // 毫秒 -> 秒
    m_lastTs = ts;
    if (dt <= 0.0 || dt > 5.0)
        return;

    const double rollAcc = qRadiansToDegrees(qAtan2(ay, az));
    const double pitchAcc = qRadiansToDegrees(qAtan2(-ax, qSqrt(ay * ay + az * az)));

    double k = dt / (dt + m_tau);

    const double mag = qSqrt(ax * ax + ay * ay + az * az);
    if (mag < 0.5 || mag > 2.0)
        k = 0.0;

    // 横滚/俯仰
    m_roll = (1.0 - k) * (m_roll + (gx - m_biasX) * dt) + k * rollAcc;
    m_pitch = (1.0 - k) * (m_pitch + (gy - m_biasY) * dt) + k * pitchAcc;

    // 偏航
    m_yaw += (gz - m_biasZ) * dt;

    m_roll = wrap180(m_roll);
    m_pitch = wrap180(m_pitch);
    m_yaw = wrap360(m_yaw);

    emit attitudeChanged(m_roll, m_pitch, m_yaw);
}

void GyroParser::setGyroBias(double gxDps, double gyDps, double gzDps)
{
    m_biasX = gxDps;
    m_biasY = gyDps;
    m_biasZ = gzDps;
    m_calibrating = false;
    m_calibX.clear();
    m_calibY.clear();
    m_calibZ.clear();
    m_roll = m_pitch = m_yaw = 0.0;
    m_lastTs = -1;
    emit calibrationDone(m_biasX, m_biasY, m_biasZ);
}

void GyroParser::startCalibration(int sampleCount)
{
    m_calibTarget = sampleCount > 0 ? sampleCount : 50;
    m_calibX.clear();
    m_calibY.clear();
    m_calibZ.clear();
    m_calibrating = true;
    m_roll = m_pitch = m_yaw = 0.0;
    m_lastTs = -1;
}

void GyroParser::finishCalibration()
{
    if (m_calibX.isEmpty())
        return;

    auto avg = [](const QVector<double> &v)
    {
        double sum = 0.0;
        for (double x : v)
            sum += x;
        return sum / v.size();
    };

    m_biasX = avg(m_calibX);
    m_biasY = avg(m_calibY);
    m_biasZ = avg(m_calibZ);

    m_calibX.clear();
    m_calibY.clear();
    m_calibZ.clear();
    m_calibrating = false;
    m_roll = m_pitch = m_yaw = 0.0;
    m_lastTs = -1;

    emit calibrationDone(m_biasX, m_biasY, m_biasZ);
}

void GyroParser::reset()
{
    m_roll = m_pitch = m_yaw = 0.0;
    m_lastTs = -1;
    m_calibX.clear();
    m_calibY.clear();
    m_calibZ.clear();
    m_calibrating = true;
}
