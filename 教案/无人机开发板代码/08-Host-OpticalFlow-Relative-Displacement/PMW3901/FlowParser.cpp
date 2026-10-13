#include "FlowParser.h"
#include <QRegularExpression>

FlowParser::FlowParser(QObject *parent)
    : QObject(parent)
{
    m_timer.start();
}

void FlowParser::processLine(const QString &line)
{
    if (line.contains(QStringLiteral("squal")))
    {
        static const QRegularExpression squalRe(QStringLiteral(R"(squal\s*=\s*(\d+))"));
        const auto m = squalRe.match(line);
        if (!m.hasMatch())
            return;
        m_squal = m.captured(1).toInt();

        double vx = 0.0, vy = 0.0;

        if (m_hasPending)
        {
            if (m_squal >= m_minSqual)
            {
                const double dmmX = m_pendingDx * m_heightMm * kFlowScale;
                const double dmmY = m_pendingDy * m_heightMm * kFlowScale;

                const qint64 nowMs = m_timer.elapsed();
                if (m_hasLastTs)
                {
                    const double dtSec = (nowMs - m_lastTsMs) / 1000.0;
                    if (dtSec > 1e-3)
                    {
                        vx = dmmX / dtSec;
                        vy = dmmY / dtSec;
                    }
                }
                m_lastTsMs = nowMs;
                m_hasLastTs = true;

                m_xMm += dmmX;
                m_yMm += dmmY;
            }
            m_hasPending = false;
        }

        emit sampleReady(m_xMm, m_yMm, m_heightMm, vx, vy, m_squal);
        return;
    }

    if (line.contains(QStringLiteral("Distance"), Qt::CaseInsensitive))
    {
        static const QRegularExpression distRe(QStringLiteral(R"(Distance\s*:\s*([-+]?\d+(?:\.\d+)?))"),
                                               QRegularExpression::CaseInsensitiveOption);
        const auto m = distRe.match(line);
        if (m.hasMatch())
            m_heightMm = m.captured(1).toDouble();
        return;
    }

    static const QRegularExpression dxRe(QStringLiteral(R"(dx\s*=\s*([-+]?\d+))"));
    static const QRegularExpression dyRe(QStringLiteral(R"(dy\s*=\s*([-+]?\d+))"));
    const auto dxM = dxRe.match(line);
    const auto dyM = dyRe.match(line);
    if (!dxM.hasMatch() || !dyM.hasMatch())
        return;

    m_pendingDx = dxM.captured(1).toDouble();
    m_pendingDy = dyM.captured(1).toDouble();
    m_hasPending = true;
}

void FlowParser::reset()
{
    m_xMm = m_yMm = 0.0;
    m_hasPending = false;
    m_pendingDx = m_pendingDy = 0.0;
    m_hasLastTs = false;
    emit sampleReady(m_xMm, m_yMm, m_heightMm, 0.0, 0.0, m_squal);
}
