#ifndef FLOWPARSER_H
#define FLOWPARSER_H

#include <QObject>
#include <QElapsedTimer>

class FlowParser : public QObject
{
    Q_OBJECT

public:
    explicit FlowParser(QObject *parent = nullptr);

    void processLine(const QString &line);

    double xMm() const { return m_xMm; }
    double yMm() const { return m_yMm; }

    double heightMm() const { return m_heightMm; }
    int squal() const { return m_squal; }

    void setMinSqual(int s) { m_minSqual = s; }
    int minSqual() const { return m_minSqual; }

    void reset();

signals:
    void sampleReady(double xMm, double yMm, double heightMm,
                     double vxMmS, double vyMmS, int squal);

private:

    static constexpr double kFlowScale = 1.0 / 385.0;

    double m_xMm = 0.0;
    double m_yMm = 0.0;
    double m_heightMm = 0.0;
    int    m_squal = 0;
    int    m_minSqual = 10;


    bool   m_hasPending = false;
    double m_pendingDx = 0.0;
    double m_pendingDy = 0.0;


    QElapsedTimer m_timer;
    bool          m_hasLastTs = false;
    qint64        m_lastTsMs = 0;
};

#endif // FLOWPARSER_H
