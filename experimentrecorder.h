#ifndef EXPERIMENTRECORDER_H
#define EXPERIMENTRECORDER_H

#include <QElapsedTimer>
#include <QFile>
#include <QImage>
#include <QJsonObject>
#include <QMutex>
#include <QObject>
#include <QTextStream>
#include <QThread>
#include <QWaitCondition>

struct ExperimentFramePacket
{
    QImage image;
    qulonglong frameId;
    qulonglong typeFrameId;
    qint64 timestampNs;
    bool keyFrame;
    qulonglong triggerStep;
    int channels;
    double zCommand;
    double zEncoder;
    double xEncoder;
    double yEncoder;
};

struct ExperimentMotionPacket
{
    qint64 timestampNs;
    double zCommand;
    double zEncoder;
    double xEncoder;
    double yEncoder;
};

struct ExperimentEventPacket
{
    QString event;
    qint64 timestampNs;
    qlonglong nearestFrameId;
    double zCommand;
    double zEncoder;
};

Q_DECLARE_METATYPE(ExperimentFramePacket)
Q_DECLARE_METATYPE(ExperimentMotionPacket)
Q_DECLARE_METATYPE(ExperimentEventPacket)

class ExperimentRecorderWorker : public QObject
{
    Q_OBJECT

public:
    explicit ExperimentRecorderWorker(QObject *parent = nullptr);

    Q_INVOKABLE QString startTrial(const QString &rootDirectory,
                                   const QJsonObject &metadata);
    Q_INVOKABLE void stopTrial(qulonglong frameCount,
                               qulonglong mhiFrameCount,
                               qulonglong keyFrameCount,
                               qulonglong droppedFrameCount,
                               qulonglong droppedMhiFrameCount,
                               qulonglong droppedKeyFrameCount,
                               double durationSeconds);

public slots:
    void writeFrame(const ExperimentFramePacket &packet);
    void writeMotion(const ExperimentMotionPacket &packet);
    void writeEvent(const ExperimentEventPacket &packet);

signals:
    void frameHandled(bool keyFrame);
    void recorderError(const QString &message);

private:
    void closeFiles();
    bool openLog(QFile &file, QTextStream &stream, const QString &path,
                 const QString &header);
    bool writeMetadata() const;

    bool m_active;
    QString m_trialDirectory;
    QString m_mhiFramesDirectory;
    QString m_keyFramesDirectory;
    QFile m_frameLogFile;
    QFile m_motionLogFile;
    QFile m_eventLogFile;
    QTextStream m_frameLog;
    QTextStream m_motionLog;
    QTextStream m_eventLog;
    QJsonObject m_metadata;
    qulonglong m_failedFrameCount;
    qulonglong m_failedMhiFrameCount;
    qulonglong m_failedKeyFrameCount;
};

class ExperimentRecorder : public QObject
{
    Q_OBJECT

public:
    explicit ExperimentRecorder(QObject *parent = nullptr);
    ~ExperimentRecorder();

    void setMetadata(const QJsonObject &metadata);
    bool startTrial(const QString &rootDirectory = QString());
    void stopTrial();

    bool isRecording() const;
    qulonglong frameCount() const;
    qulonglong droppedFrameCount() const;
    double elapsedSeconds() const;
    QString trialDirectory() const;
    QString trialId() const;

public slots:
    void recordMhiFrame(const QImage &image);
    void recordKeyFrame(const QImage &image, qulonglong triggerStep);
    void recordMotionCommand(double zCommandUm);
    void recordMotionData(double zEncoderUm, double xEncoderUm,
                          double yEncoderUm);
    void markEvent(const QString &event = QStringLiteral("manual_contact"));

signals:
    void writeFrameRequested(const ExperimentFramePacket &packet);
    void writeMotionRequested(const ExperimentMotionPacket &packet);
    void writeEventRequested(const ExperimentEventPacket &packet);
    void frameCountChanged(qulonglong frameCount);
    void droppedFrameCountChanged(qulonglong droppedFrameCount);
    void recordingChanged(bool recording);
    void recorderError(const QString &message);

private slots:
    void onFrameHandled(bool keyFrame);

private:
    qint64 nextTimestampNsLocked();
    ExperimentMotionPacket motionPacketLocked(qint64 timestampNs) const;
    void recordFrame(const QImage &image, bool keyFrame,
                     qulonglong triggerStep);

    mutable QMutex m_mutex;
    QElapsedTimer m_elapsedTimer;
    qint64 m_lastTimestampNs;
    bool m_recording;
    qulonglong m_nextFrameId;
    qulonglong m_nextMhiFrameId;
    qulonglong m_nextKeyFrameId;
    qlonglong m_nearestFrameId;
    qulonglong m_frameCount;
    qulonglong m_mhiFrameCount;
    qulonglong m_keyFrameCount;
    qulonglong m_droppedFrameCount;
    qulonglong m_droppedMhiFrameCount;
    qulonglong m_droppedKeyFrameCount;
    int m_pendingFrames;
    int m_pendingMhiFrames;
    int m_pendingKeyFrames;
    int m_maxPendingMhiFrames;
    int m_maxPendingKeyFrames;
    double m_zCommand;
    double m_zEncoder;
    double m_xEncoder;
    double m_yEncoder;
    QJsonObject m_metadata;
    QString m_trialDirectory;
    QString m_trialId;
    QWaitCondition m_pendingFramesFinished;

    QThread m_workerThread;
    ExperimentRecorderWorker *m_worker;
};

#endif // EXPERIMENTRECORDER_H
