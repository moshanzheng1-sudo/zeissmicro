#include "experimentrecorder.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QImageWriter>
#include <QJsonDocument>
#include <QMutexLocker>
#include <QRegularExpression>
#include <QSaveFile>

#include <cmath>
#include <limits>

namespace {

QString csvNumber(double value)
{
    if (!std::isfinite(value)) {
        return QStringLiteral("NaN");
    }
    return QString::number(value, 'f', 6);
}

double secondsFromNanoseconds(qint64 nanoseconds)
{
    return static_cast<double>(nanoseconds) / 1000000000.0;
}

QString safeDirectoryComponent(QString value, const QString &fallback)
{
    value = value.trimmed();
    if (value.isEmpty()) {
        value = fallback;
    }
    value.replace(QRegularExpression(QStringLiteral("[\\\\/:*?\"<>|]")),
                  QStringLiteral("_"));
    return value;
}

} // namespace

ExperimentRecorderWorker::ExperimentRecorderWorker(QObject *parent)
    : QObject(parent),
      m_active(false),
      m_frameLog(&m_frameLogFile),
      m_motionLog(&m_motionLogFile),
      m_eventLog(&m_eventLogFile),
      m_failedFrameCount(0)
{
    m_frameLog.setCodec("UTF-8");
    m_motionLog.setCodec("UTF-8");
    m_eventLog.setCodec("UTF-8");
}

QString ExperimentRecorderWorker::startTrial(const QString &rootDirectory,
                                             const QJsonObject &metadata)
{
    if (m_active) {
        return QString();
    }

    QString root = rootDirectory;
    if (root.isEmpty()) {
        root = QStringLiteral("MicroSystemExperiments");
    }

    QDir rootDir;
    if (!rootDir.mkpath(root)) {
        emit recorderError(QStringLiteral("Cannot create experiment root: %1").arg(root));
        return QString();
    }

    const QString timestamp = QDateTime::currentDateTime().toString(
        QStringLiteral("yyyyMMdd_HHmmss_zzz"));
    QString experimentName = QStringLiteral("Experiment_%1").arg(timestamp);
    QString experimentDirectory = QDir(root).filePath(experimentName);
    int suffix = 2;
    while (QFileInfo::exists(experimentDirectory)) {
        experimentName = QStringLiteral("Experiment_%1_%2")
                             .arg(timestamp)
                             .arg(suffix++, 2, 10, QLatin1Char('0'));
        experimentDirectory = QDir(root).filePath(experimentName);
    }

    const QString cellId = safeDirectoryComponent(
        metadata.value(QStringLiteral("cell_id")).toString(),
        QStringLiteral("Cell_001"));
    const QString trialId = QStringLiteral("%1_Trial_01").arg(cellId);
    m_trialDirectory = QDir(experimentDirectory).filePath(trialId);
    m_framesDirectory = QDir(m_trialDirectory).filePath(QStringLiteral("raw_frames"));

    if (!rootDir.mkpath(m_framesDirectory)) {
        emit recorderError(QStringLiteral("Cannot create trial directory: %1")
                               .arg(m_trialDirectory));
        m_trialDirectory.clear();
        return QString();
    }

    if (!openLog(m_frameLogFile, m_frameLog,
                 QDir(m_trialDirectory).filePath(QStringLiteral("frame_log.csv")),
                 QStringLiteral("frame_id,timestamp_s,z_command,z_encoder,x_encoder,y_encoder\n")) ||
        !openLog(m_motionLogFile, m_motionLog,
                 QDir(m_trialDirectory).filePath(QStringLiteral("motion_log.csv")),
                 QStringLiteral("timestamp_s,z_command,z_encoder,x_encoder,y_encoder\n")) ||
        !openLog(m_eventLogFile, m_eventLog,
                 QDir(m_trialDirectory).filePath(QStringLiteral("event_log.csv")),
                 QStringLiteral("event,timestamp_s,nearest_frame_id,z_command,z_encoder\n"))) {
        emit recorderError(QStringLiteral("Cannot open one or more trial log files in: %1")
                               .arg(m_trialDirectory));
        closeFiles();
        m_trialDirectory.clear();
        return QString();
    }

    m_metadata = metadata;
    m_metadata.insert(QStringLiteral("trial_id"), trialId);
    m_metadata.insert(QStringLiteral("recording_complete"), false);
    m_metadata.insert(QStringLiteral("frame_count"), 0);
    m_metadata.insert(QStringLiteral("dropped_frame_count"), 0);
    m_failedFrameCount = 0;
    m_active = true;

    if (!writeMetadata()) {
        emit recorderError(QStringLiteral("Cannot write metadata.json in: %1")
                               .arg(m_trialDirectory));
        closeFiles();
        m_active = false;
        m_trialDirectory.clear();
        return QString();
    }

    return m_trialDirectory;
}

void ExperimentRecorderWorker::stopTrial(qulonglong frameCount,
                                         qulonglong droppedFrameCount,
                                         double durationSeconds)
{
    if (!m_active) {
        return;
    }

    m_frameLog.flush();
    m_motionLog.flush();
    m_eventLog.flush();

    m_metadata.insert(QStringLiteral("recording_complete"), true);
    m_metadata.insert(QStringLiteral("frame_count"),
                      static_cast<double>(frameCount));
    m_metadata.insert(QStringLiteral("dropped_frame_count"),
                      static_cast<double>(droppedFrameCount));
    m_metadata.insert(QStringLiteral("failed_frame_count"),
                      static_cast<double>(m_failedFrameCount));
    m_metadata.insert(QStringLiteral("duration_s"), durationSeconds);
    m_metadata.insert(QStringLiteral("experiment_end_time"),
                      QDateTime::currentDateTime().toString(Qt::ISODate));
    if (!writeMetadata()) {
        emit recorderError(QStringLiteral("Failed to finalize metadata.json in: %1")
                               .arg(m_trialDirectory));
    }

    closeFiles();
    m_active = false;
}

void ExperimentRecorderWorker::writeFrame(const ExperimentFramePacket &packet)
{
    if (!m_active) {
        emit frameHandled();
        return;
    }

    const QString fileName = QStringLiteral("frame_%1.png")
                                 .arg(packet.frameId + 1, 6, 10, QLatin1Char('0'));
    const QString path = QDir(m_framesDirectory).filePath(fileName);
    QImageWriter writer(path, "png");
    writer.setCompression(1);
    if (!writer.write(packet.image)) {
        ++m_failedFrameCount;
        emit recorderError(QStringLiteral("Failed to save frame: %1").arg(path));
        emit frameHandled();
        return;
    }

    m_frameLog << packet.frameId << ','
               << csvNumber(secondsFromNanoseconds(packet.timestampNs)) << ','
               << csvNumber(packet.zCommand) << ','
               << csvNumber(packet.zEncoder) << ','
               << csvNumber(packet.xEncoder) << ','
               << csvNumber(packet.yEncoder) << '\n';
    m_frameLog.flush();
    emit frameHandled();
}

void ExperimentRecorderWorker::writeMotion(const ExperimentMotionPacket &packet)
{
    if (!m_active) {
        return;
    }

    m_motionLog << csvNumber(secondsFromNanoseconds(packet.timestampNs)) << ','
                << csvNumber(packet.zCommand) << ','
                << csvNumber(packet.zEncoder) << ','
                << csvNumber(packet.xEncoder) << ','
                << csvNumber(packet.yEncoder) << '\n';
    m_motionLog.flush();
}

void ExperimentRecorderWorker::writeEvent(const ExperimentEventPacket &packet)
{
    if (!m_active) {
        return;
    }

    m_eventLog << packet.event << ','
               << csvNumber(secondsFromNanoseconds(packet.timestampNs)) << ',';
    if (packet.nearestFrameId >= 0) {
        m_eventLog << packet.nearestFrameId;
    } else {
        m_eventLog << QStringLiteral("NaN");
    }
    m_eventLog << ',' << csvNumber(packet.zCommand)
               << ',' << csvNumber(packet.zEncoder) << '\n';
    m_eventLog.flush();
}

void ExperimentRecorderWorker::closeFiles()
{
    m_frameLog.flush();
    m_motionLog.flush();
    m_eventLog.flush();
    m_frameLogFile.close();
    m_motionLogFile.close();
    m_eventLogFile.close();
}

bool ExperimentRecorderWorker::openLog(QFile &file, QTextStream &stream,
                                       const QString &path,
                                       const QString &header)
{
    file.setFileName(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    stream << header;
    stream.flush();
    return true;
}

bool ExperimentRecorderWorker::writeMetadata() const
{
    QSaveFile file(QDir(m_trialDirectory).filePath(QStringLiteral("metadata.json")));
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }
    file.write(QJsonDocument(m_metadata).toJson(QJsonDocument::Indented));
    return file.commit();
}

ExperimentRecorder::ExperimentRecorder(QObject *parent)
    : QObject(parent),
      m_lastTimestampNs(-1),
      m_recording(false),
      m_nextFrameId(0),
      m_nearestFrameId(-1),
      m_frameCount(0),
      m_droppedFrameCount(0),
      m_pendingFrames(0),
      m_maxPendingFrames(16),
      m_zCommand(std::numeric_limits<double>::quiet_NaN()),
      m_zEncoder(std::numeric_limits<double>::quiet_NaN()),
      m_xEncoder(std::numeric_limits<double>::quiet_NaN()),
      m_yEncoder(std::numeric_limits<double>::quiet_NaN()),
      m_worker(new ExperimentRecorderWorker)
{
    qRegisterMetaType<ExperimentFramePacket>("ExperimentFramePacket");
    qRegisterMetaType<ExperimentMotionPacket>("ExperimentMotionPacket");
    qRegisterMetaType<ExperimentEventPacket>("ExperimentEventPacket");

    m_worker->moveToThread(&m_workerThread);
    connect(&m_workerThread, &QThread::finished,
            m_worker, &QObject::deleteLater);
    connect(this, &ExperimentRecorder::writeFrameRequested,
            m_worker, &ExperimentRecorderWorker::writeFrame,
            Qt::QueuedConnection);
    connect(this, &ExperimentRecorder::writeMotionRequested,
            m_worker, &ExperimentRecorderWorker::writeMotion,
            Qt::QueuedConnection);
    connect(this, &ExperimentRecorder::writeEventRequested,
            m_worker, &ExperimentRecorderWorker::writeEvent,
            Qt::QueuedConnection);
    connect(m_worker, &ExperimentRecorderWorker::frameHandled,
            this, &ExperimentRecorder::onFrameHandled,
            Qt::DirectConnection);
    connect(m_worker, &ExperimentRecorderWorker::recorderError,
            this, &ExperimentRecorder::recorderError,
            Qt::QueuedConnection);
    m_workerThread.setObjectName(QStringLiteral("ExperimentRecorderWorker"));
    m_workerThread.start();
}

ExperimentRecorder::~ExperimentRecorder()
{
    stopTrial();
    m_workerThread.quit();
    m_workerThread.wait();
}

void ExperimentRecorder::setMetadata(const QJsonObject &metadata)
{
    QMutexLocker locker(&m_mutex);
    if (!m_recording) {
        m_metadata = metadata;
    }
}

bool ExperimentRecorder::startTrial(const QString &rootDirectory)
{
    {
        QMutexLocker locker(&m_mutex);
        if (m_recording) {
            return false;
        }
    }

    QJsonObject metadata;
    {
        QMutexLocker locker(&m_mutex);
        metadata = m_metadata;
    }
    metadata.insert(QStringLiteral("experiment_time"),
                    QDateTime::currentDateTime().toString(Qt::ISODate));
    metadata.insert(QStringLiteral("timestamp_clock"),
                    QStringLiteral("QElapsedTimer monotonic nanoseconds"));
    metadata.insert(QStringLiteral("position_unit"), QStringLiteral("um"));

    QString directory;
    const bool invoked = QMetaObject::invokeMethod(
        m_worker, "startTrial", Qt::BlockingQueuedConnection,
        Q_RETURN_ARG(QString, directory),
        Q_ARG(QString, rootDirectory),
        Q_ARG(QJsonObject, metadata));
    if (!invoked || directory.isEmpty()) {
        return false;
    }

    {
        QMutexLocker locker(&m_mutex);
        m_trialDirectory = directory;
        m_trialId = QFileInfo(directory).fileName();
        m_nextFrameId = 0;
        m_nearestFrameId = -1;
        m_frameCount = 0;
        m_droppedFrameCount = 0;
        m_pendingFrames = 0;
        m_zCommand = std::numeric_limits<double>::quiet_NaN();
        m_zEncoder = std::numeric_limits<double>::quiet_NaN();
        m_xEncoder = std::numeric_limits<double>::quiet_NaN();
        m_yEncoder = std::numeric_limits<double>::quiet_NaN();
        m_lastTimestampNs = -1;
        m_elapsedTimer.start();
        m_recording = true;
    }

    emit frameCountChanged(0);
    emit droppedFrameCountChanged(0);
    emit recordingChanged(true);
    return true;
}

void ExperimentRecorder::stopTrial()
{
    qulonglong frames = 0;
    qulonglong dropped = 0;
    double duration = 0.0;
    {
        QMutexLocker locker(&m_mutex);
        if (!m_recording) {
            return;
        }
        duration = secondsFromNanoseconds(nextTimestampNsLocked());
        m_recording = false;
        frames = m_frameCount;
        dropped = m_droppedFrameCount;
    }

    QMetaObject::invokeMethod(
        m_worker, "stopTrial", Qt::BlockingQueuedConnection,
        Q_ARG(qulonglong, frames),
        Q_ARG(qulonglong, dropped),
        Q_ARG(double, duration));
    emit recordingChanged(false);
}

bool ExperimentRecorder::isRecording() const
{
    QMutexLocker locker(&m_mutex);
    return m_recording;
}

qulonglong ExperimentRecorder::frameCount() const
{
    QMutexLocker locker(&m_mutex);
    return m_frameCount;
}

qulonglong ExperimentRecorder::droppedFrameCount() const
{
    QMutexLocker locker(&m_mutex);
    return m_droppedFrameCount;
}

double ExperimentRecorder::elapsedSeconds() const
{
    QMutexLocker locker(&m_mutex);
    if (!m_recording || !m_elapsedTimer.isValid()) {
        return 0.0;
    }
    return secondsFromNanoseconds(m_elapsedTimer.nsecsElapsed());
}

QString ExperimentRecorder::trialDirectory() const
{
    QMutexLocker locker(&m_mutex);
    return m_trialDirectory;
}

QString ExperimentRecorder::trialId() const
{
    QMutexLocker locker(&m_mutex);
    return m_trialId;
}

void ExperimentRecorder::recordFrame(const QImage &image)
{
    qulonglong count = 0;
    qulonglong dropped = 0;
    bool accepted = false;
    bool overflow = false;
    {
        QMutexLocker locker(&m_mutex);
        if (!m_recording || image.isNull()) {
            return;
        }
        if (m_pendingFrames >= m_maxPendingFrames) {
            ++m_droppedFrameCount;
            dropped = m_droppedFrameCount;
            overflow = true;
        } else {
            ExperimentFramePacket packet;
            packet.image = image;
            packet.frameId = m_nextFrameId++;
            packet.timestampNs = nextTimestampNsLocked();
            packet.zCommand = m_zCommand;
            packet.zEncoder = m_zEncoder;
            packet.xEncoder = m_xEncoder;
            packet.yEncoder = m_yEncoder;
            m_nearestFrameId = static_cast<qlonglong>(packet.frameId);
            ++m_frameCount;
            ++m_pendingFrames;
            count = m_frameCount;
            emit writeFrameRequested(packet);
            accepted = true;
        }
    }

    if (accepted) {
        emit frameCountChanged(count);
    } else if (overflow) {
        emit droppedFrameCountChanged(dropped);
    }
}

void ExperimentRecorder::recordMotionCommand(double zCommandUm)
{
    QMutexLocker locker(&m_mutex);
    if (!m_recording) {
        return;
    }
    if (std::isfinite(zCommandUm)) {
        m_zCommand = zCommandUm;
    }
    emit writeMotionRequested(motionPacketLocked(nextTimestampNsLocked()));
}

void ExperimentRecorder::recordMotionData(double zEncoderUm, double xEncoderUm,
                                          double yEncoderUm)
{
    QMutexLocker locker(&m_mutex);
    if (!m_recording) {
        return;
    }
    m_zEncoder = zEncoderUm;
    m_xEncoder = xEncoderUm;
    m_yEncoder = yEncoderUm;
    emit writeMotionRequested(motionPacketLocked(nextTimestampNsLocked()));
}

void ExperimentRecorder::markEvent(const QString &event)
{
    QMutexLocker locker(&m_mutex);
    if (!m_recording) {
        return;
    }
    ExperimentEventPacket packet;
    packet.event = event;
    packet.timestampNs = nextTimestampNsLocked();
    packet.nearestFrameId = m_nearestFrameId;
    packet.zCommand = m_zCommand;
    packet.zEncoder = m_zEncoder;
    emit writeEventRequested(packet);
}

void ExperimentRecorder::onFrameHandled()
{
    QMutexLocker locker(&m_mutex);
    if (m_pendingFrames > 0) {
        --m_pendingFrames;
    }
}

qint64 ExperimentRecorder::nextTimestampNsLocked()
{
    qint64 timestamp = m_elapsedTimer.nsecsElapsed();
    if (timestamp <= m_lastTimestampNs) {
        timestamp = m_lastTimestampNs + 1;
    }
    m_lastTimestampNs = timestamp;
    return timestamp;
}

ExperimentMotionPacket ExperimentRecorder::motionPacketLocked(qint64 timestampNs) const
{
    ExperimentMotionPacket packet;
    packet.timestampNs = timestampNs;
    packet.zCommand = m_zCommand;
    packet.zEncoder = m_zEncoder;
    packet.xEncoder = m_xEncoder;
    packet.yEncoder = m_yEncoder;
    return packet;
}
