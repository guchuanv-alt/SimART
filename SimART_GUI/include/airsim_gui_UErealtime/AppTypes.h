#pragma once

#include <QString>
#include <QMetaType>
#include <vector>
#include <cmath>
#include <limits>

namespace airsim_gui {

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct BaseStation {
    QString id;
    QString name;
    Vec3 position;
    Vec3 color{0.95, 0.55, 0.20};
    QString previewCameraName;
    QString previewRosTopic;
    double previewOffsetZ{0.0};
    double previewFps{60.0};
    bool previewCameraTargetEnabled{false};
    Vec3 previewCameraTarget{};
};

struct SceneBlock {
    QString id;
    Vec3 center;
    Vec3 size{10.0, 10.0, 10.0};
    Vec3 color{0.70, 0.70, 0.75};
};

struct BeamPath {
    QString id;
    QString txId;
    QString rxId;
    std::vector<Vec3> points;
    double powerDb{-70.0};
    double delayNs{0.0};
    QString kind{"specular"};
};

using BeamPathList = std::vector<BeamPath>;

struct RfPathObservationData {
    QString pathId;
    int pathType{0};
    int pathOrder{0};
    int pathIndex{0};
    bool isValid{false};
    bool hasAmplitude{false};
    double amplitudeAbs{0.0};
    bool hasPathGain{false};
    double pathGainLinear{0.0};
    bool hasPowerDb{false};
    double powerDb{0.0};
    bool hasTau{false};
    double tauS{0.0};
    double tauStdS{0.0};
    bool hasDoppler{false};
    double dopplerHz{0.0};
    double dopplerStdHz{0.0};
    bool hasAoa{false};
    double aoaAzRad{0.0};
    double aoaElRad{0.0};
    double aoaAzStdRad{0.0};
    double aoaElStdRad{0.0};
    Vec3 txPosition;
    Vec3 rxPosition;
    std::vector<Vec3> pathPoints;
};

struct RfAnchorObservationData {
    int anchorId{0};
    QString anchorName;
    Vec3 anchorPosition;
    std::vector<RfPathObservationData> paths;
};

struct RfObservationSnapshot {
    bool valid{false};
    QString frameId;
    Vec3 rxPosition;
    Vec3 rxVelocity;
    std::vector<RfAnchorObservationData> anchors;
};

struct DroneState {
    QString id{"uav_1"};
    Vec3 position;
    double rollDeg{0.0};
    double pitchDeg{0.0};
    double yawDeg{0.0};
    QString sourceFrameId;
    std::vector<Vec3> history;
    bool valid{false};
};

struct WorldConfig {
    double groundSizeX{200.0};
    double groundSizeY{200.0};
    std::vector<SceneBlock> blocks;
};

inline double clamp01(double v) {
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
}

inline Vec3 colorFromPowerDb(double powerDb) {
    // Map [-100, -40] dB to blue -> green -> red.
    const double t = clamp01((powerDb + 100.0) / 60.0);
    Vec3 out;
    if (t < 0.5) {
        const double u = t / 0.5;
        out = {0.1 * (1.0 - u) + 0.1 * u,
               0.35 * (1.0 - u) + 0.9 * u,
               0.95 * (1.0 - u) + 0.2 * u};
    } else {
        const double u = (t - 0.5) / 0.5;
        out = {0.1 * (1.0 - u) + 0.95 * u,
               0.9 * (1.0 - u) + 0.2 * u,
               0.2 * (1.0 - u) + 0.15 * u};
    }
    return out;
}


inline QString sanitizedCameraToken(QString value) {
    value = value.trimmed();
    for (int i = 0; i < value.size(); ++i) {
        const QChar ch = value.at(i);
        const bool ok = ch.isLetterOrNumber() || ch == QChar('_');
        if (!ok) value[i] = QChar('_');
    }
    while (value.contains(QStringLiteral("__"))) {
        value.replace(QStringLiteral("__"), QStringLiteral("_"));
    }
    value = value.trimmed();
    while (value.startsWith(QChar('_'))) value.remove(0, 1);
    while (value.endsWith(QChar('_'))) value.chop(1);
    if (value.isEmpty()) value = QStringLiteral("station");
    if (!value.at(0).isLetter() && value.at(0) != QChar('_')) {
        value.prepend(QStringLiteral("bs_"));
    }
    return value;
}

inline QString stationTokenForResources(const BaseStation& station) {
    const QString token = !station.id.trimmed().isEmpty() ? station.id : station.name;
    return sanitizedCameraToken(token);
}

inline QString defaultPreviewCameraNameForStation(const BaseStation& station) {
    return stationTokenForResources(station) + QStringLiteral("_camera");
}

inline QString defaultPreviewRosTopicForStation(const BaseStation& station) {
    return QStringLiteral("/airsim_gui/station_cameras/%1/image_raw").arg(stationTokenForResources(station));
}

inline QString siblingCameraTopicForImageTopic(QString imageTopic, const QString& leafName) {
    imageTopic = imageTopic.trimmed();
    const QString leaf = leafName.trimmed();
    if (imageTopic.isEmpty()) {
        return leaf;
    }
    while (imageTopic.size() > 1 && imageTopic.endsWith(QChar('/'))) {
        imageTopic.chop(1);
    }
    const int slashIndex = imageTopic.lastIndexOf(QChar('/'));
    if (slashIndex <= 0) {
        return imageTopic + QChar('/') + leaf;
    }
    return imageTopic.left(slashIndex + 1) + leaf;
}

inline QString cameraInfoTopicForImageTopic(const QString& imageTopic) {
    return siblingCameraTopicForImageTopic(imageTopic, QStringLiteral("camera_info"));
}

inline QString cameraPoseTopicForImageTopic(const QString& imageTopic) {
    return siblingCameraTopicForImageTopic(imageTopic, QStringLiteral("camera_pose_ned"));
}

inline QString vecToString(const Vec3& v) {
    return QString("(%1, %2, %3)")
        .arg(v.x, 0, 'f', 2)
        .arg(v.y, 0, 'f', 2)
        .arg(v.z, 0, 'f', 2);
}

}  // namespace airsim_gui

Q_DECLARE_METATYPE(airsim_gui::Vec3)
Q_DECLARE_METATYPE(airsim_gui::BaseStation)
Q_DECLARE_METATYPE(airsim_gui::SceneBlock)
Q_DECLARE_METATYPE(airsim_gui::BeamPath)
Q_DECLARE_METATYPE(airsim_gui::BeamPathList)
Q_DECLARE_METATYPE(airsim_gui::DroneState)
Q_DECLARE_METATYPE(std::vector<airsim_gui::BaseStation>)
Q_DECLARE_METATYPE(std::vector<airsim_gui::SceneBlock>)

Q_DECLARE_METATYPE(airsim_gui::RfPathObservationData)
Q_DECLARE_METATYPE(airsim_gui::RfAnchorObservationData)
Q_DECLARE_METATYPE(airsim_gui::RfObservationSnapshot)
Q_DECLARE_METATYPE(std::vector<airsim_gui::RfAnchorObservationData>)
Q_DECLARE_METATYPE(std::vector<airsim_gui::RfPathObservationData>)
