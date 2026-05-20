#include "airsim_gui_UErealtime/JsonConfig.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace airsim_gui {
namespace {

bool readJsonObject(const QString& filePath, QJsonObject& root, QString* errorMessage) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = QString("Cannot open JSON file: %1").arg(filePath);
        }
        return false;
    }

    const QByteArray bytes = file.readAll();
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(bytes, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage) {
            *errorMessage = QString("JSON parse failed for %1: %2")
                                .arg(filePath, parseError.errorString());
        }
        return false;
    }

    root = doc.object();
    return true;
}

Vec3 toVec3(const QJsonArray& arr, const Vec3& fallback = {}) {
    if (arr.size() < 3) {
        return fallback;
    }
    return {arr.at(0).toDouble(fallback.x),
            arr.at(1).toDouble(fallback.y),
            arr.at(2).toDouble(fallback.z)};
}

BaseStation normalizedStationForSave(BaseStation station, int fallbackIndex) {
    station.id = station.id.trimmed();
    station.name = station.name.trimmed();
    if (station.id.isEmpty()) {
        station.id = QStringLiteral("bs_%1").arg(fallbackIndex + 1);
    }
    if (station.name.isEmpty()) {
        station.name = station.id;
    }
    station.color.x = clamp01(station.color.x);
    station.color.y = clamp01(station.color.y);
    station.color.z = clamp01(station.color.z);
    if (station.previewCameraName.trimmed().isEmpty()) {
        station.previewCameraName = defaultPreviewCameraNameForStation(station);
    } else {
        station.previewCameraName = sanitizedCameraToken(station.previewCameraName);
    }
    if (station.previewRosTopic.trimmed().isEmpty()) {
        station.previewRosTopic = defaultPreviewRosTopicForStation(station);
    } else {
        station.previewRosTopic = station.previewRosTopic.trimmed();
    }
    if (station.previewFps <= 0.0) {
        station.previewFps = 60.0;
    }
    return station;
}

}  // namespace

bool JsonConfig::loadBaseStations(const QString& filePath,
                                  std::vector<BaseStation>& stations,
                                  QString* errorMessage) {
    stations.clear();

    QJsonObject root;
    if (!readJsonObject(filePath, root, errorMessage)) {
        return false;
    }

    const QJsonArray items = root.value("base_stations").toArray();
    for (const QJsonValue& value : items) {
        const QJsonObject obj = value.toObject();
        BaseStation station;
        station.id = obj.value("id").toString();
        station.name = obj.value("name").toString(station.id);
        station.position = toVec3(obj.value("position").toArray());
        station.color = toVec3(obj.value("color").toArray(), {0.95, 0.55, 0.20});
        station.previewCameraName = obj.value("preview_camera_name").toString().trimmed();
        if (station.previewCameraName.isEmpty()) {
            station.previewCameraName = defaultPreviewCameraNameForStation(station);
        }
        station.previewRosTopic = obj.value("preview_ros_topic").toString().trimmed();
        station.previewOffsetZ = obj.value("preview_offset_z").toDouble(0.0);
        station.previewFps = obj.value("preview_fps").toDouble(60.0);
        if (obj.value("preview_camera_target").isArray()) {
            const QJsonArray targetArr = obj.value("preview_camera_target").toArray();
            if (targetArr.size() >= 3) {
                station.previewCameraTargetEnabled = true;
                station.previewCameraTarget = toVec3(targetArr);
            }
        }
        station = normalizedStationForSave(station, static_cast<int>(stations.size()));
        if (!station.id.isEmpty()) {
            stations.push_back(station);
        }
    }

    if (stations.empty() && errorMessage) {
        *errorMessage = QString("No base stations found in %1").arg(filePath);
    }
    return !stations.empty();
}

bool JsonConfig::loadWorldConfig(const QString& filePath,
                                 WorldConfig& world,
                                 QString* errorMessage) {
    world = WorldConfig{};

    QJsonObject root;
    if (!readJsonObject(filePath, root, errorMessage)) {
        return false;
    }

    world.groundSizeX = root.value("ground_size_x").toDouble(220.0);
    world.groundSizeY = root.value("ground_size_y").toDouble(220.0);

    const QJsonArray items = root.value("blocks").toArray();
    for (const QJsonValue& value : items) {
        const QJsonObject obj = value.toObject();
        SceneBlock block;
        block.id = obj.value("id").toString();
        block.center = toVec3(obj.value("center").toArray());
        block.size = toVec3(obj.value("size").toArray(), {10.0, 10.0, 10.0});
        block.color = toVec3(obj.value("color").toArray(), {0.7, 0.7, 0.75});
        world.blocks.push_back(block);
    }

    return true;
}


bool JsonConfig::saveBaseStations(const QString& filePath,
                                  const std::vector<BaseStation>& stations,
                                  QString* errorMessage) {
    QJsonArray items;
    for (int i = 0; i < static_cast<int>(stations.size()); ++i) {
        const BaseStation station = normalizedStationForSave(stations[static_cast<size_t>(i)], i);
        QJsonObject obj;
        obj["id"] = station.id;
        obj["name"] = station.name;
        obj["position"] = QJsonArray{station.position.x, station.position.y, station.position.z};
        obj["color"] = QJsonArray{station.color.x, station.color.y, station.color.z};
        obj["preview_camera_name"] = station.previewCameraName;
        obj["preview_ros_topic"] = station.previewRosTopic;
        obj["preview_offset_z"] = station.previewOffsetZ;
        obj["preview_fps"] = station.previewFps;
        if (station.previewCameraTargetEnabled) {
            obj["preview_camera_target"] = QJsonArray{
                station.previewCameraTarget.x,
                station.previewCameraTarget.y,
                station.previewCameraTarget.z,
            };
        }
        items.push_back(obj);
    }

    QJsonObject root;
    root["base_stations"] = items;

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (errorMessage) {
            *errorMessage = QString("Cannot open JSON file for writing: %1").arg(filePath);
        }
        return false;
    }

    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();
    return true;
}

}  // namespace airsim_gui
