#include "airsim_gui_UErealtime/FrameTransformManager.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <ros/duration.h>
#include <ros/time.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <utility>

namespace airsim_gui {
namespace {

struct NamedTransformEntry {
    QString frameId;
    TransformMatrix4x4 sourceToTarget;
};

geometry_msgs::TransformStamped makeTransformStamped(const TransformMatrix4x4& matrix,
                                                     const QString& parentFrame,
                                                     const QString& childFrame) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time(0);
    transform.header.frame_id = parentFrame.toStdString();
    transform.child_frame_id = childFrame.toStdString();
    transform.transform.translation.x = matrix.m[0][3];
    transform.transform.translation.y = matrix.m[1][3];
    transform.transform.translation.z = matrix.m[2][3];

    tf2::Matrix3x3 basis(
        matrix.m[0][0], matrix.m[0][1], matrix.m[0][2],
        matrix.m[1][0], matrix.m[1][1], matrix.m[1][2],
        matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);
    tf2::Quaternion quaternion;
    basis.getRotation(quaternion);
    transform.transform.rotation = tf2::toMsg(quaternion.normalized());
    return transform;
}

TransformMatrix4x4 loadTransformOrIdentity(const QString& filePath, QString* errorMessage) {
    if (filePath.trimmed().isEmpty()) {
        if (errorMessage) {
            errorMessage->clear();
        }
        return identityTransformMatrix4x4();
    }
    return loadTransformMatrixFromJson(filePath, errorMessage);
}

QString normalizeLocalFrameId(const QString& frameId, const QString& fallback = QString()) {
    const QString trimmed = frameId.trimmed();
    if (!trimmed.isEmpty()) {
        return trimmed;
    }
    return fallback.trimmed();
}

QString defaultAntennaFrameId(int index) {
    return QStringLiteral("rx_ant_%1").arg(std::max(index, 0));
}

QString normalizeRxArrayFacingDirectionKey(const QString& facingDirection) {
    const QString normalized = facingDirection.trimmed().toLower();
    if (normalized == QStringLiteral("back")
        || normalized == QStringLiteral("left")
        || normalized == QStringLiteral("right")
        || normalized == QStringLiteral("up")
        || normalized == QStringLiteral("down")) {
        return normalized;
    }
    return QStringLiteral("front");
}

Vec3 crossProduct(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

double norm3(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 normalizedOrFallback(const Vec3& value, const Vec3& fallback) {
    const double valueNorm = norm3(value);
    if (valueNorm > 1e-12 && std::isfinite(valueNorm)) {
        return {value.x / valueNorm, value.y / valueNorm, value.z / valueNorm};
    }
    const double fallbackNorm = norm3(fallback);
    if (fallbackNorm > 1e-12 && std::isfinite(fallbackNorm)) {
        return {fallback.x / fallbackNorm, fallback.y / fallbackNorm, fallback.z / fallbackNorm};
    }
    return {1.0, 0.0, 0.0};
}

TransformMatrix4x4 buildUavToRxArrayCenterTransform(const QString& facingDirection,
                                                    double tx,
                                                    double ty,
                                                    double tz) {
    const QString normalizedFacing = normalizeRxArrayFacingDirectionKey(facingDirection);
    Vec3 xAxis{1.0, 0.0, 0.0};
    if (normalizedFacing == QStringLiteral("back")) {
        xAxis = {-1.0, 0.0, 0.0};
    } else if (normalizedFacing == QStringLiteral("left")) {
        xAxis = {0.0, 1.0, 0.0};
    } else if (normalizedFacing == QStringLiteral("right")) {
        xAxis = {0.0, -1.0, 0.0};
    } else if (normalizedFacing == QStringLiteral("up")) {
        xAxis = {0.0, 0.0, 1.0};
    } else if (normalizedFacing == QStringLiteral("down")) {
        xAxis = {0.0, 0.0, -1.0};
    }

    xAxis = normalizedOrFallback(xAxis, {1.0, 0.0, 0.0});
    Vec3 yAxis{};
    Vec3 zAxis{};
    if (normalizedFacing == QStringLiteral("up") || normalizedFacing == QStringLiteral("down")) {
        yAxis = normalizedOrFallback({0.0, 1.0, 0.0}, {0.0, 1.0, 0.0});
        zAxis = normalizedOrFallback(crossProduct(xAxis, yAxis), {1.0, 0.0, 0.0});
        yAxis = normalizedOrFallback(crossProduct(zAxis, xAxis), {0.0, 1.0, 0.0});
    } else {
        zAxis = normalizedOrFallback({0.0, 0.0, 1.0}, {0.0, 0.0, 1.0});
        yAxis = normalizedOrFallback(crossProduct(zAxis, xAxis), {0.0, 1.0, 0.0});
        zAxis = normalizedOrFallback(crossProduct(xAxis, yAxis), {0.0, 0.0, 1.0});
    }

    TransformMatrix4x4 rxArrayInUav = identityTransformMatrix4x4();
    rxArrayInUav.m[0][0] = xAxis.x;
    rxArrayInUav.m[1][0] = xAxis.y;
    rxArrayInUav.m[2][0] = xAxis.z;
    rxArrayInUav.m[0][1] = yAxis.x;
    rxArrayInUav.m[1][1] = yAxis.y;
    rxArrayInUav.m[2][1] = yAxis.z;
    rxArrayInUav.m[0][2] = zAxis.x;
    rxArrayInUav.m[1][2] = zAxis.y;
    rxArrayInUav.m[2][2] = zAxis.z;
    rxArrayInUav.m[0][3] = std::isfinite(tx) ? tx : 0.0;
    rxArrayInUav.m[1][3] = std::isfinite(ty) ? ty : 0.0;
    rxArrayInUav.m[2][3] = std::isfinite(tz) ? tz : 0.0;
    rxArrayInUav.valid = true;

    TransformMatrix4x4 uavToRxArray = identityTransformMatrix4x4();
    if (!invertTransformMatrix4x4(rxArrayInUav, &uavToRxArray)) {
        return identityTransformMatrix4x4();
    }
    uavToRxArray.valid = true;
    return uavToRxArray;
}

double carrierWavelengthMeters(double carrierFrequencyHz) {
    constexpr double kSpeedOfLight = 299792458.0;
    if (!std::isfinite(carrierFrequencyHz) || carrierFrequencyHz <= 0.0) {
        return 1.0;
    }
    return kSpeedOfLight / carrierFrequencyHz;
}

bool jsonArrayLooksLikeMatrix4x4(const QJsonArray& array) {
    if (array.size() != 4) {
        return false;
    }
    for (int rowIndex = 0; rowIndex < 4; ++rowIndex) {
        const QJsonValue rowValue = array.at(rowIndex);
        if (!rowValue.isArray()) {
            return false;
        }
        const QJsonArray row = rowValue.toArray();
        if (row.size() != 4) {
            return false;
        }
    }
    return true;
}

bool parseMatrixFromJsonArray(const QJsonArray& array,
                              TransformMatrix4x4* matrix,
                              QString* errorMessage,
                              const QString& contextLabel) {
    if (!matrix) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("%1 has no output matrix destination.").arg(contextLabel);
        }
        return false;
    }
    if (!jsonArrayLooksLikeMatrix4x4(array)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("%1 must be a 4x4 JSON array.").arg(contextLabel);
        }
        return false;
    }
    *matrix = identityTransformMatrix4x4();
    for (int rowIndex = 0; rowIndex < 4; ++rowIndex) {
        const QJsonArray row = array.at(rowIndex).toArray();
        for (int colIndex = 0; colIndex < 4; ++colIndex) {
            matrix->m[rowIndex][colIndex] = row.at(colIndex).toDouble((rowIndex == colIndex) ? 1.0 : 0.0);
        }
    }
    matrix->valid = true;
    if (errorMessage) {
        errorMessage->clear();
    }
    return true;
}

bool parseNamedTransformEntry(const QJsonValue& value,
                              const QString& fallbackFrameId,
                              NamedTransformEntry* output,
                              QString* errorMessage,
                              const QString& contextLabel) {
    if (!output) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("%1 has no output entry destination.").arg(contextLabel);
        }
        return false;
    }

    if (value.isArray()) {
        output->frameId = normalizeLocalFrameId(fallbackFrameId, QStringLiteral("rx_ant_0"));
        return parseMatrixFromJsonArray(value.toArray(), &output->sourceToTarget, errorMessage, contextLabel);
    }

    if (!value.isObject()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("%1 must be either a 4x4 JSON array or an object with a matrix field.").arg(contextLabel);
        }
        return false;
    }

    const QJsonObject obj = value.toObject();
    output->frameId = normalizeLocalFrameId(
        obj.value(QStringLiteral("frame_id")).toString(),
        normalizeLocalFrameId(obj.value(QStringLiteral("name")).toString(),
                              normalizeLocalFrameId(obj.value(QStringLiteral("id")).toString(),
                                                    fallbackFrameId)));
    if (output->frameId.isEmpty()) {
        output->frameId = QStringLiteral("rx_ant_0");
    }

    const QStringList matrixKeys = {
        QStringLiteral("matrix"),
        QStringLiteral("transform"),
        QStringLiteral("T"),
    };
    for (const QString& key : matrixKeys) {
        if (obj.contains(key) && obj.value(key).isArray()) {
            return parseMatrixFromJsonArray(obj.value(key).toArray(), &output->sourceToTarget, errorMessage, contextLabel);
        }
    }

    if (errorMessage) {
        *errorMessage = QStringLiteral("%1 is missing a 4x4 matrix field ('matrix', 'transform', or 'T').").arg(contextLabel);
    }
    return false;
}

std::vector<NamedTransformEntry> loadNamedTransformsFromJson(const QString& filePath, QString* errorMessage) {
    if (errorMessage) {
        errorMessage->clear();
    }
    const QString trimmedPath = filePath.trimmed();
    if (trimmedPath.isEmpty()) {
        return {};
    }

    QFile file(trimmedPath);
    if (!file.exists()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Transform list file does not exist: %1").arg(trimmedPath);
        }
        return {};
    }
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Could not open transform list file: %1").arg(trimmedPath);
        }
        return {};
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || (!doc.isObject() && !doc.isArray())) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to parse transform list JSON %1: %2")
                .arg(trimmedPath, parseError.errorString());
        }
        return {};
    }

    std::vector<NamedTransformEntry> entries;
    auto appendEntry = [&](const QJsonValue& value, const QString& fallbackFrameId, const QString& contextLabel) -> bool {
        NamedTransformEntry entry;
        QString parseErrorText;
        if (!parseNamedTransformEntry(value, fallbackFrameId, &entry, &parseErrorText, contextLabel)) {
            if (errorMessage) {
                *errorMessage = parseErrorText;
            }
            return false;
        }
        entries.push_back(entry);
        return true;
    };

    if (doc.isArray()) {
        const QJsonArray rootArray = doc.array();
        if (jsonArrayLooksLikeMatrix4x4(rootArray)) {
            if (!appendEntry(rootArray, defaultAntennaFrameId(0), QStringLiteral("Antenna transform root"))) {
                return {};
            }
        } else {
            for (int idx = 0; idx < rootArray.size(); ++idx) {
                if (!appendEntry(rootArray.at(idx),
                                 defaultAntennaFrameId(idx),
                                 QStringLiteral("Antenna transform entry %1").arg(idx))) {
                    return {};
                }
            }
        }
        return entries;
    }

    const QJsonObject root = doc.object();
    if (root.contains(QStringLiteral("elements")) && root.value(QStringLiteral("elements")).isArray()) {
        const QJsonArray array = root.value(QStringLiteral("elements")).toArray();
        for (int idx = 0; idx < array.size(); ++idx) {
            if (!appendEntry(array.at(idx),
                             defaultAntennaFrameId(idx),
                             QStringLiteral("Antenna transform entry %1").arg(idx))) {
                return {};
            }
        }
        return entries;
    }
    if (root.contains(QStringLiteral("antennas")) && root.value(QStringLiteral("antennas")).isArray()) {
        const QJsonArray array = root.value(QStringLiteral("antennas")).toArray();
        for (int idx = 0; idx < array.size(); ++idx) {
            if (!appendEntry(array.at(idx),
                             defaultAntennaFrameId(idx),
                             QStringLiteral("Antenna transform entry %1").arg(idx))) {
                return {};
            }
        }
        return entries;
    }

    int autoIndex = 0;
    for (auto it = root.constBegin(); it != root.constEnd(); ++it) {
        const QString key = it.key();
        if (key == QStringLiteral("parent_frame")
            || key == QStringLiteral("source_frame")
            || key == QStringLiteral("frame_prefix")) {
            continue;
        }
        if (!appendEntry(it.value(),
                         normalizeLocalFrameId(key, defaultAntennaFrameId(autoIndex)),
                         QStringLiteral("Antenna transform entry '%1'").arg(key))) {
            return {};
        }
        ++autoIndex;
    }
    return entries;
}

std::vector<NamedTransformEntry> generatePlanarArrayTransforms(int numRows,
                                                               int numCols,
                                                               double verticalSpacingLambda,
                                                               double horizontalSpacingLambda,
                                                               double carrierFrequencyHz) {
    const int rows = std::max(numRows, 1);
    const int cols = std::max(numCols, 1);
    const double wavelength = carrierWavelengthMeters(carrierFrequencyHz);
    const double dv = std::max(0.0, verticalSpacingLambda) * wavelength;
    const double dh = std::max(0.0, horizontalSpacingLambda) * wavelength;

    std::vector<NamedTransformEntry> entries;
    entries.reserve(static_cast<size_t>(rows * cols));
    int antennaIndex = 0;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const double y = dh * static_cast<double>(col) - (static_cast<double>(cols) - 1.0) * dh / 2.0;
            const double z = -dv * static_cast<double>(row) + (static_cast<double>(rows) - 1.0) * dv / 2.0;
            NamedTransformEntry entry;
            entry.frameId = defaultAntennaFrameId(antennaIndex++);
            entry.sourceToTarget = identityTransformMatrix4x4();
            entry.sourceToTarget.m[0][3] = 0.0;
            entry.sourceToTarget.m[1][3] = -y;
            entry.sourceToTarget.m[2][3] = -z;
            entry.sourceToTarget.valid = true;
            entries.push_back(entry);
        }
    }
    return entries;
}

TransformMatrix4x4 transformMatrixFromScenePose(const Vec3& position,
                                                double rollDeg,
                                                double pitchDeg,
                                                double yawDeg) {
    TransformMatrix4x4 matrix = identityTransformMatrix4x4();
    tf2::Quaternion quaternion;
    quaternion.setRPY(rollDeg * M_PI / 180.0,
                      pitchDeg * M_PI / 180.0,
                      yawDeg * M_PI / 180.0);
    tf2::Matrix3x3 basis(quaternion.normalized());
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            matrix.m[row][col] = basis[row][col];
        }
    }
    matrix.m[0][3] = position.x;
    matrix.m[1][3] = position.y;
    matrix.m[2][3] = position.z;
    matrix.valid = true;
    return matrix;
}

}  // namespace

QString coordinateOutputFrameToKey(CoordinateOutputFrame frame) {
    switch (frame) {
    case CoordinateOutputFrame::Ros:
        return QStringLiteral("ros");
    case CoordinateOutputFrame::AirSim:
        return QStringLiteral("airsim");
    case CoordinateOutputFrame::Scene3D:
    default:
        return QStringLiteral("3d");
    }
}

CoordinateOutputFrame coordinateOutputFrameFromKey(const QString& key) {
    const QString normalized = key.trimmed().toLower();
    if (normalized == QStringLiteral("ros")) {
        return CoordinateOutputFrame::Ros;
    }
    if (normalized == QStringLiteral("airsim")) {
        return CoordinateOutputFrame::AirSim;
    }
    return CoordinateOutputFrame::Scene3D;
}

FrameTransformManager::FrameTransformManager() = default;

FrameTransformManager::~FrameTransformManager() = default;

QString FrameTransformManager::sceneFrameId() {
    return QStringLiteral("3D");
}

QString FrameTransformManager::airsimFrameId() {
    return QStringLiteral("airsim");
}

QString FrameTransformManager::uavComFrameId() {
    return QStringLiteral("uav_com");
}

QString FrameTransformManager::rxArrayCenterFrameId() {
    return QStringLiteral("rx_array_center");
}

QString FrameTransformManager::normalizeFrameId(const QString& frameId, const QString& fallback) {
    const QString trimmed = frameId.trimmed();
    if (!trimmed.isEmpty()) {
        return trimmed;
    }
    return fallback.trimmed();
}

void FrameTransformManager::setRosFrameId(const QString& frameId) {
    const QString normalized = normalizeFrameId(frameId, QStringLiteral("ROS"));
    std::lock_guard<std::mutex> lock(mutex_);
    if (rosFrameId_ == normalized) {
        return;
    }
    rosFrameId_ = normalized;
    bufferDirty_ = true;
}

QString FrameTransformManager::rosFrameId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return rosFrameId_;
}

QString FrameTransformManager::resolvedRosFrameId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return normalizeFrameId(rosFrameId_, QStringLiteral("ROS"));
}

void FrameTransformManager::setRosToSceneTransformPath(const QString& filePath) {
    std::lock_guard<std::mutex> lock(mutex_);
    const QString normalized = filePath.trimmed();
    if (rosToSceneTransformPath_ == normalized) {
        return;
    }
    rosToSceneTransformPath_ = normalized;
    bufferDirty_ = true;
}

void FrameTransformManager::setAirSimToSceneTransformPath(const QString& filePath) {
    std::lock_guard<std::mutex> lock(mutex_);
    const QString normalized = filePath.trimmed();
    if (airsimToSceneTransformPath_ == normalized) {
        return;
    }
    airsimToSceneTransformPath_ = normalized;
    bufferDirty_ = true;
}

void FrameTransformManager::setRxArrayFacingDirection(const QString& facingDirection) {
    std::lock_guard<std::mutex> lock(mutex_);
    const QString normalized = normalizeRxArrayFacingDirectionKey(facingDirection);
    if (rxArrayFacingDirection_ == normalized) {
        return;
    }
    rxArrayFacingDirection_ = normalized;
    bufferDirty_ = true;
}

void FrameTransformManager::setUavToRxArrayOffset(double tx, double ty, double tz) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double normalizedTx = std::isfinite(tx) ? tx : 0.0;
    const double normalizedTy = std::isfinite(ty) ? ty : 0.0;
    const double normalizedTz = std::isfinite(tz) ? tz : 0.0;
    const bool unchanged = std::abs(uavToRxArrayTx_ - normalizedTx) < 1e-12
        && std::abs(uavToRxArrayTy_ - normalizedTy) < 1e-12
        && std::abs(uavToRxArrayTz_ - normalizedTz) < 1e-12;
    if (unchanged) {
        return;
    }
    uavToRxArrayTx_ = normalizedTx;
    uavToRxArrayTy_ = normalizedTy;
    uavToRxArrayTz_ = normalizedTz;
    bufferDirty_ = true;
}

QString FrameTransformManager::rosToSceneTransformPath() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return rosToSceneTransformPath_;
}

QString FrameTransformManager::airsimToSceneTransformPath() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return airsimToSceneTransformPath_;
}

QString FrameTransformManager::rxArrayFacingDirection() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return rxArrayFacingDirection_;
}

Vec3 FrameTransformManager::uavToRxArrayOffset() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {uavToRxArrayTx_, uavToRxArrayTy_, uavToRxArrayTz_};
}

void FrameTransformManager::setUavToRxArrayCenterTransformPath(const QString& filePath) {
    std::lock_guard<std::mutex> lock(mutex_);
    const QString normalized = filePath.trimmed();
    if (uavToRxArrayCenterTransformPath_ == normalized) {
        return;
    }
    uavToRxArrayCenterTransformPath_ = normalized;
    bufferDirty_ = true;
}

void FrameTransformManager::setRxArrayElementsTransformPath(const QString& filePath) {
    std::lock_guard<std::mutex> lock(mutex_);
    const QString normalized = filePath.trimmed();
    if (rxArrayElementsTransformPath_ == normalized) {
        return;
    }
    rxArrayElementsTransformPath_ = normalized;
    bufferDirty_ = true;
}

QString FrameTransformManager::uavToRxArrayCenterTransformPath() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return uavToRxArrayCenterTransformPath_;
}

QString FrameTransformManager::rxArrayElementsTransformPath() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return rxArrayElementsTransformPath_;
}

void FrameTransformManager::setCurrentDronePoseInScene(const Vec3& position,
                                                       double rollDeg,
                                                       double pitchDeg,
                                                       double yawDeg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool unchanged = hasCurrentDronePose_
        && std::abs(currentDronePosition_.x - position.x) < 1e-9
        && std::abs(currentDronePosition_.y - position.y) < 1e-9
        && std::abs(currentDronePosition_.z - position.z) < 1e-9
        && std::abs(currentDroneRollDeg_ - rollDeg) < 1e-9
        && std::abs(currentDronePitchDeg_ - pitchDeg) < 1e-9
        && std::abs(currentDroneYawDeg_ - yawDeg) < 1e-9;
    if (unchanged) {
        return;
    }
    currentDronePosition_ = position;
    currentDroneRollDeg_ = rollDeg;
    currentDronePitchDeg_ = pitchDeg;
    currentDroneYawDeg_ = yawDeg;
    hasCurrentDronePose_ = true;
    bufferDirty_ = true;
}

void FrameTransformManager::clearCurrentDronePoseInScene() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasCurrentDronePose_) {
        return;
    }
    hasCurrentDronePose_ = false;
    bufferDirty_ = true;
}

void FrameTransformManager::setRxArrayGeometry(int numRows,
                                               int numCols,
                                               double verticalSpacingLambda,
                                               double horizontalSpacingLambda,
                                               double carrierFrequencyHz) {
    std::lock_guard<std::mutex> lock(mutex_);
    const int normalizedRows = std::max(numRows, 1);
    const int normalizedCols = std::max(numCols, 1);
    const double normalizedV = std::isfinite(verticalSpacingLambda) ? std::max(0.0, verticalSpacingLambda) : 0.5;
    const double normalizedH = std::isfinite(horizontalSpacingLambda) ? std::max(0.0, horizontalSpacingLambda) : 0.5;
    const double normalizedFc = (std::isfinite(carrierFrequencyHz) && carrierFrequencyHz > 0.0) ? carrierFrequencyHz : 3.5e9;
    const bool unchanged = rxArrayNumRows_ == normalizedRows
        && rxArrayNumCols_ == normalizedCols
        && std::abs(rxArrayVerticalSpacingLambda_ - normalizedV) < 1e-12
        && std::abs(rxArrayHorizontalSpacingLambda_ - normalizedH) < 1e-12
        && std::abs(carrierFrequencyHz_ - normalizedFc) < 1e-6;
    if (unchanged) {
        return;
    }
    rxArrayNumRows_ = normalizedRows;
    rxArrayNumCols_ = normalizedCols;
    rxArrayVerticalSpacingLambda_ = normalizedV;
    rxArrayHorizontalSpacingLambda_ = normalizedH;
    carrierFrequencyHz_ = normalizedFc;
    bufferDirty_ = true;
}

QString FrameTransformManager::frameIdForOutput(CoordinateOutputFrame outputFrame) const {
    switch (outputFrame) {
    case CoordinateOutputFrame::Ros:
        return resolvedRosFrameId();
    case CoordinateOutputFrame::AirSim:
        return airsimFrameId();
    case CoordinateOutputFrame::Scene3D:
    default:
        return sceneFrameId();
    }
}

QStringList FrameTransformManager::antennaFrameIds() const {
    ensureBufferReady(nullptr);
    std::lock_guard<std::mutex> lock(mutex_);
    return antennaFrameIds_;
}

void FrameTransformManager::ensureBufferReady(QString* errorMessage) const {
    if (errorMessage) {
        errorMessage->clear();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_ && !bufferDirty_) {
        return;
    }

    QString loadError;
    const TransformMatrix4x4 rosToScene = loadTransformOrIdentity(rosToSceneTransformPath_, &loadError);
    if (!loadError.trimmed().isEmpty()) {
        lastBuildError_ = loadError;
        if (errorMessage) {
            *errorMessage = loadError;
        }
        return;
    }

    const TransformMatrix4x4 airsimToScene = loadTransformOrIdentity(airsimToSceneTransformPath_, &loadError);
    if (!loadError.trimmed().isEmpty()) {
        lastBuildError_ = loadError;
        if (errorMessage) {
            *errorMessage = loadError;
        }
        return;
    }

    auto rebuiltBuffer = std::make_unique<tf2_ros::Buffer>(ros::Duration(3600.0), false);
    const QString rosFrame = normalizeFrameId(rosFrameId_, QStringLiteral("ROS"));
    rebuiltBuffer->setTransform(
        makeTransformStamped(rosToScene, sceneFrameId(), rosFrame),
        QStringLiteral("gui_frame_transformer_ros_to_scene").toStdString(),
        true);
    rebuiltBuffer->setTransform(
        makeTransformStamped(airsimToScene, sceneFrameId(), airsimFrameId()),
        QStringLiteral("gui_frame_transformer_airsim_to_scene").toStdString(),
        true);

    TransformMatrix4x4 uavToRxArray = identityTransformMatrix4x4();
    if (!uavToRxArrayCenterTransformPath_.trimmed().isEmpty()) {
        uavToRxArray = loadTransformOrIdentity(uavToRxArrayCenterTransformPath_, &loadError);
        if (!loadError.trimmed().isEmpty()) {
            lastBuildError_ = loadError;
            if (errorMessage) {
                *errorMessage = loadError;
            }
            return;
        }
    } else {
        uavToRxArray = buildUavToRxArrayCenterTransform(
            rxArrayFacingDirection_,
            uavToRxArrayTx_,
            uavToRxArrayTy_,
            uavToRxArrayTz_);
    }
    TransformMatrix4x4 rxArrayToUav = identityTransformMatrix4x4();
    if (!invertTransformMatrix4x4(uavToRxArray, &rxArrayToUav)) {
        lastBuildError_ = QStringLiteral("Could not invert UAV COM -> RX array center transform.");
        if (errorMessage) {
            *errorMessage = lastBuildError_;
        }
        return;
    }
    rebuiltBuffer->setTransform(
        makeTransformStamped(rxArrayToUav, uavComFrameId(), rxArrayCenterFrameId()),
        QStringLiteral("gui_frame_transformer_uav_to_rx_array_center").toStdString(),
        true);

    std::vector<NamedTransformEntry> antennaTransforms =
        loadNamedTransformsFromJson(rxArrayElementsTransformPath_, &loadError);
    if (!loadError.trimmed().isEmpty()) {
        lastBuildError_ = loadError;
        if (errorMessage) {
            *errorMessage = loadError;
        }
        return;
    }
    if (antennaTransforms.empty()) {
        antennaTransforms = generatePlanarArrayTransforms(
            rxArrayNumRows_,
            rxArrayNumCols_,
            rxArrayVerticalSpacingLambda_,
            rxArrayHorizontalSpacingLambda_,
            carrierFrequencyHz_);
    }
    QStringList rebuiltAntennaFrameIds;
    for (const NamedTransformEntry& entry : antennaTransforms) {
        TransformMatrix4x4 antennaToArray = identityTransformMatrix4x4();
        if (!invertTransformMatrix4x4(entry.sourceToTarget, &antennaToArray)) {
            lastBuildError_ = QStringLiteral("Could not invert RX array center -> antenna transform for frame '%1'.").arg(entry.frameId);
            if (errorMessage) {
                *errorMessage = lastBuildError_;
            }
            return;
        }
        rebuiltBuffer->setTransform(
            makeTransformStamped(antennaToArray, rxArrayCenterFrameId(), entry.frameId),
            QStringLiteral("gui_frame_transformer_rx_array_antenna_%1").arg(entry.frameId).toStdString(),
            true);
        rebuiltAntennaFrameIds.push_back(entry.frameId);
    }

    if (hasCurrentDronePose_) {
        const TransformMatrix4x4 uavPoseInScene = transformMatrixFromScenePose(
            currentDronePosition_,
            currentDroneRollDeg_,
            currentDronePitchDeg_,
            currentDroneYawDeg_);
        rebuiltBuffer->setTransform(
            makeTransformStamped(uavPoseInScene, sceneFrameId(), uavComFrameId()),
            QStringLiteral("gui_frame_transformer_scene_to_uav_pose").toStdString(),
            true);
    }

    buffer_ = std::move(rebuiltBuffer);
    antennaFrameIds_ = rebuiltAntennaFrameIds;
    bufferDirty_ = false;
    lastBuildError_.clear();
}

bool FrameTransformManager::transformPoint(const Vec3& input,
                                           const QString& sourceFrameId,
                                           const QString& targetFrameId,
                                           Vec3* output,
                                           QString* errorMessage) const {
    if (!output) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Output point pointer is null.");
        }
        return false;
    }

    const QString source = normalizeFrameId(sourceFrameId, sceneFrameId());
    const QString target = normalizeFrameId(targetFrameId, sceneFrameId());
    if (source == target) {
        *output = input;
        if (errorMessage) {
            errorMessage->clear();
        }
        return true;
    }

    ensureBufferReady(errorMessage);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!buffer_) {
        *output = input;
        return false;
    }

    geometry_msgs::PointStamped point;
    point.header.stamp = ros::Time(0);
    point.header.frame_id = source.toStdString();
    point.point.x = input.x;
    point.point.y = input.y;
    point.point.z = input.z;

    try {
        geometry_msgs::PointStamped transformed;
        buffer_->transform(point, transformed, target.toStdString());
        *output = {transformed.point.x, transformed.point.y, transformed.point.z};
        if (errorMessage) {
            errorMessage->clear();
        }
        return true;
    } catch (const std::exception& ex) {
        if (errorMessage) {
            *errorMessage = QString::fromUtf8(ex.what());
        }
        *output = input;
        return false;
    }
}

bool FrameTransformManager::transformVector(const Vec3& input,
                                            const QString& sourceFrameId,
                                            const QString& targetFrameId,
                                            Vec3* output,
                                            QString* errorMessage) const {
    if (!output) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Output vector pointer is null.");
        }
        return false;
    }

    const QString source = normalizeFrameId(sourceFrameId, sceneFrameId());
    const QString target = normalizeFrameId(targetFrameId, sceneFrameId());
    if (source == target) {
        *output = input;
        if (errorMessage) {
            errorMessage->clear();
        }
        return true;
    }

    ensureBufferReady(errorMessage);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!buffer_) {
        *output = input;
        return false;
    }

    geometry_msgs::Vector3Stamped vector;
    vector.header.stamp = ros::Time(0);
    vector.header.frame_id = source.toStdString();
    vector.vector.x = input.x;
    vector.vector.y = input.y;
    vector.vector.z = input.z;

    try {
        geometry_msgs::Vector3Stamped transformed;
        buffer_->transform(vector, transformed, target.toStdString());
        *output = {transformed.vector.x, transformed.vector.y, transformed.vector.z};
        if (errorMessage) {
            errorMessage->clear();
        }
        return true;
    } catch (const std::exception& ex) {
        if (errorMessage) {
            *errorMessage = QString::fromUtf8(ex.what());
        }
        *output = input;
        return false;
    }
}

bool FrameTransformManager::transformQuaternion(double x, double y, double z, double w,
                                                const QString& sourceFrameId,
                                                const QString& targetFrameId,
                                                double* outX,
                                                double* outY,
                                                double* outZ,
                                                double* outW,
                                                QString* errorMessage) const {
    if (!outX || !outY || !outZ || !outW) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Quaternion output pointer is null.");
        }
        return false;
    }

    const QString source = normalizeFrameId(sourceFrameId, sceneFrameId());
    const QString target = normalizeFrameId(targetFrameId, sceneFrameId());
    if (source == target) {
        *outX = x;
        *outY = y;
        *outZ = z;
        *outW = w;
        if (errorMessage) {
            errorMessage->clear();
        }
        return true;
    }

    ensureBufferReady(errorMessage);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!buffer_) {
        *outX = x;
        *outY = y;
        *outZ = z;
        *outW = w;
        return false;
    }

    try {
        const geometry_msgs::TransformStamped transform =
            buffer_->lookupTransform(target.toStdString(), source.toStdString(), ros::Time(0));
        tf2::Quaternion inputQuaternion(x, y, z, w);
        tf2::Quaternion frameQuaternion;
        tf2::fromMsg(transform.transform.rotation, frameQuaternion);
        const tf2::Quaternion outputQuaternion = (frameQuaternion * inputQuaternion).normalized();
        *outX = outputQuaternion.x();
        *outY = outputQuaternion.y();
        *outZ = outputQuaternion.z();
        *outW = outputQuaternion.w();
        if (errorMessage) {
            errorMessage->clear();
        }
        return true;
    } catch (const std::exception& ex) {
        if (errorMessage) {
            *errorMessage = QString::fromUtf8(ex.what());
        }
        *outX = x;
        *outY = y;
        *outZ = z;
        *outW = w;
        return false;
    }
}

}  // namespace airsim_gui
