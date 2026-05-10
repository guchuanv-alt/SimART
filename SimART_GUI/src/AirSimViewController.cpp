#include "airsim_gui_UErealtime/AirSimViewController.h"
#include "airsim_gui_UErealtime/FrameTransformManager.h"

#include <QImage>
#include <QMetaObject>
#include <QPointF>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <ros/master.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#ifdef AIRSIM_GUI_ENABLE_AIRSIM
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "common/VectorMath.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#endif

namespace airsim_gui {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinDistance = 1.0;
constexpr double kMaxDistance = 3000.0;
constexpr double kMinPitch = -89.0;
constexpr double kMaxPitch = 89.0;
constexpr double kProjectionNear = 0.05;
constexpr float kAirSimRpcTimeoutSec = 2.5f;
constexpr auto kRosImageSubscriptionStaleTimeout = std::chrono::seconds(3);
std::mutex g_airSimRpcRequestMutex;

inline double degToRad(double deg) {
    return deg * kPi / 180.0;
}

inline double clampValue(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}

inline double normalizeSignedDegrees(double value) {
    double normalized = std::fmod(value, 360.0);
    if (normalized > 180.0) {
        normalized -= 360.0;
    } else if (normalized <= -180.0) {
        normalized += 360.0;
    }
    return normalized;
}

QString formatAirSimDisabledMessage() {
    return QStringLiteral(
        "AirSim C++ RPC live view is not compiled in. Rebuild with -DAIRSIM_GUI_ENABLE_AIRSIM=ON and set AIRSIM_CLIENT_ROOT to your AirSim checkout.");
}

bool isGuiPreviewCameraName(const QString& cameraName) {
    return cameraName.trimmed().compare(QStringLiteral("GuiPreview"), Qt::CaseInsensitive) == 0;
}

struct Basis3 {
    Vec3 right;
    Vec3 up;
    Vec3 forward;
};

inline Vec3 makeVec3(double x, double y, double z) {
    return {x, y, z};
}

inline Vec3 addVec(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 subtractVec(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 scaleVec(const Vec3& v, double s) {
    return {v.x * s, v.y * s, v.z * s};
}

inline double dotVec(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 crossVec(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline double normVec(const Vec3& v) {
    return std::sqrt(dotVec(v, v));
}

inline Vec3 normalizeVec(const Vec3& v, const Vec3& fallback = {1.0, 0.0, 0.0}) {
    const double n = normVec(v);
    if (n < 1e-8) {
        return fallback;
    }
    return {v.x / n, v.y / n, v.z / n};
}

geometry_msgs::Quaternion cameraQuaternionFromBasisNed(const Vec3& right,
                                                       const Vec3& up,
                                                       const Vec3& forward) {
    const Vec3 normalizedForward = normalizeVec(forward, {1.0, 0.0, 0.0});
    const Vec3 normalizedRight = normalizeVec(right, {0.0, 1.0, 0.0});
    const Vec3 down = normalizeVec(scaleVec(up, -1.0), {0.0, 0.0, 1.0});
    tf2::Matrix3x3 basis(
        normalizedForward.x, normalizedRight.x, down.x,
        normalizedForward.y, normalizedRight.y, down.y,
        normalizedForward.z, normalizedRight.z, down.z);
    tf2::Quaternion quaternion;
    basis.getRotation(quaternion);
    return tf2::toMsg(quaternion.normalized());
}

inline Vec3 directionFromYawPitch(double yawDeg, double pitchDeg) {
    const double yaw = degToRad(yawDeg);
    const double pitch = degToRad(pitchDeg);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    return normalizeVec({cp * cy, cp * sy, sp}, {1.0, 0.0, 0.0});
}

inline Vec3 directionFromYawPitchNed(double yawDeg, double pitchDeg) {
    const double yaw = degToRad(yawDeg);
    const double pitch = degToRad(pitchDeg);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    return normalizeVec({cp * cy, cp * sy, -sp}, {1.0, 0.0, 0.0});
}

inline Vec3 computeCameraPosition(double fx, double fy, double fz,
                                  double yawDeg, double pitchDeg, double distance) {
    const Vec3 forward = directionFromYawPitch(yawDeg, pitchDeg);
    return {
        fx - distance * forward.x,
        fy - distance * forward.y,
        fz - distance * forward.z
    };
}

inline double safeAtanTanHalf(double fovDeg) {
    return std::tan(degToRad(clampValue(fovDeg, 20.0, 170.0)) * 0.5);
}

inline bool isValidDepthSample(float value) {
    return std::isfinite(value) && value > 0.0f;
}

float sampleDepthBilinear(const std::vector<float>& depth, int width, int height, double nx, double ny) {
    if (width <= 0 || height <= 0 || depth.size() != static_cast<size_t>(width * height)) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    const double x = clampValue(nx, 0.0, 1.0) * static_cast<double>(std::max(0, width - 1));
    const double y = clampValue(ny, 0.0, 1.0) * static_cast<double>(std::max(0, height - 1));
    const int x0 = std::max(0, std::min(width - 1, static_cast<int>(std::floor(x))));
    const int y0 = std::max(0, std::min(height - 1, static_cast<int>(std::floor(y))));
    const int x1 = std::max(0, std::min(width - 1, x0 + 1));
    const int y1 = std::max(0, std::min(height - 1, y0 + 1));
    const double tx = clampValue(x - static_cast<double>(x0), 0.0, 1.0);
    const double ty = clampValue(y - static_cast<double>(y0), 0.0, 1.0);

    struct Sample { int x; int y; double w; };
    const Sample samples[4] = {
        {x0, y0, (1.0 - tx) * (1.0 - ty)},
        {x1, y0, tx * (1.0 - ty)},
        {x0, y1, (1.0 - tx) * ty},
        {x1, y1, tx * ty}
    };

    double weightedSum = 0.0;
    double weightSum = 0.0;
    for (const Sample& sample : samples) {
        const float value = depth[static_cast<size_t>(sample.y * width + sample.x)];
        if (!isValidDepthSample(value)) continue;
        weightedSum += static_cast<double>(value) * sample.w;
        weightSum += sample.w;
    }
    if (weightSum > 1e-8) {
        return static_cast<float>(weightedSum / weightSum);
    }

    const int cx = std::max(0, std::min(width - 1, static_cast<int>(std::round(x))));
    const int cy = std::max(0, std::min(height - 1, static_cast<int>(std::round(y))));
    double bestDist2 = std::numeric_limits<double>::max();
    float bestValue = std::numeric_limits<float>::quiet_NaN();
    for (int radius = 1; radius <= 2; ++radius) {
        for (int oy = -radius; oy <= radius; ++oy) {
            for (int ox = -radius; ox <= radius; ++ox) {
                const int sx = std::max(0, std::min(width - 1, cx + ox));
                const int sy = std::max(0, std::min(height - 1, cy + oy));
                const float value = depth[static_cast<size_t>(sy * width + sx)];
                if (!isValidDepthSample(value)) continue;
                const double dist2 = static_cast<double>(ox * ox + oy * oy);
                if (dist2 < bestDist2) {
                    bestDist2 = dist2;
                    bestValue = value;
                }
            }
        }
        if (isValidDepthSample(bestValue)) {
            return bestValue;
        }
    }

    return std::numeric_limits<float>::quiet_NaN();
}

std::vector<float> resampleDepthToScene(const std::vector<float>& src, int srcWidth, int srcHeight, int dstWidth, int dstHeight) {
    if (srcWidth <= 0 || srcHeight <= 0 || dstWidth <= 0 || dstHeight <= 0
        || src.size() != static_cast<size_t>(srcWidth * srcHeight)) {
        return {};
    }
    if (srcWidth == dstWidth && srcHeight == dstHeight) {
        return src;
    }

    std::vector<float> dst(static_cast<size_t>(dstWidth * dstHeight), std::numeric_limits<float>::quiet_NaN());
    for (int y = 0; y < dstHeight; ++y) {
        const double ny = (dstHeight > 1) ? (static_cast<double>(y) / static_cast<double>(dstHeight - 1)) : 0.5;
        for (int x = 0; x < dstWidth; ++x) {
            const double nx = (dstWidth > 1) ? (static_cast<double>(x) / static_cast<double>(dstWidth - 1)) : 0.5;
            dst[static_cast<size_t>(y * dstWidth + x)] = sampleDepthBilinear(src, srcWidth, srcHeight, nx, ny);
        }
    }
    return dst;
}

QVariantMap makePointMap(double x, double y) {
    QVariantMap point;
    point.insert(QStringLiteral("x"), x);
    point.insert(QStringLiteral("y"), y);
    return point;
}

QVariantList makeSegmentVariant(const std::vector<QPointF>& points) {
    QVariantList segment;
    for (const QPointF& point : points) {
        segment.push_back(makePointMap(point.x(), point.y()));
    }
    return segment;
}

QString formatVecForUi(const Vec3& value) {
    return QStringLiteral("(%1, %2, %3)")
        .arg(value.x, 0, 'f', 2)
        .arg(value.y, 0, 'f', 2)
        .arg(value.z, 0, 'f', 2);
}

QImage adjustDisplayBrightness(const QImage& source, double factor) {
    if (source.isNull() || qFuzzyCompare(factor, 1.0)) {
        return source;
    }

    QImage adjusted = source.convertToFormat(QImage::Format_ARGB32);
    const auto scaleChannel = [factor](int channel) -> int {
        return std::max(0, std::min(255, static_cast<int>(std::lround(static_cast<double>(channel) * factor))));
    };

    for (int y = 0; y < adjusted.height(); ++y) {
        auto* line = reinterpret_cast<QRgb*>(adjusted.scanLine(y));
        for (int x = 0; x < adjusted.width(); ++x) {
            const QRgb pixel = line[x];
            line[x] = qRgba(scaleChannel(qRed(pixel)),
                            scaleChannel(qGreen(pixel)),
                            scaleChannel(qBlue(pixel)),
                            qAlpha(pixel));
        }
    }
    return adjusted;
}

QImage imageFromRosMessage(const sensor_msgs::Image& msg) {
    if (msg.width == 0 || msg.height == 0 || msg.step == 0 || msg.data.empty()) {
        return QImage();
    }
    const size_t requiredBytes = static_cast<size_t>(msg.step) * static_cast<size_t>(msg.height);
    if (msg.data.size() < requiredBytes) {
        return QImage();
    }

    const auto* data = msg.data.data();
    const int width = static_cast<int>(msg.width);
    const int height = static_cast<int>(msg.height);
    const int bytesPerLine = static_cast<int>(msg.step);
    const std::string& encoding = msg.encoding;

    if (encoding == "rgb8") {
        QImage view(data, width, height, bytesPerLine, QImage::Format_RGB888);
        return view.copy();
    }
    if (encoding == "bgr8") {
        QImage view(data, width, height, bytesPerLine, QImage::Format_RGB888);
        return view.rgbSwapped().copy();
    }
    if (encoding == "rgba8") {
        QImage view(data, width, height, bytesPerLine, QImage::Format_RGBA8888);
        return view.convertToFormat(QImage::Format_RGB32);
    }
    if (encoding == "bgra8") {
        QImage view(data, width, height, bytesPerLine, QImage::Format_ARGB32);
        return view.convertToFormat(QImage::Format_RGB32);
    }
    if (encoding == "mono8" || encoding == "8UC1") {
        QImage view(data, width, height, bytesPerLine, QImage::Format_Grayscale8);
        return view.copy();
    }

    return QImage();
}

#ifdef AIRSIM_GUI_ENABLE_AIRSIM
inline Vec3 toVec3(const msr::airlib::Vector3r& v) {
    return {static_cast<double>(v.x()), static_cast<double>(v.y()), static_cast<double>(v.z())};
}

inline Vec3 rotateByQuaternion(const Vec3& v, const msr::airlib::Quaternionr& q) {
    const double qw = static_cast<double>(q.w());
    const double qx = static_cast<double>(q.x());
    const double qy = static_cast<double>(q.y());
    const double qz = static_cast<double>(q.z());

    const Vec3 qv{qx, qy, qz};
    const Vec3 uv = crossVec(qv, v);
    const Vec3 uuv = crossVec(qv, uv);
    return addVec(v, scaleVec(addVec(scaleVec(uv, qw), uuv), 2.0));
}

inline Basis3 basisFromQuaternion(const msr::airlib::Quaternionr& q) {
    Vec3 forward = normalizeVec(rotateByQuaternion({1.0, 0.0, 0.0}, q), {1.0, 0.0, 0.0});
    Vec3 right = normalizeVec(rotateByQuaternion({0.0, 1.0, 0.0}, q), {0.0, 1.0, 0.0});
    Vec3 up = normalizeVec(rotateByQuaternion({0.0, 0.0, -1.0}, q), {0.0, 0.0, -1.0});

    right = normalizeVec(subtractVec(right, scaleVec(forward, dotVec(right, forward))), {0.0, 1.0, 0.0});
    up = normalizeVec(crossVec(right, forward), up);
    return {right, up, forward};
}

inline void applyRollToBasis(const Vec3& forward, double rollDeg, Vec3* right, Vec3* down) {
    if (!right || !down) {
        return;
    }
    const double roll = degToRad(rollDeg);
    const double c = std::cos(roll);
    const double s = std::sin(roll);
    const Vec3 baseRight = *right;
    const Vec3 baseDown = *down;
    *right = normalizeVec(addVec(scaleVec(baseRight, c), scaleVec(baseDown, s)), baseRight);
    *down = normalizeVec(addVec(scaleVec(baseRight, -s), scaleVec(baseDown, c)), baseDown);
    *right = normalizeVec(subtractVec(*right, scaleVec(forward, dotVec(*right, forward))), baseRight);
    *down = normalizeVec(crossVec(forward, *right), *down);
}

inline msr::airlib::Quaternionr quaternionFromBasisNed(const Vec3& forward, const Vec3& right, const Vec3& down) {
    const double m00 = forward.x;
    const double m01 = right.x;
    const double m02 = down.x;
    const double m10 = forward.y;
    const double m11 = right.y;
    const double m12 = down.y;
    const double m20 = forward.z;
    const double m21 = right.z;
    const double m22 = down.z;

    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    const double trace = m00 + m11 + m22;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * s;
        qx = (m21 - m12) / s;
        qy = (m02 - m20) / s;
        qz = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        qw = (m21 - m12) / s;
        qx = 0.25 * s;
        qy = (m01 + m10) / s;
        qz = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        qw = (m02 - m20) / s;
        qx = (m01 + m10) / s;
        qy = 0.25 * s;
        qz = (m12 + m21) / s;
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        qw = (m10 - m01) / s;
        qx = (m02 + m20) / s;
        qy = (m12 + m21) / s;
        qz = 0.25 * s;
    }

    const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    msr::airlib::Quaternionr q;
    if (norm > 1e-8) {
        q.w() = static_cast<float>(qw / norm);
        q.x() = static_cast<float>(qx / norm);
        q.y() = static_cast<float>(qy / norm);
        q.z() = static_cast<float>(qz / norm);
    } else {
        q.w() = 1.0f;
        q.x() = 0.0f;
        q.y() = 0.0f;
        q.z() = 0.0f;
    }
    return q;
}

inline msr::airlib::Quaternionr makeCameraQuaternionForDirectionAndRoll(const Vec3& direction, double rollDeg) {
    const Vec3 forward = normalizeVec(direction, {1.0, 0.0, 0.0});
    const Vec3 worldDown{0.0, 0.0, 1.0};
    Vec3 right = crossVec(worldDown, forward);
    if (normVec(right) < 1e-6) {
        right = {0.0, 1.0, 0.0};
    }
    right = normalizeVec(right, {0.0, 1.0, 0.0});
    Vec3 down = normalizeVec(crossVec(forward, right), worldDown);
    right = normalizeVec(crossVec(worldDown, forward), right);
    down = normalizeVec(crossVec(forward, right), down);
    applyRollToBasis(forward, rollDeg, &right, &down);
    return quaternionFromBasisNed(forward, right, down);
}

inline msr::airlib::Quaternionr makeAnchoredCameraQuaternionForDirection(const Vec3& direction) {
    return makeCameraQuaternionForDirectionAndRoll(direction, 0.0);
}

inline msr::airlib::Quaternionr makeAnchoredCameraQuaternion(double yawDeg, double pitchDeg, double rollDeg = 0.0) {
    return makeCameraQuaternionForDirectionAndRoll(directionFromYawPitchNed(yawDeg, pitchDeg), rollDeg);
}
#endif

}  // namespace

AirSimViewController::AirSimViewController(QObject* parent)
    : QObject(parent),
      frameTransformManager_(std::make_shared<FrameTransformManager>()) {
}

AirSimViewController::~AirSimViewController() {
    stopExternalPoseControlWorker();
    stop();
}

void AirSimViewController::ensureScratchPaths() {}

void AirSimViewController::setFrameTransformManager(const std::shared_ptr<FrameTransformManager>& manager) {
    frameTransformManager_ = manager ? manager : std::make_shared<FrameTransformManager>();
    if (frameTransformManager_) {
        frameTransformManager_->setAirSimToSceneTransformPath(transformMatrixPath_);
    }
    rebuildStationCameraPoseRequests();
    rebuildOverlayData();
}

void AirSimViewController::setAirsimToGuiTransformPath(const QString& filePath) {
    const QString normalized = filePath.trimmed();
    if (frameTransformManager_) {
        frameTransformManager_->setAirSimToSceneTransformPath(normalized);
    }
    {
        std::lock_guard<std::mutex> lock(transformMutex_);
        transformMatrixPath_ = normalized;
        transformLoaded_ = false;
        airsimToGuiTransform_ = identityTransformMatrix4x4();
        guiToAirsimTransform_ = identityTransformMatrix4x4();
    }
    rebuildStationCameraPoseRequests();
    rebuildOverlayData();
}

void AirSimViewController::ensureCoordinateTransformLoaded() {
    std::lock_guard<std::mutex> lock(transformMutex_);
    if (transformLoaded_) {
        return;
    }

    airsimToGuiTransform_ = identityTransformMatrix4x4();
    guiToAirsimTransform_ = identityTransformMatrix4x4();

    if (!transformMatrixPath_.trimmed().isEmpty()) {
        QString error;
        const TransformMatrix4x4 loaded = loadTransformMatrixFromJson(transformMatrixPath_, &error);
        if (error.trimmed().isEmpty()) {
            airsimToGuiTransform_ = loaded;
            if (!invertTransformMatrix4x4(airsimToGuiTransform_, &guiToAirsimTransform_)) {
                airsimToGuiTransform_ = identityTransformMatrix4x4();
                guiToAirsimTransform_ = identityTransformMatrix4x4();
            }
        }
    }

    transformLoaded_ = true;
}

Vec3 AirSimViewController::guiToAirSimPoint(const Vec3& guiPoint) const {
    if (frameTransformManager_) {
        Vec3 transformed = guiPoint;
        if (frameTransformManager_->transformPoint(guiPoint,
                                                   FrameTransformManager::sceneFrameId(),
                                                   FrameTransformManager::airsimFrameId(),
                                                   &transformed)) {
            return transformed;
        }
    }
    const_cast<AirSimViewController*>(this)->ensureCoordinateTransformLoaded();
    std::lock_guard<std::mutex> lock(transformMutex_);
    return transformPointByMatrix(guiToAirsimTransform_, guiPoint);
}

Vec3 AirSimViewController::airSimToGuiPoint(const Vec3& airsimPoint) const {
    if (frameTransformManager_) {
        Vec3 transformed = airsimPoint;
        if (frameTransformManager_->transformPoint(airsimPoint,
                                                   FrameTransformManager::airsimFrameId(),
                                                   FrameTransformManager::sceneFrameId(),
                                                   &transformed)) {
            return transformed;
        }
    }
    const_cast<AirSimViewController*>(this)->ensureCoordinateTransformLoaded();
    std::lock_guard<std::mutex> lock(transformMutex_);
    return transformPointByMatrix(airsimToGuiTransform_, airsimPoint);
}

void AirSimViewController::rebuildStationCameraPoseRequests() {
    std::vector<StationCameraPoseRequest> requests;
    requests.reserve(baseStations_.size());

    for (const BaseStation& station : baseStations_) {
        const QString cameraName = station.previewCameraName.trimmed().isEmpty()
            ? defaultPreviewCameraNameForStation(station)
            : station.previewCameraName.trimmed();
        if (cameraName.isEmpty() || isGuiPreviewCameraName(cameraName)) {
            continue;
        }

        StationCameraPoseRequest request;
        request.cameraName = cameraName;
        const Vec3 airsimPosition = guiToAirSimPoint(station.position);
        request.position = {airsimPosition.x, airsimPosition.y, airsimPosition.z + station.previewOffsetZ};
        if (station.previewCameraTargetEnabled) {
            request.hasTarget = true;
            request.target = guiToAirSimPoint(station.previewCameraTarget);
        }
        requests.push_back(request);
    }

    {
        std::lock_guard<std::mutex> lock(stationCameraPoseMutex_);
        stationCameraPoseRequests_ = std::move(requests);
        ++stationCameraPoseRevision_;
    }
}

void AirSimViewController::setBaseStations(const std::vector<BaseStation>& stations) {
    baseStations_ = stations;
    if (selectedBaseStationIndex_ >= static_cast<int>(baseStations_.size())) {
        selectedBaseStationIndex_ = baseStations_.empty() ? -1 : static_cast<int>(baseStations_.size()) - 1;
    }
    rebuildStationCameraPoseRequests();
    rebuildOverlayData();
}

void AirSimViewController::setBeamPaths(const BeamPathList& paths) {
    beamPaths_ = paths;
    rebuildOverlayData();
}

void AirSimViewController::setDroneState(const DroneState& state) {
    droneState_ = state;
    rebuildOverlayData();
}

void AirSimViewController::setSelectedBaseStationIndex(int index) {
    const int normalized = (index >= 0 && index < static_cast<int>(baseStations_.size())) ? index : -1;
    if (selectedBaseStationIndex_ == normalized) {
        return;
    }
    selectedBaseStationIndex_ = normalized;
    rebuildOverlayData();
}

void AirSimViewController::setBaseStationsVisible(bool visible) {
    if (baseStationsVisible_ == visible) {
        return;
    }
    baseStationsVisible_ = visible;
    rebuildOverlayData();
}

void AirSimViewController::setDroneVisible(bool visible) {
    if (droneVisible_ == visible) {
        return;
    }
    droneVisible_ = visible;
    rebuildOverlayData();
}

void AirSimViewController::setTrajectoryVisible(bool visible) {
    if (trajectoryVisible_ == visible) {
        return;
    }
    trajectoryVisible_ = visible;
    rebuildOverlayData();
}

void AirSimViewController::setPathsVisible(bool visible) {
    if (pathsVisible_ == visible) {
        return;
    }
    pathsVisible_ = visible;
    rebuildOverlayData();
}

void AirSimViewController::setViewGizmoVisible(bool visible) {
    if (viewGizmoVisible_ == visible) {
        return;
    }
    viewGizmoVisible_ = visible;
    emit viewGizmoVisibleChanged();
}

void AirSimViewController::setHost(const QString& value) {
    const QString trimmed = value.trimmed().isEmpty() ? QStringLiteral("127.0.0.1") : value.trimmed();
    const bool changed = host_ != trimmed;
    host_ = trimmed;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.host = host_;
        if (changed) {
            sharedState_.configDirty = true;
            if (sharedState_.externalPoseControlEnabled) {
                ++sharedState_.externalPoseControlRevision;
            }
        }
    }
    if (changed) {
        emit configChanged();
    }
}

void AirSimViewController::setPort(int value) {
    const int normalized = value > 0 ? value : 41451;
    const bool changed = port_ != normalized;
    port_ = normalized;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.port = port_;
        if (changed) {
            sharedState_.configDirty = true;
            if (sharedState_.externalPoseControlEnabled) {
                ++sharedState_.externalPoseControlRevision;
            }
        }
    }
    if (changed) {
        emit configChanged();
    }
}

void AirSimViewController::setCameraName(const QString& value) {
    const QString trimmed = value.trimmed().isEmpty() ? QStringLiteral("front_center") : value.trimmed();
    const bool changed = cameraName_ != trimmed;
    cameraName_ = trimmed;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.cameraName = cameraName_;
        if (changed) {
            sharedState_.configDirty = true;
            if (sharedState_.externalPoseControlEnabled) {
                ++sharedState_.externalPoseControlRevision;
            }
        }
    }
    if (changed) {
        emit configChanged();
        rebuildOverlayData();
    }
}

void AirSimViewController::setVehicleName(const QString& value) {
    const QString trimmed = value.trimmed();
    const bool changed = vehicleName_ != trimmed;
    vehicleName_ = trimmed;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.vehicleName = vehicleName_;
        if (changed) {
            sharedState_.configDirty = true;
        }
    }
    if (changed) {
        emit configChanged();
    }
}

void AirSimViewController::setViewLabel(const QString& value) {
    const QString trimmed = value.trimmed().isEmpty() ? QStringLiteral("AirSim Camera") : value.trimmed();
    if (viewLabel_ == trimmed) return;
    viewLabel_ = trimmed;
    emit viewLabelChanged();
}

void AirSimViewController::setFollowVehicle(bool value) {
    const bool changed = followVehicle_ != value;
    followVehicle_ = value;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.followVehicle = followVehicle_;
    }
    if (changed) {
        emit configChanged();
    }
}

void AirSimViewController::setFramesPerSecond(double value) {
    const double normalized = clampValue(value, 1.0, 100.0);
    const bool changed = !qFuzzyCompare(framesPerSecond_ + 1.0, normalized + 1.0);
    framesPerSecond_ = normalized;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.fps = framesPerSecond_;
    }
    if (changed) {
        emit configChanged();
    }
}

void AirSimViewController::setDepthFetchEnabled(bool value) {
    depthFetchEnabled_ = value;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.depthFetchEnabled = depthFetchEnabled_;
    }
}

void AirSimViewController::setAnchoredCamera(bool value) {
    const bool changed = anchoredCamera_ != value;
    anchoredCamera_ = value;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.anchoredCamera = anchoredCamera_;
        if (anchoredCamera_) {
            sharedState_.focusX = anchorPoint_.x;
            sharedState_.focusY = anchorPoint_.y;
            sharedState_.focusZ = anchorPoint_.z;
            sharedState_.focusInitialized = true;
            sharedState_.followVehicle = false;
        }
        if (changed && sharedState_.externalPoseControlEnabled) {
            ++sharedState_.externalPoseControlRevision;
        }
    }
    if (changed) {
        emit configChanged();
    }
}

void AirSimViewController::setAnchorPoint(const Vec3& value) {
    const bool changed = !qFuzzyCompare(anchorPoint_.x + 1.0, value.x + 1.0)
        || !qFuzzyCompare(anchorPoint_.y + 1.0, value.y + 1.0)
        || !qFuzzyCompare(anchorPoint_.z + 1.0, value.z + 1.0);
    anchorPoint_ = value;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.anchorX = value.x;
        sharedState_.anchorY = value.y;
        sharedState_.anchorZ = value.z;
        sharedState_.focusX = value.x;
        sharedState_.focusY = value.y;
        sharedState_.focusZ = value.z;
        sharedState_.focusInitialized = true;
        if (changed && sharedState_.externalPoseControlEnabled) {
            ++sharedState_.externalPoseControlRevision;
        }
    }
    if (changed) {
        emit configChanged();
        emit anchorPointChanged();
    }
}

void AirSimViewController::setViewAngles(double yawDeg, double pitchDeg) {
    const double normalizedPitch = clampValue(pitchDeg, kMinPitch, kMaxPitch);
    const bool changed = !qFuzzyCompare(yawDeg_ + 1.0, yawDeg + 1.0)
        || !qFuzzyCompare(pitchDeg_ + 1.0, normalizedPitch + 1.0)
        || !qFuzzyCompare(rollDeg_ + 1.0, 1.0);
    yawDeg_ = yawDeg;
    pitchDeg_ = normalizedPitch;
    rollDeg_ = 0.0;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.yawDeg = yawDeg_;
        sharedState_.pitchDeg = pitchDeg_;
        sharedState_.rollDeg = rollDeg_;
        if (anchoredCamera_) {
            sharedState_.focusX = anchorPoint_.x;
            sharedState_.focusY = anchorPoint_.y;
            sharedState_.focusZ = anchorPoint_.z;
            sharedState_.focusInitialized = true;
            sharedState_.followVehicle = false;
        }
        if (changed && sharedState_.externalPoseControlEnabled) {
            ++sharedState_.externalPoseControlRevision;
        }
    }
    if (changed) {
        emit configChanged();
        emit cameraStateChanged();
    }
}

void AirSimViewController::setViewPreset(const QString& preset) {
    const QString key = preset.trimmed().toLower();
    if (key == QStringLiteral("x_pos")) {
        setViewAngles(0.0, 0.0);
    } else if (key == QStringLiteral("x_neg")) {
        setViewAngles(180.0, 0.0);
    } else if (key == QStringLiteral("y_pos")) {
        setViewAngles(90.0, 0.0);
    } else if (key == QStringLiteral("y_neg")) {
        setViewAngles(-90.0, 0.0);
    } else if (key == QStringLiteral("z_pos") || key == QStringLiteral("top")) {
        setViewAngles(0.0, 89.0);
    } else if (key == QStringLiteral("z_neg") || key == QStringLiteral("bottom")) {
        setViewAngles(0.0, -89.0);
    } else if (key == QStringLiteral("iso")) {
        setViewAngles(45.0, 45.0);
    }
}

void AirSimViewController::rollViewQuarter(int direction) {
    if (direction == 0) {
        return;
    }
    const double delta = direction > 0 ? 90.0 : -90.0;
    bool changed = false;
    double roll = rollDeg_;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        const double nextRoll = normalizeSignedDegrees(sharedState_.rollDeg + delta);
        changed = !qFuzzyCompare(rollDeg_ + 1.0, nextRoll + 1.0);
        sharedState_.rollDeg = nextRoll;
        sharedState_.followVehicle = false;
        if (changed && sharedState_.externalPoseControlEnabled) {
            ++sharedState_.externalPoseControlRevision;
        }
        roll = sharedState_.rollDeg;
    }
    if (changed) {
        rollDeg_ = roll;
        emit configChanged();
        emit cameraStateChanged();
    }
}

void AirSimViewController::setRosPublishingEnabled(bool value) {
    if (rosPublishingEnabled_ == value) {
        if (rosPublishingEnabled_) {
            ensureRosPublisher();
        }
        return;
    }

    rosPublishingEnabled_ = value;
    if (!rosPublishingEnabled_) {
        shutdownRosPublisher();
        return;
    }

    ensureRosPublisher();
}

void AirSimViewController::setRosImageTopic(const QString& topicName) {
    const QString trimmed = topicName.trimmed();
    if (rosImageTopic_ == trimmed) {
        if (rosPublishingEnabled_) {
            ensureRosPublisher();
        }
        return;
    }
    rosImageTopic_ = trimmed;
    shutdownRosPublisher();
    if (rosPublishingEnabled_) {
        ensureRosPublisher();
    }
}

void AirSimViewController::setRosFrameId(const QString& frameId) {
    rosFrameId_ = frameId.trimmed();
}

bool AirSimViewController::hasRosImageSubscribers() const {
    return (rosImagePublisher_ && rosImagePublisher_.getNumSubscribers() > 0)
        || (rosCameraInfoPublisher_ && rosCameraInfoPublisher_.getNumSubscribers() > 0)
        || (rosCameraPosePublisher_ && rosCameraPosePublisher_.getNumSubscribers() > 0);
}

void AirSimViewController::setRosImageSubscriptionTopic(const QString& topicName) {
    const QString trimmed = topicName.trimmed();
    if (rosImageSubscribeTopic_ == trimmed) {
        if (rosImageSubscriptionEnabled_) {
            ensureRosImageSubscriber();
        }
        return;
    }

    rosImageSubscribeTopic_ = trimmed;
    if (rosImageSubscriptionEnabled_) {
        shutdownRosImageSubscriber();
        ensureRosImageSubscriber();
    }
}

void AirSimViewController::setRosImageSubscriptionEnabled(bool value) {
    if (rosImageSubscriptionEnabled_ == value) {
        if (rosImageSubscriptionEnabled_) {
            ensureRosImageSubscriber();
        }
        return;
    }

    rosImageSubscriptionEnabled_ = value;
    if (rosImageSubscriptionEnabled_) {
        ensureRosImageSubscriber();
    } else {
        shutdownRosImageSubscriber();
    }
}

void AirSimViewController::setExternalPoseControlEnabled(bool value) {
    if (externalPoseControlEnabled_ == value) {
        return;
    }

    externalPoseControlEnabled_ = value;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.externalPoseControlEnabled = externalPoseControlEnabled_;
        ++sharedState_.externalPoseControlRevision;
    }

    if (externalPoseControlEnabled_) {
        startExternalPoseControlWorker();
    } else {
        stopExternalPoseControlWorker();
    }
}

void AirSimViewController::start() {
#ifdef AIRSIM_GUI_ENABLE_AIRSIM
    if (workerRunning_.load()) {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.host = host_;
        sharedState_.port = port_;
        sharedState_.cameraName = cameraName_;
        sharedState_.vehicleName = vehicleName_;
        sharedState_.followVehicle = followVehicle_;
        sharedState_.anchoredCamera = anchoredCamera_;
        sharedState_.anchorX = anchorPoint_.x;
        sharedState_.anchorY = anchorPoint_.y;
        sharedState_.anchorZ = anchorPoint_.z;
        sharedState_.fps = framesPerSecond_;
        sharedState_.depthFetchEnabled = depthFetchEnabled_;
        return;
    }
    workerStop_.store(false);
    workerRunning_.store(true);
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.host = host_;
        sharedState_.port = port_;
        sharedState_.cameraName = cameraName_;
        sharedState_.vehicleName = vehicleName_;
        sharedState_.followVehicle = followVehicle_;
        sharedState_.anchoredCamera = anchoredCamera_;
        sharedState_.anchorX = anchorPoint_.x;
        sharedState_.anchorY = anchorPoint_.y;
        sharedState_.anchorZ = anchorPoint_.z;
        sharedState_.fps = framesPerSecond_;
        sharedState_.depthFetchEnabled = depthFetchEnabled_;
        sharedState_.configDirty = true;
    }
    workerThread_ = std::thread(&AirSimViewController::workerLoop, this);
    setStatusText(QStringLiteral("Starting AirSim C++ RPC live view... (timeout %1 ms)").arg(static_cast<int>(kAirSimRpcTimeoutSec * 1000.0f)));
    emit runningChanged();
#else
    setStatusText(formatAirSimDisabledMessage());
#endif
}

void AirSimViewController::stop() {
    shutdownRosImageSubscriber();
    if (!workerRunning_.load() && !workerThread_.joinable()) {
        return;
    }
    workerStop_.store(true);
    if (workerThread_.joinable()) {
        workerThread_.join();
    }
    const bool runningBefore = workerRunning_.exchange(false);
    if (connected_) {
        connected_ = false;
        emit connectedChanged();
    }
    if (runningBefore) {
        emit runningChanged();
    }
}

void AirSimViewController::restart() {
    stop();
    start();
}

void AirSimViewController::pause() {
    stop();
}

void AirSimViewController::resume() {
    start();
}

void AirSimViewController::startExternalPoseControlWorker() {
#ifdef AIRSIM_GUI_ENABLE_AIRSIM
    if (externalPoseControlRunning_.load()) {
        return;
    }
    externalPoseControlStop_.store(false);
    externalPoseControlRunning_.store(true);
    externalPoseControlThread_ = std::thread(&AirSimViewController::externalPoseControlLoop, this);
#else
    setStatusText(formatAirSimDisabledMessage());
#endif
}

void AirSimViewController::stopExternalPoseControlWorker() {
    externalPoseControlStop_.store(true);
    if (externalPoseControlThread_.joinable()) {
        externalPoseControlThread_.join();
    }
    externalPoseControlRunning_.store(false);
}

void AirSimViewController::externalPoseControlLoop() {
#ifdef AIRSIM_GUI_ENABLE_AIRSIM
    using msr::airlib::MultirotorRpcLibClient;
    using msr::airlib::Pose;
    using msr::airlib::Vector3r;

    std::unique_ptr<MultirotorRpcLibClient> client;
    QString lastHost;
    int lastPort = -1;
    int lastAppliedRevision = -1;
    QString lastError;

    while (!externalPoseControlStop_.load()) {
        SharedState snapshot;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            snapshot = sharedState_;
        }

        if (!snapshot.externalPoseControlEnabled) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        const bool endpointChanged = !client
            || snapshot.host != lastHost
            || snapshot.port != lastPort;
        if (!endpointChanged && snapshot.externalPoseControlRevision == lastAppliedRevision) {
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            continue;
        }

        try {
            if (endpointChanged) {
                std::unique_ptr<MultirotorRpcLibClient> nextClient(
                    new MultirotorRpcLibClient(snapshot.host.toStdString(),
                                               static_cast<uint16_t>(snapshot.port),
                                               kAirSimRpcTimeoutSec));
                {
                    std::lock_guard<std::mutex> requestLock(g_airSimRpcRequestMutex);
                    if (!nextClient->ping()) {
                        throw std::runtime_error(QStringLiteral("AirSim did not respond within %1 ms.")
                                                     .arg(static_cast<int>(kAirSimRpcTimeoutSec * 1000.0f))
                                                     .toStdString());
                    }
                }
                client = std::move(nextClient);
                lastHost = snapshot.host;
                lastPort = snapshot.port;
                lastAppliedRevision = -1;
            }

            const QString cameraName = snapshot.cameraName.trimmed();
            if (cameraName.isEmpty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            const Vector3r position(static_cast<float>(snapshot.anchorX),
                                    static_cast<float>(snapshot.anchorY),
                                    static_cast<float>(snapshot.anchorZ));
            const auto rotation = makeAnchoredCameraQuaternion(snapshot.yawDeg, snapshot.pitchDeg, snapshot.rollDeg);
            {
                std::lock_guard<std::mutex> requestLock(g_airSimRpcRequestMutex);
                client->simSetCameraPose(cameraName.toStdString(),
                                         Pose(position, rotation),
                                         std::string(),
                                         true);
            }
            lastAppliedRevision = snapshot.externalPoseControlRevision;
            lastError.clear();
        }
        catch (const std::exception& e) {
            const QString error = QStringLiteral("Station camera pose control waiting for AirSim: %1").arg(QString::fromStdString(e.what()));
            if (error != lastError) {
                lastError = error;
                QMetaObject::invokeMethod(this, [this, error]() {
                    emit helperMessage(error);
                }, Qt::QueuedConnection);
            }
            client.reset();
            lastAppliedRevision = -1;
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
    }

    externalPoseControlRunning_.store(false);
#endif
}

void AirSimViewController::orbitCamera(double dx, double dy, int buttons, bool shiftHeld) {
    bool changed = false;
    double yaw = yawDeg_;
    double pitch = pitchDeg_;
    double distance = distance_;
    double focusX = focusX_;
    double focusY = focusY_;
    double focusZ = focusZ_;

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (sharedState_.anchoredCamera) {
            Q_UNUSED(buttons);
            Q_UNUSED(shiftHeld);
            sharedState_.yawDeg -= dx * 0.25;
            sharedState_.pitchDeg = clampValue(sharedState_.pitchDeg - dy * 0.2, kMinPitch, kMaxPitch);
            sharedState_.focusX = sharedState_.anchorX;
            sharedState_.focusY = sharedState_.anchorY;
            sharedState_.focusZ = sharedState_.anchorZ;
            sharedState_.focusInitialized = true;
            sharedState_.followVehicle = false;
            if (sharedState_.externalPoseControlEnabled) {
                ++sharedState_.externalPoseControlRevision;
            }
            changed = true;
            yaw = sharedState_.yawDeg;
            pitch = sharedState_.pitchDeg;
            distance = sharedState_.distance;
            focusX = sharedState_.focusX;
            focusY = sharedState_.focusY;
            focusZ = sharedState_.focusZ;
        } else if (shiftHeld || (buttons & Qt::RightButton)) {
        const double yaw = degToRad(sharedState_.yawDeg);
        const double pitch = degToRad(sharedState_.pitchDeg);
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);

        const double forwardX = cp * cy;
        const double forwardY = cp * sy;
        const double forwardZ = sp;

        constexpr double worldUpX = 0.0;
        constexpr double worldUpY = 0.0;
        constexpr double worldUpZ = 1.0;
        double rightX = worldUpY * forwardZ - worldUpZ * forwardY;
        double rightY = worldUpZ * forwardX - worldUpX * forwardZ;
        double rightZ = worldUpX * forwardY - worldUpY * forwardX;
        double rightNorm = std::sqrt(rightX * rightX + rightY * rightY + rightZ * rightZ);
        if (rightNorm < 1e-6) {
            rightX = -std::sin(yaw);
            rightY = std::cos(yaw);
            rightZ = 0.0;
            rightNorm = std::sqrt(rightX * rightX + rightY * rightY);
        }
        rightX /= rightNorm;
        rightY /= rightNorm;
        rightZ /= rightNorm;

        double upX = rightY * forwardZ - rightZ * forwardY;
        double upY = rightZ * forwardX - rightX * forwardZ;
        double upZ = rightX * forwardY - rightY * forwardX;
        const double upNorm = std::sqrt(upX * upX + upY * upY + upZ * upZ);
        if (upNorm > 1e-6) {
            upX /= upNorm;
            upY /= upNorm;
            upZ /= upNorm;
        }

        const double scale = std::max(1.0, sharedState_.distance) * 0.0035;
        sharedState_.focusX += (-dx * rightX + dy * upX) * scale;
        sharedState_.focusY += (-dx * rightY + dy * upY) * scale;
        sharedState_.focusZ += (-dx * rightZ + dy * upZ) * scale;
        sharedState_.focusInitialized = true;
        sharedState_.followVehicle = false;
            changed = true;
        } else {
        sharedState_.yawDeg += dx * 0.25;
        sharedState_.pitchDeg = clampValue(sharedState_.pitchDeg - dy * 0.2, kMinPitch, kMaxPitch);
            changed = true;
        }

        if (changed) {
            yaw = sharedState_.yawDeg;
            pitch = sharedState_.pitchDeg;
            distance = sharedState_.distance;
            focusX = sharedState_.focusX;
            focusY = sharedState_.focusY;
            focusZ = sharedState_.focusZ;
        }
    }

    if (changed) {
        yawDeg_ = yaw;
        pitchDeg_ = pitch;
        distance_ = distance;
        focusX_ = focusX;
        focusY_ = focusY;
        focusZ_ = focusZ;
        emit cameraStateChanged();
    }
}

void AirSimViewController::zoomCamera(double wheelDelta) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (sharedState_.anchoredCamera) {
        Q_UNUSED(wheelDelta);
        return;
    }
    const double factor = wheelDelta > 0.0 ? 0.9 : 1.1;
    sharedState_.distance = clampValue(sharedState_.distance * factor, kMinDistance, kMaxDistance);
}

void AirSimViewController::resetCamera() {
    double yaw = 0.0;
    double pitch = -18.0;
    double roll = 0.0;
    double distance = 45.0;
    double focusX = 0.0;
    double focusY = 0.0;
    double focusZ = 0.0;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sharedState_.yawDeg = 0.0;
        sharedState_.pitchDeg = sharedState_.anchoredCamera ? 0.0 : -18.0;
        sharedState_.rollDeg = 0.0;
        sharedState_.distance = 45.0;
        if (sharedState_.anchoredCamera) {
            sharedState_.focusX = sharedState_.anchorX;
            sharedState_.focusY = sharedState_.anchorY;
            sharedState_.focusZ = sharedState_.anchorZ;
            sharedState_.focusInitialized = true;
            sharedState_.followVehicle = false;
            if (sharedState_.externalPoseControlEnabled) {
                ++sharedState_.externalPoseControlRevision;
            }
        } else {
            sharedState_.focusX = 0.0;
            sharedState_.focusY = 0.0;
            sharedState_.focusZ = 0.0;
            sharedState_.focusInitialized = false;
        }
        sharedState_.resetRequested = true;
        yaw = sharedState_.yawDeg;
        pitch = sharedState_.pitchDeg;
        roll = sharedState_.rollDeg;
        distance = sharedState_.distance;
        focusX = sharedState_.focusX;
        focusY = sharedState_.focusY;
        focusZ = sharedState_.focusZ;
    }

    yawDeg_ = yaw;
    pitchDeg_ = pitch;
    rollDeg_ = roll;
    distance_ = distance;
    focusX_ = focusX;
    focusY_ = focusY;
    focusZ_ = focusZ;
    emit cameraStateChanged();
}

void AirSimViewController::nudgePan(double dx, double dy) {
    orbitCamera(dx, dy, Qt::RightButton, true);
}

void AirSimViewController::requestStationSelection(int index) {
    emit stationSelectionRequested(index);
}

QString AirSimViewController::stationInfoText(int index) const {
    if (index < 0 || index >= static_cast<int>(baseStations_.size())) {
        return QString();
    }

    const BaseStation& station = baseStations_[static_cast<size_t>(index)];
    int count = 0;
    for (const auto& path : beamPaths_) {
        if (path.txId == station.name || path.txId == station.id) {
            ++count;
        }
    }
    return QStringLiteral("Name: %1\nID: %2\nPosition: %3\nActive paths: %4")
        .arg(station.name, station.id, vecToString(station.position))
        .arg(count);
}

double AirSimViewController::displayBrightness() const {
    std::lock_guard<std::mutex> lock(frameMutex_);
    return displayBrightness_;
}

QImage AirSimViewController::latestFrameCopy() const {
    QImage frame;
    double brightness = 1.0;
    {
        std::lock_guard<std::mutex> lock(frameMutex_);
        frame = latestFrame_;
        brightness = displayBrightness_;
    }
    return adjustDisplayBrightness(frame, brightness);
}

void AirSimViewController::setDisplayBrightness(double value) {
    const double clamped = std::max(0.25, std::min(3.0, value));
    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(frameMutex_);
        if (qFuzzyCompare(displayBrightness_, clamped)) {
            return;
        }
        displayBrightness_ = clamped;
        if (!latestFrame_.isNull()) {
            ++frameToken_;
        }
        changed = true;
    }
    if (changed) {
        emit displayBrightnessChanged();
        emit frameTokenChanged();
    }
}

void AirSimViewController::setStatusText(const QString& text) {
    if (statusText_ == text) return;
    statusText_ = text;
    emit statusTextChanged();
    emit helperMessage(text);
}

QVariantMap AirSimViewController::worldPointFromNormalized(double nx, double ny) const {
    QVariantMap out;
    out.insert(QStringLiteral("valid"), false);

    Vec3 cameraPos;
    Vec3 right;
    Vec3 up;
    Vec3 forward;
    double hFovDeg = 90.0;
    int width = 0;
    int height = 0;
    std::vector<float> depth;
    {
        std::lock_guard<std::mutex> lock(geometryMutex_);
        cameraPos = cameraPosition_;
        right = cameraRight_;
        up = cameraUp_;
        forward = cameraForward_;
        hFovDeg = horizontalFovDeg_;
        width = depthWidth_;
        height = depthHeight_;
        depth = depthValues_;
    }

    if (width <= 0 || height <= 0 || depth.size() != static_cast<size_t>(width * height)) {
        return out;
    }

    const double clampedNx = clampValue(nx, 0.0, 1.0);
    const double clampedNy = clampValue(ny, 0.0, 1.0);
    const int col = std::max(0, std::min(width - 1, static_cast<int>(std::round(clampedNx * (width - 1)))));
    const int row = std::max(0, std::min(height - 1, static_cast<int>(std::round(clampedNy * (height - 1)))));
    const float depthValue = sampleDepthBilinear(depth, width, height, clampedNx, clampedNy);
    if (!isValidDepthSample(depthValue)) {
        return out;
    }

    const double aspect = static_cast<double>(width) / static_cast<double>(height);
    const double tanHalfHFov = safeAtanTanHalf(hFovDeg);
    const double tanHalfVFov = tanHalfHFov / std::max(1e-6, aspect);
    const double xNdc = 2.0 * clampedNx - 1.0;
    const double yNdc = 1.0 - 2.0 * clampedNy;
    const Vec3 rayDirection = normalizeVec(addVec(addVec(forward, scaleVec(right, xNdc * tanHalfHFov)),
                                                  scaleVec(up, yNdc * tanHalfVFov)),
                                           forward);
    const double perspectiveDepth = static_cast<double>(depthValue);
    const Vec3 world = addVec(cameraPos, scaleVec(rayDirection, perspectiveDepth));
    out.insert(QStringLiteral("valid"), true);
    const Vec3 guiWorld = airSimToGuiPoint(world);
    out.insert(QStringLiteral("x"), guiWorld.x);
    out.insert(QStringLiteral("y"), guiWorld.y);
    out.insert(QStringLiteral("z"), guiWorld.z);
    out.insert(QStringLiteral("airsimX"), world.x);
    out.insert(QStringLiteral("airsimY"), world.y);
    out.insert(QStringLiteral("airsimZ"), world.z);
    out.insert(QStringLiteral("depth"), perspectiveDepth);
    out.insert(QStringLiteral("row"), row);
    out.insert(QStringLiteral("col"), col);
    return out;
}

double AirSimViewController::depthAtNormalized(double nx, double ny) const {
    const QVariantMap point = worldPointFromNormalized(nx, ny);
    return point.value(QStringLiteral("valid")).toBool()
        ? point.value(QStringLiteral("depth")).toDouble()
        : -1.0;
}

QString AirSimViewController::anchorPointText() const {
    return formatVecForUi(anchorPoint_);
}

QString AirSimViewController::cameraPositionText() const {
    Vec3 airsimPos;
    bool geometryValid = false;
    {
        std::lock_guard<std::mutex> lock(geometryMutex_);
        airsimPos = cameraPosition_;
        geometryValid = cameraGeometryValid_;
    }

    const Vec3 guiPos = geometryValid ? airSimToGuiPoint(airsimPos) : anchorPoint_;
    const QString guiPrefix = geometryValid ? QStringLiteral("Live GUI") : QStringLiteral("Anchor GUI");
    const QString airsimText = geometryValid ? formatVecForUi(airsimPos) : QStringLiteral("(n/a, n/a, n/a)");
    return QStringLiteral("%1: %2\nLive AirSim NED: %3")
        .arg(guiPrefix, formatVecForUi(guiPos), airsimText);
}

bool AirSimViewController::projectWorldPoint(const Vec3& point, int frameWidth, int frameHeight, QPointF* outNormalized) const {
    if (!outNormalized || frameWidth <= 0 || frameHeight <= 0) {
        return false;
    }

    Vec3 cameraPos;
    Vec3 right;
    Vec3 up;
    Vec3 forward;
    double hFovDeg = 90.0;
    bool geometryValid = false;
    {
        std::lock_guard<std::mutex> lock(geometryMutex_);
        cameraPos = cameraPosition_;
        right = cameraRight_;
        up = cameraUp_;
        forward = cameraForward_;
        hFovDeg = horizontalFovDeg_;
        geometryValid = cameraGeometryValid_;
    }
    if (!geometryValid) {
        return false;
    }

    const Vec3 airsimPoint = guiToAirSimPoint(point);
    const Vec3 delta = subtractVec(airsimPoint, cameraPos);
    const double xCam = dotVec(delta, right);
    const double yCam = dotVec(delta, up);
    const double zCam = dotVec(delta, forward);
    if (zCam <= kProjectionNear) {
        return false;
    }

    const double aspect = static_cast<double>(frameWidth) / static_cast<double>(frameHeight);
    const double tanHalfHFov = safeAtanTanHalf(hFovDeg);
    const double tanHalfVFov = tanHalfHFov / std::max(1e-6, aspect);
    if (std::abs(tanHalfHFov) < 1e-6 || std::abs(tanHalfVFov) < 1e-6) {
        return false;
    }

    const double xNdc = xCam / (zCam * tanHalfHFov);
    const double yNdc = yCam / (zCam * tanHalfVFov);
    const double u = 0.5 * (xNdc + 1.0);
    const double v = 0.5 * (1.0 - yNdc);
    outNormalized->setX(u);
    outNormalized->setY(v);
    return std::isfinite(u) && std::isfinite(v);
}

void AirSimViewController::rebuildOverlayData() {
    int frameWidth = 0;
    int frameHeight = 0;
    {
        std::lock_guard<std::mutex> lock(geometryMutex_);
        frameWidth = sceneWidth_;
        frameHeight = sceneHeight_;
    }
    if (frameWidth <= 0 || frameHeight <= 0) {
        const QImage image = latestFrameCopy();
        frameWidth = image.width();
        frameHeight = image.height();
    }

    QVariantList stationOverlays;
    if (baseStationsVisible_) {
        for (size_t i = 0; i < baseStations_.size(); ++i) {
            const BaseStation& station = baseStations_[i];
            QPointF projected;
            const bool projectedOk = projectWorldPoint(station.position, frameWidth, frameHeight, &projected);
            const bool insideFrame = projectedOk
                && projected.x() >= 0.0 && projected.x() <= 1.0
                && projected.y() >= 0.0 && projected.y() <= 1.0;

            QVariantMap item;
            item.insert(QStringLiteral("index"), static_cast<int>(i));
            item.insert(QStringLiteral("id"), station.id);
            item.insert(QStringLiteral("name"), station.name);
            item.insert(QStringLiteral("x"), projected.x());
            item.insert(QStringLiteral("y"), projected.y());
            item.insert(QStringLiteral("visible"), insideFrame);
            item.insert(QStringLiteral("selected"), static_cast<int>(i) == selectedBaseStationIndex_);
            item.insert(QStringLiteral("worldX"), station.position.x);
            item.insert(QStringLiteral("worldY"), station.position.y);
            item.insert(QStringLiteral("worldZ"), station.position.z);
            item.insert(QStringLiteral("colorR"), station.color.x);
            item.insert(QStringLiteral("colorG"), station.color.y);
            item.insert(QStringLiteral("colorB"), station.color.z);
            stationOverlays.push_back(item);
        }
    }

    QVariantList pathOverlays;
    QVariantMap droneOverlay;
    droneOverlay.insert(QStringLiteral("visible"), false);
    if (isGuiPreviewCameraName(cameraName_)) {
        auto appendProjectedPathOverlay = [this, frameWidth, frameHeight, &pathOverlays](const BeamPath& path,
                                                                                          const Vec3* overrideColor = nullptr,
                                                                                          double overrideOpacity = -1.0,
                                                                                          double overrideWidth = -1.0) {
            QVariantList segments;
            std::vector<QPointF> currentSegment;
            for (const Vec3& worldPoint : path.points) {
                QPointF projected;
                const bool ok = projectWorldPoint(worldPoint, frameWidth, frameHeight, &projected)
                    && projected.x() >= 0.0 && projected.x() <= 1.0
                    && projected.y() >= 0.0 && projected.y() <= 1.0;
                if (ok) {
                    currentSegment.push_back(projected);
                } else {
                    if (currentSegment.size() >= 2) {
                        segments.push_back(makeSegmentVariant(currentSegment));
                    }
                    currentSegment.clear();
                }
            }
            if (currentSegment.size() >= 2) {
                segments.push_back(makeSegmentVariant(currentSegment));
            }
            if (segments.isEmpty()) {
                return;
            }

            const bool isLos = path.kind.compare(QStringLiteral("los"), Qt::CaseInsensitive) == 0;
            const bool isTrajectory = path.kind.compare(QStringLiteral("trajectory"), Qt::CaseInsensitive) == 0;
            const Vec3 color = overrideColor ? *overrideColor : (isTrajectory ? Vec3{1.0, 0.95, 0.35} : colorFromPowerDb(path.powerDb));
            const double opacity = overrideOpacity >= 0.0 ? overrideOpacity : (isLos ? 0.95 : (isTrajectory ? 0.88 : 0.70));
            const double width = overrideWidth >= 0.0 ? overrideWidth : (isLos ? 3.0 : (isTrajectory ? 3.0 : 2.0));

            QVariantMap item;
            item.insert(QStringLiteral("id"), path.id);
            item.insert(QStringLiteral("txId"), path.txId);
            item.insert(QStringLiteral("rxId"), path.rxId);
            item.insert(QStringLiteral("segments"), segments);
            item.insert(QStringLiteral("powerDb"), path.powerDb);
            item.insert(QStringLiteral("delayNs"), path.delayNs);
            item.insert(QStringLiteral("kind"), path.kind);
            item.insert(QStringLiteral("colorR"), color.x);
            item.insert(QStringLiteral("colorG"), color.y);
            item.insert(QStringLiteral("colorB"), color.z);
            item.insert(QStringLiteral("opacity"), opacity);
            item.insert(QStringLiteral("width"), width);
            pathOverlays.push_back(item);
        };

        bool hasTrajectoryPath = false;
        for (const BeamPath& path : beamPaths_) {
            const bool isTrajectory = path.kind.compare(QStringLiteral("trajectory"), Qt::CaseInsensitive) == 0;
            if (isTrajectory) {
                hasTrajectoryPath = true;
                if (!trajectoryVisible_) {
                    continue;
                }
            } else if (!pathsVisible_) {
                continue;
            }
            appendProjectedPathOverlay(path);
        }

        if (trajectoryVisible_ && !hasTrajectoryPath && droneState_.history.size() >= 2) {
            BeamPath trajectoryPath;
            trajectoryPath.id = QStringLiteral("drone_history_overlay");
            trajectoryPath.txId = QStringLiteral("trajectory");
            trajectoryPath.rxId = droneState_.id;
            trajectoryPath.kind = QStringLiteral("trajectory");
            trajectoryPath.powerDb = -50.0;
            trajectoryPath.delayNs = 0.0;
            trajectoryPath.points = droneState_.history;
            appendProjectedPathOverlay(trajectoryPath);
        }

        if (droneVisible_ && droneState_.valid) {
            QPointF projected;
            const bool ok = projectWorldPoint(droneState_.position, frameWidth, frameHeight, &projected)
                && projected.x() >= 0.0 && projected.x() <= 1.0
                && projected.y() >= 0.0 && projected.y() <= 1.0;
            if (ok) {
                droneOverlay.insert(QStringLiteral("visible"), true);
                droneOverlay.insert(QStringLiteral("x"), projected.x());
                droneOverlay.insert(QStringLiteral("y"), projected.y());
                droneOverlay.insert(QStringLiteral("name"), droneState_.id);
                droneOverlay.insert(QStringLiteral("worldX"), droneState_.position.x);
                droneOverlay.insert(QStringLiteral("worldY"), droneState_.position.y);
                droneOverlay.insert(QStringLiteral("worldZ"), droneState_.position.z);
            }
        }
    }

    stationOverlays_ = stationOverlays;
    pathOverlays_ = pathOverlays;
    droneOverlay_ = droneOverlay;
    emit overlayDataChanged();
}

void AirSimViewController::applyFrameUpdate(const QImage& image,
                                            bool connected,
                                            double yawDeg,
                                            double pitchDeg,
                                            double distance,
                                            double focusX,
                                            double focusY,
                                            double focusZ,
                                            const Vec3& cameraPos,
                                            const Vec3& cameraRight,
                                            const Vec3& cameraUp,
                                            const Vec3& cameraForward,
                                            double horizontalFovDeg,
                                            int sceneWidth,
                                            int sceneHeight,
                                            int depthWidth,
                                            int depthHeight,
                                            const std::vector<float>& depthValues,
                                            const QString& message) {
    if (!image.isNull()) {
        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            latestFrame_ = image;
            ++frameToken_;
        }
        emit frameTokenChanged();
    }
    if (connected_ != connected) {
        connected_ = connected;
        emit connectedChanged();
    }
    const bool cameraChanged = !qFuzzyCompare(yawDeg_ + 1.0, yawDeg + 1.0)
        || !qFuzzyCompare(pitchDeg_ + 1.0, pitchDeg + 1.0)
        || !qFuzzyCompare(distance_ + 1.0, distance + 1.0)
        || !qFuzzyCompare(focusX_ + 1.0, focusX + 1.0)
        || !qFuzzyCompare(focusY_ + 1.0, focusY + 1.0)
        || !qFuzzyCompare(focusZ_ + 1.0, focusZ + 1.0);
    yawDeg_ = yawDeg;
    pitchDeg_ = pitchDeg;
    distance_ = distance;
    focusX_ = focusX;
    focusY_ = focusY;
    focusZ_ = focusZ;
    if (cameraChanged) {
        emit cameraStateChanged();
    }

    {
        std::lock_guard<std::mutex> lock(geometryMutex_);
        cameraPosition_ = cameraPos;
        cameraRight_ = cameraRight;
        cameraUp_ = cameraUp;
        cameraForward_ = cameraForward;
        horizontalFovDeg_ = horizontalFovDeg;
        sceneWidth_ = sceneWidth;
        sceneHeight_ = sceneHeight;
        depthWidth_ = depthWidth;
        depthHeight_ = depthHeight;
        depthValues_ = depthValues;
        depthAvailable_ = (depthWidth_ > 0)
            && (depthHeight_ > 0)
            && (depthValues_.size() == static_cast<size_t>(depthWidth_ * depthHeight_));
        cameraGeometryValid_ = true;
    }
    emit cameraPositionChanged();

    rebuildOverlayData();
    publishRosFrame(image);
    setStatusText(message);
}

void AirSimViewController::ensureRosImageSubscriber() {
    if (!rosImageSubscriptionEnabled_ || rosImageSubscribeTopic_.isEmpty() || !ros::isInitialized()) {
        return;
    }

    ros::start();
    if (!ros::master::check()) {
        shutdownRosImageSubscriber();
        return;
    }

    if (rosImageSubscriptionActive_ && rosImageSubscriber_) {
        const auto now = std::chrono::steady_clock::now();
        const bool startupTimedOut = rosImageLastFrameTime_ == std::chrono::steady_clock::time_point{}
            && rosImageSubscribeStartTime_ != std::chrono::steady_clock::time_point{}
            && now - rosImageSubscribeStartTime_ > kRosImageSubscriptionStaleTimeout;
        const bool streamStale = rosImageLastFrameTime_ != std::chrono::steady_clock::time_point{}
            && now - rosImageLastFrameTime_ > kRosImageSubscriptionStaleTimeout;
        if (!startupTimedOut && !streamStale) {
            return;
        }
        shutdownRosImageSubscriber();
    }

    rosImageSubscribeNodeHandle_ = std::make_unique<ros::NodeHandle>();
    rosImageSubscriber_ = rosImageSubscribeNodeHandle_->subscribe<sensor_msgs::Image>(
        rosImageSubscribeTopic_.toStdString(),
        1,
        &AirSimViewController::rosImageCallback,
        this,
        ros::TransportHints().tcpNoDelay());
    rosImageSubscribeSpinner_ = std::make_unique<ros::AsyncSpinner>(1);
    rosImageSubscribeSpinner_->start();

    rosImageSubscriptionActive_ = true;
    rosImageSubscribeStartTime_ = std::chrono::steady_clock::now();
    rosImageLastFrameTime_ = {};
    setStatusText(QStringLiteral("Waiting for ROS image topic %1...").arg(rosImageSubscribeTopic_));
    emit runningChanged();
}

void AirSimViewController::shutdownRosImageSubscriber() {
    const bool wasActive = rosImageSubscriptionActive_;
    rosImageSubscriber_.shutdown();
    if (rosImageSubscribeSpinner_) {
        rosImageSubscribeSpinner_->stop();
        rosImageSubscribeSpinner_.reset();
    }
    rosImageSubscribeNodeHandle_.reset();
    rosImageSubscriptionActive_ = false;
    rosImageSubscribeStartTime_ = {};
    rosImageLastFrameTime_ = {};

    if (!workerRunning_.load() && connected_) {
        connected_ = false;
        emit connectedChanged();
    }
    if (wasActive) {
        setStatusText(QStringLiteral("ROS image subscription stopped."));
        emit runningChanged();
    }
}

void AirSimViewController::rosImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    const QImage decoded = imageFromRosMessage(*msg);
    if (decoded.isNull()) {
        const QString encoding = QString::fromStdString(msg->encoding);
        QMetaObject::invokeMethod(this,
            [this, encoding]() {
                setStatusText(QStringLiteral("Unsupported ROS image encoding: %1").arg(encoding));
            },
            Qt::QueuedConnection);
        return;
    }

    const QString topic = rosImageSubscribeTopic_;
    QMetaObject::invokeMethod(this,
        [this, decoded, topic]() {
            if (!rosImageSubscriptionEnabled_ || topic != rosImageSubscribeTopic_) {
                return;
            }
            const QImage frameImage = adjustDisplayBrightness(decoded, displayBrightness_);
            {
                std::lock_guard<std::mutex> lock(frameMutex_);
                latestFrame_ = frameImage;
                ++frameToken_;
            }
            {
                std::lock_guard<std::mutex> lock(geometryMutex_);
                sceneWidth_ = frameImage.width();
                sceneHeight_ = frameImage.height();
                depthWidth_ = 0;
                depthHeight_ = 0;
                depthValues_.clear();
                depthAvailable_ = false;
                cameraGeometryValid_ = false;
            }
            rosImageSubscribeStartTime_ = {};
            rosImageLastFrameTime_ = std::chrono::steady_clock::now();
            if (!connected_) {
                connected_ = true;
                emit connectedChanged();
            }
            emit frameTokenChanged();
            emit overlayDataChanged();
            setStatusText(QStringLiteral("ROS image topic %1 · %2x%3")
                              .arg(topic)
                              .arg(frameImage.width())
                              .arg(frameImage.height()));
        },
        Qt::QueuedConnection);
}

void AirSimViewController::shutdownRosPublisher() {
    if (rosImagePublisher_) {
        rosImagePublisher_.shutdown();
    }
    if (rosCameraInfoPublisher_) {
        rosCameraInfoPublisher_.shutdown();
    }
    if (rosCameraPosePublisher_) {
        rosCameraPosePublisher_.shutdown();
    }
    rosNodeHandle_.reset();
}

void AirSimViewController::ensureRosPublisher() {
    if (!rosPublishingEnabled_ || rosImageTopic_.isEmpty() || !ros::isInitialized()) {
        return;
    }

    ros::start();

    if (!ros::master::check()) {
        shutdownRosPublisher();
        return;
    }

    if (!rosNodeHandle_) {
        rosNodeHandle_ = std::make_unique<ros::NodeHandle>();
    }
    if (!rosImagePublisher_) {
        rosImagePublisher_ = rosNodeHandle_->advertise<sensor_msgs::Image>(rosImageTopic_.toStdString(), 1);
    }
    if (!rosCameraInfoPublisher_) {
        rosCameraInfoPublisher_ = rosNodeHandle_->advertise<sensor_msgs::CameraInfo>(
            cameraInfoTopicForImageTopic(rosImageTopic_).toStdString(), 1);
    }
    if (!rosCameraPosePublisher_) {
        rosCameraPosePublisher_ = rosNodeHandle_->advertise<geometry_msgs::PoseStamped>(
            cameraPoseTopicForImageTopic(rosImageTopic_).toStdString(), 1);
    }
}

void AirSimViewController::publishRosFrame(const QImage& image) {
    if (!rosPublishingEnabled_ || rosImageTopic_.isEmpty() || image.isNull() || !ros::isInitialized()) {
        return;
    }

    ensureRosPublisher();
    if (!rosImagePublisher_) {
        return;
    }

    Vec3 cameraPos;
    Vec3 right;
    Vec3 up;
    Vec3 forward;
    double hFovDeg = 90.0;
    bool geometryValid = false;
    {
        std::lock_guard<std::mutex> lock(geometryMutex_);
        cameraPos = cameraPosition_;
        right = cameraRight_;
        up = cameraUp_;
        forward = cameraForward_;
        hFovDeg = horizontalFovDeg_;
        geometryValid = cameraGeometryValid_;
    }

    const ros::Time stamp = ros::Time::now();
    const std::string cameraFrameId = rosFrameId_.isEmpty() ? std::string("airsim_camera") : rosFrameId_.toStdString();
    const QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    sensor_msgs::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = cameraFrameId;
    msg.height = static_cast<uint32_t>(rgb.height());
    msg.width = static_cast<uint32_t>(rgb.width());
    msg.encoding = "rgb8";
    msg.is_bigendian = 0;
    msg.step = static_cast<uint32_t>(rgb.bytesPerLine());
    msg.data.resize(static_cast<size_t>(msg.step) * static_cast<size_t>(msg.height));
    std::memcpy(msg.data.data(), rgb.constBits(), msg.data.size());
    rosImagePublisher_.publish(msg);

    if (rosCameraInfoPublisher_) {
        sensor_msgs::CameraInfo info;
        info.header = msg.header;
        info.height = msg.height;
        info.width = msg.width;
        info.distortion_model = "plumb_bob";
        info.D.assign(5, 0.0);
        const double clampedFov = clampValue(hFovDeg, 20.0, 170.0);
        const double focalPx = static_cast<double>(msg.width) / (2.0 * std::tan(degToRad(clampedFov) * 0.5));
        const double cx = (static_cast<double>(msg.width) - 1.0) * 0.5;
        const double cy = (static_cast<double>(msg.height) - 1.0) * 0.5;
        info.K.fill(0.0);
        info.K[0] = focalPx;
        info.K[2] = cx;
        info.K[4] = focalPx;
        info.K[5] = cy;
        info.K[8] = 1.0;
        info.R.fill(0.0);
        info.R[0] = 1.0;
        info.R[4] = 1.0;
        info.R[8] = 1.0;
        info.P.fill(0.0);
        info.P[0] = focalPx;
        info.P[2] = cx;
        info.P[5] = focalPx;
        info.P[6] = cy;
        info.P[10] = 1.0;
        rosCameraInfoPublisher_.publish(info);
    }

    if (rosCameraPosePublisher_ && geometryValid) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = FrameTransformManager::airsimFrameId().toStdString();
        pose.pose.position.x = cameraPos.x;
        pose.pose.position.y = cameraPos.y;
        pose.pose.position.z = cameraPos.z;
        pose.pose.orientation = cameraQuaternionFromBasisNed(right, up, forward);
        rosCameraPosePublisher_.publish(pose);
    }
}

void AirSimViewController::workerLoop() {
#ifdef AIRSIM_GUI_ENABLE_AIRSIM
    using msr::airlib::ImageCaptureBase;
    using msr::airlib::MultirotorRpcLibClient;
    using msr::airlib::Pose;
    using msr::airlib::Vector3r;

    std::unique_ptr<MultirotorRpcLibClient> client;
    QString lastHost;
    int lastPort = -1;
    QString lastVehicle;
    QString lastCamera;
    int lastAppliedStationCameraPoseRevision = -1;
    bool cameraInfoCacheValid = false;
    double cachedHorizontalFovDeg = 90.0;
    Vec3 cachedCameraInfoPos;
    Basis3 cachedCameraInfoBasis;
    bool lastSetPoseValid = false;
    Vec3 lastSetCameraPos;
    Vec3 lastSetLookTarget;
    double lastSetRollDeg = 0.0;

    while (!workerStop_.load()) {
        const auto loopStart = std::chrono::steady_clock::now();
        SharedState snapshot;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            snapshot = sharedState_;
            sharedState_.resetRequested = false;
        }

        QString status = QStringLiteral("AirSim live view running");
        bool frameConnected = false;
        bool success = false;

        try {
            const bool needReconnect = !client
                || snapshot.configDirty
                || snapshot.host != lastHost
                || snapshot.port != lastPort
                || snapshot.vehicleName != lastVehicle
                || snapshot.cameraName != lastCamera;

            if (needReconnect) {
                std::unique_ptr<MultirotorRpcLibClient> nextClient(
                    new MultirotorRpcLibClient(snapshot.host.toStdString(),
                                               static_cast<uint16_t>(snapshot.port),
                                               kAirSimRpcTimeoutSec));
                {
                    std::lock_guard<std::mutex> requestLock(g_airSimRpcRequestMutex);
                    if (!nextClient->ping()) {
                        throw std::runtime_error(QStringLiteral("AirSim did not respond within %1 ms.")
                                                     .arg(static_cast<int>(kAirSimRpcTimeoutSec * 1000.0f))
                                                     .toStdString());
                    }
                    Q_UNUSED(nextClient->getServerVersion());
                }
                client = std::move(nextClient);
                lastHost = snapshot.host;
                lastPort = snapshot.port;
                lastVehicle = snapshot.vehicleName;
                lastCamera = snapshot.cameraName;
                lastAppliedStationCameraPoseRevision = -1;
                cameraInfoCacheValid = false;
                lastSetPoseValid = false;
                lastSetRollDeg = 0.0;
                status = QStringLiteral("Connected to AirSim RPC %1:%2").arg(snapshot.host).arg(snapshot.port);
                std::lock_guard<std::mutex> lock(stateMutex_);
                sharedState_.configDirty = false;
            }

            std::vector<StationCameraPoseRequest> stationPoseRequests;
            int stationPoseRevision = 0;
            {
                std::lock_guard<std::mutex> lock(stationCameraPoseMutex_);
                stationPoseRevision = stationCameraPoseRevision_;
                if (stationPoseRevision != lastAppliedStationCameraPoseRevision) {
                    stationPoseRequests = stationCameraPoseRequests_;
                }
            }
            if (stationPoseRevision != lastAppliedStationCameraPoseRevision) {
                {
                    std::lock_guard<std::mutex> requestLock(g_airSimRpcRequestMutex);
                    for (const StationCameraPoseRequest& request : stationPoseRequests) {
                        const Vector3r position(static_cast<float>(request.position.x),
                                                static_cast<float>(request.position.y),
                                                static_cast<float>(request.position.z));
                        const Vec3 direction = request.hasTarget
                            ? subtractVec(request.target, request.position)
                            : directionFromYawPitchNed(0.0, 0.0);
                        const auto rotation = makeAnchoredCameraQuaternionForDirection(direction);
                        client->simSetCameraPose(request.cameraName.toStdString(),
                                                 Pose(position, rotation),
                                                 std::string(),
                                                 true);
                    }
                }
                lastAppliedStationCameraPoseRevision = stationPoseRevision;
                if (!stationPoseRequests.empty()) {
                    QMetaObject::invokeMethod(this, [this, count = stationPoseRequests.size()]() {
                        emit helperMessage(QStringLiteral("Synced %1 base-station external camera poses from current 3D coordinate transform.").arg(count));
                    }, Qt::QueuedConnection);
                }
            }

            if (snapshot.anchoredCamera) {
                snapshot.focusX = snapshot.anchorX;
                snapshot.focusY = snapshot.anchorY;
                snapshot.focusZ = snapshot.anchorZ;
                snapshot.focusInitialized = true;
                std::lock_guard<std::mutex> lock(stateMutex_);
                sharedState_.focusX = snapshot.focusX;
                sharedState_.focusY = snapshot.focusY;
                sharedState_.focusZ = snapshot.focusZ;
                sharedState_.focusInitialized = true;
                sharedState_.followVehicle = false;
            } else if ((snapshot.followVehicle && !snapshot.vehicleName.isEmpty()) || !snapshot.focusInitialized) {
                if (!snapshot.vehicleName.isEmpty()) {
                    std::lock_guard<std::mutex> requestLock(g_airSimRpcRequestMutex);
                    const auto state = client->getMultirotorState(snapshot.vehicleName.toStdString());
                    const auto pos = state.getPosition();
                    snapshot.focusX = pos.x();
                    snapshot.focusY = pos.y();
                    snapshot.focusZ = pos.z();
                } else if (!snapshot.focusInitialized) {
                    snapshot.focusX = 0.0;
                    snapshot.focusY = 0.0;
                    snapshot.focusZ = 0.0;
                }
                snapshot.focusInitialized = true;
                std::lock_guard<std::mutex> lock(stateMutex_);
                sharedState_.focusX = snapshot.focusX;
                sharedState_.focusY = snapshot.focusY;
                sharedState_.focusZ = snapshot.focusZ;
                sharedState_.focusInitialized = true;
            }

            const Vec3 requestedCameraPos = snapshot.anchoredCamera
                ? makeVec3(snapshot.anchorX, snapshot.anchorY, snapshot.anchorZ)
                : computeCameraPosition(snapshot.focusX, snapshot.focusY, snapshot.focusZ,
                                        snapshot.yawDeg, snapshot.pitchDeg, snapshot.distance);
            const Vec3 requestedLookTarget = snapshot.anchoredCamera
                ? addVec(requestedCameraPos, scaleVec(directionFromYawPitchNed(snapshot.yawDeg, snapshot.pitchDeg), 10.0))
                : makeVec3(snapshot.focusX, snapshot.focusY, snapshot.focusZ);
            const auto cameraPos = Vector3r(static_cast<float>(requestedCameraPos.x),
                                            static_cast<float>(requestedCameraPos.y),
                                            static_cast<float>(requestedCameraPos.z));
            const auto target = Vector3r(static_cast<float>(requestedLookTarget.x),
                                         static_cast<float>(requestedLookTarget.y),
                                         static_cast<float>(requestedLookTarget.z));
            const auto cameraRot = snapshot.anchoredCamera
                ? makeAnchoredCameraQuaternion(snapshot.yawDeg, snapshot.pitchDeg, snapshot.rollDeg)
                : (std::abs(snapshot.rollDeg) > 1e-6
                    ? makeCameraQuaternionForDirectionAndRoll(subtractVec(requestedLookTarget, requestedCameraPos), snapshot.rollDeg)
                    : msr::airlib::VectorMath::lookAt(cameraPos, target));

            const std::string cameraNameStd = snapshot.cameraName.toStdString();
            std::vector<ImageCaptureBase::ImageResponse> responses;
            double actualHorizontalFovDeg = 90.0;
            Vec3 fallbackCameraPos = requestedCameraPos;
            Basis3 fallbackCameraBasis;
            {
                std::lock_guard<std::mutex> requestLock(g_airSimRpcRequestMutex);
                const bool poseChanged = !lastSetPoseValid
                    || normVec(subtractVec(requestedCameraPos, lastSetCameraPos)) > 1e-4
                    || normVec(subtractVec(requestedLookTarget, lastSetLookTarget)) > 1e-4
                    || std::abs(snapshot.rollDeg - lastSetRollDeg) > 1e-4;
                if (poseChanged) {
                    client->simSetCameraPose(cameraNameStd, Pose(cameraPos, cameraRot), std::string(), true);
                    lastSetCameraPos = requestedCameraPos;
                    lastSetLookTarget = requestedLookTarget;
                    lastSetRollDeg = snapshot.rollDeg;
                    lastSetPoseValid = true;
                }
                if (!cameraInfoCacheValid) {
                    const auto cameraInfo = client->simGetCameraInfo(cameraNameStd, std::string(), true);
                    cachedHorizontalFovDeg = static_cast<double>(cameraInfo.fov);
                    cachedCameraInfoPos = toVec3(cameraInfo.pose.position);
                    cachedCameraInfoBasis = basisFromQuaternion(cameraInfo.pose.orientation);
                    cameraInfoCacheValid = true;
                }
                actualHorizontalFovDeg = cachedHorizontalFovDeg;
                fallbackCameraPos = cachedCameraInfoPos;
                fallbackCameraBasis = cachedCameraInfoBasis;

                std::vector<ImageCaptureBase::ImageRequest> requests;
                requests.emplace_back(cameraNameStd, ImageCaptureBase::ImageType::Scene, false, false);
                if (snapshot.depthFetchEnabled) {
                    requests.emplace_back(cameraNameStd, ImageCaptureBase::ImageType::DepthPerspective, true, false);
                }
                responses = client->simGetImages(requests, std::string(), true);
            }

            if (!responses.empty()) {
                const ImageCaptureBase::ImageResponse& sceneResponse = responses.front();
                const Vec3 actualCameraPos = sceneResponse.width > 0
                    ? toVec3(sceneResponse.camera_position)
                    : fallbackCameraPos;
                const Basis3 cameraBasis = sceneResponse.width > 0
                    ? basisFromQuaternion(sceneResponse.camera_orientation)
                    : fallbackCameraBasis;
                QImage image;

                const int width = static_cast<int>(sceneResponse.width);
                const int height = static_cast<int>(sceneResponse.height);
                const int channels = (width > 0 && height > 0)
                    ? static_cast<int>(sceneResponse.image_data_uint8.size()) / (width * height)
                    : 0;

                if (width > 0 && height > 0 && channels >= 3) {
                    if (channels == 3) {
                        QImage tmp(sceneResponse.image_data_uint8.data(), width, height, width * 3, QImage::Format_RGB888);
                        image = tmp.rgbSwapped().copy();
                    } else if (channels >= 4) {
                        QImage tmp(sceneResponse.image_data_uint8.data(), width, height, width * 4, QImage::Format_ARGB32);
                        image = tmp.convertToFormat(QImage::Format_RGB32).copy();
                    }
                }

                if (image.isNull()) {
                    const QByteArray bytes(reinterpret_cast<const char*>(sceneResponse.image_data_uint8.data()),
                                           static_cast<int>(sceneResponse.image_data_uint8.size()));
                    image.loadFromData(bytes);
                }

                std::vector<float> depthValues;
                int depthWidth = 0;
                int depthHeight = 0;
                int rawDepthWidth = 0;
                int rawDepthHeight = 0;
                if (responses.size() >= 2) {
                    const ImageCaptureBase::ImageResponse& depthResponse = responses[1];
                    rawDepthWidth = static_cast<int>(depthResponse.width);
                    rawDepthHeight = static_cast<int>(depthResponse.height);
                    depthWidth = rawDepthWidth;
                    depthHeight = rawDepthHeight;
                    if (!depthResponse.image_data_float.empty()) {
                        depthValues = depthResponse.image_data_float;
                    }
                }

                if (!image.isNull()) {
                    if (!depthValues.empty() && rawDepthWidth > 0 && rawDepthHeight > 0
                        && (rawDepthWidth != image.width() || rawDepthHeight != image.height())) {
                        std::vector<float> sceneDepth = resampleDepthToScene(depthValues, rawDepthWidth, rawDepthHeight,
                                                                            image.width(), image.height());
                        if (!sceneDepth.empty()) {
                            depthValues = std::move(sceneDepth);
                            depthWidth = image.width();
                            depthHeight = image.height();
                        }
                    }
                    image.setDevicePixelRatio(1.0);
                    frameConnected = true;
                    success = true;
                    const QString depthStatus = depthValues.empty()
                        ? (snapshot.depthFetchEnabled ? QStringLiteral("unavailable") : QStringLiteral("disabled"))
                        : ((rawDepthWidth > 0 && rawDepthHeight > 0
                            && (rawDepthWidth != depthWidth || rawDepthHeight != depthHeight))
                            ? QStringLiteral("perspective %1x%2->%3x%4").arg(rawDepthWidth).arg(rawDepthHeight).arg(depthWidth).arg(depthHeight)
                            : QStringLiteral("perspective %1x%2").arg(depthWidth).arg(depthHeight));
                    status = QStringLiteral("AirSim %1 view running · %2x%3 · FOV %4° · depth %5")
                        .arg(snapshot.anchoredCamera ? QStringLiteral("anchored") : QStringLiteral("orbit"))
                        .arg(image.width())
                        .arg(image.height())
                        .arg(actualHorizontalFovDeg, 0, 'f', 1)
                        .arg(depthStatus);

                    const QImage frameImage = image;
                    const double yaw = snapshot.yawDeg;
                    const double pitch = snapshot.pitchDeg;
                    const double dist = snapshot.distance;
                    const double focusX = snapshot.focusX;
                    const double focusY = snapshot.focusY;
                    const double focusZ = snapshot.focusZ;
                    const Vec3 posCopy = actualCameraPos;
                    const Vec3 rightCopy = cameraBasis.right;
                    const Vec3 upCopy = cameraBasis.up;
                    const Vec3 forwardCopy = cameraBasis.forward;
                    const double fovCopy = actualHorizontalFovDeg;
                    const int sceneWidth = image.width();
                    const int sceneHeight = image.height();
                    const int depthWidthCopy = depthWidth;
                    const int depthHeightCopy = depthHeight;
                    QMetaObject::invokeMethod(this,
                        [this, frameImage, yaw, pitch, dist, focusX, focusY, focusZ,
                         posCopy, rightCopy, upCopy, forwardCopy, fovCopy,
                         sceneWidth, sceneHeight, depthWidthCopy, depthHeightCopy, depthValues, status]() {
                            applyFrameUpdate(frameImage, true, yaw, pitch, dist, focusX, focusY, focusZ,
                                             posCopy, rightCopy, upCopy, forwardCopy, fovCopy,
                                             sceneWidth, sceneHeight, depthWidthCopy, depthHeightCopy, depthValues, status);
                        },
                        Qt::QueuedConnection);
                } else {
                    status = QStringLiteral("AirSim returned an image frame that Qt could not decode.");
                }
            } else {
                status = QStringLiteral("AirSim returned no image data for external preview camera '%1'.").arg(snapshot.cameraName);
            }
        } catch (const std::exception& ex) {
            status = QStringLiteral("AirSim RPC error: %1").arg(QString::fromUtf8(ex.what()));
            client.reset();
        } catch (...) {
            status = QStringLiteral("Unknown AirSim RPC error while fetching live frame.");
            client.reset();
        }

        if (!success) {
            const double yaw = snapshot.yawDeg;
            const double pitch = snapshot.pitchDeg;
            const double dist = snapshot.distance;
            const double focusX = snapshot.focusX;
            const double focusY = snapshot.focusY;
            const double focusZ = snapshot.focusZ;
            Vec3 posCopy;
            Vec3 rightCopy;
            Vec3 upCopy;
            Vec3 forwardCopy;
            double fovCopy = 90.0;
            int sceneWidthCopy = 0;
            int sceneHeightCopy = 0;
            int depthWidthCopy = 0;
            int depthHeightCopy = 0;
            std::vector<float> depthCopy;
            {
                std::lock_guard<std::mutex> lock(geometryMutex_);
                posCopy = cameraPosition_;
                rightCopy = cameraRight_;
                upCopy = cameraUp_;
                forwardCopy = cameraForward_;
                fovCopy = horizontalFovDeg_;
                sceneWidthCopy = sceneWidth_;
                sceneHeightCopy = sceneHeight_;
                depthWidthCopy = depthWidth_;
                depthHeightCopy = depthHeight_;
                depthCopy = depthValues_;
            }
            QMetaObject::invokeMethod(this,
                [this, frameConnected, yaw, pitch, dist, focusX, focusY, focusZ,
                 posCopy, rightCopy, upCopy, forwardCopy, fovCopy,
                 sceneWidthCopy, sceneHeightCopy, depthWidthCopy, depthHeightCopy, depthCopy, status]() {
                    applyFrameUpdate(QImage(), frameConnected, yaw, pitch, dist, focusX, focusY, focusZ,
                                     posCopy, rightCopy, upCopy, forwardCopy, fovCopy,
                                     sceneWidthCopy, sceneHeightCopy, depthWidthCopy, depthHeightCopy, depthCopy, status);
                },
                Qt::QueuedConnection);
        }

        const double fps = std::max(1.0, snapshot.fps);
        const auto framePeriod = std::chrono::duration<double>(1.0 / fps);
        const auto nextFrameTime = loopStart + std::chrono::duration_cast<std::chrono::steady_clock::duration>(framePeriod);
        const auto now = std::chrono::steady_clock::now();
        if (nextFrameTime > now) {
            std::this_thread::sleep_until(nextFrameTime);
        }
    }
#else
    QMetaObject::invokeMethod(this, [this]() { setStatusText(formatAirSimDisabledMessage()); }, Qt::QueuedConnection);
#endif

    workerRunning_.store(false);
    QMetaObject::invokeMethod(this, [this]() {
        if (connected_) {
            connected_ = false;
            emit connectedChanged();
        }
        emit runningChanged();
    }, Qt::QueuedConnection);
}

}  // namespace airsim_gui
