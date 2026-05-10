#include "airsim_gui_UErealtime/RosBridge.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <ros/master.h>

#include <cmath>
#include <limits>

namespace {
constexpr double kDroneEmitMinIntervalSec = 1.0 / 20.0;
constexpr double kPathsEmitMinIntervalSec = 1.0 / 5.0;
constexpr double kGroundHeight = 0.0;

double squaredDistance(const airsim_gui::Vec3& a, const airsim_gui::Vec3& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

bool isNearZeroPoint(const airsim_gui::Vec3& p) {
    return std::abs(p.x) < 1e-6 && std::abs(p.y) < 1e-6 && std::abs(p.z) < 1e-6;
}

int tbStatusToAckState(int tbStatus) {
    if (tbStatus == 1) {
        return 1;
    }
    if (tbStatus == 2) {
        return 0;
    }
    return -1;
}

QJsonObject pointToJson(const geometry_msgs::Point& point) {
    return QJsonObject{{QStringLiteral("x"), point.x}, {QStringLiteral("y"), point.y}, {QStringLiteral("z"), point.z}};
}

QJsonObject pointToJson(const airsim_gui::Vec3& point) {
    return QJsonObject{{QStringLiteral("x"), point.x}, {QStringLiteral("y"), point.y}, {QStringLiteral("z"), point.z}};
}
}

namespace airsim_gui {

RosBridge::RosBridge(QObject* parent)
    : QObject(parent) {
}

RosBridge::~RosBridge() {
    stop();
}

void RosBridge::setFrameTransformManager(const std::shared_ptr<FrameTransformManager>& manager) {
    frameTransformManager_ = manager;
    if (frameTransformManager_) {
        frameTransformManager_->setRosToSceneTransformPath(rosToSceneTransformPath_);
    }
}

void RosBridge::setRosToSceneTransformPath(const QString& filePath) {
    rosToSceneTransformPath_ = filePath.trimmed();
    if (frameTransformManager_) {
        frameTransformManager_->setRosToSceneTransformPath(rosToSceneTransformPath_);
    }
    rosToSceneTransformLoaded_ = false;
    rosToSceneTransform_ = identityTransformMatrix4x4();
}

void RosBridge::ensureRosToSceneTransformLoaded() const {
    if (rosToSceneTransformLoaded_) {
        return;
    }
    rosToSceneTransform_ = identityTransformMatrix4x4();
    if (!rosToSceneTransformPath_.trimmed().isEmpty()) {
        QString error;
        const TransformMatrix4x4 loaded = loadTransformMatrixFromJson(rosToSceneTransformPath_, &error);
        if (error.trimmed().isEmpty()) {
            rosToSceneTransform_ = loaded;
        }
    }
    rosToSceneTransformLoaded_ = true;
}

void RosBridge::updateRosFrameIdFromInput(const QString& frameId) {
    if (!frameTransformManager_) {
        return;
    }
    const QString trimmed = frameId.trimmed();
    if (trimmed.isEmpty()) {
        return;
    }
    frameTransformManager_->setRosFrameId(trimmed);
}

Vec3 RosBridge::transformPointToScene(const Vec3& point,
                                      const QString& sourceFrameId,
                                      bool clampGround) const {
    const QString sourceFrame = sourceFrameId.trimmed();
    Vec3 transformed = point;
    bool success = false;
    if (frameTransformManager_ && !sourceFrame.isEmpty()) {
        success = frameTransformManager_->transformPoint(point,
                                                         sourceFrame,
                                                         FrameTransformManager::sceneFrameId(),
                                                         &transformed);
    }
    if (!success && frameTransformManager_ && sourceFrame.isEmpty()) {
        const QString fallbackRosFrame = frameTransformManager_->resolvedRosFrameId();
        success = frameTransformManager_->transformPoint(point,
                                                         fallbackRosFrame,
                                                         FrameTransformManager::sceneFrameId(),
                                                         &transformed);
    }
    if (!success && (sourceFrame.isEmpty()
                     || !frameTransformManager_
                     || sourceFrame == frameTransformManager_->resolvedRosFrameId())) {
        ensureRosToSceneTransformLoaded();
        transformed = transformPointByMatrix(rosToSceneTransform_, point);
    } else if (!success) {
        transformed = point;
    }
    if (clampGround) {
        transformed.z = std::max(transformed.z, kGroundHeight);
    }
    return transformed;
}

Vec3 RosBridge::transformVectorToScene(const Vec3& vector, const QString& sourceFrameId) const {
    const QString sourceFrame = sourceFrameId.trimmed();
    Vec3 transformed = vector;
    bool success = false;
    if (frameTransformManager_ && !sourceFrame.isEmpty()) {
        success = frameTransformManager_->transformVector(vector,
                                                          sourceFrame,
                                                          FrameTransformManager::sceneFrameId(),
                                                          &transformed);
    }
    if (!success && frameTransformManager_ && sourceFrame.isEmpty()) {
        const QString fallbackRosFrame = frameTransformManager_->resolvedRosFrameId();
        success = frameTransformManager_->transformVector(vector,
                                                          fallbackRosFrame,
                                                          FrameTransformManager::sceneFrameId(),
                                                          &transformed);
    }
    if (!success && (sourceFrame.isEmpty()
                     || !frameTransformManager_
                     || sourceFrame == frameTransformManager_->resolvedRosFrameId())) {
        ensureRosToSceneTransformLoaded();
        transformed = transformVectorByMatrix(rosToSceneTransform_, vector);
    } else if (!success) {
        transformed = vector;
    }
    return transformed;
}

bool RosBridge::transformQuaternionToScene(double x, double y, double z, double w,
                                           const QString& sourceFrameId,
                                           double* outX,
                                           double* outY,
                                           double* outZ,
                                           double* outW) const {
    if (!outX || !outY || !outZ || !outW) {
        return false;
    }
    const QString sourceFrame = sourceFrameId.trimmed();
    if (frameTransformManager_ && !sourceFrame.isEmpty()) {
        if (frameTransformManager_->transformQuaternion(x, y, z, w,
                                                        sourceFrame,
                                                        FrameTransformManager::sceneFrameId(),
                                                        outX, outY, outZ, outW)) {
            return true;
        }
    }
    if (frameTransformManager_ && sourceFrame.isEmpty()) {
        const QString fallbackRosFrame = frameTransformManager_->resolvedRosFrameId();
        if (frameTransformManager_->transformQuaternion(x, y, z, w,
                                                        fallbackRosFrame,
                                                        FrameTransformManager::sceneFrameId(),
                                                        outX, outY, outZ, outW)) {
            return true;
        }
    }
    *outX = x;
    *outY = y;
    *outZ = z;
    *outW = w;
    return false;
}

std::vector<QPair<QString, QString>> RosBridge::listTopics() {
    std::vector<QPair<QString, QString>> out;
    if (!ros::master::check()) {
        return out;
    }

    ros::master::V_TopicInfo masterTopics;
    if (!ros::master::getTopics(masterTopics)) {
        return out;
    }

    out.reserve(masterTopics.size());
    for (const auto& info : masterTopics) {
        out.push_back(qMakePair(QString::fromStdString(info.name),
                                QString::fromStdString(info.datatype)));
    }
    return out;
}

QString RosBridge::topicType(const QString& topicName) {
    if (topicName.trimmed().isEmpty()) {
        return QString();
    }
    for (const auto& topic : listTopics()) {
        if (topic.first == topicName.trimmed()) {
            return topic.second;
        }
    }
    return QString();
}

bool RosBridge::isSupportedPoseType(const QString& topicType) {
    return topicType == QLatin1String("geometry_msgs/PoseStamped") ||
           topicType == QLatin1String("nav_msgs/Odometry");
}

bool RosBridge::isSupportedTrajectoryType(const QString& topicType) {
    return topicType == QLatin1String("nav_msgs/Path");
}

bool RosBridge::isSupportedRfType(const QString& topicType) {
    return topicType == QLatin1String("rf_msgs/RfObservationArray") ||
           topicType == QLatin1String("std_msgs/String");
}

bool RosBridge::isSupportedSysType(const QString& topicType) {
    return topicType == QLatin1String("sionna_sys_msgs/SysObservationArray");
}

bool RosBridge::isSupportedBeamType(const QString& topicType) {
    return topicType == QLatin1String("sionna_beam_msgs/BeamObservationArray");
}

bool RosBridge::start(const QString& poseTopic, const QString& trajectoryTopic, const QString& rfTopic, const QString& sysTopic, const QString& beamTopic) {
    const QString poseName = poseTopic.trimmed();
    const QString trajectoryName = trajectoryTopic.trimmed();
    const QString rfName = rfTopic.trimmed();
    const QString sysName = sysTopic.trimmed();
    const QString beamName = beamTopic.trimmed();

    if (running_) {
        stop();
    }

    if (!ros::master::check()) {
        emit statusMessage("ROS master is not available. Start roscore first.");
        return false;
    }

    if (poseName.isEmpty()) {
        emit statusMessage("Pose topic is empty.");
        return false;
    }
    currentPoseType_ = topicType(poseName);
    if (!isSupportedPoseType(currentPoseType_)) {
        emit statusMessage(QString("Pose topic %1 has unsupported type %2")
                               .arg(poseName, currentPoseType_.isEmpty() ? QString("<not found>") : currentPoseType_));
        return false;
    }

    currentTrajectoryType_.clear();
    if (!trajectoryName.isEmpty()) {
        currentTrajectoryType_ = topicType(trajectoryName);
        if (!isSupportedTrajectoryType(currentTrajectoryType_)) {
            emit statusMessage(QString("Trajectory topic %1 has unsupported type %2")
                                   .arg(trajectoryName, currentTrajectoryType_.isEmpty() ? QString("<not found>") : currentTrajectoryType_));
            return false;
        }
    } else {
        currentTrajectoryType_ = QStringLiteral("<internal from pose>");
    }

    currentRfType_.clear();
    if (!rfName.isEmpty()) {
        currentRfType_ = topicType(rfName);
        if (!isSupportedRfType(currentRfType_)) {
            emit statusMessage(QString("RF topic %1 has unsupported type %2")
                                   .arg(rfName, currentRfType_.isEmpty() ? QString("<not found>") : currentRfType_));
            return false;
        }
    } else {
        currentRfType_ = QStringLiteral("<disabled>");
    }

    currentSysType_.clear();
    if (!sysName.isEmpty()) {
        currentSysType_ = topicType(sysName);
        if (!isSupportedSysType(currentSysType_)) {
            emit statusMessage(QString("SYS topic %1 has unsupported type %2")
                                   .arg(sysName, currentSysType_.isEmpty() ? QString("<not found>") : currentSysType_));
            return false;
        }
    } else {
        currentSysType_ = QStringLiteral("<disabled>");
    }

    currentBeamType_.clear();
    if (!beamName.isEmpty()) {
        currentBeamType_ = topicType(beamName);
        if (!currentBeamType_.isEmpty() && !isSupportedBeamType(currentBeamType_)) {
            emit statusMessage(QString("Beam topic %1 has unsupported type %2")
                                   .arg(beamName, currentBeamType_));
            return false;
        }
        if (currentBeamType_.isEmpty()) {
            currentBeamType_ = QStringLiteral("<not found>");
        }
    } else {
        currentBeamType_ = QStringLiteral("<disabled>");
    }

    nodeHandle_ = std::make_unique<ros::NodeHandle>();
    currentPoseTopic_ = poseName;
    currentTrajectoryTopic_ = trajectoryName;
    currentRfTopic_ = rfName;
    currentSysTopic_ = sysName;
    currentBeamTopic_ = beamName;

    if (currentPoseType_ == QLatin1String("nav_msgs/Odometry")) {
        poseSub_ = nodeHandle_->subscribe<nav_msgs::Odometry>(
            poseName.toStdString(), 10, &RosBridge::odometryCallback, this);
    } else {
        poseSub_ = nodeHandle_->subscribe<geometry_msgs::PoseStamped>(
            poseName.toStdString(), 10, &RosBridge::poseStampedCallback, this);
    }

    if (!trajectoryName.isEmpty()) {
        trajectorySub_ = nodeHandle_->subscribe<nav_msgs::Path>(
            trajectoryName.toStdString(), 3, &RosBridge::trajectoryCallback, this);
    }

    if (!rfName.isEmpty()) {
        if (currentRfType_ == QLatin1String("rf_msgs/RfObservationArray")) {
            rfSub_ = nodeHandle_->subscribe<rf_msgs::RfObservationArray>(
                rfName.toStdString(), 3, &RosBridge::rfObservationCallback, this);
        } else {
            rfSub_ = nodeHandle_->subscribe<std_msgs::String>(
                rfName.toStdString(), 3,
                [this](const std_msgs::String::ConstPtr& msg) { handlePaths(parsePathsJson(msg->data)); });
        }
    }

    if (!sysName.isEmpty()) {
        sysSub_ = nodeHandle_->subscribe<sionna_sys_msgs::SysObservationArray>(
            sysName.toStdString(), 3, &RosBridge::sysObservationCallback, this);
    }

    if (!beamName.isEmpty() &&
        (currentBeamType_ == QLatin1String("sionna_beam_msgs/BeamObservationArray") ||
         currentBeamType_ == QLatin1String("<not found>"))) {
        beamSub_ = nodeHandle_->subscribe<sionna_beam_msgs::BeamObservationArray>(
            beamName.toStdString(), 3, &RosBridge::beamObservationCallback, this);
    }

    droneState_ = DroneState{};
    lastDroneEmitTime_ = ros::WallTime(0.0);
    lastPathsEmitTime_ = ros::WallTime(0.0);

    spinner_ = std::make_unique<ros::AsyncSpinner>(1);
    spinner_->start();
    running_ = true;
    const QString trajectoryStatusTopic = currentTrajectoryTopic_.trimmed().isEmpty() ? QStringLiteral("<internal>") : currentTrajectoryTopic_;
    const QString rfStatusTopic = currentRfTopic_.trimmed().isEmpty() ? QStringLiteral("<disabled>") : currentRfTopic_;
    const QString sysStatusTopic = currentSysTopic_.trimmed().isEmpty() ? QStringLiteral("<disabled>") : currentSysTopic_;
    const QString beamStatusTopic = currentBeamTopic_.trimmed().isEmpty() ? QStringLiteral("<disabled>") : currentBeamTopic_;
    emit statusMessage(QString("ROS1 bridge connected: pose=%1 [%2], trajectory=%3 [%4], rf=%5 [%6], sys=%7 [%8], beam=%9 [%10]")
                           .arg(currentPoseTopic_)
                           .arg(currentPoseType_)
                           .arg(trajectoryStatusTopic)
                           .arg(currentTrajectoryType_)
                           .arg(rfStatusTopic)
                           .arg(currentRfType_)
                           .arg(sysStatusTopic)
                           .arg(currentSysType_)
                           .arg(beamStatusTopic)
                           .arg(currentBeamType_));
    return true;
}

void RosBridge::stop() {
    if (!running_ && !spinner_ && !nodeHandle_) {
        return;
    }

    poseSub_.shutdown();
    trajectorySub_.shutdown();
    rfSub_.shutdown();
    sysSub_.shutdown();
    beamSub_.shutdown();

    if (spinner_) {
        spinner_->stop();
        spinner_.reset();
    }
    nodeHandle_.reset();
    running_ = false;
    currentPoseTopic_.clear();
    currentTrajectoryTopic_.clear();
    currentRfTopic_.clear();
    currentSysTopic_.clear();
    currentBeamTopic_.clear();
    currentPoseType_.clear();
    currentTrajectoryType_.clear();
    currentRfType_.clear();
    currentSysType_.clear();
    currentBeamType_.clear();
    emit statusMessage("ROS1 bridge disconnected.");
}

bool RosBridge::isRunning() const {
    return running_;
}

void RosBridge::poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    const QString sourceFrameId = QString::fromStdString(msg->header.frame_id).trimmed();
    updateRosFrameIdFromInput(sourceFrameId);
    const Vec3 transformed = transformPointToScene({msg->pose.position.x, msg->pose.position.y, msg->pose.position.z},
                                                   sourceFrameId,
                                                   true);
    double qx = msg->pose.orientation.x;
    double qy = msg->pose.orientation.y;
    double qz = msg->pose.orientation.z;
    double qw = msg->pose.orientation.w;
    transformQuaternionToScene(qx, qy, qz, qw, sourceFrameId, &qx, &qy, &qz, &qw);
    handlePose(transformed, qx, qy, qz, qw, true, sourceFrameId);
}

void RosBridge::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    const QString sourceFrameId = QString::fromStdString(msg->header.frame_id).trimmed();
    updateRosFrameIdFromInput(sourceFrameId);
    const Vec3 transformed = transformPointToScene({msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z},
                                                   sourceFrameId,
                                                   true);
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    transformQuaternionToScene(qx, qy, qz, qw, sourceFrameId, &qx, &qy, &qz, &qw);
    handlePose(transformed, qx, qy, qz, qw, true, sourceFrameId);
}

void RosBridge::trajectoryCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const QString pathFrameId = QString::fromStdString(msg->header.frame_id).trimmed();
    updateRosFrameIdFromInput(pathFrameId);
    std::vector<Vec3> history;
    history.reserve(msg->poses.size());
    for (const auto& poseStamped : msg->poses) {
        const QString poseFrameId = QString::fromStdString(poseStamped.header.frame_id).trimmed();
        const QString sourceFrameId = poseFrameId.isEmpty() ? pathFrameId : poseFrameId;
        history.push_back(transformPointToScene({poseStamped.pose.position.x,
                                                 poseStamped.pose.position.y,
                                                 poseStamped.pose.position.z},
                                                sourceFrameId,
                                                true));
    }
    droneState_.history = history;

    if (!msg->poses.empty()) {
        const auto& poseStamped = msg->poses.back();
        const QString poseFrameId = QString::fromStdString(poseStamped.header.frame_id).trimmed();
        const QString sourceFrameId = poseFrameId.isEmpty() ? pathFrameId : poseFrameId;
        const auto& pose = poseStamped.pose;
        const Vec3 transformed = transformPointToScene({pose.position.x, pose.position.y, pose.position.z},
                                                       sourceFrameId,
                                                       true);
        double qx = pose.orientation.x;
        double qy = pose.orientation.y;
        double qz = pose.orientation.z;
        double qw = pose.orientation.w;
        transformQuaternionToScene(qx, qy, qz, qw, sourceFrameId, &qx, &qy, &qz, &qw);
        handlePose(transformed, qx, qy, qz, qw, false, sourceFrameId);
    } else {
        const ros::WallTime now = ros::WallTime::now();
        if (lastDroneEmitTime_.isZero() || (now - lastDroneEmitTime_).toSec() >= kDroneEmitMinIntervalSec) {
            lastDroneEmitTime_ = now;
            emit droneUpdated(droneState_);
        }
    }
}

void RosBridge::rfObservationCallback(const rf_msgs::RfObservationArray::ConstPtr& msg) {
    const RfObservationSnapshot snapshot = fromRfObservationSnapshot(msg);
    emit rfDataUpdated(snapshot);
    handlePaths(fromRfObservationArray(msg));
}

void RosBridge::sysObservationCallback(const sionna_sys_msgs::SysObservationArray::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    QJsonObject root;
    root.insert(QStringLiteral("valid"), true);
    root.insert(QStringLiteral("enabled"), msg->enabled);
    root.insert(QStringLiteral("scheduler_policy"), QString::fromStdString(msg->scheduler_policy_name));
    root.insert(QStringLiteral("sim_idx"), static_cast<int>(msg->sim_idx));
    root.insert(QStringLiteral("odom_stamp_s"), msg->odom_stamp_s);
    root.insert(QStringLiteral("frame_id"), FrameTransformManager::sceneFrameId());
    root.insert(QStringLiteral("source_frame_id"), QString::fromStdString(msg->header.frame_id));

    const auto& serving = msg->serving;
    root.insert(QStringLiteral("serving_idx"), serving.serving_index);
    root.insert(QStringLiteral("serving_bs_name"), QString::fromStdString(serving.anchor_name));
    root.insert(QStringLiteral("serving_anchor_id"), serving.anchor_id);
    root.insert(QStringLiteral("serving_mcs_index"), serving.mcs_index);
    root.insert(QStringLiteral("serving_sinr_eff_db"), serving.sinr_eff_db);
    root.insert(QStringLiteral("serving_spectral_efficiency_bpshz"), serving.spectral_efficiency_bpshz);
    if (tbStatusToAckState(static_cast<int>(serving.tb_status)) >= 0) {
        root.insert(QStringLiteral("serving_tb_ok"), tbStatusToAckState(static_cast<int>(serving.tb_status)) == 1);
    }

    QJsonArray candidates;
    for (const auto& candidate : msg->candidates) {
        const Vec3 anchorPosition = transformPointToScene({candidate.anchor_position.x,
                                                           candidate.anchor_position.y,
                                                           candidate.anchor_position.z},
                                                          QString::fromStdString(msg->header.frame_id));
        QJsonObject item;
        item.insert(QStringLiteral("sim_idx"), candidate.sim_idx);
        item.insert(QStringLiteral("odom_stamp_s"), candidate.odom_stamp_s);
        item.insert(QStringLiteral("bs_name"), QString::fromStdString(candidate.anchor_name));
        item.insert(QStringLiteral("anchor_id"), candidate.anchor_id);
        item.insert(QStringLiteral("anchor_position"), pointToJson(anchorPosition));
        item.insert(QStringLiteral("num_paths"), static_cast<int>(candidate.num_paths));
        item.insert(QStringLiteral("power_db"), candidate.strongest_path_power_db);
        item.insert(QStringLiteral("path_type"), QString::fromStdString(candidate.strongest_path_type));
        item.insert(QStringLiteral("path_order"), candidate.strongest_path_order);
        item.insert(QStringLiteral("sys_enabled"), msg->enabled);
        item.insert(QStringLiteral("sys_candidate_rate_bpshz"), candidate.candidate_rate_bpshz);
        item.insert(QStringLiteral("sys_candidate_sinr_eff_db"), candidate.candidate_sinr_eff_db);
        item.insert(QStringLiteral("sys_mcs_index"), candidate.mcs_index);
        item.insert(QStringLiteral("sys_num_decoded_bits"), candidate.num_decoded_bits);
        item.insert(QStringLiteral("sys_spectral_efficiency_bpshz"), candidate.spectral_efficiency_bpshz);
        item.insert(QStringLiteral("sys_bler_target"), candidate.bler_target);
        item.insert(QStringLiteral("sys_is_serving_bs"), candidate.is_serving);
        item.insert(QStringLiteral("sys_serving_bs_name"), QString::fromStdString(serving.anchor_name));
        item.insert(QStringLiteral("sys_serving_anchor_id"), serving.anchor_id);
        const int tbOk = tbStatusToAckState(static_cast<int>(candidate.tb_status));
        if (tbOk >= 0) {
            item.insert(QStringLiteral("sys_tb_ok"), tbOk == 1);
        }
        candidates.push_back(item);
    }
    root.insert(QStringLiteral("candidates"), candidates);

    emit sysObservationUpdated(QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Compact)));
}

void RosBridge::beamObservationCallback(const sionna_beam_msgs::BeamObservationArray::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    QJsonObject root;
    root.insert(QStringLiteral("valid"), true);
    root.insert(QStringLiteral("sim_idx"), static_cast<int>(msg->sim_idx));
    root.insert(QStringLiteral("odom_stamp_s"), msg->odom_stamp_s);
    root.insert(QStringLiteral("frame_id"), QString::fromStdString(msg->frame_id));
    root.insert(QStringLiteral("source_frame_id"), QString::fromStdString(msg->header.frame_id));
    root.insert(QStringLiteral("codebook_id"), QString::fromStdString(msg->codebook_id));
    root.insert(QStringLiteral("selection_mode"), QString::fromStdString(msg->selection_mode));

    QJsonArray observations;
    for (const auto& obs : msg->observations) {
        QJsonObject item;
        item.insert(QStringLiteral("sim_idx"), static_cast<int>(obs.sim_idx));
        item.insert(QStringLiteral("odom_stamp_s"), obs.odom_stamp_s);
        item.insert(QStringLiteral("bs_index"), static_cast<int>(obs.bs_index));
        item.insert(QStringLiteral("bs_name"), QString::fromStdString(obs.bs_name));
        item.insert(QStringLiteral("anchor_id"), static_cast<int>(obs.anchor_id));
        item.insert(QStringLiteral("tx_idx"), obs.tx_idx);
        item.insert(QStringLiteral("beam_enabled"), obs.beamforming_enabled);
        item.insert(QStringLiteral("beam_selection_mode"), QString::fromStdString(obs.selection_mode));
        item.insert(QStringLiteral("beam_selection_source"), QString::fromStdString(obs.selection_source));
        item.insert(QStringLiteral("beam_serving_source"), QString::fromStdString(obs.serving_source));
        item.insert(QStringLiteral("beam_codebook_id"), QString::fromStdString(obs.codebook_id));
        item.insert(QStringLiteral("beam_codebook_type"), QString::fromStdString(obs.codebook_type));
        item.insert(QStringLiteral("beam_num_beams"), static_cast<int>(obs.num_beams));
        item.insert(QStringLiteral("beam_manual_index"), obs.manual_beam_index);
        item.insert(QStringLiteral("beam_feature_mode"), QString::fromStdString(obs.feature_mode));
        item.insert(QStringLiteral("beam_top_k"), obs.top_k);
        item.insert(QStringLiteral("beam_predictor_available"), obs.predictor_available);
        item.insert(QStringLiteral("beam_predictor_status"), QString::fromStdString(obs.predictor_status));
        item.insert(QStringLiteral("beam_predictor_error"), QString::fromStdString(obs.predictor_error));
        item.insert(QStringLiteral("beam_selected_index"), obs.selected_beam_index);
        item.insert(QStringLiteral("beam_selected_gain_db"), obs.selected_beam_gain_db);
        item.insert(QStringLiteral("beam_oracle_index"), obs.oracle_beam_index);
        item.insert(QStringLiteral("beam_oracle_gain_db"), obs.oracle_beam_gain_db);
        item.insert(QStringLiteral("beam_predicted_index"), obs.predicted_beam_index);
        item.insert(QStringLiteral("beam_predicted_confidence"), obs.predicted_beam_confidence);
        item.insert(QStringLiteral("sys_is_serving_bs"), obs.is_serving_bs);
        item.insert(QStringLiteral("beam_hit"), obs.beam_hit);
        item.insert(QStringLiteral("beam_oracle_in_topk"), obs.oracle_in_topk);
        QJsonArray topkIndices;
        for (const auto& v : obs.topk_indices) {
            topkIndices.push_back(static_cast<int>(v));
        }
        item.insert(QStringLiteral("beam_topk_indices"), topkIndices);
        observations.push_back(item);
    }
    root.insert(QStringLiteral("observations"), observations);

    emit beamObservationUpdated(QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Compact)));
}

void RosBridge::appendInternalHistoryPoint(const Vec3& point) {
    if (!droneState_.history.empty()) {
        if (squaredDistance(droneState_.history.back(), point) < internalTrajectoryMinDist_ * internalTrajectoryMinDist_) {
            return;
        }
    }
    droneState_.history.push_back(point);
    if (internalTrajectoryMaxLen_ > 0 && static_cast<int>(droneState_.history.size()) > internalTrajectoryMaxLen_) {
        droneState_.history.erase(droneState_.history.begin(),
                                  droneState_.history.begin() + (droneState_.history.size() - static_cast<size_t>(internalTrajectoryMaxLen_)));
    }
}

void RosBridge::handlePose(const Vec3& position,
                           double qx, double qy, double qz, double qw,
                           bool appendHistoryPoint,
                           const QString& sourceFrameId) {
    droneState_.valid = true;
    droneState_.id = "uav_ros";
    droneState_.position = position;
    droneState_.sourceFrameId = sourceFrameId.trimmed();
    if (droneState_.sourceFrameId.isEmpty() && frameTransformManager_) {
        droneState_.sourceFrameId = frameTransformManager_->resolvedRosFrameId();
    }

    quaternionToEulerDeg(qx, qy, qz, qw,
                         droneState_.rollDeg,
                         droneState_.pitchDeg,
                         droneState_.yawDeg);

    if (appendHistoryPoint) {
        appendInternalHistoryPoint(droneState_.position);
    }

    const ros::WallTime now = ros::WallTime::now();
    if (lastDroneEmitTime_.isZero() || (now - lastDroneEmitTime_).toSec() >= kDroneEmitMinIntervalSec) {
        lastDroneEmitTime_ = now;
        emit droneUpdated(droneState_);
    }
}

void RosBridge::handlePaths(const BeamPathList& paths) {
    const ros::WallTime now = ros::WallTime::now();
    if (lastPathsEmitTime_.isZero() || (now - lastPathsEmitTime_).toSec() >= kPathsEmitMinIntervalSec) {
        lastPathsEmitTime_ = now;
        emit pathsUpdated(paths);
    }
}

BeamPathList RosBridge::fromNavPath(const nav_msgs::Path::ConstPtr& msg) {
    BeamPathList paths;
    if (!msg) {
        return paths;
    }

    BeamPath path;
    path.id = QString("trajectory_%1").arg(msg->header.seq);
    path.txId = "trajectory";
    path.rxId = "drone";
    path.kind = "trajectory";
    path.powerDb = -50.0;
    path.delayNs = 0.0;
    path.points.reserve(msg->poses.size());
    for (const auto& poseStamped : msg->poses) {
        path.points.push_back({poseStamped.pose.position.x,
                               poseStamped.pose.position.y,
                               poseStamped.pose.position.z});
    }
    if (path.points.size() >= 2) {
        paths.push_back(path);
    }
    return paths;
}

RfObservationSnapshot RosBridge::fromRfObservationSnapshot(const rf_msgs::RfObservationArray::ConstPtr& msg) const {
    RfObservationSnapshot snapshot;
    if (!msg) {
        return snapshot;
    }

    const QString sourceFrameId = QString::fromStdString(msg->header.frame_id).trimmed();
    snapshot.valid = true;
    snapshot.frameId = FrameTransformManager::sceneFrameId();
    snapshot.rxPosition = transformPointToScene({msg->rx_position.x, msg->rx_position.y, msg->rx_position.z},
                                                sourceFrameId);
    snapshot.rxVelocity = transformVectorToScene({msg->rx_velocity.x, msg->rx_velocity.y, msg->rx_velocity.z},
                                                 sourceFrameId);
    snapshot.anchors.reserve(msg->anchors.size());

    for (const auto& anchor : msg->anchors) {
        RfAnchorObservationData anchorData;
        anchorData.anchorId = anchor.anchor_id;
        anchorData.anchorName = QString::fromStdString(anchor.anchor_name);
        anchorData.anchorPosition = transformPointToScene({anchor.anchor_position.x,
                                                           anchor.anchor_position.y,
                                                           anchor.anchor_position.z},
                                                          sourceFrameId);
        anchorData.paths.reserve(anchor.paths.size());

        for (const auto& rfPath : anchor.paths) {
            RfPathObservationData pathData;
            pathData.pathId = QString::fromStdString(rfPath.path_id);
            pathData.pathType = static_cast<int>(rfPath.path_type);
            pathData.pathOrder = rfPath.path_order;
            pathData.pathIndex = rfPath.path_index;
            pathData.isValid = rfPath.is_valid;
            pathData.hasAmplitude = rfPath.has_amplitude;
            pathData.amplitudeAbs = rfPath.amplitude_abs;
            pathData.hasPathGain = rfPath.has_path_gain;
            pathData.pathGainLinear = rfPath.path_gain_linear;
            pathData.hasPowerDb = rfPath.has_power_db;
            pathData.powerDb = rfPath.power_db;
            pathData.hasTau = rfPath.has_tau;
            pathData.tauS = rfPath.tau_s;
            pathData.tauStdS = rfPath.tau_std_s;
            pathData.hasDoppler = rfPath.has_doppler;
            pathData.dopplerHz = rfPath.doppler_hz;
            pathData.dopplerStdHz = rfPath.doppler_std_hz;
            pathData.hasAoa = rfPath.has_aoa;
            pathData.aoaAzRad = rfPath.aoa_az_rad;
            pathData.aoaElRad = rfPath.aoa_el_rad;
            pathData.aoaAzStdRad = rfPath.aoa_az_std_rad;
            pathData.aoaElStdRad = rfPath.aoa_el_std_rad;
            pathData.txPosition = transformPointToScene({rfPath.tx_position.x, rfPath.tx_position.y, rfPath.tx_position.z},
                                                        sourceFrameId);
            pathData.rxPosition = transformPointToScene({rfPath.rx_position.x, rfPath.rx_position.y, rfPath.rx_position.z},
                                                        sourceFrameId);
            pathData.pathPoints.reserve(rfPath.path_points.size());
            for (const auto& p : rfPath.path_points) {
                pathData.pathPoints.push_back(transformPointToScene({p.x, p.y, p.z}, sourceFrameId));
            }
            anchorData.paths.push_back(std::move(pathData));
        }

        snapshot.anchors.push_back(std::move(anchorData));
    }

    return snapshot;
}

BeamPathList RosBridge::fromRfObservationArray(const rf_msgs::RfObservationArray::ConstPtr& msg) const {
    BeamPathList paths;
    if (!msg) {
        return paths;
    }

    const QString sourceFrameId = QString::fromStdString(msg->header.frame_id).trimmed();
    for (const auto& anchor : msg->anchors) {
        for (const auto& rfPath : anchor.paths) {
            if (!rfPath.is_valid) {
                continue;
            }
            BeamPath path;
            path.id = QString::fromStdString(rfPath.path_id);
            path.txId = QString::fromStdString(anchor.anchor_name);
            path.rxId = QString("uav");
            path.kind = rfPath.path_type == 1 ? QString("los") : QString("rf");
            path.powerDb = rfPath.has_power_db ? rfPath.power_db : -100.0;
            path.delayNs = rfPath.has_tau ? rfPath.tau_s * 1e9 : 0.0;
            path.points.push_back(transformPointToScene({rfPath.tx_position.x, rfPath.tx_position.y, rfPath.tx_position.z},
                                                        sourceFrameId));
            for (const auto& p : rfPath.path_points) {
                Vec3 pt = transformPointToScene({p.x, p.y, p.z}, sourceFrameId);
                if (isNearZeroPoint(pt) && rfPath.path_order == 0) {
                    continue;
                }
                path.points.push_back(pt);
            }
            path.points.push_back(transformPointToScene({rfPath.rx_position.x, rfPath.rx_position.y, rfPath.rx_position.z},
                                                        sourceFrameId));
            if (path.points.size() >= 2) {
                paths.push_back(path);
            }
        }
    }
    return paths;
}

BeamPathList RosBridge::parsePathsJson(const std::string& jsonText) {
    BeamPathList paths;
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(jsonText), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        return paths;
    }

    const QJsonArray pathArray = doc.object().value("paths").toArray();
    for (const QJsonValue& pathValue : pathArray) {
        const QJsonObject obj = pathValue.toObject();
        BeamPath path;
        path.id = obj.value("id").toString();
        path.txId = obj.value("tx_id").toString();
        path.rxId = obj.value("rx_id").toString();
        path.powerDb = obj.value("power_db").toDouble(-80.0);
        path.delayNs = obj.value("delay_ns").toDouble(0.0);
        path.kind = obj.value("kind").toString("specular");

        const QJsonArray pointsArray = obj.value("points").toArray();
        for (const QJsonValue& pointValue : pointsArray) {
            const QJsonArray point = pointValue.toArray();
            if (point.size() >= 3) {
                path.points.push_back({point.at(0).toDouble(),
                                       point.at(1).toDouble(),
                                       point.at(2).toDouble()});
            }
        }

        if (!path.id.isEmpty() && path.points.size() >= 2) {
            paths.push_back(path);
        }
    }

    return paths;
}

void RosBridge::quaternionToEulerDeg(double x, double y, double z, double w,
                                     double& rollDeg, double& pitchDeg, double& yawDeg) {
    const double sinrCosp = 2.0 * (w * x + y * z);
    const double cosrCosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinrCosp, cosrCosp);

    const double sinp = 2.0 * (w * y - z * x);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const double sinyCosp = 2.0 * (w * z + x * y);
    const double cosyCosp = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(sinyCosp, cosyCosp);

    constexpr double kRadToDeg = 180.0 / M_PI;
    rollDeg = roll * kRadToDeg;
    pitchDeg = pitch * kRadToDeg;
    yawDeg = yaw * kRadToDeg;
}

}  // namespace airsim_gui
