#pragma once

#include "airsim_gui_UErealtime/AppTypes.h"
#include "airsim_gui_UErealtime/FrameTransformManager.h"
#include "airsim_gui_UErealtime/TransformUtils.h"

#include <QObject>
#include <QString>
#include <QPair>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <rf_msgs/RfObservationArray.h>
#include <sionna_sys_msgs/SysObservationArray.h>
#include <sionna_beam_msgs/BeamObservationArray.h>

namespace airsim_gui {

class RosBridge : public QObject {
    Q_OBJECT
public:
    explicit RosBridge(QObject* parent = nullptr);
    ~RosBridge() override;

    bool start(const QString& poseTopic, const QString& trajectoryTopic, const QString& rfTopic, const QString& sysTopic = QString(), const QString& beamTopic = QString());
    void setFrameTransformManager(const std::shared_ptr<FrameTransformManager>& manager);
    void setRosToSceneTransformPath(const QString& filePath);
    QString rosToSceneTransformPath() const { return rosToSceneTransformPath_; }
    void stop();
    bool isRunning() const;

    static std::vector<QPair<QString, QString>> listTopics();
    static QString topicType(const QString& topicName);
    static bool isSupportedPoseType(const QString& topicType);
    static bool isSupportedTrajectoryType(const QString& topicType);
    static bool isSupportedRfType(const QString& topicType);
    static bool isSupportedSysType(const QString& topicType);
    static bool isSupportedBeamType(const QString& topicType);

signals:
    void droneUpdated(const airsim_gui::DroneState& state);
    void pathsUpdated(const airsim_gui::BeamPathList& paths);
    void rfDataUpdated(const airsim_gui::RfObservationSnapshot& snapshot);
    void sysObservationUpdated(const QString& jsonText);
    void beamObservationUpdated(const QString& jsonText);
    void statusMessage(const QString& message);

private:
    void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg);
    void rfObservationCallback(const rf_msgs::RfObservationArray::ConstPtr& msg);
    void sysObservationCallback(const sionna_sys_msgs::SysObservationArray::ConstPtr& msg);
    void beamObservationCallback(const sionna_beam_msgs::BeamObservationArray::ConstPtr& msg);
    void handlePose(const Vec3& position,
                    double qx, double qy, double qz, double qw,
                    bool appendHistoryPoint,
                    const QString& sourceFrameId = QString());
    void appendInternalHistoryPoint(const Vec3& point);
    void handlePaths(const BeamPathList& paths);
    void updateRosFrameIdFromInput(const QString& frameId);
    void ensureRosToSceneTransformLoaded() const;
    Vec3 transformPointToScene(const Vec3& point,
                              const QString& sourceFrameId = QString(),
                              bool clampGround = false) const;
    Vec3 transformVectorToScene(const Vec3& vector,
                               const QString& sourceFrameId = QString()) const;
    bool transformQuaternionToScene(double x, double y, double z, double w,
                                    const QString& sourceFrameId,
                                    double* outX,
                                    double* outY,
                                    double* outZ,
                                    double* outW) const;
    static BeamPathList parsePathsJson(const std::string& jsonText);
    static BeamPathList fromNavPath(const nav_msgs::Path::ConstPtr& msg);
    airsim_gui::RfObservationSnapshot fromRfObservationSnapshot(const rf_msgs::RfObservationArray::ConstPtr& msg) const;
    BeamPathList fromRfObservationArray(const rf_msgs::RfObservationArray::ConstPtr& msg) const;
    static void quaternionToEulerDeg(double x, double y, double z, double w,
                                     double& rollDeg, double& pitchDeg, double& yawDeg);

    std::unique_ptr<ros::NodeHandle> nodeHandle_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::Subscriber poseSub_;
    ros::Subscriber trajectorySub_;
    ros::Subscriber rfSub_;
    ros::Subscriber sysSub_;
    ros::Subscriber beamSub_;
    bool running_{false};
    DroneState droneState_;
    ros::WallTime lastDroneEmitTime_;
    ros::WallTime lastPathsEmitTime_;
    QString currentPoseTopic_;
    QString currentTrajectoryTopic_;
    QString currentRfTopic_;
    QString currentSysTopic_;
    QString currentBeamTopic_;
    QString currentPoseType_;
    QString currentTrajectoryType_;
    QString currentRfType_;
    QString currentSysType_;
    QString currentBeamType_;
    std::shared_ptr<FrameTransformManager> frameTransformManager_;
    mutable TransformMatrix4x4 rosToSceneTransform_{};
    QString rosToSceneTransformPath_;
    mutable bool rosToSceneTransformLoaded_{false};
    double internalTrajectoryMinDist_{0.02};
    int internalTrajectoryMaxLen_{0};
};

}  // namespace airsim_gui
