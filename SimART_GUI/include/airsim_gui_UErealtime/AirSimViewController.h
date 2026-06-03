#pragma once

#include <QObject>

#include <ros/ros.h>
#include <QTemporaryDir>
#include <QString>
#include <QImage>
#include <QPointF>
#include <QVariant>

#include <sensor_msgs/Image.h>

#include <vector>
#include <memory>

#include "airsim_gui_UErealtime/AppTypes.h"
#include "airsim_gui_UErealtime/TransformUtils.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace airsim_gui {

class FrameTransformManager;

class AirSimViewController : public QObject {
    Q_OBJECT
    Q_PROPERTY(int frameToken READ frameToken NOTIFY frameTokenChanged)
    Q_PROPERTY(QString statusText READ statusText NOTIFY statusTextChanged)
    Q_PROPERTY(bool running READ isRunning NOTIFY runningChanged)
    Q_PROPERTY(bool connected READ isConnected NOTIFY connectedChanged)
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY configChanged)
    Q_PROPERTY(int port READ port WRITE setPort NOTIFY configChanged)
    Q_PROPERTY(QString cameraName READ cameraName WRITE setCameraName NOTIFY configChanged)
    Q_PROPERTY(QString vehicleName READ vehicleName WRITE setVehicleName NOTIFY configChanged)
    Q_PROPERTY(bool followVehicle READ followVehicle WRITE setFollowVehicle NOTIFY configChanged)
    Q_PROPERTY(double yawDeg READ yawDeg NOTIFY cameraStateChanged)
    Q_PROPERTY(double pitchDeg READ pitchDeg NOTIFY cameraStateChanged)
    Q_PROPERTY(double rollDeg READ rollDeg NOTIFY cameraStateChanged)
    Q_PROPERTY(double distance READ distance NOTIFY cameraStateChanged)
    Q_PROPERTY(double framesPerSecond READ framesPerSecond WRITE setFramesPerSecond NOTIFY configChanged)
    Q_PROPERTY(QVariantList stationOverlays READ stationOverlays NOTIFY overlayDataChanged)
    Q_PROPERTY(QVariantList pathOverlays READ pathOverlays NOTIFY overlayDataChanged)
    Q_PROPERTY(QVariantMap droneOverlay READ droneOverlay NOTIFY overlayDataChanged)
    Q_PROPERTY(bool depthAvailable READ depthAvailable NOTIFY overlayDataChanged)
    Q_PROPERTY(int depthWidth READ depthWidth NOTIFY overlayDataChanged)
    Q_PROPERTY(int depthHeight READ depthHeight NOTIFY overlayDataChanged)
    Q_PROPERTY(QString anchorPointText READ anchorPointText NOTIFY anchorPointChanged)
    Q_PROPERTY(QString cameraPositionText READ cameraPositionText NOTIFY cameraPositionChanged)
    Q_PROPERTY(QString viewLabel READ viewLabel WRITE setViewLabel NOTIFY viewLabelChanged)
    Q_PROPERTY(double displayBrightness READ displayBrightness WRITE setDisplayBrightness NOTIFY displayBrightnessChanged)
    Q_PROPERTY(bool viewGizmoVisible READ viewGizmoVisible NOTIFY viewGizmoVisibleChanged)
public:
    explicit AirSimViewController(QObject* parent = nullptr);
    ~AirSimViewController() override;

    int frameToken() const { return frameToken_; }
    QImage latestFrameCopy() const;
    QString statusText() const { return statusText_; }
    bool isRunning() const { return workerRunning_.load() || rosImageSubscriptionActive_; }
    bool isConnected() const { return connected_; }
    QString host() const { return host_; }
    int port() const { return port_; }
    QString cameraName() const { return cameraName_; }
    QString vehicleName() const { return vehicleName_; }
    bool followVehicle() const { return followVehicle_; }
    double yawDeg() const { return yawDeg_; }
    double pitchDeg() const { return pitchDeg_; }
    double rollDeg() const { return rollDeg_; }
    double distance() const { return distance_; }
    double framesPerSecond() const { return framesPerSecond_; }
    QVariantList stationOverlays() const { return stationOverlays_; }
    QVariantList pathOverlays() const { return pathOverlays_; }
    QVariantMap droneOverlay() const { return droneOverlay_; }
    bool depthAvailable() const { return depthAvailable_; }
    int depthWidth() const { return depthWidth_; }
    int depthHeight() const { return depthHeight_; }
    QString anchorPointText() const;
    QString cameraPositionText() const;
    QString viewLabel() const { return viewLabel_; }
    double displayBrightness() const;
    bool viewGizmoVisible() const { return viewGizmoVisible_; }

    void setBaseStations(const std::vector<BaseStation>& stations);
    void setBeamPaths(const BeamPathList& paths);
    void setDroneState(const DroneState& state);
    void setSelectedBaseStationIndex(int index);
    void setBaseStationsVisible(bool visible);
    void setDroneVisible(bool visible);
    void setTrajectoryVisible(bool visible);
    void setPathsVisible(bool visible);
    void setViewGizmoVisible(bool visible);

    void setHost(const QString& value);
    void setPort(int value);
    void setCameraName(const QString& value);
    void setVehicleName(const QString& value);
    void setFollowVehicle(bool value);
    void setFramesPerSecond(double value);
    void setAnchoredCamera(bool value);
    void setAnchorPoint(const Vec3& value);
    void setViewAngles(double yawDeg, double pitchDeg);
    void setViewLabel(const QString& value);
    void setDisplayBrightness(double value);
    void setDepthFetchEnabled(bool value);
    void setRosPublishingEnabled(bool value);
    void setRosImageTopic(const QString& topicName);
    void setRosFrameId(const QString& frameId);
    bool hasRosImageSubscribers() const;
    void setRosImageSubscriptionTopic(const QString& topicName);
    void setRosImageSubscriptionEnabled(bool value);
    void setExternalPoseControlEnabled(bool value);
    void setScriptPath(const QString& scriptPath) { Q_UNUSED(scriptPath) }
    void setPythonExecutable(const QString& pythonExecutable) { Q_UNUSED(pythonExecutable) }
    void setFrameTransformManager(const std::shared_ptr<FrameTransformManager>& manager);
    void setAirsimToGuiTransformPath(const QString& filePath);
    QString airsimToGuiTransformPath() const { return transformMatrixPath_; }

    Q_INVOKABLE void start();
    Q_INVOKABLE void stop();
    Q_INVOKABLE void restart();
    Q_INVOKABLE void pause();
    Q_INVOKABLE void resume();
    Q_INVOKABLE void orbitCamera(double dx, double dy, int buttons, bool shiftHeld);
    Q_INVOKABLE void setViewPreset(const QString& preset);
    Q_INVOKABLE void rollViewQuarter(int direction);
    Q_INVOKABLE void zoomCamera(double wheelDelta);
    Q_INVOKABLE void resetCamera();
    Q_INVOKABLE void nudgePan(double dx, double dy);
    Q_INVOKABLE void requestStationSelection(int index);
    Q_INVOKABLE QString stationInfoText(int index) const;
    Q_INVOKABLE QVariantMap worldPointFromNormalized(double nx, double ny) const;
    Q_INVOKABLE double depthAtNormalized(double nx, double ny) const;

signals:
    void frameTokenChanged();
    void statusTextChanged();
    void runningChanged();
    void connectedChanged();
    void configChanged();
    void cameraStateChanged();
    void helperMessage(const QString& text);
    void overlayDataChanged();
    void stationSelectionRequested(int index);
    void anchorPointChanged();
    void cameraPositionChanged();
    void viewLabelChanged();
    void displayBrightnessChanged();
    void viewGizmoVisibleChanged();

private:
    struct SharedState {
        QString host{QStringLiteral("127.0.0.1")};
        int port{41451};
        QString cameraName{QStringLiteral("front_center")};
        QString vehicleName;
        bool followVehicle{false};
        bool anchoredCamera{false};
        double anchorX{0.0};
        double anchorY{0.0};
        double anchorZ{0.0};
        double fps{10.0};
        double yawDeg{0.0};
        double pitchDeg{-18.0};
        double rollDeg{0.0};
        double distance{45.0};
        double focusX{0.0};
        double focusY{0.0};
        double focusZ{0.0};
        bool focusInitialized{false};
        bool resetRequested{false};
        bool configDirty{true};
        bool depthFetchEnabled{false};
        bool externalPoseControlEnabled{false};
        int externalPoseControlRevision{0};
    };

    struct StationCameraPoseRequest {
        QString cameraName;
        Vec3 position;
        bool hasTarget{false};
        Vec3 target;
    };

    void ensureScratchPaths();
    void ensureCoordinateTransformLoaded();
    void rebuildStationCameraPoseRequests();
    void startExternalPoseControlWorker();
    void stopExternalPoseControlWorker();
    void externalPoseControlLoop();
    void workerLoop();
    void setStatusText(const QString& text);
    void rosImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void ensureRosImageSubscriber();
    void shutdownRosImageSubscriber();
    void applyFrameUpdate(const QImage& image,
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
                          const QString& message);
    void rebuildOverlayData();
    bool projectWorldPoint(const Vec3& point, int frameWidth, int frameHeight, QPointF* outNormalized) const;
    Vec3 guiToAirSimPoint(const Vec3& guiPoint) const;
    Vec3 airSimToGuiPoint(const Vec3& airsimPoint) const;

    QTemporaryDir tempDir_;
    mutable std::mutex frameMutex_;
    QImage latestFrame_;
    int frameToken_{0};
    QString statusText_{QStringLiteral("Idle")};
    QString host_{QStringLiteral("127.0.0.1")};
    int port_{41451};
    QString cameraName_{QStringLiteral("front_center")};
    QString vehicleName_;
    QString viewLabel_{QStringLiteral("AirSim Camera")};
    bool followVehicle_{false};
    bool anchoredCamera_{false};
    Vec3 anchorPoint_{};
    double yawDeg_{0.0};
    double pitchDeg_{-18.0};
    double rollDeg_{0.0};
    double distance_{45.0};
    double framesPerSecond_{10.0};
    double displayBrightness_{1.0};
    bool depthFetchEnabled_{false};
    bool connected_{false};
    double focusX_{0.0};
    double focusY_{0.0};
    double focusZ_{0.0};
    std::vector<BaseStation> baseStations_;
    BeamPathList beamPaths_;
    DroneState droneState_;
    int selectedBaseStationIndex_{-1};
    bool baseStationsVisible_{true};
    bool droneVisible_{true};
    bool trajectoryVisible_{true};
    bool pathsVisible_{true};
    bool viewGizmoVisible_{true};
    QVariantList stationOverlays_;
    QVariantList pathOverlays_;
    QVariantMap droneOverlay_;

    mutable std::mutex transformMutex_;
    TransformMatrix4x4 airsimToGuiTransform_{};
    TransformMatrix4x4 guiToAirsimTransform_{};
    QString transformMatrixPath_;
    bool transformLoaded_{false};
    std::shared_ptr<FrameTransformManager> frameTransformManager_;


    mutable std::mutex geometryMutex_;
    Vec3 cameraPosition_{};
    Vec3 cameraRight_{0.0, 1.0, 0.0};
    Vec3 cameraUp_{0.0, 0.0, 1.0};
    Vec3 cameraForward_{1.0, 0.0, 0.0};
    double horizontalFovDeg_{90.0};
    int sceneWidth_{0};
    int sceneHeight_{0};
    int depthWidth_{0};
    int depthHeight_{0};
    std::vector<float> depthValues_;
    bool depthAvailable_{false};
    bool cameraGeometryValid_{false};

    mutable std::mutex stationCameraPoseMutex_;
    std::vector<StationCameraPoseRequest> stationCameraPoseRequests_;
    int stationCameraPoseRevision_{0};

    mutable std::mutex stateMutex_;
    SharedState sharedState_;
    std::atomic<bool> workerStop_{false};
    std::atomic<bool> workerRunning_{false};
    std::thread workerThread_;

    std::atomic<bool> externalPoseControlStop_{false};
    std::atomic<bool> externalPoseControlRunning_{false};
    std::thread externalPoseControlThread_;
    bool externalPoseControlEnabled_{false};

    bool rosPublishingEnabled_{false};
    QString rosImageTopic_;
    QString rosFrameId_;
    std::unique_ptr<ros::NodeHandle> rosNodeHandle_;
    ros::Publisher rosImagePublisher_;
    ros::Publisher rosCameraInfoPublisher_;
    ros::Publisher rosCameraPosePublisher_;

    QString rosImageSubscribeTopic_;
    bool rosImageSubscriptionEnabled_{false};
    bool rosImageSubscriptionActive_{false};
    std::unique_ptr<ros::NodeHandle> rosImageSubscribeNodeHandle_;
    std::unique_ptr<ros::AsyncSpinner> rosImageSubscribeSpinner_;
    ros::Subscriber rosImageSubscriber_;
    std::chrono::steady_clock::time_point rosImageSubscribeStartTime_{};
    std::chrono::steady_clock::time_point rosImageLastFrameTime_{};

    void publishRosFrame(const QImage& image);
    void ensureRosPublisher();
    void shutdownRosPublisher();
};

}  // namespace airsim_gui
