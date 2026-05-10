#pragma once

#include "airsim_gui_UErealtime/AppTypes.h"
#include "airsim_gui_UErealtime/TransformUtils.h"

#include <QString>
#include <QStringList>

#include <memory>
#include <mutex>

namespace tf2_ros {
class Buffer;
}

namespace airsim_gui {

enum class CoordinateOutputFrame {
    Ros = 0,
    Scene3D = 1,
    AirSim = 2,
};

QString coordinateOutputFrameToKey(CoordinateOutputFrame frame);
CoordinateOutputFrame coordinateOutputFrameFromKey(const QString& key);

class FrameTransformManager {
public:
    FrameTransformManager();
    ~FrameTransformManager();

    void setRosFrameId(const QString& frameId);
    QString rosFrameId() const;
    QString resolvedRosFrameId() const;

    void setRosToSceneTransformPath(const QString& filePath);
    void setAirSimToSceneTransformPath(const QString& filePath);
    void setRxArrayFacingDirection(const QString& facingDirection);
    void setUavToRxArrayOffset(double tx, double ty, double tz);
    void setUavToRxArrayCenterTransformPath(const QString& filePath);
    void setRxArrayElementsTransformPath(const QString& filePath);
    QString rosToSceneTransformPath() const;
    QString airsimToSceneTransformPath() const;
    QString rxArrayFacingDirection() const;
    Vec3 uavToRxArrayOffset() const;
    QString uavToRxArrayCenterTransformPath() const;
    QString rxArrayElementsTransformPath() const;

    void setCurrentDronePoseInScene(const Vec3& position,
                                    double rollDeg,
                                    double pitchDeg,
                                    double yawDeg);
    void clearCurrentDronePoseInScene();
    void setRxArrayGeometry(int numRows,
                            int numCols,
                            double verticalSpacingLambda,
                            double horizontalSpacingLambda,
                            double carrierFrequencyHz);

    static QString sceneFrameId();
    static QString airsimFrameId();
    static QString uavComFrameId();
    static QString rxArrayCenterFrameId();
    QString frameIdForOutput(CoordinateOutputFrame outputFrame) const;
    QStringList antennaFrameIds() const;

    bool transformPoint(const Vec3& input,
                        const QString& sourceFrameId,
                        const QString& targetFrameId,
                        Vec3* output,
                        QString* errorMessage = nullptr) const;
    bool transformVector(const Vec3& input,
                         const QString& sourceFrameId,
                         const QString& targetFrameId,
                         Vec3* output,
                         QString* errorMessage = nullptr) const;
    bool transformQuaternion(double x, double y, double z, double w,
                             const QString& sourceFrameId,
                             const QString& targetFrameId,
                             double* outX,
                             double* outY,
                             double* outZ,
                             double* outW,
                             QString* errorMessage = nullptr) const;

private:
    static QString normalizeFrameId(const QString& frameId, const QString& fallback = QString());
    void ensureBufferReady(QString* errorMessage = nullptr) const;

    mutable std::mutex mutex_;
    QString rosFrameId_{QStringLiteral("ROS")};
    QString rosToSceneTransformPath_;
    QString airsimToSceneTransformPath_;
    QString rxArrayFacingDirection_{QStringLiteral("front")};
    double uavToRxArrayTx_{0.0};
    double uavToRxArrayTy_{0.0};
    double uavToRxArrayTz_{0.0};
    QString uavToRxArrayCenterTransformPath_;
    QString rxArrayElementsTransformPath_;
    Vec3 currentDronePosition_{};
    double currentDroneRollDeg_{0.0};
    double currentDronePitchDeg_{0.0};
    double currentDroneYawDeg_{0.0};
    bool hasCurrentDronePose_{false};
    int rxArrayNumRows_{1};
    int rxArrayNumCols_{1};
    double rxArrayVerticalSpacingLambda_{0.5};
    double rxArrayHorizontalSpacingLambda_{0.5};
    double carrierFrequencyHz_{3.5e9};
    mutable QStringList antennaFrameIds_;
    mutable std::unique_ptr<tf2_ros::Buffer> buffer_;
    mutable bool bufferDirty_{true};
    mutable QString lastBuildError_;
};

}  // namespace airsim_gui
