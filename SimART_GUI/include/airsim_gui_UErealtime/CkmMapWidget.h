#pragma once

#include <QImage>
#include <QJsonObject>
#include <QMap>
#include <QPixmap>
#include <QPointF>
#include <QRect>
#include <QSize>
#include <QString>
#include <QVector>
#include <QVector3D>
#include <QWidget>

class QComboBox;
class QLabel;
class QPushButton;
class QScrollArea;

namespace airsim_gui {

class CkmMapWidget : public QWidget {
    Q_OBJECT
public:
    explicit CkmMapWidget(QWidget* parent = nullptr);

    bool loadFromMetaFile(const QString& metaFilePath, QString* errorMessage = nullptr);
    void clearResult(const QString& placeholderText = QString());
    QString currentMetaFilePath() const;
    void setGenerationRunning(bool running);

signals:
    void generateRequested();
    void stopRequested();
    void loadLocalRequested();
    void pointSelectionChanged(const QString& title, const QString& summary, const QString& details);
    void pointSelectionCleared(const QString& hint);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    struct SceneClickMapping {
        bool valid{false};
        bool hasCamera{false};
        bool hasExtent{false};
        bool hasQuad{false};
        QSize imageSize;
        double planeZ{0.0};
        double xMin{0.0};
        double xMax{0.0};
        double yMin{0.0};
        double yMax{0.0};
        double fovDeg{36.0};
        QVector3D cameraOrigin;
        QVector3D cameraTarget;
        QVector3D cameraUp;
        QVector<QPointF> quadImage;
        QVector<QPointF> quadWorld;
        QRect displayContentRect;
    };

    struct ScenePickMap {
        bool valid{false};
        QSize imageSize;
        double xMin{0.0};
        double xMax{0.0};
        double yMin{0.0};
        double yMax{0.0};
        QString xImagePath;
        QString yImagePath;
        QRect displayContentRect;
    };

    struct MetricEntry {
        QString metric;
        QString displayName;
        QString heatmapImagePath;
        QString sceneImagePath;
        QString npzPath;
        QString samplesJsonPath;
        double xMin{0.0};
        double xMax{0.0};
        double yMin{0.0};
        double yMax{0.0};
        double zFixed{0.0};
        bool hasExtent{false};
        SceneClickMapping sceneClickMapping;
        ScenePickMap scenePickMap;
    };

    struct SamplePoint {
        double x{0.0};
        double y{0.0};
        double metricValue{0.0};
        bool hasMetricValue{false};
        QJsonObject payload;
    };

    void setPlaceholderText(const QString& text);
    void applyMetricSelection(const QString& metric);
    void rebuildViewModeCombo();
    void setZoomFactor(double factor);
    void zoomBy(double multiplier);
    QString currentDisplayImagePath(const MetricEntry& entry, QString* resolvedMode = nullptr) const;
    void updateDisplayedPixmap();
    bool mapWidgetPointToWorld(const QPointF& widgetPoint, QPointF* worldPoint, QPointF* imagePoint = nullptr) const;
    bool mapImagePointToWorld(const QPointF& imagePoint, QPointF* worldPoint) const;
    bool mapImagePointToWorldFromPickMap(const QPointF& imagePoint, QPointF* worldPoint) const;
    bool mapWorldPointToImage(const QPointF& worldPoint, QPointF* imagePoint) const;
    bool selectNearestPoint(const QPointF& worldPoint, const QPointF* imagePoint = nullptr);
    QString formatValueWithMetric(const QString& metric, double value) const;
    QString formatJsonValue(const QJsonValue& value) const;
    void emitSelectionClearedHint(const QString& hint);

    QLabel* titleLabel_{nullptr};
    QLabel* infoLabel_{nullptr};
    QPushButton* generateButton_{nullptr};
    QPushButton* stopButton_{nullptr};
    QPushButton* loadLocalButton_{nullptr};
    QPushButton* zoomOutButton_{nullptr};
    QPushButton* zoomResetButton_{nullptr};
    QPushButton* zoomInButton_{nullptr};
    QComboBox* metricCombo_{nullptr};
    QComboBox* viewModeCombo_{nullptr};
    QLabel* imageLabel_{nullptr};
    QScrollArea* scrollArea_{nullptr};
    QString currentMetaFilePath_;
    QString commonInfoText_;
    QString defaultMetric_;
    QMap<QString, MetricEntry> metricEntries_;
    QMap<QString, QVector<SamplePoint>> metricSamples_;
    bool updatingMetricCombo_{false};
    bool updatingViewModeCombo_{false};
    double xMin_{0.0};
    double xMax_{1.0};
    double yMin_{0.0};
    double yMax_{1.0};
    double zFixed_{0.0};
    bool hasExtent_{false};
    QString currentMetric_;
    QString currentViewMode_;
    QString currentImagePath_;
    bool currentImageClickable_{false};
    QPixmap basePixmap_;
    SceneClickMapping currentSceneClickMapping_;
    ScenePickMap currentScenePickMap_;
    QImage currentScenePickMapX_;
    QImage currentScenePickMapY_;
    bool hasSelectedPoint_{false};
    QPointF selectedWorldPoint_;
    bool hasSelectedImagePoint_{false};
    QPointF selectedImagePoint_;
    bool generationRunning_{false};
    double zoomFactor_{1.0};
    QSize displayedPixmapSize_;
};

}  // namespace airsim_gui
