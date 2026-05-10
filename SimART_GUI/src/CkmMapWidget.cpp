#include "airsim_gui_UErealtime/CkmMapWidget.h"

#include <QComboBox>
#include <QDir>
#include <QEvent>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMouseEvent>
#include <QSizePolicy>
#include <QWheelEvent>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QPolygonF>
#include <QScrollArea>
#include <QSet>
#include <QTransform>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

namespace airsim_gui {

namespace {
QString ckmMetricUiKey(const QString& metric) {
    const QString key = metric.trimmed();
    if (key == QLatin1String("beam_deepsense_index")) {
        return QStringLiteral("beam_predicted_index");
    }
    if (key == QLatin1String("beam_deepsense_gain_db")) {
        return QStringLiteral("beam_predicted_gain_db");
    }
    return key;
}

QString sanitizeCkmDisplayText(QString text) {
    text.replace(QStringLiteral("DeepSense selected beam"), QStringLiteral("Predicted beam"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSense-beam"), QStringLiteral("Predicted-beam"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSense6G-style"), QStringLiteral("beam-selection"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSense-style"), QStringLiteral("beam-selection"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSenseBeamPredictor"), QStringLiteral("BeamSelectionPredictor"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSenseCheckpointMetadata"), QStringLiteral("BeamSelectionCheckpointMetadata"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("train_deepsense_beam_model.py"), QStringLiteral("train_beam_selection_model.py"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("deepsense_beam_selector.py"), QStringLiteral("beam_selection_model.py"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("deepsense_predictor"), QStringLiteral("learned_selector"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("deepsense_pos_bbox4"), QStringLiteral("pos_bbox4"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("beam_deepsense"), QStringLiteral("beam_predicted"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSense"), QStringLiteral("Beam selection"), Qt::CaseInsensitive);
    return text;
}

QString metricDisplayNameFromObject(const QJsonObject& obj) {
    const QString displayName = obj.value(QStringLiteral("display_name")).toString().trimmed();
    if (!displayName.isEmpty()) {
        return sanitizeCkmDisplayText(displayName);
    }
    return ckmMetricUiKey(obj.value(QStringLiteral("metric")).toString());
}

QString boolText(bool value) {
    return value ? QStringLiteral("true") : QStringLiteral("false");
}

QString compactNumber(double value, int precision = 3) {
    return QString::number(value, 'f', precision);
}

bool parsePoint2(const QJsonValue& value, QPointF* point) {
    if (!point || !value.isArray()) {
        return false;
    }
    const QJsonArray arr = value.toArray();
    if (arr.size() < 2 || !arr.at(0).isDouble() || !arr.at(1).isDouble()) {
        return false;
    }
    *point = QPointF(arr.at(0).toDouble(), arr.at(1).toDouble());
    return std::isfinite(point->x()) && std::isfinite(point->y());
}

bool parseVec3(const QJsonValue& value, QVector3D* vec) {
    if (!vec || !value.isArray()) {
        return false;
    }
    const QJsonArray arr = value.toArray();
    if (arr.size() < 3 || !arr.at(0).isDouble() || !arr.at(1).isDouble() || !arr.at(2).isDouble()) {
        return false;
    }
    *vec = QVector3D(
        static_cast<float>(arr.at(0).toDouble()),
        static_cast<float>(arr.at(1).toDouble()),
        static_cast<float>(arr.at(2).toDouble()));
    return std::isfinite(vec->x()) && std::isfinite(vec->y()) && std::isfinite(vec->z());
}

bool parseImageRect(const QJsonValue& value, QRect* rect) {
    if (!rect || !value.isObject()) {
        return false;
    }
    const QJsonObject obj = value.toObject();
    const int x = obj.value(QStringLiteral("x")).toInt(std::numeric_limits<int>::min());
    const int y = obj.value(QStringLiteral("y")).toInt(std::numeric_limits<int>::min());
    const int width = obj.value(QStringLiteral("width")).toInt(-1);
    const int height = obj.value(QStringLiteral("height")).toInt(-1);
    if (x == std::numeric_limits<int>::min() || y == std::numeric_limits<int>::min() || width <= 0 || height <= 0) {
        return false;
    }
    *rect = QRect(x, y, width, height);
    return true;
}

QVector<QPointF> parsePointArray(const QJsonValue& value) {
    QVector<QPointF> out;
    if (!value.isArray()) {
        return out;
    }
    const QJsonArray arr = value.toArray();
    out.reserve(arr.size());
    for (const QJsonValue& item : arr) {
        QPointF point;
        if (parsePoint2(item, &point)) {
            out.push_back(point);
        }
    }
    return out;
}

QPolygonF toPolygon(const QVector<QPointF>& points) {
    QPolygonF poly;
    for (const QPointF& point : points) {
        poly << point;
    }
    return poly;
}

bool pointNearSegment(const QPointF& p, const QPointF& a, const QPointF& b, double tolPx) {
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    const double lenSq = dx * dx + dy * dy;
    if (lenSq <= 1e-12) {
        const double ex = p.x() - a.x();
        const double ey = p.y() - a.y();
        return (ex * ex + ey * ey) <= tolPx * tolPx;
    }
    const double t = std::max(0.0, std::min(1.0, ((p.x() - a.x()) * dx + (p.y() - a.y()) * dy) / lenSq));
    const double projX = a.x() + t * dx;
    const double projY = a.y() + t * dy;
    const double ex = p.x() - projX;
    const double ey = p.y() - projY;
    return (ex * ex + ey * ey) <= tolPx * tolPx;
}

bool pointInOrNearQuad(const QVector<QPointF>& quad, const QPointF& point, double tolPx = 2.5) {
    if (quad.size() < 4) {
        return false;
    }
    const QPolygonF poly = toPolygon(quad);
    if (poly.containsPoint(point, Qt::OddEvenFill)) {
        return true;
    }
    for (int i = 0; i < quad.size(); ++i) {
        if (pointNearSegment(point, quad.at(i), quad.at((i + 1) % quad.size()), tolPx)) {
            return true;
        }
    }
    return false;
}

bool quadToQuadTransform(const QVector<QPointF>& srcQuad, const QVector<QPointF>& dstQuad, QTransform* transform) {
    if (!transform || srcQuad.size() < 4 || dstQuad.size() < 4) {
        return false;
    }
    return QTransform::quadToQuad(toPolygon(srcQuad), toPolygon(dstQuad), *transform);
}

QRect effectiveContentRect(const QRect& requestedRect, const QSize& displaySize) {
    QRect rect = requestedRect;
    if (!rect.isValid()) {
        rect = QRect(QPoint(0, 0), displaySize);
    }
    rect = rect.intersected(QRect(QPoint(0, 0), displaySize));
    if (!rect.isValid() || rect.width() <= 0 || rect.height() <= 0) {
        rect = QRect(QPoint(0, 0), displaySize);
    }
    return rect;
}

double clampZoomFactor(double factor) {
    return std::max(0.25, std::min(factor, 8.0));
}

QSize scaledSizeForZoom(const QSize& baseSize, double zoomFactor) {
    if (!baseSize.isValid()) {
        return QSize();
    }
    const double clamped = clampZoomFactor(zoomFactor);
    return QSize(
        std::max(1, static_cast<int>(std::lround(static_cast<double>(baseSize.width()) * clamped))),
        std::max(1, static_cast<int>(std::lround(static_cast<double>(baseSize.height()) * clamped))));
}

QPointF scalePointBetweenSizes(const QPointF& point, const QSize& fromSize, const QSize& toSize) {
    if (!fromSize.isValid() || !toSize.isValid()) {
        return point;
    }
    const double x = point.x() * static_cast<double>(toSize.width()) / std::max(1.0, static_cast<double>(fromSize.width()));
    const double y = point.y() * static_cast<double>(toSize.height()) / std::max(1.0, static_cast<double>(fromSize.height()));
    return QPointF(x, y);
}

bool mapDisplayPointToImagePoint(const QPointF& displayPoint,
                                 const QSize& displaySize,
                                 const QRect& requestedContentRect,
                                 const QSize& imageSize,
                                 QPointF* imagePoint) {
    if (!imagePoint || !displaySize.isValid() || !imageSize.isValid()) {
        return false;
    }
    const QRect contentRect = effectiveContentRect(requestedContentRect, displaySize);
    const QRectF contentRectF(contentRect);
    if (!contentRectF.contains(displayPoint)) {
        return false;
    }
    const double localX = displayPoint.x() - contentRectF.left();
    const double localY = displayPoint.y() - contentRectF.top();
    const double scaleX = static_cast<double>(imageSize.width()) / std::max(1.0, static_cast<double>(contentRect.width()));
    const double scaleY = static_cast<double>(imageSize.height()) / std::max(1.0, static_cast<double>(contentRect.height()));
    imagePoint->setX(std::max(0.0, std::min(localX * scaleX, static_cast<double>(imageSize.width() - 1))));
    imagePoint->setY(std::max(0.0, std::min(localY * scaleY, static_cast<double>(imageSize.height() - 1))));
    return true;
}

QPointF mapImagePointToDisplayPoint(const QPointF& imagePoint,
                                    const QSize& imageSize,
                                    const QSize& displaySize,
                                    const QRect& requestedContentRect) {
    if (!imageSize.isValid() || !displaySize.isValid()) {
        return imagePoint;
    }
    const QRect contentRect = effectiveContentRect(requestedContentRect, displaySize);
    const double scaleX = static_cast<double>(contentRect.width()) / std::max(1.0, static_cast<double>(imageSize.width()));
    const double scaleY = static_cast<double>(contentRect.height()) / std::max(1.0, static_cast<double>(imageSize.height()));
    return QPointF(contentRect.left() + imagePoint.x() * scaleX,
                   contentRect.top() + imagePoint.y() * scaleY);
}
}

CkmMapWidget::CkmMapWidget(QWidget* parent)
    : QWidget(parent) {
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    titleLabel_ = new QLabel(tr("Dense CKM Map"), this);
    titleLabel_->setStyleSheet(QStringLiteral("font-size: 18px; font-weight: 700;"));
    titleLabel_->setWordWrap(true);
    rootLayout->addWidget(titleLabel_);

    auto* controlsRow = new QWidget(this);
    auto* controlsRowLayout = new QHBoxLayout(controlsRow);
    controlsRowLayout->setContentsMargins(0, 0, 0, 0);
    controlsRowLayout->setSpacing(8);

    generateButton_ = new QPushButton(tr("Generate Dense CKM"), controlsRow);
    generateButton_->setToolTip(tr("Generate a new Dense CKM version using the current simulation settings."));
    stopButton_ = new QPushButton(tr("Stop"), controlsRow);
    stopButton_->setToolTip(tr("Stop the current Dense CKM generation and delete the unfinished output files."));
    loadLocalButton_ = new QPushButton(tr("Load Local CKM"), controlsRow);
    loadLocalButton_->setToolTip(tr("Load an existing local Dense CKM version folder."));
    controlsRowLayout->addWidget(generateButton_);
    controlsRowLayout->addWidget(stopButton_);
    controlsRowLayout->addWidget(loadLocalButton_);

    auto* metricLabel = new QLabel(tr("Metric"), controlsRow);
    metricCombo_ = new QComboBox(controlsRow);
    metricCombo_->setMinimumContentsLength(24);
    controlsRowLayout->addWidget(metricLabel);
    controlsRowLayout->addWidget(metricCombo_, 1);

    auto* viewLabel = new QLabel(tr("View"), controlsRow);
    viewModeCombo_ = new QComboBox(controlsRow);
    viewModeCombo_->setMinimumContentsLength(18);
    controlsRowLayout->addWidget(viewLabel);
    controlsRowLayout->addWidget(viewModeCombo_);

    controlsRowLayout->addSpacing(6);
    auto* zoomLabel = new QLabel(tr("Zoom"), controlsRow);
    zoomOutButton_ = new QPushButton(tr("-"), controlsRow);
    zoomResetButton_ = new QPushButton(tr("100%"), controlsRow);
    zoomInButton_ = new QPushButton(tr("+"), controlsRow);
    zoomOutButton_->setToolTip(tr("Zoom out the CKM image."));
    zoomResetButton_->setToolTip(tr("Reset CKM zoom to 100%."));
    zoomInButton_->setToolTip(tr("Zoom in on the CKM image."));
    zoomOutButton_->setFixedWidth(34);
    zoomInButton_->setFixedWidth(34);
    zoomResetButton_->setMinimumWidth(58);
    controlsRowLayout->addWidget(zoomLabel);
    controlsRowLayout->addWidget(zoomOutButton_);
    controlsRowLayout->addWidget(zoomResetButton_);
    controlsRowLayout->addWidget(zoomInButton_);

    rootLayout->addWidget(controlsRow);

    infoLabel_ = new QLabel(this);
    infoLabel_->setWordWrap(true);
    infoLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    infoLabel_->setStyleSheet(QStringLiteral("color: #4b5563;"));
    rootLayout->addWidget(infoLabel_);

    scrollArea_ = new QScrollArea(this);
    scrollArea_->setWidgetResizable(false);
    scrollArea_->setAlignment(Qt::AlignCenter);
    imageLabel_ = new QLabel(scrollArea_);
    imageLabel_->setAlignment(Qt::AlignCenter);
    imageLabel_->setMinimumSize(320, 220);
    imageLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    imageLabel_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    imageLabel_->setStyleSheet(QStringLiteral(
        "QLabel {"
        "background: #0f172a;"
        "color: #cbd5e1;"
        "border: 1px solid #1e293b;"
        "padding: 18px;"
        "}"
    ));
    imageLabel_->installEventFilter(this);
    scrollArea_->setWidget(imageLabel_);
    rootLayout->addWidget(scrollArea_, 1);

    connect(generateButton_, &QPushButton::clicked, this, &CkmMapWidget::generateRequested);
    connect(stopButton_, &QPushButton::clicked, this, &CkmMapWidget::stopRequested);
    connect(loadLocalButton_, &QPushButton::clicked, this, &CkmMapWidget::loadLocalRequested);
    connect(zoomOutButton_, &QPushButton::clicked, this, [this]() { zoomBy(1.0 / 1.2); });
    connect(zoomResetButton_, &QPushButton::clicked, this, [this]() { setZoomFactor(1.0); });
    connect(zoomInButton_, &QPushButton::clicked, this, [this]() { zoomBy(1.2); });

    connect(metricCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (updatingMetricCombo_ || index < 0) {
            return;
        }
        rebuildViewModeCombo();
        applyMetricSelection(metricCombo_->itemData(index).toString());
    });
    connect(viewModeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (updatingViewModeCombo_ || index < 0) {
            return;
        }
        const int metricIndex = metricCombo_->currentIndex();
        if (metricIndex < 0) {
            return;
        }
        applyMetricSelection(metricCombo_->itemData(metricIndex).toString());
    });

    setGenerationRunning(false);
    clearResult(tr("Dense CKM result not loaded yet. Generate Dense CKM on this page or load a local CKM version folder."));
}

bool CkmMapWidget::loadFromMetaFile(const QString& metaFilePath, QString* errorMessage) {
    if (errorMessage) {
        errorMessage->clear();
    }

    QFile file(metaFilePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = tr("Could not open CKM metadata file: %1").arg(metaFilePath);
        }
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage) {
            *errorMessage = tr("CKM metadata parse failed: %1").arg(parseError.errorString());
        }
        return false;
    }

    const QJsonObject obj = doc.object();
    const QFileInfo metaInfo(metaFilePath);
    const QDir metaDir = metaInfo.absoluteDir();

    metricEntries_.clear();
    metricSamples_.clear();
    commonInfoText_.clear();
    defaultMetric_.clear();
    hasExtent_ = false;
    hasSelectedPoint_ = false;
    basePixmap_ = QPixmap();
    currentMetric_.clear();
    currentImagePath_.clear();
    currentViewMode_.clear();
    currentImageClickable_ = false;
    currentSceneClickMapping_ = SceneClickMapping();
    currentScenePickMap_ = ScenePickMap();
    currentScenePickMapX_ = QImage();
    currentScenePickMapY_ = QImage();
    hasSelectedImagePoint_ = false;

    auto normalizePath = [&metaDir](const QString& rawPath) {
        QString path = rawPath.trimmed();
        if (path.isEmpty()) {
            return QString();
        }
        if (QDir::isRelativePath(path)) {
            path = metaDir.filePath(path);
        }
        return QFileInfo(path).absoluteFilePath();
    };

    xMin_ = obj.value(QStringLiteral("x_min")).toDouble(0.0);
    xMax_ = obj.value(QStringLiteral("x_max")).toDouble(0.0);
    yMin_ = obj.value(QStringLiteral("y_min")).toDouble(0.0);
    yMax_ = obj.value(QStringLiteral("y_max")).toDouble(0.0);
    zFixed_ = obj.value(QStringLiteral("z_fixed")).toDouble(0.0);
    hasExtent_ = std::isfinite(xMin_) && std::isfinite(xMax_) && std::isfinite(yMin_) && std::isfinite(yMax_) && xMax_ > xMin_ && yMax_ > yMin_;

    auto populateMetricEntry = [&](const QJsonObject& sourceObj, MetricEntry* entry) {
        if (!entry) {
            return;
        }
        entry->displayName = metricDisplayNameFromObject(sourceObj);
        entry->heatmapImagePath = normalizePath(sourceObj.value(QStringLiteral("image_path")).toString());
        if (entry->heatmapImagePath.isEmpty()) {
            entry->heatmapImagePath = normalizePath(sourceObj.value(QStringLiteral("png_path")).toString());
        }
        entry->sceneImagePath = normalizePath(sourceObj.value(QStringLiteral("scene_image_path")).toString());
        entry->npzPath = normalizePath(sourceObj.value(QStringLiteral("npz_path")).toString());
        entry->samplesJsonPath = normalizePath(sourceObj.value(QStringLiteral("samples_json_path")).toString());
        entry->xMin = sourceObj.value(QStringLiteral("x_min")).toDouble(xMin_);
        entry->xMax = sourceObj.value(QStringLiteral("x_max")).toDouble(xMax_);
        entry->yMin = sourceObj.value(QStringLiteral("y_min")).toDouble(yMin_);
        entry->yMax = sourceObj.value(QStringLiteral("y_max")).toDouble(yMax_);
        entry->zFixed = sourceObj.value(QStringLiteral("z_fixed")).toDouble(zFixed_);
        entry->hasExtent = std::isfinite(entry->xMin) && std::isfinite(entry->xMax)
                        && std::isfinite(entry->yMin) && std::isfinite(entry->yMax)
                        && entry->xMax > entry->xMin && entry->yMax > entry->yMin;

        SceneClickMapping mapping;
        QJsonObject mappingObj;
        if (sourceObj.contains(QStringLiteral("scene_click_mapping")) && sourceObj.value(QStringLiteral("scene_click_mapping")).isObject()) {
            mappingObj = sourceObj.value(QStringLiteral("scene_click_mapping")).toObject();
        } else if (obj.contains(QStringLiteral("scene_click_mapping")) && obj.value(QStringLiteral("scene_click_mapping")).isObject()) {
            mappingObj = obj.value(QStringLiteral("scene_click_mapping")).toObject();
        }
        if (!mappingObj.isEmpty()) {
            mapping.imageSize = QSize(mappingObj.value(QStringLiteral("image_width")).toInt(),
                                      mappingObj.value(QStringLiteral("image_height")).toInt());
            mapping.planeZ = mappingObj.value(QStringLiteral("plane_z")).toDouble(entry->zFixed);
            mapping.xMin = mappingObj.value(QStringLiteral("x_min")).toDouble(entry->xMin);
            mapping.xMax = mappingObj.value(QStringLiteral("x_max")).toDouble(entry->xMax);
            mapping.yMin = mappingObj.value(QStringLiteral("y_min")).toDouble(entry->yMin);
            mapping.yMax = mappingObj.value(QStringLiteral("y_max")).toDouble(entry->yMax);
            mapping.hasExtent = std::isfinite(mapping.xMin) && std::isfinite(mapping.xMax)
                             && std::isfinite(mapping.yMin) && std::isfinite(mapping.yMax)
                             && mapping.xMax > mapping.xMin && mapping.yMax > mapping.yMin;

            const QJsonObject cameraObj = mappingObj.value(QStringLiteral("camera")).toObject();
            if (!cameraObj.isEmpty()) {
                QVector3D origin;
                QVector3D target;
                QVector3D up;
                const bool okOrigin = parseVec3(cameraObj.value(QStringLiteral("origin")), &origin);
                const bool okTarget = parseVec3(cameraObj.value(QStringLiteral("target")), &target);
                const bool okUp = parseVec3(cameraObj.value(QStringLiteral("up")), &up);
                const double fovDeg = cameraObj.value(QStringLiteral("fov")).toDouble(36.0);
                if (okOrigin && okTarget && okUp && std::isfinite(fovDeg) && fovDeg > 1e-6) {
                    mapping.cameraOrigin = origin;
                    mapping.cameraTarget = target;
                    mapping.cameraUp = up;
                    mapping.fovDeg = fovDeg;
                    mapping.hasCamera = true;
                }
            }

            parseImageRect(mappingObj.value(QStringLiteral("display_content_rect")), &mapping.displayContentRect);
            mapping.quadImage = parsePointArray(mappingObj.value(QStringLiteral("quad_image_xy")));
            mapping.quadWorld = parsePointArray(mappingObj.value(QStringLiteral("world_corners_xy")));
            mapping.hasQuad = mapping.quadImage.size() >= 4 && mapping.quadWorld.size() >= 4;
            mapping.valid = mapping.hasExtent && ((mapping.hasCamera && mapping.imageSize.isValid()) || mapping.hasQuad);
        }
        entry->sceneClickMapping = mapping;

        ScenePickMap pickMap;
        QJsonObject pickMapObj;
        if (sourceObj.contains(QStringLiteral("scene_pick_map")) && sourceObj.value(QStringLiteral("scene_pick_map")).isObject()) {
            pickMapObj = sourceObj.value(QStringLiteral("scene_pick_map")).toObject();
        } else if (obj.contains(QStringLiteral("scene_pick_map")) && obj.value(QStringLiteral("scene_pick_map")).isObject()) {
            pickMapObj = obj.value(QStringLiteral("scene_pick_map")).toObject();
        }
        if (!pickMapObj.isEmpty()) {
            pickMap.imageSize = QSize(pickMapObj.value(QStringLiteral("image_width")).toInt(mapping.imageSize.width()),
                                      pickMapObj.value(QStringLiteral("image_height")).toInt(mapping.imageSize.height()));
            pickMap.xMin = pickMapObj.value(QStringLiteral("x_min")).toDouble(entry->xMin);
            pickMap.xMax = pickMapObj.value(QStringLiteral("x_max")).toDouble(entry->xMax);
            pickMap.yMin = pickMapObj.value(QStringLiteral("y_min")).toDouble(entry->yMin);
            pickMap.yMax = pickMapObj.value(QStringLiteral("y_max")).toDouble(entry->yMax);
            pickMap.xImagePath = normalizePath(pickMapObj.value(QStringLiteral("x_image_path")).toString());
            pickMap.yImagePath = normalizePath(pickMapObj.value(QStringLiteral("y_image_path")).toString());
            if (!parseImageRect(pickMapObj.value(QStringLiteral("display_content_rect")), &pickMap.displayContentRect)) {
                pickMap.displayContentRect = mapping.displayContentRect;
            }
            pickMap.valid = pickMap.imageSize.isValid()
                         && std::isfinite(pickMap.xMin) && std::isfinite(pickMap.xMax)
                         && std::isfinite(pickMap.yMin) && std::isfinite(pickMap.yMax)
                         && pickMap.xMax > pickMap.xMin && pickMap.yMax > pickMap.yMin
                         && !pickMap.xImagePath.trimmed().isEmpty() && !pickMap.yImagePath.trimmed().isEmpty();
        }
        entry->scenePickMap = pickMap;
    };

    if (obj.contains(QStringLiteral("metrics")) && obj.value(QStringLiteral("metrics")).isArray()) {
        const QJsonArray metrics = obj.value(QStringLiteral("metrics")).toArray();
        for (const QJsonValue& value : metrics) {
            if (!value.isObject()) {
                continue;
            }
            const QJsonObject metricObj = value.toObject();
            MetricEntry entry;
            entry.metric = metricObj.value(QStringLiteral("metric")).toString().trimmed();
            if (entry.metric.isEmpty()) {
                continue;
            }
            populateMetricEntry(metricObj, &entry);
            metricEntries_.insert(entry.metric, entry);
        }
    }

    if (metricEntries_.isEmpty()) {
        MetricEntry entry;
        entry.metric = obj.value(QStringLiteral("metric")).toString().trimmed();
        if (entry.metric.isEmpty()) {
            entry.metric = QStringLiteral("ckm_metric");
        }
        populateMetricEntry(obj, &entry);
        metricEntries_.insert(entry.metric, entry);
    }
    if (metricEntries_.isEmpty()) {
        if (errorMessage) {
            *errorMessage = tr("CKM metadata does not contain any metric entries.");
        }
        return false;
    }

    QString defaultMetric = obj.value(QStringLiteral("default_metric")).toString().trimmed();
    if (defaultMetric.isEmpty()) {
        defaultMetric = obj.value(QStringLiteral("metric")).toString().trimmed();
    }
    if (defaultMetric.isEmpty() || !metricEntries_.contains(defaultMetric)) {
        defaultMetric = metricEntries_.firstKey();
    }
    defaultMetric_ = defaultMetric;

    QStringList infoLines;
    infoLines << tr("Meta file: %1").arg(metaInfo.absoluteFilePath());
    if (obj.contains(QStringLiteral("run_id"))) {
        infoLines << tr("Run ID: %1").arg(obj.value(QStringLiteral("run_id")).toString());
    }
    if (obj.contains(QStringLiteral("output_root_dir"))) {
        infoLines << tr("Version folder: %1").arg(normalizePath(obj.value(QStringLiteral("output_root_dir")).toString()));
    }
    if (obj.contains(QStringLiteral("selected_metrics")) && obj.value(QStringLiteral("selected_metrics")).isArray()) {
        QStringList metricNames;
        const QJsonArray arr = obj.value(QStringLiteral("selected_metrics")).toArray();
        for (const QJsonValue& value : arr) {
            const QString metric = value.toString().trimmed();
            if (!metric.isEmpty()) {
                metricNames << ckmMetricUiKey(metric);
            }
        }
        if (!metricNames.isEmpty()) {
            infoLines << tr("Generated metrics: %1").arg(metricNames.join(QStringLiteral(", ")));
        }
    }
    if (obj.contains(QStringLiteral("grid_width")) && obj.contains(QStringLiteral("grid_height"))) {
        infoLines << tr("Grid size: %1 × %2").arg(obj.value(QStringLiteral("grid_width")).toInt()).arg(obj.value(QStringLiteral("grid_height")).toInt());
    }
    if (obj.contains(QStringLiteral("resolution_m"))) {
        infoLines << tr("Resolution: %1 m").arg(obj.value(QStringLiteral("resolution_m")).toDouble());
    }
    if (obj.contains(QStringLiteral("z_fixed"))) {
        infoLines << tr("Fixed RX height: %1 m").arg(obj.value(QStringLiteral("z_fixed")).toDouble());
    }
    if (hasExtent_) {
        infoLines << tr("Extent: x [%1, %2], y [%3, %4]")
                        .arg(obj.value(QStringLiteral("x_min")).toDouble())
                        .arg(obj.value(QStringLiteral("x_max")).toDouble())
                        .arg(obj.value(QStringLiteral("y_min")).toDouble())
                        .arg(obj.value(QStringLiteral("y_max")).toDouble());
    }
    if (obj.contains(QStringLiteral("range_source"))) {
        infoLines << tr("Range source: %1").arg(obj.value(QStringLiteral("range_source")).toString());
    }
    if (obj.contains(QStringLiteral("station_names")) && obj.value(QStringLiteral("station_names")).isArray()) {
        QStringList stationNames;
        const QJsonArray arr = obj.value(QStringLiteral("station_names")).toArray();
        for (const QJsonValue& value : arr) {
            const QString name = value.toString().trimmed();
            if (!name.isEmpty()) {
                stationNames << name;
            }
        }
        if (!stationNames.isEmpty()) {
            infoLines << tr("Base stations: %1").arg(stationNames.join(QStringLiteral(", ")));
        }
    }
    commonInfoText_ = infoLines.join(QStringLiteral("\n"));

    for (auto it = metricEntries_.cbegin(); it != metricEntries_.cend(); ++it) {
        QVector<SamplePoint> samples;
        const QString samplesPath = it.value().samplesJsonPath.trimmed();
        if (!samplesPath.isEmpty() && QFileInfo::exists(samplesPath)) {
            QFile samplesFile(samplesPath);
            if (samplesFile.open(QIODevice::ReadOnly)) {
                QJsonParseError sampleParseError;
                const QJsonDocument samplesDoc = QJsonDocument::fromJson(samplesFile.readAll(), &sampleParseError);
                if (sampleParseError.error == QJsonParseError::NoError && samplesDoc.isArray()) {
                    const QJsonArray arr = samplesDoc.array();
                    samples.reserve(arr.size());
                    for (const QJsonValue& value : arr) {
                        if (!value.isObject()) {
                            continue;
                        }
                        const QJsonObject sampleObj = value.toObject();
                        SamplePoint sample;
                        sample.x = sampleObj.value(QStringLiteral("x_m")).toDouble(std::numeric_limits<double>::quiet_NaN());
                        sample.y = sampleObj.value(QStringLiteral("y_m")).toDouble(std::numeric_limits<double>::quiet_NaN());
                        const QJsonValue metricValue = sampleObj.value(QStringLiteral("metric_value"));
                        sample.hasMetricValue = metricValue.isDouble();
                        sample.metricValue = metricValue.toDouble(0.0);
                        sample.payload = sampleObj;
                        if (std::isfinite(sample.x) && std::isfinite(sample.y)) {
                            samples.push_back(sample);
                        }
                    }
                }
            }
        }
        metricSamples_.insert(it.key(), samples);
    }

    updatingMetricCombo_ = true;
    metricCombo_->clear();
    for (auto it = metricEntries_.cbegin(); it != metricEntries_.cend(); ++it) {
        metricCombo_->addItem(it.value().displayName.isEmpty() ? ckmMetricUiKey(it.key()) : sanitizeCkmDisplayText(it.value().displayName), it.key());
    }
    const int defaultIndex = metricCombo_->findData(defaultMetric);
    if (defaultIndex >= 0) {
        metricCombo_->setCurrentIndex(defaultIndex);
    }
    updatingMetricCombo_ = false;

    currentMetaFilePath_ = metaInfo.absoluteFilePath();
    zoomFactor_ = 1.0;
    displayedPixmapSize_ = QSize();
    if (zoomResetButton_) {
        zoomResetButton_->setText(QStringLiteral("100%"));
    }
    rebuildViewModeCombo();
    applyMetricSelection(defaultMetric);
    emitSelectionClearedHint(tr("Click the scene-overlay image to inspect the nearest offline CKM sample for that point."));
    return true;
}

void CkmMapWidget::rebuildViewModeCombo() {
    const int metricIndex = metricCombo_->currentIndex();
    const QString metric = metricIndex >= 0 ? metricCombo_->itemData(metricIndex).toString() : QString();
    const MetricEntry entry = metricEntries_.value(metric);
    const QString previousMode = viewModeCombo_->currentData().toString();

    updatingViewModeCombo_ = true;
    viewModeCombo_->clear();
    if (!entry.sceneImagePath.trimmed().isEmpty()) {
        viewModeCombo_->addItem(tr("Scene overlay"), QStringLiteral("scene_overlay"));
    }
    if (!entry.heatmapImagePath.trimmed().isEmpty()) {
        viewModeCombo_->addItem(tr("2D heatmap"), QStringLiteral("heatmap"));
    }
    if (viewModeCombo_->count() == 0) {
        viewModeCombo_->addItem(tr("2D heatmap"), QStringLiteral("heatmap"));
    }
    int targetIndex = viewModeCombo_->findData(previousMode);
    if (targetIndex < 0) {
        targetIndex = viewModeCombo_->findData(entry.sceneImagePath.trimmed().isEmpty() ? QStringLiteral("heatmap") : QStringLiteral("scene_overlay"));
    }
    if (targetIndex < 0) {
        targetIndex = 0;
    }
    viewModeCombo_->setCurrentIndex(targetIndex);
    updatingViewModeCombo_ = false;
}

QString CkmMapWidget::currentDisplayImagePath(const MetricEntry& entry, QString* resolvedMode) const {
    const QString requestedMode = viewModeCombo_ ? viewModeCombo_->currentData().toString() : QString();
    if (requestedMode == QStringLiteral("scene_overlay") && !entry.sceneImagePath.trimmed().isEmpty()) {
        if (resolvedMode) {
            *resolvedMode = QStringLiteral("scene_overlay");
        }
        return entry.sceneImagePath;
    }
    if (!entry.heatmapImagePath.trimmed().isEmpty()) {
        if (resolvedMode) {
            *resolvedMode = QStringLiteral("heatmap");
        }
        return entry.heatmapImagePath;
    }
    if (!entry.sceneImagePath.trimmed().isEmpty()) {
        if (resolvedMode) {
            *resolvedMode = QStringLiteral("scene_overlay");
        }
        return entry.sceneImagePath;
    }
    if (resolvedMode) {
        *resolvedMode = QStringLiteral("heatmap");
    }
    return QString();
}

void CkmMapWidget::applyMetricSelection(const QString& metric) {
    const MetricEntry entry = metricEntries_.value(metric);
    if (entry.metric.isEmpty()) {
        return;
    }

    QString resolvedMode;
    const QString imagePath = currentDisplayImagePath(entry, &resolvedMode);
    const QPixmap pixmap(imagePath);
    if (pixmap.isNull()) {
        titleLabel_->setText(tr("Dense CKM Map — %1").arg(ckmMetricUiKey(entry.metric)));
        infoLabel_->setText(commonInfoText_ + QStringLiteral("\n") + tr("Image file is missing: %1").arg(imagePath));
        imageLabel_->setPixmap(QPixmap());
        basePixmap_ = QPixmap();
        currentMetric_.clear();
        currentImagePath_.clear();
        currentViewMode_.clear();
        currentImageClickable_ = false;
        currentSceneClickMapping_ = SceneClickMapping();
        currentScenePickMap_ = ScenePickMap();
        currentScenePickMapX_ = QImage();
        currentScenePickMapY_ = QImage();
        hasSelectedImagePoint_ = false;
        setPlaceholderText(tr("CKM image not found: %1").arg(imagePath));
        emitSelectionClearedHint(tr("Selected CKM image is missing. Load another CKM version folder or regenerate the result."));
        return;
    }

    currentMetric_ = entry.metric;
    xMin_ = entry.hasExtent ? entry.xMin : xMin_;
    xMax_ = entry.hasExtent ? entry.xMax : xMax_;
    yMin_ = entry.hasExtent ? entry.yMin : yMin_;
    yMax_ = entry.hasExtent ? entry.yMax : yMax_;
    zFixed_ = std::isfinite(entry.zFixed) ? entry.zFixed : zFixed_;
    hasExtent_ = entry.hasExtent;
    currentSceneClickMapping_ = entry.sceneClickMapping;
    currentScenePickMap_ = entry.scenePickMap;
    currentScenePickMapX_ = QImage();
    currentScenePickMapY_ = QImage();
    if (resolvedMode == QStringLiteral("scene_overlay") && currentScenePickMap_.valid) {
        const QImage pickX(currentScenePickMap_.xImagePath);
        const QImage pickY(currentScenePickMap_.yImagePath);
        if (!pickX.isNull() && !pickY.isNull() && pickX.size() == pickY.size()) {
            currentScenePickMapX_ = pickX;
            currentScenePickMapY_ = pickY;
            if (!currentScenePickMap_.imageSize.isValid()) {
                currentScenePickMap_.imageSize = pickX.size();
            }
            if (currentScenePickMap_.imageSize != pickX.size()) {
                currentScenePickMap_.valid = false;
                currentScenePickMapX_ = QImage();
                currentScenePickMapY_ = QImage();
            }
        } else {
            currentScenePickMap_.valid = false;
        }
    }
    currentImagePath_ = imagePath;
    currentViewMode_ = resolvedMode;
    currentImageClickable_ = resolvedMode == QStringLiteral("scene_overlay")
                          && (currentScenePickMap_.valid || currentSceneClickMapping_.valid || hasExtent_);
    basePixmap_ = pixmap;
    hasSelectedImagePoint_ = false;

    titleLabel_->setText(entry.displayName.isEmpty() ? tr("Dense CKM Map — %1").arg(ckmMetricUiKey(entry.metric))
                                                      : tr("Dense CKM Map — %1").arg(sanitizeCkmDisplayText(entry.displayName)));

    QStringList infoLines;
    if (!commonInfoText_.trimmed().isEmpty()) {
        infoLines << commonInfoText_;
    }
    infoLines << tr("Current metric: %1").arg(entry.metric);
    infoLines << tr("Current view: %1").arg(resolvedMode == QStringLiteral("scene_overlay") ? tr("Scene overlay") : tr("2D heatmap"));
    if (currentImageClickable_) {
        infoLines << tr("Click query: enabled");
        if (currentScenePickMap_.valid) {
            infoLines << tr("Mapping: pixel-aligned pick map (exact PNG -> world XY lookup)");
        } else if (currentSceneClickMapping_.valid) {
            infoLines << tr("Mapping: pixel -> camera ray -> z=%1 plane").arg(compactNumber(currentSceneClickMapping_.planeZ, 2));
        } else {
            infoLines << tr("Mapping: linear XY fallback (legacy metadata)");
        }
    } else {
        infoLines << tr("Click query: switch to Scene overlay to query offline CKM values by point");
    }
    if (!entry.sceneImagePath.trimmed().isEmpty()) {
        infoLines << tr("Scene overlay image: %1").arg(entry.sceneImagePath);
    }
    if (!entry.heatmapImagePath.trimmed().isEmpty()) {
        infoLines << tr("Heatmap image: %1").arg(entry.heatmapImagePath);
    }
    if (!entry.npzPath.trimmed().isEmpty()) {
        infoLines << tr("NPZ: %1").arg(entry.npzPath);
    }
    if (!entry.samplesJsonPath.trimmed().isEmpty()) {
        infoLines << tr("Samples JSON: %1").arg(entry.samplesJsonPath);
        infoLines << tr("Loaded sample count: %1").arg(metricSamples_.value(entry.metric).size());
    }
    infoLabel_->setText(infoLines.join(QStringLiteral("\n")));

    updateDisplayedPixmap();
    if (!hasSelectedPoint_) {
        emitSelectionClearedHint(currentImageClickable_
            ? tr("Click the scene-overlay image to inspect the nearest offline CKM sample for that point.")
            : tr("Switch to Scene overlay, then click the map to inspect offline CKM values."));
    }
}

void CkmMapWidget::updateDisplayedPixmap() {
    if (basePixmap_.isNull()) {
        imageLabel_->setPixmap(QPixmap());
        setPlaceholderText(tr("Dense CKM image is unavailable."));
        return;
    }

    QPixmap displayPixmap = basePixmap_.copy();
    if (hasSelectedPoint_) {
        QPointF imagePoint;
        bool ok = false;
        if (hasSelectedImagePoint_) {
            imagePoint = selectedImagePoint_;
            ok = std::isfinite(imagePoint.x()) && std::isfinite(imagePoint.y());
        } else {
            ok = mapWorldPointToImage(selectedWorldPoint_, &imagePoint);
            if (!ok && hasExtent_) {
                QRect contentRect = currentSceneClickMapping_.displayContentRect;
                if (!contentRect.isValid()) {
                    contentRect = currentScenePickMap_.displayContentRect;
                }
                contentRect = effectiveContentRect(contentRect, displayPixmap.size());
                const double width = static_cast<double>(contentRect.width());
                const double height = static_cast<double>(contentRect.height());
                if (width > 1.0 && height > 1.0) {
                    imagePoint = QPointF(
                        contentRect.left() + ((selectedWorldPoint_.x() - xMin_) / (xMax_ - xMin_)) * (width - 1.0),
                        contentRect.top() + ((yMax_ - selectedWorldPoint_.y()) / (yMax_ - yMin_)) * (height - 1.0));
                    ok = std::isfinite(imagePoint.x()) && std::isfinite(imagePoint.y());
                }
            }
        }
        if (ok) {
            QPainter painter(&displayPixmap);
            painter.setRenderHint(QPainter::Antialiasing, true);
            const QPointF center(imagePoint);
            QPen pen(Qt::yellow);
            pen.setWidth(3);
            painter.setPen(pen);
            painter.drawLine(QPointF(center.x() - 14.0, center.y()), QPointF(center.x() + 14.0, center.y()));
            painter.drawLine(QPointF(center.x(), center.y() - 14.0), QPointF(center.x(), center.y() + 14.0));
            QPen outline(Qt::black);
            outline.setWidth(1);
            painter.setPen(outline);
            painter.drawEllipse(center, 8.0, 8.0);
            painter.end();
        }
    }

    QPixmap shownPixmap = displayPixmap;
    const QSize targetSize = scaledSizeForZoom(displayPixmap.size(), zoomFactor_);
    if (targetSize.isValid() && targetSize != displayPixmap.size()) {
        shownPixmap = displayPixmap.scaled(targetSize, Qt::IgnoreAspectRatio, Qt::FastTransformation);
    }

    displayedPixmapSize_ = shownPixmap.size();
    imageLabel_->setPixmap(shownPixmap);
    imageLabel_->setText(QString());
    imageLabel_->setFixedSize(shownPixmap.size());
}

bool CkmMapWidget::mapWidgetPointToWorld(const QPointF& widgetPoint, QPointF* worldPoint, QPointF* imagePoint) const {
    if (!currentImageClickable_ || basePixmap_.isNull() || !imageLabel_) {
        return false;
    }

    const QSize basePixmapSize = basePixmap_.size();
    if (basePixmapSize.width() <= 0 || basePixmapSize.height() <= 0) {
        return false;
    }
    const QSize actualPixmapSize = displayedPixmapSize_.isValid() ? displayedPixmapSize_ : basePixmapSize;

    QRect interactiveRect = currentScenePickMap_.displayContentRect;
    if (!interactiveRect.isValid()) {
        interactiveRect = currentSceneClickMapping_.displayContentRect;
    }
    interactiveRect = effectiveContentRect(interactiveRect, basePixmapSize);

    const QRect labelRect = imageLabel_->contentsRect();
    const double offsetX = labelRect.x() + (labelRect.width() - actualPixmapSize.width()) * 0.5;
    const double offsetY = labelRect.y() + (labelRect.height() - actualPixmapSize.height()) * 0.5;
    const QRectF pixmapRect(offsetX, offsetY, actualPixmapSize.width(), actualPixmapSize.height());
    if (!pixmapRect.contains(widgetPoint)) {
        return false;
    }

    QPointF displayPoint(widgetPoint.x() - pixmapRect.left(), widgetPoint.y() - pixmapRect.top());
    displayPoint.setX(std::max(0.0, std::min(displayPoint.x(), static_cast<double>(actualPixmapSize.width() - 1))));
    displayPoint.setY(std::max(0.0, std::min(displayPoint.y(), static_cast<double>(actualPixmapSize.height() - 1))));

    QPointF resolvedImagePoint = scalePointBetweenSizes(displayPoint, actualPixmapSize, basePixmapSize);
    resolvedImagePoint.setX(std::max(0.0, std::min(resolvedImagePoint.x(), static_cast<double>(basePixmapSize.width() - 1))));
    resolvedImagePoint.setY(std::max(0.0, std::min(resolvedImagePoint.y(), static_cast<double>(basePixmapSize.height() - 1))));
    if (!QRectF(interactiveRect).contains(resolvedImagePoint)) {
        return false;
    }
    if (imagePoint) {
        *imagePoint = resolvedImagePoint;
    }

    if (mapImagePointToWorldFromPickMap(resolvedImagePoint, worldPoint)) {
        return true;
    }
    if (currentSceneClickMapping_.valid && mapImagePointToWorld(resolvedImagePoint, worldPoint)) {
        return true;
    }

    if (!hasExtent_) {
        return false;
    }
    const double u = (resolvedImagePoint.x() - interactiveRect.left()) / std::max(1.0, static_cast<double>(interactiveRect.width() - 1));
    const double v = (resolvedImagePoint.y() - interactiveRect.top()) / std::max(1.0, static_cast<double>(interactiveRect.height() - 1));
    const double x = xMin_ + u * (xMax_ - xMin_);
    const double y = yMax_ - v * (yMax_ - yMin_);
    if (worldPoint) {
        *worldPoint = QPointF(x, y);
    }
    return std::isfinite(x) && std::isfinite(y);
}

bool CkmMapWidget::mapImagePointToWorldFromPickMap(const QPointF& imagePoint, QPointF* worldPoint) const {
    if (!currentScenePickMap_.valid || currentScenePickMapX_.isNull() || currentScenePickMapY_.isNull()) {
        return false;
    }
    if (currentScenePickMapX_.size() != currentScenePickMapY_.size()) {
        return false;
    }
    const QSize imgSize = currentScenePickMapX_.size();
    if (imgSize.width() <= 0 || imgSize.height() <= 0) {
        return false;
    }

    QPointF mappedPoint;
    const QSize displaySize = basePixmap_.isNull() ? imgSize : basePixmap_.size();
    if (!mapDisplayPointToImagePoint(imagePoint, displaySize, currentScenePickMap_.displayContentRect, imgSize, &mappedPoint)) {
        return false;
    }

    const int px = std::max(0, std::min(static_cast<int>(std::floor(mappedPoint.x())), imgSize.width() - 1));
    const int py = std::max(0, std::min(static_cast<int>(std::floor(mappedPoint.y())), imgSize.height() - 1));
    const QRgb xPixel = currentScenePickMapX_.pixel(px, py);
    const QRgb yPixel = currentScenePickMapY_.pixel(px, py);
    const int xCode = (qRed(xPixel) << 8) | qGreen(xPixel);
    const int yCode = (qRed(yPixel) << 8) | qGreen(yPixel);
    if (xCode >= 65535 || yCode >= 65535) {
        return false;
    }

    const double xRange = currentScenePickMap_.xMax - currentScenePickMap_.xMin;
    const double yRange = currentScenePickMap_.yMax - currentScenePickMap_.yMin;
    const double x = currentScenePickMap_.xMin + (static_cast<double>(xCode) / 65534.0) * xRange;
    const double y = currentScenePickMap_.yMin + (static_cast<double>(yCode) / 65534.0) * yRange;
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return false;
    }
    if (worldPoint) {
        *worldPoint = QPointF(x, y);
    }
    return true;
}

bool CkmMapWidget::mapImagePointToWorld(const QPointF& imagePoint, QPointF* worldPoint) const {
    if (!currentSceneClickMapping_.valid) {
        return false;
    }

    QPointF mappingImagePoint;
    const QSize displaySize = basePixmap_.isNull()
        ? (currentSceneClickMapping_.imageSize.isValid() ? currentSceneClickMapping_.imageSize : QSize())
        : basePixmap_.size();
    const QSize mappingSize = currentSceneClickMapping_.imageSize.isValid() ? currentSceneClickMapping_.imageSize : displaySize;
    if (!mapDisplayPointToImagePoint(imagePoint, displaySize, currentSceneClickMapping_.displayContentRect, mappingSize, &mappingImagePoint)) {
        return false;
    }

    if (currentSceneClickMapping_.hasQuad && !pointInOrNearQuad(currentSceneClickMapping_.quadImage, mappingImagePoint)) {
        return false;
    }

    if (currentSceneClickMapping_.hasCamera && currentSceneClickMapping_.imageSize.isValid()) {
        const double width = static_cast<double>(currentSceneClickMapping_.imageSize.width());
        const double height = static_cast<double>(currentSceneClickMapping_.imageSize.height());
        const double fovRad = currentSceneClickMapping_.fovDeg * 3.14159265358979323846 / 180.0;
        const double focal = width / (2.0 * std::tan(std::max(fovRad, 1e-6) * 0.5));
        const double cx = (width - 1.0) * 0.5;
        const double cy = (height - 1.0) * 0.5;

        const QVector3D forward = (currentSceneClickMapping_.cameraTarget - currentSceneClickMapping_.cameraOrigin).normalized();
        const QVector3D right = QVector3D::crossProduct(forward, currentSceneClickMapping_.cameraUp).normalized();
        const QVector3D up = QVector3D::crossProduct(right, forward).normalized();
        QVector3D dir = right * static_cast<float>((mappingImagePoint.x() - cx) / focal)
                      + up * static_cast<float>(-(mappingImagePoint.y() - cy) / focal)
                      + forward;
        if (!std::isfinite(dir.x()) || !std::isfinite(dir.y()) || !std::isfinite(dir.z()) || std::fabs(dir.z()) < 1e-9) {
            return false;
        }
        const double t = (currentSceneClickMapping_.planeZ - currentSceneClickMapping_.cameraOrigin.z()) / dir.z();
        if (!std::isfinite(t) || t <= 0.0) {
            return false;
        }
        const QVector3D hit = currentSceneClickMapping_.cameraOrigin + dir * static_cast<float>(t);
        const double x = hit.x();
        const double y = hit.y();
        if (currentSceneClickMapping_.hasExtent) {
            const double epsX = std::max(1.0, (currentSceneClickMapping_.xMax - currentSceneClickMapping_.xMin) * 0.01);
            const double epsY = std::max(1.0, (currentSceneClickMapping_.yMax - currentSceneClickMapping_.yMin) * 0.01);
            if (x < currentSceneClickMapping_.xMin - epsX || x > currentSceneClickMapping_.xMax + epsX
                || y < currentSceneClickMapping_.yMin - epsY || y > currentSceneClickMapping_.yMax + epsY) {
                return false;
            }
        }
        if (worldPoint) {
            *worldPoint = QPointF(x, y);
        }
        return std::isfinite(x) && std::isfinite(y);
    }

    if (currentSceneClickMapping_.hasQuad) {
        QTransform transform;
        if (quadToQuadTransform(currentSceneClickMapping_.quadImage, currentSceneClickMapping_.quadWorld, &transform)) {
            const QPointF mapped = transform.map(mappingImagePoint);
            if (worldPoint) {
                *worldPoint = mapped;
            }
            return std::isfinite(mapped.x()) && std::isfinite(mapped.y());
        }
    }
    return false;
}

bool CkmMapWidget::mapWorldPointToImage(const QPointF& worldPoint, QPointF* imagePoint) const {
    if (!currentSceneClickMapping_.valid || !imagePoint) {
        return false;
    }

    QPointF rawImagePoint;
    bool ok = false;
    if (currentSceneClickMapping_.hasCamera && currentSceneClickMapping_.imageSize.isValid()) {
        const QVector3D origin = currentSceneClickMapping_.cameraOrigin;
        const QVector3D target = currentSceneClickMapping_.cameraTarget;
        const QVector3D upHint = currentSceneClickMapping_.cameraUp;
        const QVector3D forward = (target - origin).normalized();
        const QVector3D right = QVector3D::crossProduct(forward, upHint).normalized();
        const QVector3D up = QVector3D::crossProduct(right, forward).normalized();
        const QVector3D point(static_cast<float>(worldPoint.x()), static_cast<float>(worldPoint.y()), static_cast<float>(currentSceneClickMapping_.planeZ));
        const QVector3D rel = point - origin;
        const double xCam = QVector3D::dotProduct(rel, right);
        const double yCam = QVector3D::dotProduct(rel, up);
        const double zCam = QVector3D::dotProduct(rel, forward);
        if (!std::isfinite(zCam) || zCam <= 1e-6) {
            return false;
        }
        const double width = static_cast<double>(currentSceneClickMapping_.imageSize.width());
        const double height = static_cast<double>(currentSceneClickMapping_.imageSize.height());
        const double fovRad = currentSceneClickMapping_.fovDeg * 3.14159265358979323846 / 180.0;
        const double focal = width / (2.0 * std::tan(std::max(fovRad, 1e-6) * 0.5));
        const double cx = (width - 1.0) * 0.5;
        const double cy = (height - 1.0) * 0.5;
        const double u = cx + focal * (xCam / zCam);
        const double v = cy - focal * (yCam / zCam);
        rawImagePoint = QPointF(u, v);
        ok = std::isfinite(u) && std::isfinite(v);
    } else if (currentSceneClickMapping_.hasQuad) {
        QTransform transform;
        if (quadToQuadTransform(currentSceneClickMapping_.quadWorld, currentSceneClickMapping_.quadImage, &transform)) {
            const QPointF mapped = transform.map(worldPoint);
            rawImagePoint = mapped;
            ok = std::isfinite(mapped.x()) && std::isfinite(mapped.y());
        }
    }
    if (!ok) {
        return false;
    }

    const QSize displaySize = basePixmap_.isNull()
        ? (currentSceneClickMapping_.imageSize.isValid() ? currentSceneClickMapping_.imageSize : QSize())
        : basePixmap_.size();
    const QSize mappingSize = currentSceneClickMapping_.imageSize.isValid() ? currentSceneClickMapping_.imageSize : displaySize;
    *imagePoint = mapImagePointToDisplayPoint(rawImagePoint, mappingSize, displaySize, currentSceneClickMapping_.displayContentRect);
    return std::isfinite(imagePoint->x()) && std::isfinite(imagePoint->y());
}

void CkmMapWidget::setZoomFactor(double factor) {
    const double clamped = clampZoomFactor(factor);
    if (std::fabs(clamped - zoomFactor_) < 1e-6) {
        return;
    }
    zoomFactor_ = clamped;
    if (zoomResetButton_) {
        zoomResetButton_->setText(QStringLiteral("%1%").arg(static_cast<int>(std::lround(zoomFactor_ * 100.0))));
    }
    updateDisplayedPixmap();
}

void CkmMapWidget::zoomBy(double multiplier) {
    if (!std::isfinite(multiplier) || multiplier <= 0.0) {
        return;
    }
    setZoomFactor(zoomFactor_ * multiplier);
}

QString CkmMapWidget::formatValueWithMetric(const QString& metric, double value) const {
    if (!std::isfinite(value)) {
        return QStringLiteral("N/A");
    }
    if (metric == QStringLiteral("power_db") || metric == QStringLiteral("path_loss_db") || metric == QStringLiteral("sys_sinr_eff_db")
        || metric == QStringLiteral("beam_oracle_gain_db") || metric == QStringLiteral("beam_deepsense_gain_db")) {
        return QStringLiteral("%1 dB").arg(QString::number(value, 'f', 2));
    }
    if (metric == QStringLiteral("tau_std_ns")) {
        return QStringLiteral("%1 ns").arg(QString::number(value, 'f', 2));
    }
    if (metric == QStringLiteral("sys_spectral_efficiency_bpshz") || metric == QStringLiteral("best_bs_rate_bpshz")) {
        return QStringLiteral("%1 bps/Hz").arg(QString::number(value, 'f', 3));
    }
    if (metric == QStringLiteral("best_bs_index") || metric == QStringLiteral("serving_bs_index") || metric == QStringLiteral("sys_mcs_index")
        || metric == QStringLiteral("beam_oracle_index") || metric == QStringLiteral("beam_deepsense_index")) {
        return QString::number(static_cast<int>(std::llround(value)));
    }
    if (metric == QStringLiteral("los_binary") || metric == QStringLiteral("sys_tb_ok")) {
        return QString::number(static_cast<int>(std::llround(value)));
    }
    return QString::number(value, 'f', 3);
}

QString CkmMapWidget::formatJsonValue(const QJsonValue& value) const {
    if (value.isNull() || value.isUndefined()) {
        return QStringLiteral("null");
    }
    if (value.isBool()) {
        return boolText(value.toBool());
    }
    if (value.isDouble()) {
        const double v = value.toDouble();
        if (!std::isfinite(v)) {
            return QStringLiteral("null");
        }
        if (std::fabs(v - std::round(v)) < 1e-9) {
            return QString::number(static_cast<qint64>(std::llround(v)));
        }
        return QString::number(v, 'f', 3);
    }
    if (value.isString()) {
        return value.toString();
    }
    if (value.isArray() || value.isObject()) {
        return QString::fromUtf8(QJsonDocument(value.isArray() ? QJsonDocument(value.toArray()) : QJsonDocument(value.toObject()))
                                     .toJson(QJsonDocument::Compact));
    }
    return QStringLiteral("?");
}

bool CkmMapWidget::selectNearestPoint(const QPointF& worldPoint, const QPointF* imagePoint) {
    if (!hasExtent_) {
        emitSelectionClearedHint(tr("This CKM result does not expose a valid XY extent."));
        return false;
    }

    struct MatchInfo {
        bool valid{false};
        SamplePoint sample;
        double distanceSq{0.0};
    };

    QMap<QString, MatchInfo> matches;
    QStringList metricLines;
    for (auto it = metricEntries_.cbegin(); it != metricEntries_.cend(); ++it) {
        const QVector<SamplePoint> samples = metricSamples_.value(it.key());
        if (samples.isEmpty()) {
            continue;
        }
        MatchInfo best;
        best.distanceSq = std::numeric_limits<double>::infinity();
        for (const SamplePoint& sample : samples) {
            const double dx = sample.x - worldPoint.x();
            const double dy = sample.y - worldPoint.y();
            const double distSq = dx * dx + dy * dy;
            if (!best.valid || distSq < best.distanceSq) {
                best.valid = true;
                best.sample = sample;
                best.distanceSq = distSq;
            }
        }
        if (best.valid) {
            matches.insert(it.key(), best);
            const QString label = it.value().displayName.isEmpty() ? ckmMetricUiKey(it.key()) : sanitizeCkmDisplayText(it.value().displayName);
            metricLines << QStringLiteral("%1: %2  @ (%3, %4)")
                               .arg(label,
                                    formatValueWithMetric(it.key(), best.sample.hasMetricValue ? best.sample.metricValue : std::numeric_limits<double>::quiet_NaN()),
                                    compactNumber(best.sample.x, 2),
                                    compactNumber(best.sample.y, 2));
        }
    }

    if (matches.isEmpty()) {
        emitSelectionClearedHint(tr("No offline CKM samples were loaded for the current CKM version."));
        return false;
    }

    const QString anchorMetric = matches.contains(defaultMetric_) ? defaultMetric_ : matches.firstKey();
    const MatchInfo anchor = matches.value(anchorMetric);
    const QJsonObject payload = anchor.sample.payload;

    QStringList summaryLines;
    summaryLines << tr("Selected world point: (%1, %2, %3 m)")
                        .arg(compactNumber(worldPoint.x(), 2), compactNumber(worldPoint.y(), 2), compactNumber(zFixed_, 2));
    summaryLines << tr("Nearest sampled point: (%1, %2)")
                        .arg(compactNumber(anchor.sample.x, 2), compactNumber(anchor.sample.y, 2));
    summaryLines << tr("Selection distance: %1 m").arg(QString::number(std::sqrt(anchor.distanceSq), 'f', 2));
    summaryLines << tr("Reference metric: %1").arg(metricEntries_.value(anchorMetric).displayName.isEmpty()
                                                       ? ckmMetricUiKey(anchorMetric)
                                                       : sanitizeCkmDisplayText(metricEntries_.value(anchorMetric).displayName));

    QStringList detailLines;
    detailLines << tr("Per-metric offline CKM values:");
    detailLines << metricLines;
    detailLines << QString();
    detailLines << tr("Auxiliary values from the nearest sample:");

    const QStringList preferredKeys = {
        QStringLiteral("has_any_path"),
        QStringLiteral("has_any_los"),
        QStringLiteral("best_power_db"),
        QStringLiteral("best_bs_index"),
        QStringLiteral("serving_bs_index"),
        QStringLiteral("best_bs_rate_bpshz"),
        QStringLiteral("best_tau_std_ns"),
        QStringLiteral("beam_enabled"),
        QStringLiteral("beam_selection_mode"),
        QStringLiteral("beam_reference_tx_idx"),
        QStringLiteral("beam_reference_bs_name"),
        QStringLiteral("beam_reference_anchor_id"),
        QStringLiteral("beam_oracle_index"),
        QStringLiteral("beam_oracle_gain_db"),
        QStringLiteral("beam_deepsense_index"),
        QStringLiteral("beam_deepsense_gain_db"),
        QStringLiteral("beam_predictor_status"),
        QStringLiteral("beam_predictor_error"),
        QStringLiteral("beam_predictor_available"),
        QStringLiteral("beam_feature_mode"),
        QStringLiteral("beam_top_k"),
        QStringLiteral("beam_selection_source"),
        QStringLiteral("beam_topk_indices"),
        QStringLiteral("beam_topk_probabilities"),
        QStringLiteral("metric"),
        QStringLiteral("metric_value"),
        QStringLiteral("x_m"),
        QStringLiteral("y_m"),
        QStringLiteral("per_bs_power_db")
    };
    QSet<QString> emitted;
    for (const QString& key : preferredKeys) {
        if (payload.contains(key)) {
            detailLines << QStringLiteral("%1: %2").arg(ckmMetricUiKey(key), sanitizeCkmDisplayText(formatJsonValue(payload.value(key))));
            emitted.insert(key);
        }
    }
    const QStringList payloadKeys = payload.keys();
    for (const QString& key : payloadKeys) {
        if (emitted.contains(key)) {
            continue;
        }
        detailLines << QStringLiteral("%1: %2").arg(ckmMetricUiKey(key), sanitizeCkmDisplayText(formatJsonValue(payload.value(key))));
    }

    hasSelectedPoint_ = true;
    selectedWorldPoint_ = worldPoint;
    hasSelectedImagePoint_ = imagePoint != nullptr;
    if (imagePoint) {
        selectedImagePoint_ = *imagePoint;
    }
    updateDisplayedPixmap();
    emit pointSelectionChanged(
        tr("Dense CKM sample at (%1, %2)").arg(compactNumber(worldPoint.x(), 2), compactNumber(worldPoint.y(), 2)),
        summaryLines.join(QStringLiteral("\n")),
        detailLines.join(QStringLiteral("\n")));
    return true;
}

void CkmMapWidget::clearResult(const QString& placeholderText) {
    currentMetaFilePath_.clear();
    commonInfoText_.clear();
    defaultMetric_.clear();
    metricEntries_.clear();
    metricSamples_.clear();
    updatingMetricCombo_ = true;
    metricCombo_->clear();
    updatingMetricCombo_ = false;
    updatingViewModeCombo_ = true;
    viewModeCombo_->clear();
    updatingViewModeCombo_ = false;
    titleLabel_->setText(tr("Dense CKM Map"));
    infoLabel_->setText(QString());
    imageLabel_->setPixmap(QPixmap());
    basePixmap_ = QPixmap();
    currentMetric_.clear();
    currentImagePath_.clear();
    currentViewMode_.clear();
    currentImageClickable_ = false;
    currentSceneClickMapping_ = SceneClickMapping();
    currentScenePickMap_ = ScenePickMap();
    currentScenePickMapX_ = QImage();
    currentScenePickMapY_ = QImage();
    displayedPixmapSize_ = QSize();
    zoomFactor_ = 1.0;
    if (zoomResetButton_) {
        zoomResetButton_->setText(QStringLiteral("100%"));
    }
    hasSelectedPoint_ = false;
    hasSelectedImagePoint_ = false;
    hasExtent_ = false;
    setPlaceholderText(placeholderText.trimmed().isEmpty() ? tr("Dense CKM result not loaded yet.") : placeholderText);
    emitSelectionClearedHint(placeholderText.trimmed().isEmpty() ? tr("Dense CKM result not loaded yet.") : placeholderText);
}

QString CkmMapWidget::currentMetaFilePath() const {
    return currentMetaFilePath_;
}

void CkmMapWidget::setGenerationRunning(bool running) {
    generationRunning_ = running;
    if (generateButton_) {
        generateButton_->setEnabled(!generationRunning_);
    }
    if (stopButton_) {
        stopButton_->setEnabled(generationRunning_);
    }
    if (loadLocalButton_) {
        loadLocalButton_->setEnabled(true);
    }
}

void CkmMapWidget::setPlaceholderText(const QString& text) {
    displayedPixmapSize_ = QSize();
    imageLabel_->setPixmap(QPixmap());
    imageLabel_->setText(text);
    imageLabel_->setFixedSize(320, 220);
}

void CkmMapWidget::emitSelectionClearedHint(const QString& hint) {
    emit pointSelectionCleared(hint.trimmed().isEmpty() ? tr("Click the CKM scene-overlay image to inspect offline values.") : hint);
}

bool CkmMapWidget::eventFilter(QObject* watched, QEvent* event) {
    if (watched == imageLabel_ && event) {
        if (event->type() == QEvent::Wheel) {
            auto* wheelEvent = static_cast<QWheelEvent*>(event);
            if (wheelEvent->modifiers() & Qt::ControlModifier) {
                const QPoint angleDelta = wheelEvent->angleDelta();
                if (angleDelta.y() > 0) {
                    zoomBy(1.15);
                    return true;
                }
                if (angleDelta.y() < 0) {
                    zoomBy(1.0 / 1.15);
                    return true;
                }
            }
        } else if (event->type() == QEvent::MouseButtonPress) {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                QPointF worldPoint;
                QPointF imagePoint;
                if (mapWidgetPointToWorld(mouseEvent->localPos(), &worldPoint, &imagePoint)) {
                    if (selectNearestPoint(worldPoint, &imagePoint)) {
                        return true;
                    }
                } else if (currentViewMode_ != QStringLiteral("scene_overlay")) {
                    emitSelectionClearedHint(tr("Switch to Scene overlay, then click the map to inspect offline CKM values."));
                }
            }
        }
    }
    return QWidget::eventFilter(watched, event);
}

}  // namespace airsim_gui
