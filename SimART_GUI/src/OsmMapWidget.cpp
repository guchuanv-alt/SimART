#include "airsim_gui_UErealtime/OsmMapWidget.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMouseEvent>
#include <QNetworkAccessManager>
#include <QNetworkDiskCache>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPainter>
#include <QResizeEvent>
#include <QStandardPaths>
#include <QtMath>

#include <cmath>

namespace airsim_gui {
namespace {

constexpr int kTileSize = 256;
constexpr double kMinLatitude = -85.05112878;
constexpr double kMaxLatitude = 85.05112878;

QString appUserAgent() {
    return QStringLiteral("AirSimGUI-UERealtime/OSMImport (+local desktop app)");
}

double clampLat(double lat) {
    return qBound(kMinLatitude, lat, kMaxLatitude);
}

double clampLon(double lon) {
    while (lon < -180.0) {
        lon += 360.0;
    }
    while (lon > 180.0) {
        lon -= 360.0;
    }
    return lon;
}

}  // namespace

OsmMapWidget::OsmMapWidget(QWidget* parent)
    : QWidget(parent),
      network_(new QNetworkAccessManager(this)) {
    setMouseTracking(true);
    setMinimumSize(720, 420);

    QString cacheBase = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
    if (cacheBase.isEmpty()) {
        cacheBase = QDir::tempPath() + QStringLiteral("/airsim_gui_UErealtime_cache");
    }
    cacheDir_ = QDir(cacheBase).absoluteFilePath(QStringLiteral("osm_tiles"));
    QDir().mkpath(cacheDir_);

    auto* diskCache = new QNetworkDiskCache(this);
    diskCache->setCacheDirectory(cacheDir_);
    diskCache->setMaximumCacheSize(256ll * 1024ll * 1024ll);
    network_->setCache(diskCache);
    connect(network_, &QNetworkAccessManager::finished, this, &OsmMapWidget::onTileReplyFinished);

    centerOn(39.9042, 116.4074);
}

void OsmMapWidget::centerOn(double lat, double lon) {
    centerWorldPx_ = latLonToWorld(clampLat(lat), clampLon(lon));
    clampCenter();
    ensureVisibleTiles();
    update();
}

void OsmMapWidget::setZoom(int zoom) {
    const int clamped = qBound(2, zoom, 19);
    if (clamped == zoom_) {
        return;
    }

    const QPointF currentLatLon = worldToLatLon(centerWorldPx_, zoom_);
    zoom_ = clamped;
    centerWorldPx_ = latLonToWorld(currentLatLon.y(), currentLatLon.x(), zoom_);
    clampCenter();
    ensureVisibleTiles();
    update();
}

void OsmMapWidget::setSelectionMode(bool enabled) {
    selectionMode_ = enabled;
    if (!selectionMode_) {
        selecting_ = false;
    }
    update();
}

void OsmMapWidget::clearSelection() {
    hasSelection_ = false;
    selectionWorldRect_ = QRectF();
    update();
}

void OsmMapWidget::setPreviewBoundingBox(double southLat, double westLon, double northLat, double eastLon) {
    const QPointF sw = latLonToWorld(southLat, westLon);
    const QPointF ne = latLonToWorld(northLat, eastLon);
    previewWorldRect_ = QRectF(QPointF(sw.x(), ne.y()), QPointF(ne.x(), sw.y())).normalized();
    update();
}

void OsmMapWidget::clearPreviewBoundingBox() {
    previewWorldRect_ = QRectF();
    update();
}

void OsmMapWidget::setAttributionText(const QString& text) {
    attributionText_ = text;
    update();
}

QString OsmMapWidget::tileKeyString(const TileKey& key) const {
    return QStringLiteral("%1_%2_%3").arg(key.z).arg(key.x).arg(key.y);
}

QString OsmMapWidget::tileCachePath(const TileKey& key) const {
    return QDir(cacheDir_).absoluteFilePath(QStringLiteral("%1/%2/%3.png").arg(key.z).arg(key.x).arg(key.y));
}

QPointF OsmMapWidget::latLonToWorld(double lat, double lon, int zoom) const {
    const double clampedLat = clampLat(lat);
    const double clampedLon = clampLon(lon);
    const double sinLat = qSin(qDegreesToRadians(clampedLat));
    const double scale = kTileSize * static_cast<double>(1 << zoom);
    const double x = (clampedLon + 180.0) / 360.0 * scale;
    const double y = (0.5 - qLn((1.0 + sinLat) / (1.0 - sinLat)) / (4.0 * M_PI)) * scale;
    return QPointF(x, y);
}

QPointF OsmMapWidget::latLonToWorld(double lat, double lon) const {
    return latLonToWorld(lat, lon, zoom_);
}

QPointF OsmMapWidget::worldToLatLon(const QPointF& world, int zoom) const {
    const double scale = kTileSize * static_cast<double>(1 << zoom);
    const double lon = world.x() / scale * 360.0 - 180.0;
    const double mercY = M_PI * (1.0 - 2.0 * world.y() / scale);
    const double lat = qRadiansToDegrees(qAtan(std::sinh(mercY)));
    return QPointF(clampLon(lon), clampLat(lat));
}

QPointF OsmMapWidget::worldToLatLon(const QPointF& world) const {
    return worldToLatLon(world, zoom_);
}

QPointF OsmMapWidget::screenToWorld(const QPoint& screenPos) const {
    return QPointF(centerWorldPx_.x() - width() * 0.5 + screenPos.x(),
                   centerWorldPx_.y() - height() * 0.5 + screenPos.y());
}

QRectF OsmMapWidget::currentViewWorldRect() const {
    return QRectF(centerWorldPx_.x() - width() * 0.5,
                  centerWorldPx_.y() - height() * 0.5,
                  width(),
                  height());
}

void OsmMapWidget::ensureVisibleTiles() {
    const QRectF worldRect = currentViewWorldRect();
    const int minTileX = static_cast<int>(qFloor(worldRect.left() / kTileSize)) - 1;
    const int maxTileX = static_cast<int>(qFloor(worldRect.right() / kTileSize)) + 1;
    const int minTileY = static_cast<int>(qFloor(worldRect.top() / kTileSize)) - 1;
    const int maxTileY = static_cast<int>(qFloor(worldRect.bottom() / kTileSize)) + 1;
    const int maxIndex = (1 << zoom_) - 1;

    for (int tileX = minTileX; tileX <= maxTileX; ++tileX) {
        if (tileX < 0 || tileX > maxIndex) {
            continue;
        }
        for (int tileY = minTileY; tileY <= maxTileY; ++tileY) {
            if (tileY < 0 || tileY > maxIndex) {
                continue;
            }
            requestTile({zoom_, tileX, tileY});
        }
    }
}

void OsmMapWidget::requestTile(const TileKey& key) {
    const QString keyString = tileKeyString(key);
    if (tilePixmaps_.contains(keyString) || pendingTiles_.contains(keyString)) {
        return;
    }

    const QString cachePath = tileCachePath(key);
    if (QFile::exists(cachePath)) {
        QPixmap pixmap(cachePath);
        if (!pixmap.isNull()) {
            tilePixmaps_.insert(keyString, pixmap);
            return;
        }
    }

    QDir().mkpath(QFileInfo(cachePath).absolutePath());

    QNetworkRequest request(QUrl(QStringLiteral("https://tile.openstreetmap.org/%1/%2/%3.png").arg(key.z).arg(key.x).arg(key.y)));
    request.setRawHeader("User-Agent", appUserAgent().toUtf8());
    request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::PreferCache);
    auto* reply = network_->get(request);
    reply->setProperty("tile_key", keyString);
    reply->setProperty("cache_path", cachePath);
    pendingTiles_.insert(keyString, true);
}

void OsmMapWidget::clampCenter() {
    const double scale = kTileSize * static_cast<double>(1 << zoom_);
    const double minX = width() * 0.5;
    const double maxX = qMax(minX, scale - width() * 0.5);
    const double minY = height() * 0.5;
    const double maxY = qMax(minY, scale - height() * 0.5);
    centerWorldPx_.setX(qBound(minX, centerWorldPx_.x(), maxX));
    centerWorldPx_.setY(qBound(minY, centerWorldPx_.y(), maxY));
}

void OsmMapWidget::emitSelectionForScreenRect(const QRect& rect) {
    const QRect normalized = rect.normalized();
    if (normalized.width() < 6 || normalized.height() < 6) {
        return;
    }
    const QPointF topLeftWorld = screenToWorld(normalized.topLeft());
    const QPointF bottomRightWorld = screenToWorld(normalized.bottomRight());
    selectionWorldRect_ = QRectF(topLeftWorld, bottomRightWorld).normalized();
    hasSelection_ = true;

    const QPointF topLeftLatLon = worldToLatLon(QPointF(selectionWorldRect_.left(), selectionWorldRect_.top()));
    const QPointF bottomRightLatLon = worldToLatLon(QPointF(selectionWorldRect_.right(), selectionWorldRect_.bottom()));
    const double northLat = topLeftLatLon.y();
    const double southLat = bottomRightLatLon.y();
    const double westLon = topLeftLatLon.x();
    const double eastLon = bottomRightLatLon.x();
    emit selectionChanged(southLat, westLon, northLat, eastLon);
}

void OsmMapWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event)
    ensureVisibleTiles();

    QPainter painter(this);
    painter.fillRect(rect(), QColor(243, 245, 247));

    const QRectF worldRect = currentViewWorldRect();
    const int minTileX = static_cast<int>(qFloor(worldRect.left() / kTileSize)) - 1;
    const int maxTileX = static_cast<int>(qFloor(worldRect.right() / kTileSize)) + 1;
    const int minTileY = static_cast<int>(qFloor(worldRect.top() / kTileSize)) - 1;
    const int maxTileY = static_cast<int>(qFloor(worldRect.bottom() / kTileSize)) + 1;
    const int maxIndex = (1 << zoom_) - 1;

    for (int tileX = minTileX; tileX <= maxTileX; ++tileX) {
        if (tileX < 0 || tileX > maxIndex) {
            continue;
        }
        for (int tileY = minTileY; tileY <= maxTileY; ++tileY) {
            if (tileY < 0 || tileY > maxIndex) {
                continue;
            }
            const QString key = tileKeyString({zoom_, tileX, tileY});
            const QRect target(static_cast<int>(tileX * kTileSize - worldRect.left()),
                               static_cast<int>(tileY * kTileSize - worldRect.top()),
                               kTileSize,
                               kTileSize);
            if (tilePixmaps_.contains(key) && !tilePixmaps_.value(key).isNull()) {
                painter.drawPixmap(target, tilePixmaps_.value(key));
            } else {
                painter.fillRect(target, QColor(226, 231, 235));
                painter.setPen(QColor(180, 188, 194));
                painter.drawRect(target.adjusted(0, 0, -1, -1));
            }
        }
    }

    if (previewWorldRect_.isValid()) {
        QRect previewScreen(static_cast<int>(previewWorldRect_.left() - worldRect.left()),
                            static_cast<int>(previewWorldRect_.top() - worldRect.top()),
                            static_cast<int>(previewWorldRect_.width()),
                            static_cast<int>(previewWorldRect_.height()));
        QColor fill(41, 128, 185, 45);
        QColor outline(41, 128, 185, 180);
        painter.fillRect(previewScreen, fill);
        painter.setPen(QPen(outline, 2));
        painter.drawRect(previewScreen.normalized());
    }

    if (hasSelection_ && selectionWorldRect_.isValid()) {
        QRect selectedScreen(static_cast<int>(selectionWorldRect_.left() - worldRect.left()),
                             static_cast<int>(selectionWorldRect_.top() - worldRect.top()),
                             static_cast<int>(selectionWorldRect_.width()),
                             static_cast<int>(selectionWorldRect_.height()));
        QColor fill(231, 76, 60, 48);
        QColor outline(231, 76, 60, 220);
        painter.fillRect(selectedScreen, fill);
        painter.setPen(QPen(outline, 2));
        painter.drawRect(selectedScreen.normalized());
    }

    if (selecting_) {
        painter.setPen(QPen(QColor(231, 76, 60, 220), 2, Qt::DashLine));
        painter.setBrush(QColor(231, 76, 60, 35));
        painter.drawRect(QRect(selectionStart_, selectionEnd_).normalized());
    }

    painter.setPen(QColor(30, 30, 30));
    painter.setBrush(QColor(255, 255, 255, 220));
    const QRect badgeRect(12, 12, 170, 28);
    painter.drawRoundedRect(badgeRect, 6, 6);
    painter.drawText(badgeRect.adjusted(10, 0, -10, 0), Qt::AlignVCenter | Qt::AlignLeft,
                     selectionMode_ ? tr("Selection mode: drag") : tr("Pan mode"));

    const QRect attributionRect(width() - 230, height() - 28, 218, 20);
    painter.setBrush(QColor(255, 255, 255, 220));
    painter.setPen(QColor(60, 60, 60));
    painter.drawRoundedRect(attributionRect, 4, 4);
    painter.drawText(attributionRect.adjusted(8, 0, -8, 0), Qt::AlignVCenter | Qt::AlignLeft, attributionText_);
}

void OsmMapWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    clampCenter();
    ensureVisibleTiles();
}

void OsmMapWidget::mousePressEvent(QMouseEvent* event) {
    if (selectionMode_ && event->button() == Qt::LeftButton) {
        selecting_ = true;
        selectionStart_ = event->pos();
        selectionEnd_ = event->pos();
        return;
    }
    if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton || event->button() == Qt::RightButton) {
        panning_ = true;
        lastPanPos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
}

void OsmMapWidget::mouseMoveEvent(QMouseEvent* event) {
    const QPointF latLon = worldToLatLon(screenToWorld(event->pos()));
    emit statusTextChanged(QStringLiteral("lat %1, lon %2, zoom %3")
                               .arg(latLon.y(), 0, 'f', 6)
                               .arg(latLon.x(), 0, 'f', 6)
                               .arg(zoom_));

    if (selecting_) {
        selectionEnd_ = event->pos();
        update();
        return;
    }

    if (panning_) {
        const QPoint delta = event->pos() - lastPanPos_;
        centerWorldPx_ -= QPointF(delta.x(), delta.y());
        lastPanPos_ = event->pos();
        clampCenter();
        ensureVisibleTiles();
        update();
    }
}

void OsmMapWidget::mouseReleaseEvent(QMouseEvent* event) {
    Q_UNUSED(event)
    if (selecting_) {
        selecting_ = false;
        emitSelectionForScreenRect(QRect(selectionStart_, selectionEnd_));
        update();
    }
    if (panning_) {
        panning_ = false;
        unsetCursor();
    }
}

void OsmMapWidget::wheelEvent(QWheelEvent* event) {
    const QPoint angleDelta = event->angleDelta();
    if (angleDelta.y() == 0) {
        return;
    }

    const QPoint mousePos = event->pos();
    const QPointF beforeLatLon = worldToLatLon(screenToWorld(mousePos));
    setZoom(zoom_ + (angleDelta.y() > 0 ? 1 : -1));
    const QPointF afterWorld = latLonToWorld(beforeLatLon.y(), beforeLatLon.x());
    centerWorldPx_ = QPointF(afterWorld.x() - mousePos.x() + width() * 0.5,
                             afterWorld.y() - mousePos.y() + height() * 0.5);
    clampCenter();
    ensureVisibleTiles();
    update();
}

void OsmMapWidget::onTileReplyFinished(QNetworkReply* reply) {
    const QString tileKey = reply->property("tile_key").toString();
    const QString cachePath = reply->property("cache_path").toString();
    pendingTiles_.remove(tileKey);

    if (reply->error() == QNetworkReply::NoError) {
        const QByteArray bytes = reply->readAll();
        QPixmap pixmap;
        if (pixmap.loadFromData(bytes)) {
            tilePixmaps_.insert(tileKey, pixmap);
            if (!cachePath.isEmpty()) {
                QFile file(cachePath);
                if (file.open(QIODevice::WriteOnly)) {
                    file.write(bytes);
                }
            }
            update();
        }
    }
    reply->deleteLater();
}

}  // namespace airsim_gui
