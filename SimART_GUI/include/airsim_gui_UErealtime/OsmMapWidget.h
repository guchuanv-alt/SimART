#pragma once

#include <QHash>
#include <QPixmap>
#include <QRectF>
#include <QWidget>

class QNetworkAccessManager;
class QNetworkReply;

namespace airsim_gui {

class OsmMapWidget : public QWidget {
    Q_OBJECT
public:
    explicit OsmMapWidget(QWidget* parent = nullptr);

    void centerOn(double lat, double lon);
    void setZoom(int zoom);
    int zoom() const { return zoom_; }
    void setSelectionMode(bool enabled);
    bool selectionMode() const { return selectionMode_; }
    bool hasSelection() const { return hasSelection_; }
    void clearSelection();
    void setPreviewBoundingBox(double southLat, double westLon, double northLat, double eastLon);
    void clearPreviewBoundingBox();
    void setAttributionText(const QString& text);

signals:
    void selectionChanged(double southLat, double westLon, double northLat, double eastLon);
    void statusTextChanged(const QString& text);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private slots:
    void onTileReplyFinished(QNetworkReply* reply);

private:
    struct TileKey {
        int z{0};
        int x{0};
        int y{0};
    };

    QString tileKeyString(const TileKey& key) const;
    QString tileCachePath(const TileKey& key) const;
    QPointF latLonToWorld(double lat, double lon, int zoom) const;
    QPointF latLonToWorld(double lat, double lon) const;
    QPointF worldToLatLon(const QPointF& world, int zoom) const;
    QPointF worldToLatLon(const QPointF& world) const;
    QPointF screenToWorld(const QPoint& screenPos) const;
    QRectF currentViewWorldRect() const;
    void ensureVisibleTiles();
    void requestTile(const TileKey& key);
    void clampCenter();
    void emitSelectionForScreenRect(const QRect& rect);

    QNetworkAccessManager* network_{nullptr};
    int zoom_{16};
    QPointF centerWorldPx_;
    QPoint lastPanPos_;
    QPoint selectionStart_;
    QPoint selectionEnd_;
    bool panning_{false};
    bool selecting_{false};
    bool selectionMode_{false};
    bool hasSelection_{false};
    QString attributionText_{QStringLiteral("© OpenStreetMap contributors")};
    QString cacheDir_;
    QRectF previewWorldRect_;
    QRectF selectionWorldRect_;
    QHash<QString, QPixmap> tilePixmaps_;
    QHash<QString, bool> pendingTiles_;
};

}  // namespace airsim_gui
