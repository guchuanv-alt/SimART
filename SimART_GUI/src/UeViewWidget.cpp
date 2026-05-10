#include "airsim_gui_UErealtime/UeViewWidget.h"
#include "airsim_gui_UErealtime/AirSimViewController.h"

#include <QQuickImageProvider>
#include <QQuickWidget>
#include <QColor>
#include <QQmlContext>
#include <QQmlError>
#include <QVBoxLayout>
#include <QFileInfo>
#include <QUrl>

namespace airsim_gui {

namespace {
class LiveImageProvider : public QQuickImageProvider {
public:
    explicit LiveImageProvider(airsim_gui::AirSimViewController* controller)
        : QQuickImageProvider(QQuickImageProvider::Image), controller_(controller) {}

    QImage requestImage(const QString&, QSize* size, const QSize& requestedSize) override {
        QImage image = controller_ ? controller_->latestFrameCopy() : QImage();
        if (image.isNull()) {
            const int w = requestedSize.width() > 0 ? requestedSize.width() : 1280;
            const int h = requestedSize.height() > 0 ? requestedSize.height() : 720;
            image = QImage(w, h, QImage::Format_RGB32);
            image.fill(QColor(16, 19, 26));
        }
        if (size) *size = image.size();
        return image;
    }
private:
    airsim_gui::AirSimViewController* controller_;
};
}

UeViewWidget::UeViewWidget(QWidget* parent)
    : QWidget(parent) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    quickWidget_ = new QQuickWidget(this);
    quickWidget_->setResizeMode(QQuickWidget::SizeRootObjectToView);
    layout->addWidget(quickWidget_);

    controller_ = new AirSimViewController(this);
    quickWidget_->engine()->addImageProvider(QStringLiteral("airsimlive"), new LiveImageProvider(controller_));
    quickWidget_->rootContext()->setContextProperty(QStringLiteral("airSimView"), controller_);
}

QWidget* UeViewWidget::overlayHostWidget() const {
    return quickWidget_;
}

bool UeViewWidget::initialize(const QString& qmlPath, const QString& helperScriptPath, QString* errorMessage) {
    controller_->setScriptPath(helperScriptPath);
    if (!QFileInfo::exists(qmlPath)) {
        if (errorMessage) *errorMessage = QStringLiteral("QML file not found: %1").arg(qmlPath);
        return false;
    }
    quickWidget_->setSource(QUrl::fromLocalFile(QFileInfo(qmlPath).absoluteFilePath()));
    if (quickWidget_->status() == QQuickWidget::Error) {
        QStringList lines;
        for (const auto& err : quickWidget_->errors()) {
            lines << err.toString();
        }
        if (errorMessage) *errorMessage = lines.join(QStringLiteral("\n"));
        return false;
    }
    return true;
}

}  // namespace airsim_gui
