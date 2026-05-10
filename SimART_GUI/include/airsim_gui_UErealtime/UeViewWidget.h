#pragma once

#include <QWidget>

class QQuickWidget;

namespace airsim_gui {

class AirSimViewController;

class UeViewWidget : public QWidget {
    Q_OBJECT
public:
    explicit UeViewWidget(QWidget* parent = nullptr);

    AirSimViewController* controller() const { return controller_; }
    QWidget* overlayHostWidget() const;
    bool initialize(const QString& qmlPath, const QString& helperScriptPath, QString* errorMessage = nullptr);

private:
    QQuickWidget* quickWidget_{nullptr};
    AirSimViewController* controller_{nullptr};
};

}  // namespace airsim_gui
