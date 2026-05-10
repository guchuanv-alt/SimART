#pragma once

#include "airsim_gui_UErealtime/AppTypes.h"

#include <QObject>
#include <QTimer>
#include <vector>

namespace airsim_gui {

class DemoController : public QObject {
    Q_OBJECT
public:
    explicit DemoController(QObject* parent = nullptr);

    void setBaseStations(const std::vector<BaseStation>& stations);
    bool isRunning() const;

public slots:
    void start();
    void stop();

signals:
    void droneUpdated(const airsim_gui::DroneState& state);
    void pathsUpdated(const airsim_gui::BeamPathList& paths);
    void statusMessage(const QString& message);

private slots:
    void onTick();

private:
    std::vector<BaseStation> baseStations_;
    QTimer timer_;
    double timeSec_{0.0};
    DroneState drone_;
};

}  // namespace airsim_gui
