#include "airsim_gui_UErealtime/DemoController.h"

#include <QtMath>

namespace airsim_gui {

DemoController::DemoController(QObject* parent)
    : QObject(parent) {
    timer_.setInterval(50);
    connect(&timer_, &QTimer::timeout, this, &DemoController::onTick);
}

void DemoController::setBaseStations(const std::vector<BaseStation>& stations) {
    baseStations_ = stations;
}

bool DemoController::isRunning() const {
    return timer_.isActive();
}

void DemoController::start() {
    if (timer_.isActive()) {
        return;
    }
    timeSec_ = 0.0;
    drone_ = DroneState{};
    drone_.id = "uav_demo";
    drone_.valid = true;
    timer_.start();
    emit statusMessage("Demo source started.");
}

void DemoController::stop() {
    if (!timer_.isActive()) {
        return;
    }
    timer_.stop();
    emit statusMessage("Demo source stopped.");
}

void DemoController::onTick() {
    timeSec_ += 0.05;

    drone_.position.x = 45.0 * qCos(timeSec_ * 0.35);
    drone_.position.y = 30.0 * qSin(timeSec_ * 0.55);
    drone_.position.z = 20.0 + 8.0 * qSin(timeSec_ * 0.22);
    drone_.yawDeg = qRadiansToDegrees(qAtan2(0.55 * 30.0 * qCos(timeSec_ * 0.55),
                                             -0.35 * 45.0 * qSin(timeSec_ * 0.35)));
    drone_.pitchDeg = 3.5 * qSin(timeSec_ * 0.6);
    drone_.rollDeg = 6.0 * qCos(timeSec_ * 0.8);
    drone_.history.push_back(drone_.position);
    if (drone_.history.size() > 300) {
        drone_.history.erase(drone_.history.begin(), drone_.history.begin() + (drone_.history.size() - 300));
    }

    BeamPathList paths;
    int idx = 0;
    for (const BaseStation& bs : baseStations_) {
        BeamPath los;
        los.id = QString("%1_los").arg(bs.id);
        los.txId = bs.id;
        los.rxId = drone_.id;
        los.points = {bs.position, drone_.position};
        const double distance = qSqrt(qPow(bs.position.x - drone_.position.x, 2.0)
                                    + qPow(bs.position.y - drone_.position.y, 2.0)
                                    + qPow(bs.position.z - drone_.position.z, 2.0));
        los.powerDb = -48.0 - 0.35 * distance;
        los.delayNs = distance / 0.299792458;
        los.kind = "los";
        paths.push_back(los);

        BeamPath reflected;
        reflected.id = QString("%1_reflect").arg(bs.id);
        reflected.txId = bs.id;
        reflected.rxId = drone_.id;
        reflected.kind = "specular";
        reflected.powerDb = los.powerDb - 8.0 - idx * 2.0;
        reflected.delayNs = los.delayNs + 50.0 + idx * 20.0;
        Vec3 bounce;
        bounce.x = 0.55 * (bs.position.x + drone_.position.x) + 8.0 * qCos(timeSec_ + idx);
        bounce.y = 0.55 * (bs.position.y + drone_.position.y) - 5.0 * qSin(timeSec_ * 0.8 + idx);
        bounce.z = 8.0 + 2.0 * idx;
        reflected.points = {bs.position, bounce, drone_.position};
        paths.push_back(reflected);
        ++idx;
    }

    emit droneUpdated(drone_);
    emit pathsUpdated(paths);
}

}  // namespace airsim_gui
