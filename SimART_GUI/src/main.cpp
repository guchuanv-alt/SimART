#include "airsim_gui_UErealtime/MainWindow.h"
#include "airsim_gui_UErealtime/AppTypes.h"

#include <QApplication>
#include <QCoreApplication>
#include <QVTKWidget.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    ros::init(argc, argv, "airsim_gui_UErealtime", ros::init_options::AnonymousName);

    qRegisterMetaType<airsim_gui::DroneState>("airsim_gui::DroneState");
    qRegisterMetaType<airsim_gui::BeamPathList>("airsim_gui::BeamPathList");
    qRegisterMetaType<airsim_gui::RfObservationSnapshot>("airsim_gui::RfObservationSnapshot");
    qRegisterMetaType<std::vector<airsim_gui::BaseStation>>("std::vector<airsim_gui::BaseStation>");
    qRegisterMetaType<std::vector<airsim_gui::SceneBlock>>("std::vector<airsim_gui::SceneBlock>");

    QApplication app(argc, argv);

    airsim_gui::MainWindow window;
    window.show();
    return app.exec();
}
