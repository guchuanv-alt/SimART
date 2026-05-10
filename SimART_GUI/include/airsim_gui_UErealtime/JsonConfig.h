#pragma once

#include "airsim_gui_UErealtime/AppTypes.h"

#include <QString>
#include <vector>

namespace airsim_gui {

class JsonConfig {
public:
    static bool loadBaseStations(const QString& filePath,
                                 std::vector<BaseStation>& stations,
                                 QString* errorMessage = nullptr);

    static bool loadWorldConfig(const QString& filePath,
                                WorldConfig& world,
                                QString* errorMessage = nullptr);

    static bool saveBaseStations(const QString& filePath,
                                 const std::vector<BaseStation>& stations,
                                 QString* errorMessage = nullptr);
};

}  // namespace airsim_gui
