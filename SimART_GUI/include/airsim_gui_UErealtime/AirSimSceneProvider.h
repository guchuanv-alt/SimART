#pragma once

#include "airsim_gui_UErealtime/ISceneProvider.h"

namespace airsim_gui {

class AirSimSceneProvider : public ISceneProvider {
public:
    SceneGeometryBundle load(const SceneLoadRequest& request) override;

private:
    SceneGeometryBundle fetchFromAirSim(const QString& host, int port);
    bool writeCacheAsObj(const SceneGeometryBundle& bundle, const QString& filePath, QString* errorMessage);
};

}  // namespace airsim_gui
