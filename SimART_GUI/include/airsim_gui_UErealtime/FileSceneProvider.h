#pragma once

#include "airsim_gui_UErealtime/ISceneProvider.h"

namespace airsim_gui {

class FileSceneProvider : public ISceneProvider {
public:
    SceneGeometryBundle load(const SceneLoadRequest& request) override;

private:
    SceneGeometryBundle loadMeshFile(const QString& filePath);
};

}  // namespace airsim_gui
