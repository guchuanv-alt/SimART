#pragma once

#include "airsim_gui_UErealtime/ISceneProvider.h"

namespace airsim_gui {

class OsmSceneProvider : public ISceneProvider {
public:
    SceneGeometryBundle load(const SceneLoadRequest& request) override;

private:
    SceneGeometryBundle loadFromLocalFile(const SceneLoadRequest& request);
    SceneGeometryBundle loadFromBoundingBox(const SceneLoadRequest& request);
};

}  // namespace airsim_gui
