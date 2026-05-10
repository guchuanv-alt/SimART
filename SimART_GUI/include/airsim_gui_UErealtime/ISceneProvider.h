#pragma once

#include "airsim_gui_UErealtime/SceneTypes.h"

namespace airsim_gui {

class ISceneProvider {
public:
    virtual ~ISceneProvider() = default;
    virtual SceneGeometryBundle load(const SceneLoadRequest& request) = 0;
};

}  // namespace airsim_gui
