#pragma once

#include "airsim_gui_UErealtime/SceneTypes.h"
#include <memory>

namespace airsim_gui {

class ISceneProvider;

class SceneManager {
public:
    SceneManager();
    ~SceneManager();

    SceneGeometryBundle loadFromFile(const QString& filePath);
    SceneGeometryBundle loadFromAirSim(const QString& host, int port,
                                       bool cacheToFile = false,
                                       const QString& cachePath = QString());
    SceneGeometryBundle loadFromOsmFile(const QString& filePath,
                                        const QString& displayName = QString(),
                                        bool includeTrees = true,
                                        bool includeStreetFurniture = true,
                                        bool includeGreenAreas = true,
                                        bool includeWater = true);
    SceneGeometryBundle loadFromOsmBoundingBox(double southLat, double westLon,
                                               double northLat, double eastLon,
                                               const QString& displayName = QString(),
                                               bool includeTrees = true,
                                               bool includeStreetFurniture = true,
                                               bool includeGreenAreas = true,
                                               bool includeWater = true,
                                               const QString& overpassEndpoint = QStringLiteral("https://overpass-api.de/api/interpreter"),
                                               bool useOsm2World = false,
                                               const QString& osm2worldExecutable = QString(),
                                               const QString& osm2worldOutputFormat = QStringLiteral("glb"),
                                               const QString& osm2worldOutputDirectory = QString(),
                                               const QString& osm2worldExtraArguments = QString(),
                                               const QString& osm2worldJavaExecutable = QStringLiteral("java"),
                                               bool keepDownloadedOsmCopy = true,
                                               std::function<void(const QString&, int, int)> progressCallback = {});

private:
    std::unique_ptr<ISceneProvider> fileProvider_;
    std::unique_ptr<ISceneProvider> airsimProvider_;
    std::unique_ptr<ISceneProvider> osmProvider_;
};

}  // namespace airsim_gui
