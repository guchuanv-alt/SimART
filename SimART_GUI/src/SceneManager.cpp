#include "airsim_gui_UErealtime/SceneManager.h"

#include "airsim_gui_UErealtime/AirSimSceneProvider.h"
#include "airsim_gui_UErealtime/FileSceneProvider.h"
#include "airsim_gui_UErealtime/OsmSceneProvider.h"

namespace airsim_gui {

SceneManager::SceneManager()
    : fileProvider_(new FileSceneProvider()),
      airsimProvider_(new AirSimSceneProvider()),
      osmProvider_(new OsmSceneProvider()) {}

SceneManager::~SceneManager() = default;

SceneGeometryBundle SceneManager::loadFromFile(const QString& filePath) {
    SceneLoadRequest request;
    request.type = SceneSourceType::FileMesh;
    request.filePath = filePath;
    return fileProvider_->load(request);
}

SceneGeometryBundle SceneManager::loadFromAirSim(const QString& host,
                                                 int port,
                                                 bool cacheToFile,
                                                 const QString& cachePath) {
    SceneLoadRequest request;
    request.type = SceneSourceType::AirSimRuntime;
    request.host = host;
    request.port = port;
    request.cacheToFile = cacheToFile;
    request.cacheOutputPath = cachePath;
    return airsimProvider_->load(request);
}

SceneGeometryBundle SceneManager::loadFromOsmFile(const QString& filePath,
                                                  const QString& displayName,
                                                  bool includeTrees,
                                                  bool includeStreetFurniture,
                                                  bool includeGreenAreas,
                                                  bool includeWater) {
    SceneLoadRequest request;
    request.type = SceneSourceType::OsmFile;
    request.filePath = filePath;
    request.sceneDisplayName = displayName;
    request.includeTrees = includeTrees;
    request.includeStreetFurniture = includeStreetFurniture;
    request.includeGreenAreas = includeGreenAreas;
    request.includeWater = includeWater;
    return osmProvider_->load(request);
}

SceneGeometryBundle SceneManager::loadFromOsmBoundingBox(double southLat,
                                                         double westLon,
                                                         double northLat,
                                                         double eastLon,
                                                         const QString& displayName,
                                                         bool includeTrees,
                                                         bool includeStreetFurniture,
                                                         bool includeGreenAreas,
                                                         bool includeWater,
                                                         const QString& overpassEndpoint,
                                                         bool useOsm2World,
                                                         const QString& osm2worldExecutable,
                                                         const QString& osm2worldOutputFormat,
                                                         const QString& osm2worldOutputDirectory,
                                                         const QString& osm2worldExtraArguments,
                                                         const QString& osm2worldJavaExecutable,
                                                         bool keepDownloadedOsmCopy,
                                                         std::function<void(const QString&, int, int)> progressCallback) {
    SceneLoadRequest request;
    request.type = SceneSourceType::OsmBoundingBox;
    request.sceneDisplayName = displayName;
    request.southLat = southLat;
    request.westLon = westLon;
    request.northLat = northLat;
    request.eastLon = eastLon;
    request.includeTrees = includeTrees;
    request.includeStreetFurniture = includeStreetFurniture;
    request.includeGreenAreas = includeGreenAreas;
    request.includeWater = includeWater;
    if (!overpassEndpoint.trimmed().isEmpty()) {
        request.overpassEndpoint = overpassEndpoint.trimmed();
    }
    request.useOsm2World = useOsm2World;
    request.osm2worldExecutable = osm2worldExecutable.trimmed();
    request.osm2worldOutputFormat = osm2worldOutputFormat.trimmed().isEmpty()
                                       ? QStringLiteral("glb")
                                       : osm2worldOutputFormat.trimmed().toLower();
    request.osm2worldOutputDirectory = osm2worldOutputDirectory.trimmed();
    request.osm2worldExtraArguments = osm2worldExtraArguments.trimmed();
    request.osm2worldJavaExecutable = osm2worldJavaExecutable.trimmed().isEmpty()
                                          ? QStringLiteral("java")
                                          : osm2worldJavaExecutable.trimmed();
    request.keepDownloadedOsmCopy = keepDownloadedOsmCopy;
    request.progressCallback = std::move(progressCallback);
    return osmProvider_->load(request);
}

}  // namespace airsim_gui
