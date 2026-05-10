#pragma once

#include <QString>
#include <functional>
#include <QVector3D>
#include <vector>

namespace airsim_gui {

struct MeshMaterialInfo {
    QString name;
    QString texturePath;
    QVector3D diffuseColor{0.72f, 0.74f, 0.78f};
    float opacity{1.0f};
};

struct MeshGeometry {
    QString id;
    QString sourceName;
    std::vector<float> vertices;          // xyzxyzxyz...
    std::vector<unsigned int> indices;    // triangle indices
    std::vector<float> normals;           // xyzxyzxyz...
    std::vector<float> texCoords;         // uvuvuv...
    QVector3D translation{0.0f, 0.0f, 0.0f};
    QVector3D rotationDeg{0.0f, 0.0f, 0.0f};
    QVector3D scale{1.0f, 1.0f, 1.0f};
    MeshMaterialInfo material;
    bool visible{true};
};

struct SceneGeometryBundle {
    QString sceneName;
    std::vector<MeshGeometry> meshes;
    bool valid{false};
    QString errorMessage;
    QString sourcePath;
    QString downloadedOsmPath;
    QString generatedMeshPath;
    QString generatedMeshFormat;
    QString note;
};

enum class SceneSourceType {
    FileMesh,
    AirSimRuntime,
    OsmFile,
    OsmBoundingBox
};

struct SceneLoadRequest {
    SceneSourceType type{SceneSourceType::FileMesh};
    QString filePath;
    QString host{"127.0.0.1"};
    int port{41451};
    bool cacheToFile{false};
    QString cacheOutputPath;

    // OSM-specific options.
    QString sceneDisplayName;
    double southLat{0.0};
    double westLon{0.0};
    double northLat{0.0};
    double eastLon{0.0};
    bool includeTrees{true};
    bool includeStreetFurniture{true};
    bool includeGreenAreas{true};
    bool includeWater{true};
    QString overpassEndpoint{"https://overpass-api.de/api/interpreter"};
    bool useOsm2World{false};
    QString osm2worldExecutable;
    QString osm2worldJavaExecutable{"java"};
    QString osm2worldOutputFormat{"glb"};
    QString osm2worldOutputDirectory;
    QString osm2worldExtraArguments;
    bool keepDownloadedOsmCopy{true};

    std::function<void(const QString&, int, int)> progressCallback;
};

}  // namespace airsim_gui
