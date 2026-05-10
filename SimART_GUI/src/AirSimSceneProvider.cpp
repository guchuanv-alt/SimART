#include "airsim_gui_UErealtime/AirSimSceneProvider.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#ifdef AIRSIM_GUI_ENABLE_AIRSIM
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#endif

namespace airsim_gui {
namespace {

SceneGeometryBundle makeError(const QString& text) {
    SceneGeometryBundle bundle;
    bundle.valid = false;
    bundle.errorMessage = text;
    return bundle;
}

}  // namespace

SceneGeometryBundle AirSimSceneProvider::load(const SceneLoadRequest& request) {
    SceneGeometryBundle bundle = fetchFromAirSim(request.host, request.port);
    if (!bundle.valid) {
        return bundle;
    }

    if (request.cacheToFile && !request.cacheOutputPath.isEmpty()) {
        QString error;
        if (!writeCacheAsObj(bundle, request.cacheOutputPath, &error)) {
            bundle.errorMessage = error;
        }
    }
    return bundle;
}

SceneGeometryBundle AirSimSceneProvider::fetchFromAirSim(const QString& host, int port) {
#ifndef AIRSIM_GUI_ENABLE_AIRSIM
    Q_UNUSED(host)
    Q_UNUSED(port)
    return makeError(
        "AirSim mesh import is compiled as optional support and is currently disabled. "
        "Enable it in CMake with -DAIRSIM_GUI_ENABLE_AIRSIM=ON and set AIRSIM_CLIENT_ROOT to your AirSim checkout.");
#else
    using msr::airlib::MultirotorRpcLibClient;
    SceneGeometryBundle bundle;
    bundle.sceneName = "AirSim Runtime Scene";

    MultirotorRpcLibClient client(host.toStdString(), static_cast<uint16_t>(port));
    client.confirmConnection();
    const auto responses = client.simGetMeshPositionVertexBuffers();

    if (responses.empty()) {
        return makeError("AirSim returned no static meshes.");
    }

    bundle.valid = true;
    bundle.meshes.reserve(responses.size());

    for (const auto& response : responses) {
        MeshGeometry mesh;
        mesh.id = QString::fromStdString(response.name);
        mesh.sourceName = QString::fromStdString(response.name);
        mesh.vertices.reserve(response.vertices.size());
        for (const auto value : response.vertices) {
            mesh.vertices.push_back(static_cast<float>(value));
        }
        mesh.indices.reserve(response.indices.size());
        for (const auto value : response.indices) {
            mesh.indices.push_back(static_cast<unsigned int>(value));
        }
        bundle.meshes.push_back(std::move(mesh));
    }

    if (bundle.meshes.empty()) {
        return makeError("AirSim mesh import succeeded but no valid mesh buffers were returned.");
    }
    return bundle;
#endif
}

bool AirSimSceneProvider::writeCacheAsObj(const SceneGeometryBundle& bundle,
                                          const QString& filePath,
                                          QString* errorMessage) {
    QFileInfo info(filePath);
    QDir().mkpath(info.absolutePath());

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (errorMessage) {
            *errorMessage = QString("Failed to write AirSim scene cache: %1").arg(filePath);
        }
        return false;
    }

    QTextStream out(&file);
    out << "# AirSim GUI cached scene\n";

    unsigned int vertexOffset = 1;
    for (const auto& mesh : bundle.meshes) {
        out << "o " << mesh.id << "\n";
        for (size_t i = 0; i + 2 < mesh.vertices.size(); i += 3) {
            out << "v " << mesh.vertices[i] << " " << mesh.vertices[i + 1] << " " << mesh.vertices[i + 2] << "\n";
        }
        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            out << "f "
                << mesh.indices[i] + vertexOffset << " "
                << mesh.indices[i + 1] + vertexOffset << " "
                << mesh.indices[i + 2] + vertexOffset << "\n";
        }
        vertexOffset += static_cast<unsigned int>(mesh.vertices.size() / 3);
    }
    return true;
}

}  // namespace airsim_gui
