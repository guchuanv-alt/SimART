#include "airsim_gui_UErealtime/FileSceneProvider.h"
#include "airsim_gui_UErealtime/TransformUtils.h"

#include <QColor>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QStandardPaths>

#include <utility>
#include <map>
#include <limits>
#include <cmath>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace airsim_gui {
namespace {

SceneGeometryBundle makeError(const QString& text) {
    SceneGeometryBundle bundle;
    bundle.valid = false;
    bundle.errorMessage = text;
    return bundle;
}

QString cacheDirectoryFor(const QFileInfo& sourceInfo) {
    QString root = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
    if (root.isEmpty()) {
        root = QDir::tempPath() + "/airsim_gui_UErealtime_cache";
    }

    const QByteArray key = QCryptographicHash::hash(sourceInfo.absoluteFilePath().toUtf8(), QCryptographicHash::Md5).toHex();
    QDir dir(root);
    dir.mkpath(QStringLiteral("embedded_textures/%1").arg(QString::fromLatin1(key)));
    return dir.absoluteFilePath(QStringLiteral("embedded_textures/%1").arg(QString::fromLatin1(key)));
}

QString resolveTexturePath(const QFileInfo& sourceInfo, const QString& rawPath) {
    if (rawPath.isEmpty()) {
        return QString();
    }

    QFileInfo textureInfo(rawPath);
    if (textureInfo.isAbsolute() && textureInfo.exists()) {
        return textureInfo.absoluteFilePath();
    }

    const QString normalized = QDir::fromNativeSeparators(rawPath);
    const QString baseName = QFileInfo(normalized).fileName();
    const QString sourceDir = sourceInfo.absolutePath();

    const QStringList candidates = {
        QDir(sourceDir).absoluteFilePath(normalized),
        QDir(sourceDir).absoluteFilePath(baseName),
        QDir(sourceDir + "/textures").absoluteFilePath(baseName),
        QDir(sourceDir + "/Textures").absoluteFilePath(baseName)
    };

    for (const QString& candidate : candidates) {
        if (QFileInfo::exists(candidate)) {
            return QFileInfo(candidate).absoluteFilePath();
        }
    }
    return QString();
}

QString writeEmbeddedTextureToDisk(const aiScene* scene,
                                   const QString& textureRef,
                                   const QFileInfo& sourceInfo) {
    if (!scene || !textureRef.startsWith('*')) {
        return QString();
    }

    bool ok = false;
    const int textureIndex = textureRef.mid(1).toInt(&ok);
    if (!ok || textureIndex < 0 || textureIndex >= static_cast<int>(scene->mNumTextures)) {
        return QString();
    }

    const aiTexture* embedded = scene->mTextures[textureIndex];
    if (!embedded) {
        return QString();
    }

    const QString cacheDir = cacheDirectoryFor(sourceInfo);
    QString extension;
    if (embedded->achFormatHint[0] != '\0') {
        extension = QString::fromLatin1(embedded->achFormatHint).trimmed().toLower();
    }
    if (extension.isEmpty()) {
        extension = embedded->mHeight == 0 ? QStringLiteral("bin") : QStringLiteral("png");
    }

    const QString outputPath = QDir(cacheDir).absoluteFilePath(
        QStringLiteral("embedded_%1.%2").arg(textureIndex).arg(extension));

    if (QFileInfo::exists(outputPath)) {
        return outputPath;
    }

    if (embedded->mHeight == 0) {
        QFile file(outputPath);
        if (!file.open(QIODevice::WriteOnly)) {
            return QString();
        }
        file.write(reinterpret_cast<const char*>(embedded->pcData), static_cast<qint64>(embedded->mWidth));
        file.close();
        return outputPath;
    }

    QImage image(static_cast<int>(embedded->mWidth), static_cast<int>(embedded->mHeight), QImage::Format_RGBA8888);
    for (unsigned int y = 0; y < embedded->mHeight; ++y) {
        for (unsigned int x = 0; x < embedded->mWidth; ++x) {
            const aiTexel& texel = embedded->pcData[y * embedded->mWidth + x];
            image.setPixelColor(static_cast<int>(x), static_cast<int>(y), QColor(texel.r, texel.g, texel.b, texel.a));
        }
    }
    if (!image.save(outputPath)) {
        return QString();
    }
    return outputPath;
}

MeshMaterialInfo readMaterial(const aiScene* scene,
                              const aiMaterial* material,
                              const QFileInfo& sourceInfo) {
    MeshMaterialInfo info;
    if (!material) {
        return info;
    }

    aiString name;
    if (material->Get(AI_MATKEY_NAME, name) == AI_SUCCESS) {
        info.name = QString::fromUtf8(name.C_Str());
    }

    aiColor3D diffuse(0.72f, 0.74f, 0.78f);
    if (material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse) == AI_SUCCESS) {
        info.diffuseColor = QVector3D(diffuse.r, diffuse.g, diffuse.b);
    }

    float opacity = 1.0f;
    if (material->Get(AI_MATKEY_OPACITY, opacity) == AI_SUCCESS) {
        info.opacity = opacity;
    }

    aiString texturePath;
    if (material->GetTexture(aiTextureType_BASE_COLOR, 0, &texturePath) == AI_SUCCESS ||
        material->GetTexture(aiTextureType_DIFFUSE, 0, &texturePath) == AI_SUCCESS) {
        const QString rawPath = QString::fromUtf8(texturePath.C_Str());
        if (rawPath.startsWith('*')) {
            info.texturePath = writeEmbeddedTextureToDisk(scene, rawPath, sourceInfo);
        } else {
            info.texturePath = resolveTexturePath(sourceInfo, rawPath);
        }
    }

    return info;
}

QString materialGroupKey(const MeshMaterialInfo& material) {
    return QStringLiteral("%1|%2|%3|%4|%5|%6")
        .arg(material.texturePath,
             material.name,
             QString::number(material.diffuseColor.x(), 'f', 4),
             QString::number(material.diffuseColor.y(), 'f', 4),
             QString::number(material.diffuseColor.z(), 'f', 4),
             QString::number(material.opacity, 'f', 4));
}

bool usesGltfCoordinateConversion(const QString& suffix) {
    return suffix == QStringLiteral("gltf") || suffix == QStringLiteral("glb");
}

QString gltfImportTransformConfigPath() {
    return findPackagedConfigFile(QStringLiteral("config/gltf_import_transform.json"));
}

double determinant3x3(const TransformMatrix4x4& matrix) {
    return matrix.m[0][0] * (matrix.m[1][1] * matrix.m[2][2] - matrix.m[1][2] * matrix.m[2][1])
         - matrix.m[0][1] * (matrix.m[1][0] * matrix.m[2][2] - matrix.m[1][2] * matrix.m[2][0])
         + matrix.m[0][2] * (matrix.m[1][0] * matrix.m[2][1] - matrix.m[1][1] * matrix.m[2][0]);
}

void transformTriples(std::vector<float>* values,
                      const TransformMatrix4x4& matrix,
                      bool treatAsPoint) {
    if (!values) {
        return;
    }
    for (size_t i = 0; i + 2 < values->size(); i += 3) {
        const Vec3 in{(*values)[i], (*values)[i + 1], (*values)[i + 2]};
        const Vec3 out = treatAsPoint ? transformPointByMatrix(matrix, in)
                                      : transformVectorByMatrix(matrix, in);
        (*values)[i] = static_cast<float>(out.x);
        (*values)[i + 1] = static_cast<float>(out.y);
        (*values)[i + 2] = static_cast<float>(out.z);

        if (!treatAsPoint) {
            const double len = std::sqrt(static_cast<double>((*values)[i]) * (*values)[i] +
                                         static_cast<double>((*values)[i + 1]) * (*values)[i + 1] +
                                         static_cast<double>((*values)[i + 2]) * (*values)[i + 2]);
            if (len > 1e-10) {
                (*values)[i] = static_cast<float>((*values)[i] / len);
                (*values)[i + 1] = static_cast<float>((*values)[i + 1] / len);
                (*values)[i + 2] = static_cast<float>((*values)[i + 2] / len);
            }
        }
    }
}

void applyGltfCoordinateConversion(SceneGeometryBundle* bundle) {
    if (!bundle) {
        return;
    }

    QString error;
    const QString configPath = gltfImportTransformConfigPath();
    const TransformMatrix4x4 matrix = loadTransformMatrixFromJson(configPath, &error);
    const double det = determinant3x3(matrix);

    for (auto& mesh : bundle->meshes) {
        transformTriples(&mesh.vertices, matrix, true);
        transformTriples(&mesh.normals, matrix, false);

        if (det < 0.0) {
            for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                std::swap(mesh.indices[i + 1], mesh.indices[i + 2]);
            }
        }
    }

    if (!bundle->note.isEmpty()) {
        bundle->note += QLatin1Char(' ');
    }
    if (!error.isEmpty()) {
        bundle->note += QStringLiteral("GLTF/GLB import transform fallback: %1").arg(error);
    } else {
        bundle->note += QStringLiteral("Applied GLTF/GLB import transform from %1. Matrix: %2")
                            .arg(configPath, transformMatrixToDisplayString(matrix).replace(QStringLiteral("\n"), QStringLiteral(" ")));
    }
}

void normalizeBundle(SceneGeometryBundle* bundle) {
    if (!bundle || bundle->meshes.empty()) return;
    float minX=std::numeric_limits<float>::max(), minY=minX, minZ=minX;
    float maxX=std::numeric_limits<float>::lowest(), maxY=maxX, maxZ=maxX;
    for (const auto& mesh : bundle->meshes) {
        for (size_t i=0; i+2<mesh.vertices.size(); i+=3) {
            minX = std::min(minX, mesh.vertices[i]);
            minY = std::min(minY, mesh.vertices[i+1]);
            minZ = std::min(minZ, mesh.vertices[i+2]);
            maxX = std::max(maxX, mesh.vertices[i]);
            maxY = std::max(maxY, mesh.vertices[i+1]);
            maxZ = std::max(maxZ, mesh.vertices[i+2]);
        }
    }
    if (!(minX <= maxX && minY <= maxY && minZ <= maxZ)) return;
    const float cx = 0.5f * (minX + maxX);
    const float cy = 0.5f * (minY + maxY);
    const float maxCoord = std::max({std::fabs(minX), std::fabs(minY), std::fabs(minZ), std::fabs(maxX), std::fabs(maxY), std::fabs(maxZ)});
    const bool shouldRecenter = maxCoord > 5000.0f || (std::fabs(cx) > 1000.0f || std::fabs(cy) > 1000.0f);
    if (!shouldRecenter) return;
    for (auto& mesh : bundle->meshes) {
        for (size_t i=0; i+2<mesh.vertices.size(); i+=3) {
            mesh.vertices[i] -= cx;
            mesh.vertices[i+1] -= cy;
            mesh.vertices[i+2] -= minZ;
        }
    }
    bundle->note += QStringLiteral(" Recentered imported mesh near the origin for viewing.");
}

void mergeMeshesByMaterial(SceneGeometryBundle* bundle) {
    if (!bundle || bundle->meshes.size() <= 128) return;
    std::map<QString, MeshGeometry> grouped;
    for (const auto& mesh : bundle->meshes) {
        const QString key = materialGroupKey(mesh.material);
        MeshGeometry& dst = grouped[key];
        if (dst.id.isEmpty()) {
            dst = mesh;
            dst.indices.clear();
            dst.vertices.clear();
            dst.normals.clear();
            dst.texCoords.clear();
        }
        const unsigned int baseIndex = static_cast<unsigned int>(dst.vertices.size() / 3);
        dst.vertices.insert(dst.vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
        dst.normals.insert(dst.normals.end(), mesh.normals.begin(), mesh.normals.end());
        dst.texCoords.insert(dst.texCoords.end(), mesh.texCoords.begin(), mesh.texCoords.end());
        for (unsigned int idx : mesh.indices) dst.indices.push_back(baseIndex + idx);
    }
    if (grouped.size() >= bundle->meshes.size()) return;
    std::vector<MeshGeometry> merged;
    merged.reserve(grouped.size());
    for (auto& it : grouped) merged.push_back(std::move(it.second));
    bundle->meshes = std::move(merged);
    bundle->note += QStringLiteral(" Reduced draw calls by merging mesh parts with the same material.");
}

void appendMesh(const aiScene* scene,
                const aiNode* node,
                const aiMatrix4x4& parentTransform,
                const QFileInfo& sourceInfo,
                SceneGeometryBundle* bundle) {
    if (!scene || !node || !bundle) {
        return;
    }

    const aiMatrix4x4 worldTransform = parentTransform * node->mTransformation;
    aiMatrix3x3 normalTransform(worldTransform);
    normalTransform.Inverse().Transpose();

    for (unsigned int meshIndex = 0; meshIndex < node->mNumMeshes; ++meshIndex) {
        const aiMesh* aiMeshPtr = scene->mMeshes[node->mMeshes[meshIndex]];
        if (!aiMeshPtr || aiMeshPtr->mNumVertices == 0 || aiMeshPtr->mNumFaces == 0) {
            continue;
        }

        MeshGeometry mesh;
        mesh.id = QStringLiteral("%1_%2_%3")
                      .arg(sourceInfo.baseName())
                      .arg(node->mName.C_Str())
                      .arg(meshIndex);
        mesh.sourceName = sourceInfo.absoluteFilePath();
        mesh.material = readMaterial(scene,
                                     aiMeshPtr->mMaterialIndex < scene->mNumMaterials
                                         ? scene->mMaterials[aiMeshPtr->mMaterialIndex]
                                         : nullptr,
                                     sourceInfo);
        mesh.vertices.reserve(static_cast<size_t>(aiMeshPtr->mNumVertices) * 3);
        mesh.normals.reserve(static_cast<size_t>(aiMeshPtr->mNumVertices) * 3);
        if (aiMeshPtr->HasTextureCoords(0)) {
            mesh.texCoords.reserve(static_cast<size_t>(aiMeshPtr->mNumVertices) * 2);
        }

        for (unsigned int v = 0; v < aiMeshPtr->mNumVertices; ++v) {
            aiVector3D p = worldTransform * aiMeshPtr->mVertices[v];
            mesh.vertices.push_back(static_cast<float>(p.x));
            mesh.vertices.push_back(static_cast<float>(p.y));
            mesh.vertices.push_back(static_cast<float>(p.z));

            aiVector3D n(0.0f, 0.0f, 1.0f);
            if (aiMeshPtr->HasNormals()) {
                n = normalTransform * aiMeshPtr->mNormals[v];
                n.Normalize();
            }
            mesh.normals.push_back(static_cast<float>(n.x));
            mesh.normals.push_back(static_cast<float>(n.y));
            mesh.normals.push_back(static_cast<float>(n.z));

            if (aiMeshPtr->HasTextureCoords(0)) {
                const aiVector3D uv = aiMeshPtr->mTextureCoords[0][v];
                mesh.texCoords.push_back(static_cast<float>(uv.x));
                mesh.texCoords.push_back(static_cast<float>(uv.y));
            }
        }

        for (unsigned int f = 0; f < aiMeshPtr->mNumFaces; ++f) {
            const aiFace& face = aiMeshPtr->mFaces[f];
            if (face.mNumIndices != 3) {
                continue;
            }
            mesh.indices.push_back(face.mIndices[0]);
            mesh.indices.push_back(face.mIndices[1]);
            mesh.indices.push_back(face.mIndices[2]);
        }

        if (!mesh.vertices.empty() && !mesh.indices.empty()) {
            bundle->meshes.push_back(std::move(mesh));
        }
    }

    for (unsigned int i = 0; i < node->mNumChildren; ++i) {
        appendMesh(scene, node->mChildren[i], worldTransform, sourceInfo, bundle);
    }
}

}  // namespace

SceneGeometryBundle FileSceneProvider::load(const SceneLoadRequest& request) {
    const QFileInfo info(request.filePath);
    const QString suffix = info.suffix().toLower();
    if (suffix == "stl" || suffix == "obj" || suffix == "fbx" || suffix == "gltf" || suffix == "glb") {
        return loadMeshFile(request.filePath);
    }
    return makeError(QString("Unsupported scene mesh format: %1. Supported formats are OBJ, STL, FBX, GLTF, and GLB.").arg(info.suffix()));
}

SceneGeometryBundle FileSceneProvider::loadMeshFile(const QString& filePath) {
    const QFileInfo info(filePath);
    if (!info.exists()) {
        return makeError(QString("Scene mesh file does not exist: %1").arg(filePath));
    }

    Assimp::Importer importer;
    const unsigned int flags = aiProcess_Triangulate |
                               aiProcess_JoinIdenticalVertices |
                               aiProcess_ImproveCacheLocality |
                               aiProcess_SortByPType |
                               aiProcess_GenSmoothNormals |
                               aiProcess_RemoveRedundantMaterials |
                               aiProcess_ValidateDataStructure |
                               aiProcess_FlipUVs;

    const aiScene* scene = importer.ReadFile(filePath.toStdString(), flags);
    if (!scene || !scene->mRootNode) {
        return makeError(QString("Failed to import mesh: %1").arg(QString::fromUtf8(importer.GetErrorString())));
    }

    SceneGeometryBundle bundle;
    bundle.sceneName = info.baseName();
    bundle.valid = true;
    bundle.sourcePath = info.absoluteFilePath();

    appendMesh(scene, scene->mRootNode, aiMatrix4x4(), info, &bundle);
    if (usesGltfCoordinateConversion(info.suffix().toLower())) {
        applyGltfCoordinateConversion(&bundle);
    }
    mergeMeshesByMaterial(&bundle);
    normalizeBundle(&bundle);

    if (bundle.meshes.empty()) {
        return makeError(QString("No mesh geometry found in %1").arg(filePath));
    }
    return bundle;
}

}  // namespace airsim_gui
