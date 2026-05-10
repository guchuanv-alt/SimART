#include "airsim_gui_UErealtime/OsmSceneProvider.h"

#include "airsim_gui_UErealtime/FileSceneProvider.h"

#include <QByteArray>
#include <QCryptographicHash>
#include <QDir>
#include <QEventLoop>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QLineF>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QProcess>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QUrl>
#include <QXmlStreamAttributes>
#include <QXmlStreamReader>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace airsim_gui {
namespace {

constexpr double kEarthRadiusMeters = 6378137.0;

struct OsmNode {
    long long id{0};
    double lat{0.0};
    double lon{0.0};
    double x{0.0};
    double y{0.0};
    std::unordered_map<std::string, std::string> tags;
};

struct OsmWay {
    long long id{0};
    std::vector<long long> nodeRefs;
    std::unordered_map<std::string, std::string> tags;
};

struct RawOsmData {
    std::unordered_map<long long, OsmNode> nodes;
    std::vector<OsmWay> ways;
    double southLat{0.0};
    double westLon{0.0};
    double northLat{0.0};
    double eastLon{0.0};
    double originLat{0.0};
    double originLon{0.0};
};

void reportProgress(const SceneLoadRequest& request, const QString& message, int value, int maximum = 100) {
    if (request.progressCallback) {
        request.progressCallback(message, value, maximum);
    }
}

SceneGeometryBundle makeError(const QString& text) {
    SceneGeometryBundle bundle;
    bundle.valid = false;
    bundle.errorMessage = text;
    return bundle;
}

QString overpassQuery(double southLat, double westLon, double northLat, double eastLon) {
    return QStringLiteral(
               "[out:xml][timeout:45];"
               "("
               "way[\"building\"](%1,%2,%3,%4);"
               "way[\"highway\"](%1,%2,%3,%4);"
               "way[\"natural\"](%1,%2,%3,%4);"
               "way[\"landuse\"](%1,%2,%3,%4);"
               "way[\"water\"](%1,%2,%3,%4);"
               "way[\"waterway\"](%1,%2,%3,%4);"
               "way[\"leisure\"](%1,%2,%3,%4);"
               "node[\"natural\"=\"tree\"](%1,%2,%3,%4);"
               "node[\"highway\"=\"street_lamp\"](%1,%2,%3,%4);"
               ");"
               "(._;>;);"
               "out body;")
        .arg(southLat, 0, 'f', 7)
        .arg(westLon, 0, 'f', 7)
        .arg(northLat, 0, 'f', 7)
        .arg(eastLon, 0, 'f', 7);
}

QString appUserAgent() {
    return QStringLiteral("AirSimGUI-UERealtime/OSMImport (+local desktop app)");
}

QString cacheBasePath() {
    QString root = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
    if (root.isEmpty()) {
        root = QDir::tempPath() + QStringLiteral("/airsim_gui_UErealtime_cache");
    }
    return QDir(root).absoluteFilePath(QStringLiteral("osm_imports"));
}

QString bboxCacheFilePath(double southLat, double westLon, double northLat, double eastLon) {
    const QString key = QStringLiteral("v2_%1_%2_%3_%4")
                            .arg(southLat, 0, 'f', 6)
                            .arg(westLon, 0, 'f', 6)
                            .arg(northLat, 0, 'f', 6)
                            .arg(eastLon, 0, 'f', 6);
    const QString hash = QString::fromLatin1(QCryptographicHash::hash(key.toUtf8(), QCryptographicHash::Md5).toHex());
    const QString dir = cacheBasePath();
    QDir().mkpath(dir);
    return QDir(dir).absoluteFilePath(hash + QStringLiteral(".osm"));
}

QString sanitizeFileStem(const QString& raw) {
    QString stem = raw.trimmed();
    if (stem.isEmpty()) {
        stem = QStringLiteral("osm_area");
    }
    stem.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9._-]+")), QStringLiteral("_"));
    stem.remove(QRegularExpression(QStringLiteral("^_+|_+$")));
    if (stem.isEmpty()) {
        stem = QStringLiteral("osm_area");
    }
    return stem.left(80);
}

QString requestedMeshExtension(const SceneLoadRequest& request) {
    const QString format = request.osm2worldOutputFormat.trimmed().toLower();
    if (format == QStringLiteral("obj") || format == QStringLiteral("gltf") || format == QStringLiteral("glb")) {
        return format;
    }
    return QStringLiteral("glb");
}

QString effectiveOsm2WorldExecutable(const SceneLoadRequest& request) {
    return request.osm2worldExecutable.trimmed().isEmpty() ? QStringLiteral("osm2world") : request.osm2worldExecutable.trimmed();
}

QString effectiveOutputDirectory(const SceneLoadRequest& request) {
    QString dir = request.osm2worldOutputDirectory.trimmed();
    if (dir.isEmpty()) {
        dir = QDir(cacheBasePath()).absoluteFilePath(QStringLiteral("osm2world_exports"));
    }
    QDir().mkpath(dir);
    return dir;
}

QString outputMeshPathFor(const SceneLoadRequest& request, const QString& sceneName) {
    const QString stem = sanitizeFileStem(sceneName);
    const QString ext = requestedMeshExtension(request);
    return QDir(effectiveOutputDirectory(request)).absoluteFilePath(stem + QStringLiteral(".") + ext);
}

QString outputOsmCopyPathFor(const SceneLoadRequest& request, const QString& sceneName) {
    const QString stem = sanitizeFileStem(sceneName);
    return QDir(effectiveOutputDirectory(request)).absoluteFilePath(stem + QStringLiteral(".osm"));
}

struct ProcessRunResult {
    int exitCode{-1};
    QProcess::ProcessError processError{QProcess::UnknownError};
    QString stdOut;
    QString stdErr;
    bool timedOut{false};
    bool started{false};
};

ProcessRunResult runProcessBlocking(const QString& program,
                                    const QStringList& arguments,
                                    int timeoutMs = 300000,
                                    const std::function<void(const QString&, int, int)>& progressCallback = {}) {
    ProcessRunResult result;
    QProcess process;
    process.setProgram(program);
    process.setArguments(arguments);
    process.setProcessChannelMode(QProcess::SeparateChannels);
    process.start();
    result.started = process.waitForStarted(10000);
    result.processError = process.error();
    if (!result.started) {
        result.stdErr = process.errorString();
        return result;
    }
    QElapsedTimer timerStart;
    timerStart.start();
    int lastValue = 35;
    while (!process.waitForFinished(250)) {
        if (progressCallback) {
            const int elapsedMs = static_cast<int>(timerStart.elapsed());
            lastValue = std::min(92, 35 + elapsedMs / 1000);
            progressCallback(QStringLiteral("OSM2World is generating the mesh..."), lastValue, 100);
        }
        if (timerStart.elapsed() > timeoutMs) {
            result.timedOut = true;
            process.kill();
            process.waitForFinished(5000);
            break;
        }
    }
    result.exitCode = process.exitCode();
    result.processError = process.error();
    result.stdOut = QString::fromLocal8Bit(process.readAllStandardOutput());
    result.stdErr = QString::fromLocal8Bit(process.readAllStandardError());
    return result;
}

bool looksLikeJarPath(const QString& path) {
    return path.trimmed().endsWith(QStringLiteral(".jar"), Qt::CaseInsensitive);
}

QStringList splitExtraArguments(const QString& raw) {
    QStringList parts;
    QString current;
    bool inQuotes = false;
    QChar quoteChar;
    for (int i = 0; i < raw.size(); ++i) {
        const QChar ch = raw.at(i);
        if (((ch == QLatin1Char('"')) || (ch.unicode() == 39)) && (!inQuotes || ch == quoteChar)) {
            if (inQuotes && ch == quoteChar) {
                inQuotes = false;
            } else if (!inQuotes) {
                inQuotes = true;
                quoteChar = ch;
            }
            continue;
        }
        if (!inQuotes && ch.isSpace()) {
            if (!current.isEmpty()) {
                parts.push_back(current);
                current.clear();
            }
            continue;
        }
        current.append(ch);
    }
    if (!current.isEmpty()) {
        parts.push_back(current);
    }
    return parts;
}

ProcessRunResult runOsm2WorldConvert(const SceneLoadRequest& request,
                                     const QString& inputOsmPath,
                                     const QString& outputMeshPath,
                                     bool useSubcommand) {
    QString program = effectiveOsm2WorldExecutable(request);
    QStringList arguments;
    if (looksLikeJarPath(program)) {
        arguments << QStringLiteral("-jar") << program;
        program = request.osm2worldJavaExecutable.trimmed().isEmpty() ? QStringLiteral("java") : request.osm2worldJavaExecutable.trimmed();
    }
    if (useSubcommand) {
        arguments << QStringLiteral("convert");
    }
    arguments << QStringLiteral("-i") << inputOsmPath
              << QStringLiteral("-o") << outputMeshPath;
    if (!request.osm2worldExtraArguments.trimmed().isEmpty()) {
        arguments << splitExtraArguments(request.osm2worldExtraArguments);
    }
    return runProcessBlocking(program, arguments, 300000, request.progressCallback);
}

bool convertWithOsm2World(const SceneLoadRequest& request,
                          const QString& inputOsmPath,
                          const QString& sceneName,
                          QString* outputMeshPath,
                          QString* note,
                          QString* errorMessage) {
    const QString requestedPath = outputMeshPathFor(request, sceneName);
    QFile::remove(requestedPath);

    auto finalizeFailure = [&](const ProcessRunResult& res, const QString& attemptedPath) {
        QString err;
        if (!res.started) {
            err = QStringLiteral("Failed to start OSM2World. Check the executable or jar path.");
            if (!res.stdErr.trimmed().isEmpty()) {
                err += QStringLiteral(" ") + res.stdErr.trimmed();
            }
        } else if (res.timedOut) {
            err = QStringLiteral("OSM2World timed out while generating the mesh.");
        } else {
            err = QStringLiteral("OSM2World did not generate the requested mesh.");
            if (!res.stdErr.trimmed().isEmpty()) {
                err += QStringLiteral("\n") + res.stdErr.trimmed();
            }
        }
        if (errorMessage) {
            *errorMessage = err + QStringLiteral("\nExpected output: %1").arg(attemptedPath);
        }
    };

    ProcessRunResult result = runOsm2WorldConvert(request, inputOsmPath, requestedPath, true);
    if ((result.started && result.exitCode == 0 && QFileInfo::exists(requestedPath)) ||
        QFileInfo::exists(requestedPath)) {
        if (outputMeshPath) {
            *outputMeshPath = requestedPath;
        }
        if (note) {
            *note = result.stdErr.trimmed();
        }
        return true;
    }

    ProcessRunResult oldCliResult = runOsm2WorldConvert(request, inputOsmPath, requestedPath, false);
    if ((oldCliResult.started && oldCliResult.exitCode == 0 && QFileInfo::exists(requestedPath)) ||
        QFileInfo::exists(requestedPath)) {
        if (outputMeshPath) {
            *outputMeshPath = requestedPath;
        }
        if (note) {
            *note = oldCliResult.stdErr.trimmed();
        }
        return true;
    }

    const QString requestedExt = requestedMeshExtension(request);
    if (requestedExt == QStringLiteral("glb")) {
        const QString fallbackPath = requestedPath.left(requestedPath.size() - 4) + QStringLiteral(".gltf");
        QFile::remove(fallbackPath);
        SceneLoadRequest fallbackRequest = request;
        fallbackRequest.osm2worldOutputFormat = QStringLiteral("gltf");
        ProcessRunResult gltfResult = runOsm2WorldConvert(fallbackRequest, inputOsmPath, fallbackPath, true);
        if ((gltfResult.started && gltfResult.exitCode == 0 && QFileInfo::exists(fallbackPath)) ||
            QFileInfo::exists(fallbackPath)) {
            if (outputMeshPath) {
                *outputMeshPath = fallbackPath;
            }
            if (note) {
                *note = QStringLiteral("Requested GLB; generated GLTF instead because this OSM2World build did not produce a GLB file.");
            }
            return true;
        }
        ProcessRunResult gltfOldCliResult = runOsm2WorldConvert(fallbackRequest, inputOsmPath, fallbackPath, false);
        if ((gltfOldCliResult.started && gltfOldCliResult.exitCode == 0 && QFileInfo::exists(fallbackPath)) ||
            QFileInfo::exists(fallbackPath)) {
            if (outputMeshPath) {
                *outputMeshPath = fallbackPath;
            }
            if (note) {
                *note = QStringLiteral("Requested GLB; generated GLTF instead because this OSM2World build did not produce a GLB file.");
            }
            return true;
        }
        finalizeFailure(gltfOldCliResult.started ? gltfOldCliResult : gltfResult, fallbackPath);
        return false;
    }

    finalizeFailure(oldCliResult.started ? oldCliResult : result, requestedPath);
    return false;
}

struct LocalBbox {
    double minX{0.0};
    double minY{0.0};
    double maxX{0.0};
    double maxY{0.0};
};

LocalBbox localBboxFor(const RawOsmData& data, const SceneLoadRequest& request) {
    const double cosLat = std::cos(data.originLat * M_PI / 180.0);
    LocalBbox bbox;
    bbox.minX = (request.westLon - data.originLon) * M_PI / 180.0 * kEarthRadiusMeters * cosLat;
    bbox.maxX = (request.eastLon - data.originLon) * M_PI / 180.0 * kEarthRadiusMeters * cosLat;
    bbox.minY = (request.southLat - data.originLat) * M_PI / 180.0 * kEarthRadiusMeters;
    bbox.maxY = (request.northLat - data.originLat) * M_PI / 180.0 * kEarthRadiusMeters;
    if (bbox.minX > bbox.maxX) std::swap(bbox.minX, bbox.maxX);
    if (bbox.minY > bbox.maxY) std::swap(bbox.minY, bbox.maxY);
    return bbox;
}

bool pointInsideLocalBbox(const QPointF& p, const LocalBbox& bbox) {
    return p.x() >= bbox.minX && p.x() <= bbox.maxX && p.y() >= bbox.minY && p.y() <= bbox.maxY;
}

enum OutCode {
    OutInside = 0,
    OutLeft = 1,
    OutRight = 2,
    OutBottom = 4,
    OutTop = 8
};

int outCodeFor(const QPointF& p, const LocalBbox& bbox) {
    int code = OutInside;
    if (p.x() < bbox.minX) code |= OutLeft;
    else if (p.x() > bbox.maxX) code |= OutRight;
    if (p.y() < bbox.minY) code |= OutBottom;
    else if (p.y() > bbox.maxY) code |= OutTop;
    return code;
}

bool clipLineSegmentToBbox(QPointF* a, QPointF* b, const LocalBbox& bbox) {
    QPointF p0 = *a;
    QPointF p1 = *b;
    int code0 = outCodeFor(p0, bbox);
    int code1 = outCodeFor(p1, bbox);
    while (true) {
        if (!(code0 | code1)) {
            *a = p0;
            *b = p1;
            return true;
        }
        if (code0 & code1) {
            return false;
        }
        const int outCode = code0 ? code0 : code1;
        double x = 0.0;
        double y = 0.0;
        const double dx = p1.x() - p0.x();
        const double dy = p1.y() - p0.y();
        if ((outCode & OutTop) != 0) {
            if (std::abs(dy) < 1e-9) return false;
            y = bbox.maxY;
            x = p0.x() + dx * (bbox.maxY - p0.y()) / dy;
        } else if ((outCode & OutBottom) != 0) {
            if (std::abs(dy) < 1e-9) return false;
            y = bbox.minY;
            x = p0.x() + dx * (bbox.minY - p0.y()) / dy;
        } else if ((outCode & OutRight) != 0) {
            if (std::abs(dx) < 1e-9) return false;
            x = bbox.maxX;
            y = p0.y() + dy * (bbox.maxX - p0.x()) / dx;
        } else {
            if (std::abs(dx) < 1e-9) return false;
            x = bbox.minX;
            y = p0.y() + dy * (bbox.minX - p0.x()) / dx;
        }
        if (outCode == code0) {
            p0 = QPointF(x, y);
            code0 = outCodeFor(p0, bbox);
        } else {
            p1 = QPointF(x, y);
            code1 = outCodeFor(p1, bbox);
        }
    }
}

std::vector<QPointF> clipPolygonToBbox(const std::vector<QPointF>& polygon, const LocalBbox& bbox) {
    auto clipAgainstEdge = [](const std::vector<QPointF>& input,
                              auto insideFn,
                              auto intersectFn) {
        std::vector<QPointF> output;
        if (input.empty()) return output;
        QPointF prev = input.back();
        bool prevInside = insideFn(prev);
        for (const QPointF& cur : input) {
            const bool curInside = insideFn(cur);
            if (curInside) {
                if (!prevInside) output.push_back(intersectFn(prev, cur));
                output.push_back(cur);
            } else if (prevInside) {
                output.push_back(intersectFn(prev, cur));
            }
            prev = cur;
            prevInside = curInside;
        }
        return output;
    };

    std::vector<QPointF> out = polygon;
    out = clipAgainstEdge(out,
        [&](const QPointF& p) { return p.x() >= bbox.minX; },
        [&](const QPointF& a, const QPointF& b) {
            const double denom = (b.x() - a.x());
            if (std::abs(denom) < 1e-9) return QPointF(bbox.minX, a.y());
            const double t = (bbox.minX - a.x()) / denom;
            return QPointF(bbox.minX, a.y() + t * (b.y() - a.y()));
        });
    out = clipAgainstEdge(out,
        [&](const QPointF& p) { return p.x() <= bbox.maxX; },
        [&](const QPointF& a, const QPointF& b) {
            const double denom = (b.x() - a.x());
            if (std::abs(denom) < 1e-9) return QPointF(bbox.maxX, a.y());
            const double t = (bbox.maxX - a.x()) / denom;
            return QPointF(bbox.maxX, a.y() + t * (b.y() - a.y()));
        });
    out = clipAgainstEdge(out,
        [&](const QPointF& p) { return p.y() >= bbox.minY; },
        [&](const QPointF& a, const QPointF& b) {
            const double denom = (b.y() - a.y());
            if (std::abs(denom) < 1e-9) return QPointF(a.x(), bbox.minY);
            const double t = (bbox.minY - a.y()) / denom;
            return QPointF(a.x() + t * (b.x() - a.x()), bbox.minY);
        });
    out = clipAgainstEdge(out,
        [&](const QPointF& p) { return p.y() <= bbox.maxY; },
        [&](const QPointF& a, const QPointF& b) {
            const double denom = (b.y() - a.y());
            if (std::abs(denom) < 1e-9) return QPointF(a.x(), bbox.maxY);
            const double t = (bbox.maxY - a.y()) / denom;
            return QPointF(a.x() + t * (b.x() - a.x()), bbox.maxY);
        });

    if (!out.empty() && QLineF(out.front(), out.back()).length() < 1e-5) {
        out.pop_back();
    }
    return out;
}

bool nodeInsideRequestedBBox(const OsmNode& node, const SceneLoadRequest& request) {
    return node.lat >= request.southLat && node.lat <= request.northLat &&
           node.lon >= request.westLon && node.lon <= request.eastLon;
}

std::string tagValue(const std::unordered_map<std::string, std::string>& tags, const std::string& key) {
    const auto it = tags.find(key);
    return it == tags.end() ? std::string() : it->second;
}

bool hasTag(const std::unordered_map<std::string, std::string>& tags, const std::string& key) {
    return tags.find(key) != tags.end();
}

bool isAreaWay(const OsmWay& way) {
    return way.nodeRefs.size() >= 4 && way.nodeRefs.front() == way.nodeRefs.back();
}

double parseDoubleTag(const std::unordered_map<std::string, std::string>& tags,
                      const std::string& key,
                      double fallback) {
    const auto it = tags.find(key);
    if (it == tags.end() || it->second.empty()) {
        return fallback;
    }
    bool ok = false;
    const double value = QString::fromStdString(it->second).remove(QStringLiteral("m")).trimmed().toDouble(&ok);
    return ok ? value : fallback;
}

double buildingHeightFor(const OsmWay& way) {
    double height = parseDoubleTag(way.tags, "height", -1.0);
    if (height > 0.0) {
        return std::max(3.0, height);
    }
    const double levels = parseDoubleTag(way.tags, "building:levels", -1.0);
    if (levels > 0.0) {
        return std::max(3.0, levels * 3.2);
    }
    const std::string kind = tagValue(way.tags, "building");
    if (kind == "industrial" || kind == "warehouse") {
        return 9.0;
    }
    if (kind == "commercial" || kind == "office") {
        return 18.0;
    }
    return 12.0;
}

double roadWidthFor(const OsmWay& way) {
    const std::string highway = tagValue(way.tags, "highway");
    const double explicitWidth = parseDoubleTag(way.tags, "width", -1.0);
    if (explicitWidth > 0.0) {
        return explicitWidth;
    }
    if (highway == "motorway") return 18.0;
    if (highway == "trunk" || highway == "primary") return 12.0;
    if (highway == "secondary") return 10.0;
    if (highway == "tertiary") return 8.0;
    if (highway == "residential" || highway == "service") return 6.0;
    if (highway == "footway" || highway == "path" || highway == "pedestrian") return 2.0;
    return 5.0;
}

bool isGreenArea(const OsmWay& way) {
    const std::string leisure = tagValue(way.tags, "leisure");
    const std::string landuse = tagValue(way.tags, "landuse");
    const std::string natural = tagValue(way.tags, "natural");
    return leisure == "park" || leisure == "garden" || landuse == "grass" || landuse == "forest" ||
           landuse == "meadow" || natural == "wood" || natural == "grassland";
}

bool isWaterArea(const OsmWay& way) {
    const std::string natural = tagValue(way.tags, "natural");
    const std::string water = tagValue(way.tags, "water");
    const std::string waterway = tagValue(way.tags, "waterway");
    const std::string landuse = tagValue(way.tags, "landuse");
    return natural == "water" || !water.empty() || waterway == "riverbank" || landuse == "reservoir";
}

void addVertex(MeshGeometry* mesh, double x, double y, double z, double nx, double ny, double nz) {
    mesh->vertices.push_back(static_cast<float>(x));
    mesh->vertices.push_back(static_cast<float>(y));
    mesh->vertices.push_back(static_cast<float>(z));
    mesh->normals.push_back(static_cast<float>(nx));
    mesh->normals.push_back(static_cast<float>(ny));
    mesh->normals.push_back(static_cast<float>(nz));
}

unsigned int vertexCount(const MeshGeometry& mesh) {
    return static_cast<unsigned int>(mesh.vertices.size() / 3);
}

void addTriangle(MeshGeometry* mesh, unsigned int a, unsigned int b, unsigned int c) {
    mesh->indices.push_back(a);
    mesh->indices.push_back(b);
    mesh->indices.push_back(c);
}

double polygonSignedArea(const std::vector<QPointF>& poly) {
    double area = 0.0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const QPointF& p = poly[i];
        const QPointF& q = poly[(i + 1) % poly.size()];
        area += p.x() * q.y() - q.x() * p.y();
    }
    return area * 0.5;
}

bool pointInTriangle(const QPointF& p, const QPointF& a, const QPointF& b, const QPointF& c) {
    const double area = std::abs((b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y()));
    const double area1 = std::abs((a.x() - p.x()) * (b.y() - p.y()) - (b.x() - p.x()) * (a.y() - p.y()));
    const double area2 = std::abs((b.x() - p.x()) * (c.y() - p.y()) - (c.x() - p.x()) * (b.y() - p.y()));
    const double area3 = std::abs((c.x() - p.x()) * (a.y() - p.y()) - (a.x() - p.x()) * (c.y() - p.y()));
    return std::abs(area - (area1 + area2 + area3)) < 1e-4;
}

bool isEar(const std::vector<QPointF>& polygon, const std::vector<int>& indices, int i, bool ccw) {
    const int prev = indices[(i - 1 + indices.size()) % indices.size()];
    const int cur = indices[i];
    const int next = indices[(i + 1) % indices.size()];
    const QPointF& a = polygon[static_cast<size_t>(prev)];
    const QPointF& b = polygon[static_cast<size_t>(cur)];
    const QPointF& c = polygon[static_cast<size_t>(next)];
    const double cross = (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
    if (ccw ? cross <= 1e-8 : cross >= -1e-8) {
        return false;
    }
    for (int idx : indices) {
        if (idx == prev || idx == cur || idx == next) {
            continue;
        }
        if (pointInTriangle(polygon[static_cast<size_t>(idx)], a, b, c)) {
            return false;
        }
    }
    return true;
}

std::vector<unsigned int> triangulatePolygon(const std::vector<QPointF>& polygon) {
    std::vector<unsigned int> triangles;
    if (polygon.size() < 3) {
        return triangles;
    }

    std::vector<int> indices;
    indices.reserve(polygon.size());
    for (int i = 0; i < static_cast<int>(polygon.size()); ++i) {
        indices.push_back(i);
    }
    const bool ccw = polygonSignedArea(polygon) > 0.0;

    int guard = 0;
    while (indices.size() > 2 && guard < 4096) {
        bool earFound = false;
        for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
            if (!isEar(polygon, indices, i, ccw)) {
                continue;
            }
            const int prev = indices[(i - 1 + indices.size()) % indices.size()];
            const int cur = indices[i];
            const int next = indices[(i + 1) % indices.size()];
            if (ccw) {
                triangles.push_back(static_cast<unsigned int>(prev));
                triangles.push_back(static_cast<unsigned int>(cur));
                triangles.push_back(static_cast<unsigned int>(next));
            } else {
                triangles.push_back(static_cast<unsigned int>(next));
                triangles.push_back(static_cast<unsigned int>(cur));
                triangles.push_back(static_cast<unsigned int>(prev));
            }
            indices.erase(indices.begin() + i);
            earFound = true;
            break;
        }
        if (!earFound) {
            break;
        }
        ++guard;
    }
    return triangles;
}

void appendFlatPolygon(MeshGeometry* mesh,
                       const std::vector<QPointF>& polygon,
                       double z,
                       bool flipNormal) {
    if (polygon.size() < 3) {
        return;
    }
    const std::vector<unsigned int> triangles = triangulatePolygon(polygon);
    if (triangles.empty()) {
        return;
    }
    const unsigned int base = vertexCount(*mesh);
    const double nz = flipNormal ? -1.0 : 1.0;
    for (const QPointF& p : polygon) {
        addVertex(mesh, p.x(), p.y(), z, 0.0, 0.0, nz);
    }
    for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
        if (!flipNormal) {
            addTriangle(mesh, base + triangles[i], base + triangles[i + 1], base + triangles[i + 2]);
        } else {
            addTriangle(mesh, base + triangles[i + 2], base + triangles[i + 1], base + triangles[i]);
        }
    }
}

void appendExtrudedPolygon(MeshGeometry* mesh,
                           const std::vector<QPointF>& polygon,
                           double baseZ,
                           double topZ) {
    if (polygon.size() < 3) {
        return;
    }
    appendFlatPolygon(mesh, polygon, topZ, false);
    appendFlatPolygon(mesh, polygon, baseZ, true);

    for (size_t i = 0; i < polygon.size(); ++i) {
        const QPointF& a = polygon[i];
        const QPointF& b = polygon[(i + 1) % polygon.size()];
        const QPointF edge = b - a;
        const double len = std::hypot(edge.x(), edge.y());
        if (len < 1e-6) {
            continue;
        }
        const double nx = edge.y() / len;
        const double ny = -edge.x() / len;
        const unsigned int base = vertexCount(*mesh);
        addVertex(mesh, a.x(), a.y(), baseZ, nx, ny, 0.0);
        addVertex(mesh, b.x(), b.y(), baseZ, nx, ny, 0.0);
        addVertex(mesh, b.x(), b.y(), topZ, nx, ny, 0.0);
        addVertex(mesh, a.x(), a.y(), topZ, nx, ny, 0.0);
        addTriangle(mesh, base + 0, base + 1, base + 2);
        addTriangle(mesh, base + 0, base + 2, base + 3);
    }
}

void appendBox(MeshGeometry* mesh,
               double centerX,
               double centerY,
               double baseZ,
               double sizeX,
               double sizeY,
               double height) {
    const double hx = sizeX * 0.5;
    const double hy = sizeY * 0.5;
    std::vector<QPointF> poly = {
        QPointF(centerX - hx, centerY - hy),
        QPointF(centerX + hx, centerY - hy),
        QPointF(centerX + hx, centerY + hy),
        QPointF(centerX - hx, centerY + hy)
    };
    appendExtrudedPolygon(mesh, poly, baseZ, baseZ + height);
}

void appendRoadSegment(MeshGeometry* mesh,
                       const QPointF& a,
                       const QPointF& b,
                       double width,
                       double z) {
    const QPointF d = b - a;
    const double len = std::hypot(d.x(), d.y());
    if (len < 1e-3) {
        return;
    }
    const QPointF n(-d.y() / len, d.x() / len);
    const QPointF p0 = a + n * (width * 0.5);
    const QPointF p1 = a - n * (width * 0.5);
    const QPointF p2 = b - n * (width * 0.5);
    const QPointF p3 = b + n * (width * 0.5);

    const unsigned int base = vertexCount(*mesh);
    addVertex(mesh, p0.x(), p0.y(), z, 0.0, 0.0, 1.0);
    addVertex(mesh, p1.x(), p1.y(), z, 0.0, 0.0, 1.0);
    addVertex(mesh, p2.x(), p2.y(), z, 0.0, 0.0, 1.0);
    addVertex(mesh, p3.x(), p3.y(), z, 0.0, 0.0, 1.0);
    addTriangle(mesh, base + 0, base + 1, base + 2);
    addTriangle(mesh, base + 0, base + 2, base + 3);
}

std::vector<QPointF> wayPolygon(const RawOsmData& data, const OsmWay& way) {
    std::vector<QPointF> poly;
    for (size_t i = 0; i < way.nodeRefs.size(); ++i) {
        const auto it = data.nodes.find(way.nodeRefs[i]);
        if (it == data.nodes.end()) {
            continue;
        }
        if (i + 1 == way.nodeRefs.size() && way.nodeRefs[i] == way.nodeRefs.front()) {
            continue;
        }
        poly.push_back(QPointF(it->second.x, it->second.y));
    }
    return poly;
}

std::vector<QPointF> wayPolyline(const RawOsmData& data, const OsmWay& way) {
    std::vector<QPointF> line;
    for (long long ref : way.nodeRefs) {
        const auto it = data.nodes.find(ref);
        if (it == data.nodes.end()) {
            continue;
        }
        line.push_back(QPointF(it->second.x, it->second.y));
    }
    return line;
}

bool parseOsmXml(const QByteArray& xml, const SceneLoadRequest& request, RawOsmData* data, QString* errorMessage) {
    if (!data) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Internal error: null OSM output container.");
        }
        return false;
    }

    QXmlStreamReader reader(xml);
    OsmWay currentWay;
    OsmNode currentNode;
    bool inWay = false;
    bool inNode = false;
    bool sawBounds = false;

    while (!reader.atEnd()) {
        reader.readNext();
        if (!reader.isStartElement() && !reader.isEndElement()) {
            continue;
        }

        if (reader.isStartElement()) {
            const QStringRef name = reader.name();
            const QXmlStreamAttributes attrs = reader.attributes();
            if (name == QStringLiteral("bounds")) {
                data->southLat = attrs.value(QStringLiteral("minlat")).toString().toDouble();
                data->westLon = attrs.value(QStringLiteral("minlon")).toString().toDouble();
                data->northLat = attrs.value(QStringLiteral("maxlat")).toString().toDouble();
                data->eastLon = attrs.value(QStringLiteral("maxlon")).toString().toDouble();
                sawBounds = true;
            } else if (name == QStringLiteral("node")) {
                currentNode = OsmNode{};
                currentNode.id = attrs.value(QStringLiteral("id")).toLongLong();
                currentNode.lat = attrs.value(QStringLiteral("lat")).toString().toDouble();
                currentNode.lon = attrs.value(QStringLiteral("lon")).toString().toDouble();
                inNode = true;
            } else if (name == QStringLiteral("way")) {
                currentWay = OsmWay{};
                currentWay.id = attrs.value(QStringLiteral("id")).toLongLong();
                inWay = true;
            } else if (inWay && name == QStringLiteral("nd")) {
                currentWay.nodeRefs.push_back(attrs.value(QStringLiteral("ref")).toLongLong());
            } else if ((inWay || inNode) && name == QStringLiteral("tag")) {
                const std::string key = attrs.value(QStringLiteral("k")).toString().toStdString();
                const std::string value = attrs.value(QStringLiteral("v")).toString().toStdString();
                if (inWay) {
                    currentWay.tags[key] = value;
                } else if (inNode) {
                    currentNode.tags[key] = value;
                }
            }
        } else if (reader.isEndElement()) {
            const QStringRef name = reader.name();
            if (inNode && name == QStringLiteral("node")) {
                data->nodes[currentNode.id] = currentNode;
                inNode = false;
            } else if (inWay && name == QStringLiteral("way")) {
                data->ways.push_back(currentWay);
                inWay = false;
            }
        }
    }

    if (reader.hasError()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to parse OSM XML: %1").arg(reader.errorString());
        }
        return false;
    }

    if (data->nodes.empty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("The selected OSM area contained no nodes.");
        }
        return false;
    }

    if (!sawBounds) {
        data->southLat = request.southLat;
        data->westLon = request.westLon;
        data->northLat = request.northLat;
        data->eastLon = request.eastLon;
        if (!(data->southLat < data->northLat && data->westLon < data->eastLon)) {
            data->southLat = std::numeric_limits<double>::max();
            data->westLon = std::numeric_limits<double>::max();
            data->northLat = -std::numeric_limits<double>::max();
            data->eastLon = -std::numeric_limits<double>::max();
            for (const auto& kv : data->nodes) {
                data->southLat = std::min(data->southLat, kv.second.lat);
                data->westLon = std::min(data->westLon, kv.second.lon);
                data->northLat = std::max(data->northLat, kv.second.lat);
                data->eastLon = std::max(data->eastLon, kv.second.lon);
            }
        }
    }

    data->originLat = (data->southLat + data->northLat) * 0.5;
    data->originLon = (data->westLon + data->eastLon) * 0.5;
    const double cosLat = std::cos(data->originLat * M_PI / 180.0);

    for (auto& kv : data->nodes) {
        OsmNode& node = kv.second;
        node.x = (node.lon - data->originLon) * M_PI / 180.0 * kEarthRadiusMeters * cosLat;
        node.y = (node.lat - data->originLat) * M_PI / 180.0 * kEarthRadiusMeters;
    }
    return true;
}

SceneGeometryBundle buildSceneBundle(const RawOsmData& data, const SceneLoadRequest& request, const QString& sceneName) {
    const LocalBbox bbox = localBboxFor(data, request);
    MeshGeometry buildings;
    buildings.id = QStringLiteral("osm_buildings");
    buildings.material.name = QStringLiteral("Buildings");
    buildings.material.diffuseColor = QVector3D(0.82f, 0.82f, 0.84f);

    MeshGeometry roads;
    roads.id = QStringLiteral("osm_roads");
    roads.material.name = QStringLiteral("Roads");
    roads.material.diffuseColor = QVector3D(0.20f, 0.22f, 0.25f);

    MeshGeometry greenAreas;
    greenAreas.id = QStringLiteral("osm_green");
    greenAreas.material.name = QStringLiteral("GreenAreas");
    greenAreas.material.diffuseColor = QVector3D(0.48f, 0.69f, 0.47f);
    greenAreas.material.opacity = 0.94f;

    MeshGeometry water;
    water.id = QStringLiteral("osm_water");
    water.material.name = QStringLiteral("Water");
    water.material.diffuseColor = QVector3D(0.32f, 0.55f, 0.78f);
    water.material.opacity = 0.85f;

    MeshGeometry trees;
    trees.id = QStringLiteral("osm_trees");
    trees.material.name = QStringLiteral("Trees");
    trees.material.diffuseColor = QVector3D(0.31f, 0.56f, 0.29f);

    MeshGeometry streetFurniture;
    streetFurniture.id = QStringLiteral("osm_street_furniture");
    streetFurniture.material.name = QStringLiteral("StreetFurniture");
    streetFurniture.material.diffuseColor = QVector3D(0.82f, 0.78f, 0.63f);

    for (const OsmWay& way : data.ways) {
        const bool area = isAreaWay(way);
        if (area && hasTag(way.tags, "building")) {
            std::vector<QPointF> poly = clipPolygonToBbox(wayPolygon(data, way), bbox);
            if (poly.size() >= 3) {
                appendExtrudedPolygon(&buildings, poly, 0.0, buildingHeightFor(way));
            }
            continue;
        }
        if (request.includeWater && area && isWaterArea(way)) {
            std::vector<QPointF> poly = clipPolygonToBbox(wayPolygon(data, way), bbox);
            appendFlatPolygon(&water, poly, 0.02, false);
            continue;
        }
        if (request.includeGreenAreas && area && isGreenArea(way)) {
            std::vector<QPointF> poly = clipPolygonToBbox(wayPolygon(data, way), bbox);
            appendFlatPolygon(&greenAreas, poly, 0.01, false);
            continue;
        }
        if (hasTag(way.tags, "highway")) {
            const std::vector<QPointF> line = wayPolyline(data, way);
            const double width = roadWidthFor(way);
            for (size_t i = 1; i < line.size(); ++i) {
                QPointF a = line[i - 1];
                QPointF b = line[i];
                if (clipLineSegmentToBbox(&a, &b, bbox)) {
                    appendRoadSegment(&roads, a, b, width, 0.03);
                }
            }
            continue;
        }
    }

    if (request.includeTrees || request.includeStreetFurniture) {
        for (const auto& kv : data.nodes) {
            const OsmNode& node = kv.second;
            if (!nodeInsideRequestedBBox(node, request)) {
                continue;
            }
            if (request.includeTrees) {
                const auto naturalIt = node.tags.find("natural");
                if (naturalIt != node.tags.end() && naturalIt->second == "tree") {
                    appendBox(&trees, node.x, node.y, 0.0, 1.0, 1.0, 5.5);
                }
            }
            if (request.includeStreetFurniture) {
                const auto lampIt = node.tags.find("highway");
                if (lampIt != node.tags.end() && lampIt->second == "street_lamp") {
                    appendBox(&streetFurniture, node.x, node.y, 0.0, 0.25, 0.25, 5.0);
                    appendBox(&streetFurniture, node.x + 0.25, node.y, 4.6, 0.8, 0.2, 0.2);
                }
            }
        }
    }

    SceneGeometryBundle bundle;
    bundle.sceneName = sceneName;
    bundle.valid = true;
    if (!greenAreas.indices.empty()) bundle.meshes.push_back(std::move(greenAreas));
    if (!water.indices.empty()) bundle.meshes.push_back(std::move(water));
    if (!roads.indices.empty()) bundle.meshes.push_back(std::move(roads));
    if (!buildings.indices.empty()) bundle.meshes.push_back(std::move(buildings));
    if (!trees.indices.empty()) bundle.meshes.push_back(std::move(trees));
    if (!streetFurniture.indices.empty()) bundle.meshes.push_back(std::move(streetFurniture));

    if (bundle.meshes.empty()) {
        return makeError(QStringLiteral("The selected OSM area did not contain importable buildings, roads, or map features."));
    }
    bundle.sourcePath = sceneName;
    return bundle;
}

SceneGeometryBundle loadConvertedMeshBundle(const QString& meshPath,
                                           const QString& sceneName,
                                           const QString& downloadedOsmPath,
                                           const QString& note) {
    FileSceneProvider fileProvider;
    SceneLoadRequest meshRequest;
    meshRequest.type = SceneSourceType::FileMesh;
    meshRequest.filePath = meshPath;
    SceneGeometryBundle bundle = fileProvider.load(meshRequest);
    if (!bundle.valid) {
        return bundle;
    }
    bundle.sceneName = sceneName;
    bundle.sourcePath = meshPath;
    bundle.generatedMeshPath = meshPath;
    bundle.downloadedOsmPath = downloadedOsmPath;
    bundle.generatedMeshFormat = QFileInfo(meshPath).suffix().toLower();
    bundle.note = note;
    return bundle;
}

}  // namespace

SceneGeometryBundle OsmSceneProvider::load(const SceneLoadRequest& request) {
    if (request.type == SceneSourceType::OsmFile) {
        return loadFromLocalFile(request);
    }
    if (request.type == SceneSourceType::OsmBoundingBox) {
        return loadFromBoundingBox(request);
    }
    return makeError(QStringLiteral("Unsupported OSM scene request."));
}

SceneGeometryBundle OsmSceneProvider::loadFromLocalFile(const SceneLoadRequest& request) {
    QFile file(request.filePath);
    if (!file.exists()) {
        return makeError(QStringLiteral("OSM file does not exist: %1").arg(request.filePath));
    }
    if (!file.open(QIODevice::ReadOnly)) {
        return makeError(QStringLiteral("Failed to open OSM file: %1").arg(file.errorString()));
    }

    const QString sceneName = request.sceneDisplayName.isEmpty()
                                  ? QFileInfo(request.filePath).baseName()
                                  : request.sceneDisplayName;

    reportProgress(request, QStringLiteral("Preparing local OSM file..."), 5);

    if (request.useOsm2World) {
        reportProgress(request, QStringLiteral("Starting OSM2World conversion..."), 20);
        file.close();
        QString outputMeshPath;
        QString note;
        QString errorMessage;
        if (!convertWithOsm2World(request, request.filePath, sceneName, &outputMeshPath, &note, &errorMessage)) {
            return makeError(errorMessage);
        }
        reportProgress(request, QStringLiteral("Loading generated mesh into the 3D view..."), 95);
        return loadConvertedMeshBundle(outputMeshPath, sceneName, request.filePath, note);
    }

    RawOsmData data;
    QString error;
    const QByteArray xml = file.readAll();
    reportProgress(request, QStringLiteral("Parsing OSM features..."), 35);
    if (!parseOsmXml(xml, request, &data, &error)) {
        return makeError(error);
    }

    reportProgress(request, QStringLiteral("Building scene geometry..."), 80);
    SceneGeometryBundle bundle = buildSceneBundle(data, request, sceneName);
    if (bundle.valid) {
        bundle.downloadedOsmPath = request.filePath;
        bundle.sourcePath = request.filePath;
    }
    return bundle;
}

SceneGeometryBundle OsmSceneProvider::loadFromBoundingBox(const SceneLoadRequest& request) {
    if (!(request.southLat < request.northLat && request.westLon < request.eastLon)) {
        return makeError(QStringLiteral("Invalid OSM bounding box."));
    }

    reportProgress(request, QStringLiteral("Preparing OSM download..."), 5);
    const QString cacheFile = bboxCacheFilePath(request.southLat, request.westLon, request.northLat, request.eastLon);
    QByteArray xml;
    QFile cached(cacheFile);
    if (cached.exists() && cached.open(QIODevice::ReadOnly)) {
        xml = cached.readAll();
    }

    if (xml.isEmpty()) {
        reportProgress(request, QStringLiteral("Downloading OSM data from Overpass..."), 10);
        QNetworkAccessManager manager;
        QNetworkRequest networkRequest(QUrl(request.overpassEndpoint));
        networkRequest.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/x-www-form-urlencoded"));
        networkRequest.setRawHeader("User-Agent", appUserAgent().toUtf8());
        const QByteArray body = QByteArrayLiteral("data=") + QUrl::toPercentEncoding(
                                 overpassQuery(request.southLat, request.westLon, request.northLat, request.eastLon));
        QEventLoop loop;
        QNetworkReply* reply = manager.post(networkRequest, body);
        QObject::connect(reply, &QNetworkReply::downloadProgress, [&](qint64 received, qint64 total) {
            int value = 15;
            if (total > 0) {
                const double frac = static_cast<double>(received) / static_cast<double>(total);
                value = 10 + static_cast<int>(frac * 20.0);
            }
            reportProgress(request, QStringLiteral("Downloading OSM data from Overpass..."), value);
        });
        QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
        loop.exec();
        if (reply->error() != QNetworkReply::NoError) {
            const QString err = reply->errorString();
            reply->deleteLater();
            return makeError(QStringLiteral("Failed to download OSM data from Overpass: %1").arg(err));
        }
        xml = reply->readAll();
        reply->deleteLater();

        QFile out(cacheFile);
        if (out.open(QIODevice::WriteOnly)) {
            out.write(xml);
        }
    } else {
        reportProgress(request, QStringLiteral("Using cached OSM download..."), 20);
    }

    const QString sceneName = request.sceneDisplayName.isEmpty()
                                  ? QStringLiteral("OSM area")
                                  : request.sceneDisplayName;

    QString osmFilePath = cacheFile;
    if (request.useOsm2World && request.keepDownloadedOsmCopy) {
        osmFilePath = outputOsmCopyPathFor(request, sceneName);
        if (osmFilePath != cacheFile) {
            QDir().mkpath(QFileInfo(osmFilePath).absolutePath());
            QFile::remove(osmFilePath);
            QFile source(cacheFile);
            if (source.exists()) {
                source.copy(osmFilePath);
            }
        }
    }

    if (request.useOsm2World) {
        reportProgress(request, QStringLiteral("Starting OSM2World conversion..."), 30);
        if (!QFileInfo::exists(osmFilePath)) {
            QFile out(osmFilePath);
            if (out.open(QIODevice::WriteOnly)) {
                out.write(xml);
                out.close();
            }
        }
        QString outputMeshPath;
        QString note;
        QString errorMessage;
        if (!convertWithOsm2World(request, osmFilePath, sceneName, &outputMeshPath, &note, &errorMessage)) {
            return makeError(errorMessage);
        }
        reportProgress(request, QStringLiteral("Loading generated mesh into the 3D view..."), 95);
        SceneGeometryBundle bundle = loadConvertedMeshBundle(outputMeshPath, sceneName, osmFilePath, note);
        if (bundle.valid) {
            bundle.downloadedOsmPath = osmFilePath;
        }
        return bundle;
    }

    RawOsmData data;
    QString error;
    reportProgress(request, QStringLiteral("Parsing OSM features..."), 40);
    if (!parseOsmXml(xml, request, &data, &error)) {
        return makeError(error);
    }
    reportProgress(request, QStringLiteral("Building scene geometry..."), 80);
    SceneGeometryBundle bundle = buildSceneBundle(data, request, sceneName);
    if (bundle.valid) {
        bundle.downloadedOsmPath = cacheFile;
        bundle.sourcePath = cacheFile;
    }
    return bundle;
}

}  // namespace airsim_gui
