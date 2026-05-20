#include "airsim_gui_UErealtime/MainWindow.h"

#include "airsim_gui_UErealtime/DemoController.h"
#include "airsim_gui_UErealtime/JsonConfig.h"
#include "airsim_gui_UErealtime/OsmImportDialog.h"
#include "airsim_gui_UErealtime/RosBridge.h"
#include "airsim_gui_UErealtime/Scene3DWidget.h"
#include "airsim_gui_UErealtime/SceneManager.h"
#include "airsim_gui_UErealtime/UeViewWidget.h"
#include "airsim_gui_UErealtime/AirSimViewController.h"
#include "airsim_gui_UErealtime/SionnaPreviewWidget.h"
#include "airsim_gui_UErealtime/CkmMapWidget.h"

#include <QAction>
#include <QApplication>
#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDialog>
#include <QDateTime>
#include <QDialogButtonBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QActionGroup>
#include <QColor>
#include <QTextStream>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QTextEdit>
#include <QTextFormat>
#include <QThread>
#include <QElapsedTimer>
#include <QDir>
#include <QDirIterator>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QKeySequence>
#include <QLabel>
#include <QListWidget>
#include <QMap>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QProgressBar>
#include <QProcess>
#include <QProcessEnvironment>
#include <QProgressDialog>
#include <QPushButton>
#include <QRegularExpression>
#include <QFrame>
#include <QToolButton>
#include <QSplitter>
#include <QScrollArea>
#include <QSaveFile>
#include <QSettings>
#include <QSet>
#include <QSlider>
#include <QSignalBlocker>
#include <QScopeGuard>
#include <QSpinBox>
#include <QStandardPaths>
#include <QStatusBar>
#include <QToolBar>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QTreeWidgetItem>
#include <QVector>
#include <QVBoxLayout>
#include <QTimer>
#include <ros/master.h>
#include <ros/param.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <signal.h>
#include <sys/types.h>


namespace airsim_gui {
namespace {

struct PythonEnvOption {
    QString name;
    QString version;
    QString path;
    QString kind;
};

constexpr const char* kGuiConfigExtension = ".agcfg";
constexpr int kGuiConfigVersion = 1;

void useCommittedSpinBoxEdits(QWidget* root) {
    if (!root) {
        return;
    }
    const auto spinBoxes = root->findChildren<QAbstractSpinBox*>();
    for (QAbstractSpinBox* spinBox : spinBoxes) {
        spinBox->setKeyboardTracking(false);
    }
}

QString defaultDenseCkmOutputDir() {
    QString base = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    if (base.trimmed().isEmpty()) {
        base = QDir::homePath();
    }
    return QDir(base).filePath(QStringLiteral("airsim_gui_UErealtime_ckm"));
}

QStringList splitBeamTrainingDatasetPaths(const QString& text) {
    QStringList paths;
    QSet<QString> seen;
    const QStringList tokens = text.split(QRegularExpression(QStringLiteral("[;\r\n]+")), QString::SkipEmptyParts);
    for (const QString& rawToken : tokens) {
        const QString path = rawToken.trimmed();
        if (path.isEmpty() || seen.contains(path)) {
            continue;
        }
        seen.insert(path);
        paths << path;
    }
    return paths;
}

constexpr double kPreviewCameraTargetDistanceMeters = 10.0;
constexpr qint64 kStationCameraPublishLeaseRefreshMs = 2500;
constexpr qint64 kStationCameraPublishRetryMs = 2000;
constexpr qint64 kStationCameraAirSimDisconnectGraceMs = 5000;
constexpr const char* kGuiPreviewCameraName = "GuiPreview";
constexpr const char* kCameraFpsGuiPreviewKey = "gui_preview";
constexpr const char* kCameraFpsRosTopicPrefix = "ros:";
constexpr const char* kCameraFpsStationPrefix = "station:";
constexpr const char* kDroneCameraImageLayerPrefix = "drone_camera_image:";
constexpr const char* kUntitledGuiConfigName = "untitled";
constexpr int kDefaultGuiPreviewFps = 10;
constexpr int kMaxGuiPreviewFps = 30;
constexpr int kDefaultCameraFps = 60;
constexpr int kMaxCameraFps = 100;

bool isGuiPreviewFpsKey(const QString& key) {
    return key.trimmed() == QLatin1String(kCameraFpsGuiPreviewKey);
}

int maxCameraFpsForControlKey(const QString& key) {
    return isGuiPreviewFpsKey(key) ? kMaxGuiPreviewFps : kMaxCameraFps;
}

int defaultCameraFpsForControlKey(const QString& key) {
    return isGuiPreviewFpsKey(key) ? kDefaultGuiPreviewFps : kDefaultCameraFps;
}

int clampCameraFpsForControlKey(const QString& key, int fps) {
    return std::max(1, std::min(maxCameraFpsForControlKey(key), fps));
}

bool isDroneCameraImageLayerKeyStatic(const QString& key) {
    return key.startsWith(QLatin1String(kDroneCameraImageLayerPrefix));
}

bool isAirSimLayerKey(const QString& key) {
    return key == QStringLiteral("base_stations")
        || key == QStringLiteral("drone")
        || key == QStringLiteral("history")
        || key == QStringLiteral("paths")
        || key == QStringLiteral("view_gizmo")
        || isDroneCameraImageLayerKeyStatic(key);
}

double degToRadLocal(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}

double radToDegLocal(double rad) {
    return rad * 180.0 / 3.14159265358979323846;
}

bool nearlyEqual(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) < eps;
}

bool nearlyEqualVec3(const Vec3& a, const Vec3& b, double eps = 1e-9) {
    return nearlyEqual(a.x, b.x, eps)
        && nearlyEqual(a.y, b.y, eps)
        && nearlyEqual(a.z, b.z, eps);
}

bool baseStationEquivalent(const BaseStation& a, const BaseStation& b) {
    return a.id == b.id
        && a.name == b.name
        && nearlyEqualVec3(a.position, b.position)
        && nearlyEqualVec3(a.color, b.color)
        && a.previewCameraName == b.previewCameraName
        && a.previewRosTopic == b.previewRosTopic
        && nearlyEqual(a.previewOffsetZ, b.previewOffsetZ)
        && nearlyEqual(a.previewFps, b.previewFps)
        && a.previewCameraTargetEnabled == b.previewCameraTargetEnabled
        && nearlyEqualVec3(a.previewCameraTarget, b.previewCameraTarget);
}

Vec3 directionFromYawPitchNedDeg(double yawDeg, double pitchDeg) {
    const double yaw = degToRadLocal(yawDeg);
    const double pitch = degToRadLocal(pitchDeg);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    return {cp * cy, cp * sy, -sp};
}

bool yawPitchFromDirectionNed(const Vec3& direction, double* yawDeg, double* pitchDeg) {
    const double norm = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    if (norm < 1e-9 || !yawDeg || !pitchDeg) {
        return false;
    }
    const double dx = direction.x / norm;
    const double dy = direction.y / norm;
    const double dz = direction.z / norm;
    *yawDeg = radToDegLocal(std::atan2(dy, dx));
    *pitchDeg = radToDegLocal(std::atan2(-dz, std::hypot(dx, dy)));
    return std::isfinite(*yawDeg) && std::isfinite(*pitchDeg);
}

QString joinBeamTrainingDatasetPaths(const QStringList& paths) {
    QStringList cleaned;
    QSet<QString> seen;
    for (const QString& rawPath : paths) {
        const QString path = rawPath.trimmed();
        if (path.isEmpty() || seen.contains(path)) {
            continue;
        }
        seen.insert(path);
        cleaned << path;
    }
    return cleaned.join(QStringLiteral("\n"));
}

QString effectiveBeamTrainingInputText(const QString& explicitText, const QString& fallbackDatasetPath) {
    const QStringList explicitPaths = splitBeamTrainingDatasetPaths(explicitText);
    if (!explicitPaths.isEmpty()) {
        return joinBeamTrainingDatasetPaths(explicitPaths);
    }
    return fallbackDatasetPath.trimmed();
}

QStringList beamTrainingDatasetPathsFromListWidget(const QListWidget* listWidget) {
    QStringList paths;
    if (!listWidget) {
        return paths;
    }
    QSet<QString> seen;
    for (int row = 0; row < listWidget->count(); ++row) {
        const QListWidgetItem* item = listWidget->item(row);
        if (!item) {
            continue;
        }
        const QString path = item->text().trimmed();
        if (path.isEmpty() || seen.contains(path)) {
            continue;
        }
        seen.insert(path);
        paths << path;
    }
    return paths;
}

void addBeamTrainingDatasetPathToListWidget(QListWidget* listWidget, const QString& rawPath) {
    if (!listWidget) {
        return;
    }
    const QString path = rawPath.trimmed();
    if (path.isEmpty()) {
        return;
    }
    const QString normalizedPath = QFileInfo(path).absoluteFilePath();
    for (int row = 0; row < listWidget->count(); ++row) {
        QListWidgetItem* existingItem = listWidget->item(row);
        if (!existingItem) {
            continue;
        }
        const QString existingPath = existingItem->text().trimmed();
        if (existingPath == path || QFileInfo(existingPath).absoluteFilePath() == normalizedPath) {
            listWidget->setCurrentRow(row);
            listWidget->scrollToItem(existingItem);
            return;
        }
    }
    listWidget->addItem(path);
    listWidget->setCurrentRow(listWidget->count() - 1);
}

void setBeamTrainingDatasetPathsOnListWidget(QListWidget* listWidget, const QStringList& paths) {
    if (!listWidget) {
        return;
    }
    listWidget->clear();
    for (const QString& path : paths) {
        addBeamTrainingDatasetPathToListWidget(listWidget, path);
    }
    if (listWidget->count() > 0 && listWidget->currentRow() < 0) {
        listWidget->setCurrentRow(0);
    }
}

enum class ConfigPathUse {
    ExistingPath,
    OutputPath
};

QString configFileBaseDir(const QString& configFilePath) {
    const QString trimmed = configFilePath.trimmed();
    if (trimmed.isEmpty()) {
        return QString();
    }
    return QFileInfo(trimmed).absoluteDir().absolutePath();
}

bool pathLooksLikePlainCommand(const QString& path) {
    const QString trimmed = path.trimmed();
    return !trimmed.isEmpty()
        && !QFileInfo(trimmed).isAbsolute()
        && !trimmed.contains(QLatin1Char('/'))
        && !trimmed.contains(QLatin1Char('\\'));
}

QString stationExternalCameraName(const BaseStation& station) {
    const QString configured = station.previewCameraName.trimmed();
    return configured.isEmpty() ? defaultPreviewCameraNameForStation(station) : sanitizedCameraToken(configured);
}

int findJsonObjectEndInText(const QString& text, int objectStart) {
    if (objectStart < 0 || objectStart >= text.size() || text.at(objectStart) != QChar('{')) {
        return -1;
    }

    bool inString = false;
    bool escaped = false;
    int depth = 0;
    for (int i = objectStart; i < text.size(); ++i) {
        const QChar ch = text.at(i);
        if (inString) {
            if (escaped) {
                escaped = false;
            } else if (ch == QChar('\\')) {
                escaped = true;
            } else if (ch == QChar('"')) {
                inString = false;
            }
            continue;
        }

        if (ch == QChar('"')) {
            inString = true;
        } else if (ch == QChar('{')) {
            ++depth;
        } else if (ch == QChar('}')) {
            --depth;
            if (depth == 0) {
                return i;
            }
        }
    }
    return -1;
}

QString lineIndentAtPosition(const QString& text, int pos) {
    if (text.isEmpty()) {
        return QString();
    }
    const int clamped = std::max(0, std::min(pos, text.size() - 1));
    int lineStart = text.lastIndexOf(QChar('\n'), clamped);
    lineStart = lineStart < 0 ? 0 : lineStart + 1;

    QString indent;
    for (int i = lineStart; i < text.size(); ++i) {
        const QChar ch = text.at(i);
        if (ch == QChar(' ') || ch == QChar('\t')) {
            indent += ch;
        } else {
            break;
        }
    }
    return indent;
}

bool textRangeHasNonWhitespace(const QString& text, int start, int end) {
    const int from = std::max(0, start);
    const int to = std::min(end, text.size());
    for (int i = from; i < to; ++i) {
        if (!text.at(i).isSpace()) {
            return true;
        }
    }
    return false;
}

bool textRangeHasLineBreak(const QString& text, int start, int end) {
    const int from = std::max(0, start);
    const int to = std::min(end, text.size());
    for (int i = from; i < to; ++i) {
        const QChar ch = text.at(i);
        if (ch == QChar('\n') || ch == QChar('\r')) {
            return true;
        }
    }
    return false;
}

bool readJsonStringLiteral(const QString& text, int quoteStart, QString* value, int* afterEnd) {
    if (quoteStart < 0 || quoteStart >= text.size() || text.at(quoteStart) != QChar('"')) {
        return false;
    }

    QString parsed;
    bool escaped = false;
    for (int i = quoteStart + 1; i < text.size(); ++i) {
        const QChar ch = text.at(i);
        if (escaped) {
            parsed += ch;
            escaped = false;
            continue;
        }
        if (ch == QChar('\\')) {
            escaped = true;
            continue;
        }
        if (ch == QChar('"')) {
            if (value) {
                *value = parsed;
            }
            if (afterEnd) {
                *afterEnd = i + 1;
            }
            return true;
        }
        parsed += ch;
    }
    return false;
}

int skipJsonWhitespace(const QString& text, int pos) {
    while (pos < text.size() && text.at(pos).isSpace()) {
        ++pos;
    }
    return pos;
}

int findTopLevelObjectPropertyObjectStart(const QString& text,
                                          int rootStart,
                                          int rootEnd,
                                          const QString& propertyName,
                                          int* keyStart) {
    if (keyStart) {
        *keyStart = -1;
    }
    if (rootStart < 0 || rootEnd <= rootStart || rootEnd > text.size()) {
        return -1;
    }

    int depth = 1;
    bool inString = false;
    bool escaped = false;
    for (int i = rootStart + 1; i < rootEnd; ++i) {
        const QChar ch = text.at(i);
        if (inString) {
            if (escaped) {
                escaped = false;
            } else if (ch == QChar('\\')) {
                escaped = true;
            } else if (ch == QChar('"')) {
                inString = false;
            }
            continue;
        }

        if (ch == QChar('"')) {
            if (depth == 1) {
                QString name;
                int afterString = -1;
                if (readJsonStringLiteral(text, i, &name, &afterString)) {
                    int cursor = skipJsonWhitespace(text, afterString);
                    if (cursor < text.size() && text.at(cursor) == QChar(':')) {
                        cursor = skipJsonWhitespace(text, cursor + 1);
                        if (name == propertyName && cursor < text.size() && text.at(cursor) == QChar('{')) {
                            if (keyStart) {
                                *keyStart = i;
                            }
                            return cursor;
                        }
                    }
                    i = afterString - 1;
                    continue;
                }
            }
            inString = true;
        } else if (ch == QChar('{') || ch == QChar('[')) {
            ++depth;
        } else if (ch == QChar('}') || ch == QChar(']')) {
            --depth;
        }
    }
    return -1;
}

int jsonInsertionPointBeforeClosingBrace(const QString& text, int objectStart, int objectEnd) {
    int insertionPos = objectEnd;
    while (insertionPos > objectStart + 1 && text.at(insertionPos - 1).isSpace()) {
        --insertionPos;
    }
    return insertionPos;
}

QString externalCameraEntriesInsertionText(const QJsonObject& entries, const QString& entryIndent) {
    const QString objectText = QString::fromUtf8(QJsonDocument(entries).toJson(QJsonDocument::Indented)).trimmed();
    QStringList lines = objectText.split(QChar('\n'));
    if (lines.size() <= 2) {
        return QString();
    }
    lines.removeFirst();
    lines.removeLast();
    for (QString& line : lines) {
        if (line.startsWith(QStringLiteral("    "))) {
            line = line.mid(4);
        }
        line.prepend(entryIndent);
    }
    return lines.join(QChar('\n'));
}

QVector<QPair<int, int>> highlightedJsonObjectRanges(const QString& text, const QStringList& objectNames) {
    QVector<QPair<int, int>> ranges;
    const QStringList lines = text.split(QChar('\n'));
    for (const QString& name : objectNames) {
        const QString marker = QStringLiteral("\"%1\"").arg(name);
        for (int i = 0; i < lines.size(); ++i) {
            if (!lines.at(i).contains(marker) || !lines.at(i).contains(QChar('{'))) {
                continue;
            }
            int braceDepth = 0;
            bool started = false;
            int endLine = i;
            for (int j = i; j < lines.size(); ++j) {
                const QString& line = lines.at(j);
                bool inString = false;
                bool escaped = false;
                for (const QChar ch : line) {
                    if (inString) {
                        if (escaped) {
                            escaped = false;
                        } else if (ch == QChar('\\')) {
                            escaped = true;
                        } else if (ch == QChar('"')) {
                            inString = false;
                        }
                        continue;
                    }
                    if (ch == QChar('"')) {
                        inString = true;
                    } else if (ch == QChar('{')) {
                        ++braceDepth;
                        started = true;
                    } else if (ch == QChar('}')) {
                        --braceDepth;
                    }
                }
                if (started && braceDepth <= 0) {
                    endLine = j;
                    break;
                }
            }
            ranges.push_back(qMakePair(i, endLine));
            break;
        }
    }
    return ranges;
}

QString portablePathFileName(const QString& relativePath, const QString& absolutePath = QString()) {
    const QString relativeFileName = QFileInfo(relativePath.trimmed()).fileName();
    if (!relativeFileName.isEmpty()) {
        return relativeFileName;
    }
    return QFileInfo(absolutePath.trimmed()).fileName();
}

QString configLocalFallbackExistingPath(const QString& fileName, const QString& configFilePath) {
    const QString trimmedFileName = fileName.trimmed();
    const QString baseDir = configFileBaseDir(configFilePath);
    if (trimmedFileName.isEmpty() || baseDir.isEmpty()) {
        return QString();
    }

    const QStringList fallbackPaths = {
        trimmedFileName,
        QStringLiteral("config/") + trimmedFileName
    };
    for (const QString& relativePath : fallbackPaths) {
        const QString candidate = QFileInfo(QDir(baseDir), relativePath).absoluteFilePath();
        if (QFileInfo::exists(candidate)) {
            return candidate;
        }
    }
    return QString();
}

QString configRelativePathToAbsolute(const QString& path, const QString& configFilePath) {
    const QString trimmed = path.trimmed();
    if (trimmed.isEmpty()) {
        return QString();
    }
    const QFileInfo info(trimmed);
    if (info.isAbsolute()) {
        return info.absoluteFilePath();
    }
    const QString baseDir = configFileBaseDir(configFilePath);
    if (baseDir.isEmpty()) {
        return QFileInfo(trimmed).absoluteFilePath();
    }
    return QFileInfo(QDir(baseDir), trimmed).absoluteFilePath();
}

bool configPathCandidateUsable(const QString& path, ConfigPathUse use) {
    if (path.trimmed().isEmpty()) {
        return false;
    }
    if (use == ConfigPathUse::OutputPath) {
        return true;
    }
    return QFileInfo::exists(path);
}

QJsonValue portableConfigPathValue(const QString& path,
                                   const QString& configFilePath,
                                   bool allowPlainCommand = false) {
    const QString trimmed = path.trimmed();
    if (trimmed.isEmpty()) {
        return QString();
    }
    if (allowPlainCommand && pathLooksLikePlainCommand(trimmed)) {
        return trimmed;
    }

    const QString absolutePath = configRelativePathToAbsolute(trimmed, configFilePath);
    const QString baseDir = configFileBaseDir(configFilePath);
    if (baseDir.isEmpty()) {
        return absolutePath;
    }

    QJsonObject obj;
    obj[QStringLiteral("relative_path")] = QDir(baseDir).relativeFilePath(absolutePath);
    obj[QStringLiteral("absolute_path")] = absolutePath;
    return obj;
}

QString resolvePortableConfigPathValue(const QJsonValue& value,
                                       const QString& configFilePath,
                                       ConfigPathUse use,
                                       bool allowPlainCommand = false) {
    if (value.isObject()) {
        const QJsonObject obj = value.toObject();
        const QString relativePath = obj.value(QStringLiteral("relative_path")).toString().trimmed();
        const QString absolutePath = obj.value(QStringLiteral("absolute_path")).toString().trimmed();

        QString relativeCandidate;
        if (!relativePath.isEmpty()) {
            relativeCandidate = configRelativePathToAbsolute(relativePath, configFilePath);
            if (configPathCandidateUsable(relativeCandidate, use)) {
                return relativeCandidate;
            }
        }

        QString absoluteCandidate;
        if (!absolutePath.isEmpty()) {
            absoluteCandidate = QFileInfo(absolutePath).isAbsolute()
                ? QFileInfo(absolutePath).absoluteFilePath()
                : configRelativePathToAbsolute(absolutePath, configFilePath);
            if (configPathCandidateUsable(absoluteCandidate, use)) {
                return absoluteCandidate;
            }
        }

        if (use == ConfigPathUse::ExistingPath) {
            const QString localFallback = configLocalFallbackExistingPath(
                portablePathFileName(relativePath, absolutePath),
                configFilePath);
            if (!localFallback.isEmpty()) {
                return localFallback;
            }
        }

        if (!relativeCandidate.isEmpty()) {
            return relativeCandidate;
        }
        if (!absoluteCandidate.isEmpty()) {
            return absoluteCandidate;
        }
        return absolutePath;
    }

    const QString path = value.toString().trimmed();
    if (path.isEmpty()) {
        return QString();
    }
    if (allowPlainCommand && pathLooksLikePlainCommand(path)) {
        return path;
    }
    const QString resolvedPath = configRelativePathToAbsolute(path, configFilePath);
    if (use == ConfigPathUse::ExistingPath && !QFileInfo::exists(resolvedPath)) {
        const QString localFallback = configLocalFallbackExistingPath(
            portablePathFileName(path),
            configFilePath);
        if (!localFallback.isEmpty()) {
            return localFallback;
        }
    }
    return resolvedPath;
}

QJsonValue portableConfigPathListValue(const QString& text,
                                       const QString& configFilePath) {
    const QStringList paths = splitBeamTrainingDatasetPaths(text);
    if (paths.isEmpty()) {
        return QString();
    }

    const QString baseDir = configFileBaseDir(configFilePath);
    if (baseDir.isEmpty()) {
        QStringList absolutePaths;
        for (const QString& path : paths) {
            absolutePaths << configRelativePathToAbsolute(path, configFilePath);
        }
        return joinBeamTrainingDatasetPaths(absolutePaths);
    }

    QJsonArray relativePaths;
    QJsonArray absolutePaths;
    for (const QString& path : paths) {
        const QString absolutePath = configRelativePathToAbsolute(path, configFilePath);
        relativePaths.push_back(QDir(baseDir).relativeFilePath(absolutePath));
        absolutePaths.push_back(absolutePath);
    }
    QJsonObject obj;
    obj[QStringLiteral("relative_paths")] = relativePaths;
    obj[QStringLiteral("absolute_paths")] = absolutePaths;
    return obj;
}

QString resolvePortableConfigPathListValue(const QJsonValue& value,
                                           const QString& configFilePath,
                                           ConfigPathUse use) {
    if (value.isObject()) {
        const QJsonObject obj = value.toObject();
        const QJsonArray relativePaths = obj.value(QStringLiteral("relative_paths")).toArray();
        const QJsonArray absolutePaths = obj.value(QStringLiteral("absolute_paths")).toArray();
        const int count = std::max(relativePaths.size(), absolutePaths.size());
        QStringList resolvedPaths;
        for (int i = 0; i < count; ++i) {
            QJsonObject pair;
            if (i < relativePaths.size()) {
                pair[QStringLiteral("relative_path")] = relativePaths.at(i).toString();
            }
            if (i < absolutePaths.size()) {
                pair[QStringLiteral("absolute_path")] = absolutePaths.at(i).toString();
            }
            const QString resolved = resolvePortableConfigPathValue(pair, configFilePath, use);
            if (!resolved.trimmed().isEmpty()) {
                resolvedPaths << resolved;
            }
        }
        return joinBeamTrainingDatasetPaths(resolvedPaths);
    }

    if (value.isArray()) {
        QStringList resolvedPaths;
        const QJsonArray arr = value.toArray();
        for (const QJsonValue& item : arr) {
            const QString resolved = resolvePortableConfigPathValue(item, configFilePath, use);
            if (!resolved.trimmed().isEmpty()) {
                resolvedPaths << resolved;
            }
        }
        return joinBeamTrainingDatasetPaths(resolvedPaths);
    }

    QStringList resolvedPaths;
    for (const QString& path : splitBeamTrainingDatasetPaths(value.toString())) {
        const QString resolved = resolvePortableConfigPathValue(path, configFilePath, use);
        if (!resolved.trimmed().isEmpty()) {
            resolvedPaths << resolved;
        }
    }
    return joinBeamTrainingDatasetPaths(resolvedPaths);
}

void makePathFieldPortable(QJsonObject& obj,
                           const QString& key,
                           const QString& configFilePath,
                           bool allowPlainCommand = false) {
    if (obj.contains(key)) {
        obj[key] = portableConfigPathValue(obj.value(key).toString(), configFilePath, allowPlainCommand);
    }
}

void makePathListFieldPortable(QJsonObject& obj,
                               const QString& key,
                               const QString& configFilePath) {
    if (obj.contains(key)) {
        obj[key] = portableConfigPathListValue(obj.value(key).toString(), configFilePath);
    }
}

void resolvePathField(QJsonObject& obj,
                      const QString& key,
                      const QString& configFilePath,
                      ConfigPathUse use,
                      bool allowPlainCommand = false) {
    if (obj.contains(key)) {
        obj[key] = resolvePortableConfigPathValue(obj.value(key), configFilePath, use, allowPlainCommand);
    }
}

void resolvePathListField(QJsonObject& obj,
                          const QString& key,
                          const QString& configFilePath,
                          ConfigPathUse use) {
    if (obj.contains(key)) {
        obj[key] = resolvePortableConfigPathListValue(obj.value(key), configFilePath, use);
    }
}

void makeGuiConfigPathsPortable(QJsonObject& root, const QString& configFilePath) {
    makePathFieldPortable(root, QStringLiteral("base_stations_file_path"), configFilePath);
    makePathFieldPortable(root, QStringLiteral("imported_scene_mesh_path"), configFilePath);

    if (root.value(QStringLiteral("simulation_settings")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("simulation_settings")).toObject();
        makePathFieldPortable(obj, QStringLiteral("scene_path"), configFilePath);
        makePathFieldPortable(obj, QStringLiteral("python_executable"), configFilePath, true);
        makePathFieldPortable(obj, QStringLiteral("beam_codebook_file_path"), configFilePath);
        makePathFieldPortable(obj, QStringLiteral("beam_model_checkpoint_path"), configFilePath);
        makePathFieldPortable(obj, QStringLiteral("beam_training_dataset_path"), configFilePath);
        makePathListFieldPortable(obj, QStringLiteral("beam_training_input_paths"), configFilePath);
        makePathFieldPortable(obj, QStringLiteral("beam_training_output_checkpoint_path"), configFilePath);
        makePathFieldPortable(obj, QStringLiteral("csv_path"), configFilePath);
        root[QStringLiteral("simulation_settings")] = obj;
    }

    if (root.value(QStringLiteral("ckm_settings")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("ckm_settings")).toObject();
        makePathFieldPortable(obj, QStringLiteral("output_dir"), configFilePath);
        makePathFieldPortable(obj, QStringLiteral("render_scene_path"), configFilePath);
        root[QStringLiteral("ckm_settings")] = obj;
    }

    if (root.value(QStringLiteral("live_view")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("live_view")).toObject();
        makePathFieldPortable(obj, QStringLiteral("airsim_settings_path"), configFilePath);
        root[QStringLiteral("live_view")] = obj;
    }

    if (root.value(QStringLiteral("coordinate_transforms")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("coordinate_transforms")).toObject();
        makePathFieldPortable(obj, QStringLiteral("ros_to_scene_matrix_path"), configFilePath);
        makePathFieldPortable(obj, QStringLiteral("airsim_to_scene_matrix_path"), configFilePath);
        makePathFieldPortable(obj, QStringLiteral("uav_to_rx_array_center_matrix_path"), configFilePath);
        makePathFieldPortable(obj, QStringLiteral("rx_array_elements_json_path"), configFilePath);
        root[QStringLiteral("coordinate_transforms")] = obj;
    }

    if (root.value(QStringLiteral("rosbag_record")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("rosbag_record")).toObject();
        makePathFieldPortable(obj, QStringLiteral("output_path"), configFilePath);
        root[QStringLiteral("rosbag_record")] = obj;
    }

    if (root.value(QStringLiteral("rosbag_playback")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("rosbag_playback")).toObject();
        makePathFieldPortable(obj, QStringLiteral("input_bag"), configFilePath);
        root[QStringLiteral("rosbag_playback")] = obj;
    }

    if (root.value(QStringLiteral("rosbag_resim")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("rosbag_resim")).toObject();
        makePathFieldPortable(obj, QStringLiteral("input_bag"), configFilePath);
        makePathFieldPortable(obj, QStringLiteral("output_bag"), configFilePath);
        root[QStringLiteral("rosbag_resim")] = obj;
    }
}

void resolveGuiConfigPaths(QJsonObject& root, const QString& configFilePath) {
    resolvePathField(root, QStringLiteral("base_stations_file_path"), configFilePath, ConfigPathUse::ExistingPath);
    resolvePathField(root, QStringLiteral("imported_scene_mesh_path"), configFilePath, ConfigPathUse::ExistingPath);

    if (root.value(QStringLiteral("simulation_settings")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("simulation_settings")).toObject();
        resolvePathField(obj, QStringLiteral("scene_path"), configFilePath, ConfigPathUse::ExistingPath);
        resolvePathField(obj, QStringLiteral("python_executable"), configFilePath, ConfigPathUse::ExistingPath, true);
        resolvePathField(obj, QStringLiteral("beam_codebook_file_path"), configFilePath, ConfigPathUse::ExistingPath);
        resolvePathField(obj, QStringLiteral("beam_model_checkpoint_path"), configFilePath, ConfigPathUse::ExistingPath);
        resolvePathField(obj, QStringLiteral("beam_training_dataset_path"), configFilePath, ConfigPathUse::OutputPath);
        resolvePathListField(obj, QStringLiteral("beam_training_input_paths"), configFilePath, ConfigPathUse::ExistingPath);
        resolvePathField(obj, QStringLiteral("beam_training_output_checkpoint_path"), configFilePath, ConfigPathUse::OutputPath);
        resolvePathField(obj, QStringLiteral("csv_path"), configFilePath, ConfigPathUse::OutputPath);
        root[QStringLiteral("simulation_settings")] = obj;
    }

    if (root.value(QStringLiteral("ckm_settings")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("ckm_settings")).toObject();
        resolvePathField(obj, QStringLiteral("output_dir"), configFilePath, ConfigPathUse::OutputPath);
        resolvePathField(obj, QStringLiteral("render_scene_path"), configFilePath, ConfigPathUse::ExistingPath);
        root[QStringLiteral("ckm_settings")] = obj;
    }

    if (root.value(QStringLiteral("live_view")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("live_view")).toObject();
        resolvePathField(obj, QStringLiteral("airsim_settings_path"), configFilePath, ConfigPathUse::ExistingPath);
        root[QStringLiteral("live_view")] = obj;
    }

    if (root.value(QStringLiteral("coordinate_transforms")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("coordinate_transforms")).toObject();
        resolvePathField(obj, QStringLiteral("ros_to_scene_matrix_path"), configFilePath, ConfigPathUse::ExistingPath);
        resolvePathField(obj, QStringLiteral("airsim_to_scene_matrix_path"), configFilePath, ConfigPathUse::ExistingPath);
        resolvePathField(obj, QStringLiteral("uav_to_rx_array_center_matrix_path"), configFilePath, ConfigPathUse::ExistingPath);
        resolvePathField(obj, QStringLiteral("rx_array_elements_json_path"), configFilePath, ConfigPathUse::ExistingPath);
        root[QStringLiteral("coordinate_transforms")] = obj;
    }

    if (root.value(QStringLiteral("rosbag_record")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("rosbag_record")).toObject();
        resolvePathField(obj, QStringLiteral("output_path"), configFilePath, ConfigPathUse::OutputPath);
        root[QStringLiteral("rosbag_record")] = obj;
    }

    if (root.value(QStringLiteral("rosbag_playback")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("rosbag_playback")).toObject();
        resolvePathField(obj, QStringLiteral("input_bag"), configFilePath, ConfigPathUse::ExistingPath);
        root[QStringLiteral("rosbag_playback")] = obj;
    }

    if (root.value(QStringLiteral("rosbag_resim")).isObject()) {
        QJsonObject obj = root.value(QStringLiteral("rosbag_resim")).toObject();
        resolvePathField(obj, QStringLiteral("input_bag"), configFilePath, ConfigPathUse::ExistingPath);
        resolvePathField(obj, QStringLiteral("output_bag"), configFilePath, ConfigPathUse::OutputPath);
        root[QStringLiteral("rosbag_resim")] = obj;
    }
}

QString coordinateOutputFrameDisplayName(const QString& key) {
    switch (coordinateOutputFrameFromKey(key)) {
    case CoordinateOutputFrame::Ros:
        return QStringLiteral("ROS");
    case CoordinateOutputFrame::AirSim:
        return QStringLiteral("AirSim");
    case CoordinateOutputFrame::Scene3D:
    default:
        return QStringLiteral("3D");
    }
}

QString normalizeRxArrayFacingDirectionKey(const QString& key) {
    const QString normalized = key.trimmed().toLower();
    if (normalized == QStringLiteral("back")
        || normalized == QStringLiteral("left")
        || normalized == QStringLiteral("right")
        || normalized == QStringLiteral("up")
        || normalized == QStringLiteral("down")) {
        return normalized;
    }
    return QStringLiteral("front");
}

QString rxArrayFacingDirectionDisplayName(const QString& key) {
    const QString normalized = normalizeRxArrayFacingDirectionKey(key);
    if (normalized == QStringLiteral("back")) {
        return QObject::tr("Back (-X, Y-Z plane)");
    }
    if (normalized == QStringLiteral("left")) {
        return QObject::tr("Left (+Y, X-Z plane)");
    }
    if (normalized == QStringLiteral("right")) {
        return QObject::tr("Right (-Y, X-Z plane)");
    }
    if (normalized == QStringLiteral("up")) {
        return QObject::tr("Up (+Z, X-Y plane)");
    }
    if (normalized == QStringLiteral("down")) {
        return QObject::tr("Down (-Z, X-Y plane)");
    }
    return QObject::tr("Front (+X, Y-Z plane)");
}

Vec3 crossProduct(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

double vectorNorm3(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 normalizedVectorOrFallback(const Vec3& value, const Vec3& fallback) {
    const double valueNorm = vectorNorm3(value);
    if (valueNorm > 1e-12 && std::isfinite(valueNorm)) {
        return {value.x / valueNorm, value.y / valueNorm, value.z / valueNorm};
    }
    const double fallbackNorm = vectorNorm3(fallback);
    if (fallbackNorm > 1e-12 && std::isfinite(fallbackNorm)) {
        return {fallback.x / fallbackNorm, fallback.y / fallbackNorm, fallback.z / fallbackNorm};
    }
    return {1.0, 0.0, 0.0};
}

TransformMatrix4x4 buildUavToRxArrayCenterTransform(const QString& facingDirection,
                                                    double tx,
                                                    double ty,
                                                    double tz) {
    const QString normalizedFacing = normalizeRxArrayFacingDirectionKey(facingDirection);
    Vec3 xAxis{1.0, 0.0, 0.0};
    if (normalizedFacing == QStringLiteral("back")) {
        xAxis = {-1.0, 0.0, 0.0};
    } else if (normalizedFacing == QStringLiteral("left")) {
        xAxis = {0.0, 1.0, 0.0};
    } else if (normalizedFacing == QStringLiteral("right")) {
        xAxis = {0.0, -1.0, 0.0};
    } else if (normalizedFacing == QStringLiteral("up")) {
        xAxis = {0.0, 0.0, 1.0};
    } else if (normalizedFacing == QStringLiteral("down")) {
        xAxis = {0.0, 0.0, -1.0};
    }

    xAxis = normalizedVectorOrFallback(xAxis, {1.0, 0.0, 0.0});
    Vec3 yAxis{};
    Vec3 zAxis{};
    if (normalizedFacing == QStringLiteral("up") || normalizedFacing == QStringLiteral("down")) {
        yAxis = normalizedVectorOrFallback({0.0, 1.0, 0.0}, {0.0, 1.0, 0.0});
        zAxis = normalizedVectorOrFallback(crossProduct(xAxis, yAxis), {1.0, 0.0, 0.0});
        yAxis = normalizedVectorOrFallback(crossProduct(zAxis, xAxis), {0.0, 1.0, 0.0});
    } else {
        zAxis = normalizedVectorOrFallback({0.0, 0.0, 1.0}, {0.0, 0.0, 1.0});
        yAxis = normalizedVectorOrFallback(crossProduct(zAxis, xAxis), {0.0, 1.0, 0.0});
        zAxis = normalizedVectorOrFallback(crossProduct(xAxis, yAxis), {0.0, 0.0, 1.0});
    }

    TransformMatrix4x4 rxArrayInUav = identityTransformMatrix4x4();
    rxArrayInUav.m[0][0] = xAxis.x;
    rxArrayInUav.m[1][0] = xAxis.y;
    rxArrayInUav.m[2][0] = xAxis.z;
    rxArrayInUav.m[0][1] = yAxis.x;
    rxArrayInUav.m[1][1] = yAxis.y;
    rxArrayInUav.m[2][1] = yAxis.z;
    rxArrayInUav.m[0][2] = zAxis.x;
    rxArrayInUav.m[1][2] = zAxis.y;
    rxArrayInUav.m[2][2] = zAxis.z;
    rxArrayInUav.m[0][3] = std::isfinite(tx) ? tx : 0.0;
    rxArrayInUav.m[1][3] = std::isfinite(ty) ? ty : 0.0;
    rxArrayInUav.m[2][3] = std::isfinite(tz) ? tz : 0.0;
    rxArrayInUav.valid = true;

    TransformMatrix4x4 uavToRxArray = identityTransformMatrix4x4();
    if (!invertTransformMatrix4x4(rxArrayInUav, &uavToRxArray)) {
        return identityTransformMatrix4x4();
    }
    uavToRxArray.valid = true;
    return uavToRxArray;
}

bool isSupportedSceneMeshFilePath(const QString& path) {
    const QString normalizedPath = path.trimmed();
    if (normalizedPath.isEmpty()) {
        return false;
    }
    const QFileInfo fileInfo(normalizedPath);
    if (!fileInfo.exists() || !fileInfo.isFile()) {
        return false;
    }
    const QString suffix = fileInfo.suffix().trimmed().toLower();
    return suffix == QStringLiteral("obj")
        || suffix == QStringLiteral("fbx")
        || suffix == QStringLiteral("stl")
        || suffix == QStringLiteral("gltf")
        || suffix == QStringLiteral("glb");
}

QString findLatestDenseCkmMetaFile(const QString& rootDirPath) {
    const QString rootPath = rootDirPath.trimmed();
    if (rootPath.isEmpty() || !QFileInfo::exists(rootPath)) {
        return QString();
    }

    QFileInfo bestInfo;
    QDirIterator it(rootPath, QStringList{QStringLiteral("meta.json")}, QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        const QString candidatePath = it.next();
        const QFileInfo info(candidatePath);
        if (!info.exists()) {
            continue;
        }
        if (!bestInfo.exists() || info.lastModified() > bestInfo.lastModified()) {
            bestInfo = info;
        }
    }
    return bestInfo.exists() ? bestInfo.absoluteFilePath() : QString();
}

QStringList denseCkmMetricList() {
    return {
        QStringLiteral("power_db"),
        QStringLiteral("path_loss_db"),
        QStringLiteral("tau_std_ns"),
        QStringLiteral("los_binary"),
        QStringLiteral("best_bs_index"),
        QStringLiteral("serving_bs_index"),
        QStringLiteral("sys_sinr_eff_db"),
        QStringLiteral("sys_spectral_efficiency_bpshz"),
        QStringLiteral("sys_mcs_index"),
        QStringLiteral("sys_tb_ok"),
        QStringLiteral("best_bs_rate_bpshz"),
        QStringLiteral("beam_oracle_index"),
        QStringLiteral("beam_oracle_gain_db"),
        QStringLiteral("beam_deepsense_index"),
        QStringLiteral("beam_deepsense_gain_db")
    };
}

QStringList denseCkmSysMetricList() {
    return {
        QStringLiteral("serving_bs_index"),
        QStringLiteral("sys_sinr_eff_db"),
        QStringLiteral("sys_spectral_efficiency_bpshz"),
        QStringLiteral("sys_mcs_index"),
        QStringLiteral("sys_tb_ok"),
        QStringLiteral("best_bs_rate_bpshz")
    };
}

QString denseCkmMetricDisplayName(const QString& metric) {
    static const QMap<QString, QString> labels = {
        {QStringLiteral("power_db"), QStringLiteral("Power (dB)")},
        {QStringLiteral("path_loss_db"), QStringLiteral("Path loss (dB)")},
        {QStringLiteral("tau_std_ns"), QStringLiteral("RMS delay spread (ns)")},
        {QStringLiteral("los_binary"), QStringLiteral("LoS binary")},
        {QStringLiteral("best_bs_index"), QStringLiteral("Best BS index")},
        {QStringLiteral("serving_bs_index"), QStringLiteral("Serving BS index")},
        {QStringLiteral("sys_sinr_eff_db"), QStringLiteral("SYS effective SINR (dB)")},
        {QStringLiteral("sys_spectral_efficiency_bpshz"), QStringLiteral("SYS spectral efficiency (bps/Hz)")},
        {QStringLiteral("sys_mcs_index"), QStringLiteral("SYS MCS index")},
        {QStringLiteral("sys_tb_ok"), QStringLiteral("SYS TB OK")},
        {QStringLiteral("best_bs_rate_bpshz"), QStringLiteral("Best BS rate (bps/Hz)")},
        {QStringLiteral("beam_oracle_index"), QStringLiteral("Oracle beam index")},
        {QStringLiteral("beam_oracle_gain_db"), QStringLiteral("Oracle beam gain (dB)")},
        {QStringLiteral("beam_deepsense_index"), QStringLiteral("Predicted beam index")},
        {QStringLiteral("beam_deepsense_gain_db"), QStringLiteral("Predicted beam gain (dB)")},
    };
    const QString key = metric.trimmed();
    return labels.value(key, key);
}

QString denseCkmMetricUiKey(const QString& metric) {
    const QString key = metric.trimmed();
    if (key == QLatin1String("beam_deepsense_index")) {
        return QStringLiteral("beam_predicted_index");
    }
    if (key == QLatin1String("beam_deepsense_gain_db")) {
        return QStringLiteral("beam_predicted_gain_db");
    }
    return key;
}

QString denseCkmMetricUiListText(const QStringList& metrics) {
    QStringList labels;
    for (const QString& metric : metrics) {
        labels << denseCkmMetricUiKey(metric);
    }
    return labels.join(QStringLiteral(", "));
}

QString sanitizeBeamTerminologyForUi(QString text) {
    text.replace(QStringLiteral("DeepSense selected beam"), QStringLiteral("Predicted beam"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSense-beam"), QStringLiteral("Predicted-beam"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSense6G-style"), QStringLiteral("beam-selection"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSense-style"), QStringLiteral("beam-selection"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSenseBeamPredictor"), QStringLiteral("BeamSelectionPredictor"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSenseCheckpointMetadata"), QStringLiteral("BeamSelectionCheckpointMetadata"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("train_deepsense_beam_model.py"), QStringLiteral("train_beam_selection_model.py"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("deepsense_beam_selector.py"), QStringLiteral("beam_selection_model.py"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("deepsense_predictor"), QStringLiteral("learned_selector"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("deepsense_pos_bbox4"), QStringLiteral("pos_bbox4"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("beam_deepsense"), QStringLiteral("beam_predicted"), Qt::CaseInsensitive);
    text.replace(QStringLiteral("DeepSense"), QStringLiteral("Beam selection"), Qt::CaseInsensitive);
    return text;
}

QString beamSelectionModeToUi(const QString& mode) {
    const QString trimmed = mode.trimmed();
    if (trimmed.compare(QStringLiteral("deepsense_predictor"), Qt::CaseInsensitive) == 0) {
        return QStringLiteral("learned_selector");
    }
    return trimmed;
}

QString beamSelectionModeFromUi(const QString& mode) {
    const QString trimmed = mode.trimmed();
    if (trimmed.compare(QStringLiteral("learned_selector"), Qt::CaseInsensitive) == 0
        || trimmed.compare(QStringLiteral("beam_selector"), Qt::CaseInsensitive) == 0
        || trimmed.compare(QStringLiteral("beam_selection"), Qt::CaseInsensitive) == 0
        || trimmed.compare(QStringLiteral("beam_selection_model"), Qt::CaseInsensitive) == 0) {
        return QStringLiteral("deepsense_predictor");
    }
    return trimmed;
}

QString beamFeatureModeToUi(const QString& mode) {
    const QString trimmed = mode.trimmed();
    if (trimmed.compare(QStringLiteral("deepsense_pos_bbox4"), Qt::CaseInsensitive) == 0) {
        return QStringLiteral("pos_bbox4");
    }
    return trimmed;
}

QString beamFeatureModeFromUi(const QString& mode) {
    const QString trimmed = mode.trimmed();
    if (trimmed.compare(QStringLiteral("pos_bbox4"), Qt::CaseInsensitive) == 0
        || trimmed.compare(QStringLiteral("beam_pos_bbox4"), Qt::CaseInsensitive) == 0) {
        return QStringLiteral("deepsense_pos_bbox4");
    }
    return trimmed;
}

bool denseCkmMetricNeedsSys(const QString& metric) {
    return denseCkmSysMetricList().contains(metric.trimmed());
}

bool denseCkmMetricNeedsBeam(const QString& metric) {
    static const QStringList kBeamMetrics = {
        QStringLiteral("beam_oracle_index"),
        QStringLiteral("beam_oracle_gain_db"),
        QStringLiteral("beam_deepsense_index"),
        QStringLiteral("beam_deepsense_gain_db"),
    };
    return kBeamMetrics.contains(metric.trimmed());
}

bool denseCkmMetricNeedsPredictedBeamCheckpoint(const QString& metric) {
    static const QStringList kPredictedBeamMetrics = {
        QStringLiteral("beam_deepsense_index"),
        QStringLiteral("beam_deepsense_gain_db"),
    };
    return kPredictedBeamMetrics.contains(metric.trimmed());
}

QStringList normalizeDenseCkmMetrics(const QStringList& rawMetrics, const QString& fallbackMetric = QStringLiteral("power_db")) {
    const QStringList supported = denseCkmMetricList();
    QStringList normalized;
    QSet<QString> seen;
    for (const QString& rawMetric : rawMetrics) {
        const QString metric = rawMetric.trimmed();
        if (metric.isEmpty() || !supported.contains(metric) || seen.contains(metric)) {
            continue;
        }
        normalized << metric;
        seen.insert(metric);
    }
    const QString fallback = supported.contains(fallbackMetric.trimmed()) ? fallbackMetric.trimmed() : QStringLiteral("power_db");
    if (normalized.isEmpty()) {
        normalized << fallback;
    }
    return normalized;
}

QJsonObject ckmSettingsToJson(const CkmSettings& value) {
    QJsonObject obj;
    obj[QStringLiteral("metric")] = value.metric;
    obj[QStringLiteral("metrics")] = QJsonArray::fromStringList(normalizeDenseCkmMetrics(value.selectedMetrics, value.metric));
    obj[QStringLiteral("use_scene_bounds")] = value.useSceneBounds;
    obj[QStringLiteral("x_min")] = value.xMin;
    obj[QStringLiteral("x_max")] = value.xMax;
    obj[QStringLiteral("y_min")] = value.yMin;
    obj[QStringLiteral("y_max")] = value.yMax;
    obj[QStringLiteral("z_fixed")] = value.zFixed;
    obj[QStringLiteral("resolution_m")] = value.resolutionM;
    obj[QStringLiteral("output_dir")] = value.outputDir;
    obj[QStringLiteral("auto_load_result")] = value.autoLoadResult;
    obj[QStringLiteral("render_scene_overlay")] = value.renderSceneOverlay;
    obj[QStringLiteral("render_scene_path")] = value.renderScenePath;
    return obj;
}

CkmSettings ckmSettingsFromJson(const QJsonObject& obj, const CkmSettings& fallback) {
    CkmSettings value = fallback;
    if (obj.contains(QStringLiteral("metric"))) value.metric = obj.value(QStringLiteral("metric")).toString(value.metric).trimmed();
    if (obj.contains(QStringLiteral("metrics")) && obj.value(QStringLiteral("metrics")).isArray()) {
        QStringList metrics;
        const QJsonArray arr = obj.value(QStringLiteral("metrics")).toArray();
        for (const QJsonValue& item : arr) {
            metrics << item.toString();
        }
        value.selectedMetrics = normalizeDenseCkmMetrics(metrics, value.metric);
    } else {
        value.selectedMetrics = normalizeDenseCkmMetrics(QStringList{value.metric}, value.metric);
    }
    if (obj.contains(QStringLiteral("use_scene_bounds"))) value.useSceneBounds = obj.value(QStringLiteral("use_scene_bounds")).toBool(value.useSceneBounds);
    if (obj.contains(QStringLiteral("x_min"))) value.xMin = obj.value(QStringLiteral("x_min")).toDouble(value.xMin);
    if (obj.contains(QStringLiteral("x_max"))) value.xMax = obj.value(QStringLiteral("x_max")).toDouble(value.xMax);
    if (obj.contains(QStringLiteral("y_min"))) value.yMin = obj.value(QStringLiteral("y_min")).toDouble(value.yMin);
    if (obj.contains(QStringLiteral("y_max"))) value.yMax = obj.value(QStringLiteral("y_max")).toDouble(value.yMax);
    if (obj.contains(QStringLiteral("z_fixed"))) value.zFixed = obj.value(QStringLiteral("z_fixed")).toDouble(value.zFixed);
    if (obj.contains(QStringLiteral("resolution_m"))) value.resolutionM = obj.value(QStringLiteral("resolution_m")).toDouble(value.resolutionM);
    if (obj.contains(QStringLiteral("output_dir"))) value.outputDir = obj.value(QStringLiteral("output_dir")).toString(value.outputDir);
    if (obj.contains(QStringLiteral("auto_load_result"))) value.autoLoadResult = obj.value(QStringLiteral("auto_load_result")).toBool(value.autoLoadResult);
    if (obj.contains(QStringLiteral("render_scene_overlay"))) value.renderSceneOverlay = obj.value(QStringLiteral("render_scene_overlay")).toBool(value.renderSceneOverlay);
    if (obj.contains(QStringLiteral("render_scene_path"))) value.renderScenePath = obj.value(QStringLiteral("render_scene_path")).toString(value.renderScenePath);
    value.metric = normalizeDenseCkmMetrics(QStringList{value.metric}, QStringLiteral("power_db")).value(0);
    value.selectedMetrics = normalizeDenseCkmMetrics(value.selectedMetrics, value.metric);
    if (!value.selectedMetrics.contains(value.metric)) {
        value.metric = value.selectedMetrics.value(0, QStringLiteral("power_db"));
    }
    return value;
}

QJsonArray vec3ToJson(const Vec3& value) {
    return QJsonArray{value.x, value.y, value.z};
}

Vec3 vec3FromJson(const QJsonValue& value, const Vec3& fallback = {}) {
    const QJsonArray arr = value.toArray();
    if (arr.size() < 3) {
        return fallback;
    }
    return {arr.at(0).toDouble(fallback.x),
            arr.at(1).toDouble(fallback.y),
            arr.at(2).toDouble(fallback.z)};
}

BaseStation baseStationFromJson(const QJsonObject& obj) {
    BaseStation station;
    station.id = obj.value(QStringLiteral("id")).toString().trimmed();
    station.name = obj.value(QStringLiteral("name")).toString().trimmed();
    station.position = vec3FromJson(obj.value(QStringLiteral("position")));
    station.color = vec3FromJson(obj.value(QStringLiteral("color")), {0.95, 0.55, 0.20});
    station.previewCameraName = obj.value(QStringLiteral("preview_camera_name")).toString().trimmed();
    station.previewRosTopic = obj.value(QStringLiteral("preview_ros_topic")).toString().trimmed();
    station.previewOffsetZ = obj.value(QStringLiteral("preview_offset_z")).toDouble(0.0);
    station.previewFps = obj.value(QStringLiteral("preview_fps")).toDouble(kDefaultCameraFps);
    if (obj.value(QStringLiteral("preview_camera_target")).isArray()) {
        const QJsonArray targetArr = obj.value(QStringLiteral("preview_camera_target")).toArray();
        if (targetArr.size() >= 3) {
            station.previewCameraTargetEnabled = true;
            station.previewCameraTarget = vec3FromJson(obj.value(QStringLiteral("preview_camera_target")));
        }
    }
    if (station.id.isEmpty()) {
        station.id = QStringLiteral("bs");
    }
    if (station.name.isEmpty()) {
        station.name = station.id;
    }
    if (station.previewCameraName.isEmpty()) {
        station.previewCameraName = defaultPreviewCameraNameForStation(station);
    } else {
        station.previewCameraName = sanitizedCameraToken(station.previewCameraName);
    }
    if (station.previewRosTopic.isEmpty()) {
        station.previewRosTopic = defaultPreviewRosTopicForStation(station);
    }
    if (station.previewFps <= 0.0) {
        station.previewFps = kDefaultCameraFps;
    }
    return station;
}

QJsonObject simulationSettingsToJson(const SimulationSettings& value) {
    QJsonObject obj;
    obj[QStringLiteral("scene_path")] = value.scenePath;
    obj[QStringLiteral("fc_hz")] = value.fcHz;
    obj[QStringLiteral("mi_variant")] = value.miVariant;

    obj[QStringLiteral("tx_array_num_rows")] = value.txArrayNumRows;
    obj[QStringLiteral("tx_array_num_cols")] = value.txArrayNumCols;
    obj[QStringLiteral("tx_array_vertical_spacing")] = value.txArrayVerticalSpacing;
    obj[QStringLiteral("tx_array_horizontal_spacing")] = value.txArrayHorizontalSpacing;
    obj[QStringLiteral("tx_array_pattern")] = value.txArrayPattern;
    obj[QStringLiteral("tx_array_polarization")] = value.txArrayPolarization;

    obj[QStringLiteral("rx_array_num_rows")] = value.rxArrayNumRows;
    obj[QStringLiteral("rx_array_num_cols")] = value.rxArrayNumCols;
    obj[QStringLiteral("rx_array_vertical_spacing")] = value.rxArrayVerticalSpacing;
    obj[QStringLiteral("rx_array_horizontal_spacing")] = value.rxArrayHorizontalSpacing;
    obj[QStringLiteral("rx_array_pattern")] = value.rxArrayPattern;
    obj[QStringLiteral("rx_array_polarization")] = value.rxArrayPolarization;

    obj[QStringLiteral("max_depth")] = value.maxDepth;
    obj[QStringLiteral("samples_per_src")] = value.samplesPerSrc;
    obj[QStringLiteral("max_num_paths_per_src")] = value.maxNumPathsPerSrc;
    obj[QStringLiteral("synthetic_array")] = value.syntheticArray;
    obj[QStringLiteral("merge_shapes")] = value.mergeShapes;

    obj[QStringLiteral("enable_sys_integration")] = value.enableSysIntegration;
    obj[QStringLiteral("sys_num_subcarriers")] = value.sysNumSubcarriers;
    obj[QStringLiteral("sys_subcarrier_spacing_hz")] = value.sysSubcarrierSpacingHz;
    obj[QStringLiteral("sys_num_ofdm_symbols")] = value.sysNumOfdmSymbols;
    obj[QStringLiteral("sys_temperature_k")] = value.sysTemperatureK;
    obj[QStringLiteral("sys_bler_target")] = value.sysBlerTarget;
    obj[QStringLiteral("sys_mcs_table_index")] = value.sysMcsTableIndex;
    obj[QStringLiteral("sys_bs_tx_power_dbm")] = value.sysBsTxPowerDbm;

    obj[QStringLiteral("enable_beamforming")] = value.enableBeamforming;
    obj[QStringLiteral("beam_selection_mode")] = value.beamSelectionMode;
    obj[QStringLiteral("beam_codebook_type")] = value.beamCodebookType;
    obj[QStringLiteral("beam_codebook_num_beams")] = value.beamCodebookNumBeams;
    obj[QStringLiteral("beam_oversampling_v")] = value.beamOversamplingV;
    obj[QStringLiteral("beam_oversampling_h")] = value.beamOversamplingH;
    obj[QStringLiteral("beam_manual_index")] = value.beamManualIndex;
    obj[QStringLiteral("beam_normalize_power")] = value.beamNormalizePower;
    obj[QStringLiteral("beam_codebook_file_path")] = value.beamCodebookFilePath;
    obj[QStringLiteral("beam_model_checkpoint_path")] = value.beamModelCheckpointPath;
    obj[QStringLiteral("beam_feature_mode")] = value.beamFeatureMode;
    obj[QStringLiteral("beam_top_k")] = value.beamTopK;
    obj[QStringLiteral("beam_export_training_dataset")] = value.beamExportTrainingDataset;
    obj[QStringLiteral("beam_training_dataset_path")] = value.beamTrainingDatasetPath;
    obj[QStringLiteral("beam_training_input_paths")] = value.beamTrainingInputPaths;
    obj[QStringLiteral("beam_training_reset_dataset_on_start")] = value.beamTrainingResetDatasetOnStart;
    obj[QStringLiteral("beam_training_output_checkpoint_path")] = value.beamTrainingOutputCheckpointPath;
    obj[QStringLiteral("beam_training_epochs")] = value.beamTrainingEpochs;
    obj[QStringLiteral("beam_training_batch_size")] = value.beamTrainingBatchSize;
    obj[QStringLiteral("beam_training_learning_rate")] = value.beamTrainingLearningRate;
    obj[QStringLiteral("beam_training_validation_split")] = value.beamTrainingValidationSplit;
    obj[QStringLiteral("beam_training_hidden_dim")] = value.beamTrainingHiddenDim;

    obj[QStringLiteral("los")] = value.los;
    obj[QStringLiteral("specular_reflection")] = value.specularReflection;
    obj[QStringLiteral("diffuse_reflection")] = value.diffuseReflection;
    obj[QStringLiteral("refraction")] = value.refraction;
    obj[QStringLiteral("diffraction")] = value.diffraction;
    obj[QStringLiteral("edge_diffraction")] = value.edgeDiffraction;
    obj[QStringLiteral("diffraction_lit_region")] = value.diffractionLitRegion;

    obj[QStringLiteral("sim_hz")] = value.simHz;
    obj[QStringLiteral("max_pose_staleness_s")] = value.maxPoseStalenessS;
    obj[QStringLiteral("rf_frame_id")] = value.rfFrameId;
    obj[QStringLiteral("csv_enabled")] = value.csvEnabled;
    obj[QStringLiteral("csv_path")] = value.csvPath;
    obj[QStringLiteral("python_executable")] = value.pythonExecutable;
    return obj;
}

SimulationSettings simulationSettingsFromJson(const QJsonObject& obj, const SimulationSettings& fallback) {
    SimulationSettings value = fallback;
    if (obj.contains(QStringLiteral("scene_path"))) value.scenePath = obj.value(QStringLiteral("scene_path")).toString();
    if (obj.contains(QStringLiteral("fc_hz"))) value.fcHz = obj.value(QStringLiteral("fc_hz")).toDouble(value.fcHz);
    if (obj.contains(QStringLiteral("mi_variant"))) value.miVariant = obj.value(QStringLiteral("mi_variant")).toString(value.miVariant);

    if (obj.contains(QStringLiteral("tx_array_num_rows"))) value.txArrayNumRows = obj.value(QStringLiteral("tx_array_num_rows")).toInt(value.txArrayNumRows);
    if (obj.contains(QStringLiteral("tx_array_num_cols"))) value.txArrayNumCols = obj.value(QStringLiteral("tx_array_num_cols")).toInt(value.txArrayNumCols);
    if (obj.contains(QStringLiteral("tx_array_vertical_spacing"))) value.txArrayVerticalSpacing = obj.value(QStringLiteral("tx_array_vertical_spacing")).toDouble(value.txArrayVerticalSpacing);
    if (obj.contains(QStringLiteral("tx_array_horizontal_spacing"))) value.txArrayHorizontalSpacing = obj.value(QStringLiteral("tx_array_horizontal_spacing")).toDouble(value.txArrayHorizontalSpacing);
    if (obj.contains(QStringLiteral("tx_array_pattern"))) value.txArrayPattern = obj.value(QStringLiteral("tx_array_pattern")).toString(value.txArrayPattern);
    if (obj.contains(QStringLiteral("tx_array_polarization"))) value.txArrayPolarization = obj.value(QStringLiteral("tx_array_polarization")).toString(value.txArrayPolarization);

    if (obj.contains(QStringLiteral("rx_array_num_rows"))) value.rxArrayNumRows = obj.value(QStringLiteral("rx_array_num_rows")).toInt(value.rxArrayNumRows);
    if (obj.contains(QStringLiteral("rx_array_num_cols"))) value.rxArrayNumCols = obj.value(QStringLiteral("rx_array_num_cols")).toInt(value.rxArrayNumCols);
    if (obj.contains(QStringLiteral("rx_array_vertical_spacing"))) value.rxArrayVerticalSpacing = obj.value(QStringLiteral("rx_array_vertical_spacing")).toDouble(value.rxArrayVerticalSpacing);
    if (obj.contains(QStringLiteral("rx_array_horizontal_spacing"))) value.rxArrayHorizontalSpacing = obj.value(QStringLiteral("rx_array_horizontal_spacing")).toDouble(value.rxArrayHorizontalSpacing);
    if (obj.contains(QStringLiteral("rx_array_pattern"))) value.rxArrayPattern = obj.value(QStringLiteral("rx_array_pattern")).toString(value.rxArrayPattern);
    if (obj.contains(QStringLiteral("rx_array_polarization"))) value.rxArrayPolarization = obj.value(QStringLiteral("rx_array_polarization")).toString(value.rxArrayPolarization);

    if (obj.contains(QStringLiteral("max_depth"))) value.maxDepth = obj.value(QStringLiteral("max_depth")).toInt(value.maxDepth);
    if (obj.contains(QStringLiteral("samples_per_src"))) value.samplesPerSrc = obj.value(QStringLiteral("samples_per_src")).toInt(value.samplesPerSrc);
    if (obj.contains(QStringLiteral("max_num_paths_per_src"))) value.maxNumPathsPerSrc = obj.value(QStringLiteral("max_num_paths_per_src")).toInt(value.maxNumPathsPerSrc);
    if (obj.contains(QStringLiteral("synthetic_array"))) value.syntheticArray = obj.value(QStringLiteral("synthetic_array")).toBool(value.syntheticArray);
    if (obj.contains(QStringLiteral("merge_shapes"))) value.mergeShapes = obj.value(QStringLiteral("merge_shapes")).toBool(value.mergeShapes);

    if (obj.contains(QStringLiteral("enable_sys_integration"))) value.enableSysIntegration = obj.value(QStringLiteral("enable_sys_integration")).toBool(value.enableSysIntegration);
    if (obj.contains(QStringLiteral("sys_num_subcarriers"))) value.sysNumSubcarriers = obj.value(QStringLiteral("sys_num_subcarriers")).toInt(value.sysNumSubcarriers);
    if (obj.contains(QStringLiteral("sys_subcarrier_spacing_hz"))) value.sysSubcarrierSpacingHz = obj.value(QStringLiteral("sys_subcarrier_spacing_hz")).toDouble(value.sysSubcarrierSpacingHz);
    if (obj.contains(QStringLiteral("sys_num_ofdm_symbols"))) value.sysNumOfdmSymbols = obj.value(QStringLiteral("sys_num_ofdm_symbols")).toInt(value.sysNumOfdmSymbols);
    if (obj.contains(QStringLiteral("sys_temperature_k"))) value.sysTemperatureK = obj.value(QStringLiteral("sys_temperature_k")).toDouble(value.sysTemperatureK);
    if (obj.contains(QStringLiteral("sys_bler_target"))) value.sysBlerTarget = obj.value(QStringLiteral("sys_bler_target")).toDouble(value.sysBlerTarget);
    if (obj.contains(QStringLiteral("sys_mcs_table_index"))) value.sysMcsTableIndex = obj.value(QStringLiteral("sys_mcs_table_index")).toInt(value.sysMcsTableIndex);
    if (obj.contains(QStringLiteral("sys_bs_tx_power_dbm"))) value.sysBsTxPowerDbm = obj.value(QStringLiteral("sys_bs_tx_power_dbm")).toDouble(value.sysBsTxPowerDbm);

    if (obj.contains(QStringLiteral("enable_beamforming"))) value.enableBeamforming = obj.value(QStringLiteral("enable_beamforming")).toBool(value.enableBeamforming);
    if (obj.contains(QStringLiteral("beam_selection_mode"))) value.beamSelectionMode = obj.value(QStringLiteral("beam_selection_mode")).toString(value.beamSelectionMode);
    if (obj.contains(QStringLiteral("beam_codebook_type"))) value.beamCodebookType = obj.value(QStringLiteral("beam_codebook_type")).toString(value.beamCodebookType);
    if (obj.contains(QStringLiteral("beam_codebook_num_beams"))) value.beamCodebookNumBeams = obj.value(QStringLiteral("beam_codebook_num_beams")).toInt(value.beamCodebookNumBeams);
    if (obj.contains(QStringLiteral("beam_oversampling_v"))) value.beamOversamplingV = obj.value(QStringLiteral("beam_oversampling_v")).toInt(value.beamOversamplingV);
    if (obj.contains(QStringLiteral("beam_oversampling_h"))) value.beamOversamplingH = obj.value(QStringLiteral("beam_oversampling_h")).toInt(value.beamOversamplingH);
    if (obj.contains(QStringLiteral("beam_manual_index"))) value.beamManualIndex = obj.value(QStringLiteral("beam_manual_index")).toInt(value.beamManualIndex);
    if (obj.contains(QStringLiteral("beam_normalize_power"))) value.beamNormalizePower = obj.value(QStringLiteral("beam_normalize_power")).toBool(value.beamNormalizePower);
    if (obj.contains(QStringLiteral("beam_codebook_file_path"))) value.beamCodebookFilePath = obj.value(QStringLiteral("beam_codebook_file_path")).toString(value.beamCodebookFilePath);
    if (obj.contains(QStringLiteral("beam_model_checkpoint_path"))) value.beamModelCheckpointPath = obj.value(QStringLiteral("beam_model_checkpoint_path")).toString(value.beamModelCheckpointPath);
    if (obj.contains(QStringLiteral("beam_feature_mode"))) value.beamFeatureMode = obj.value(QStringLiteral("beam_feature_mode")).toString(value.beamFeatureMode);
    if (obj.contains(QStringLiteral("beam_top_k"))) value.beamTopK = obj.value(QStringLiteral("beam_top_k")).toInt(value.beamTopK);
    if (obj.contains(QStringLiteral("beam_export_training_dataset"))) value.beamExportTrainingDataset = obj.value(QStringLiteral("beam_export_training_dataset")).toBool(value.beamExportTrainingDataset);
    if (obj.contains(QStringLiteral("beam_training_dataset_path"))) value.beamTrainingDatasetPath = obj.value(QStringLiteral("beam_training_dataset_path")).toString(value.beamTrainingDatasetPath);
    if (obj.contains(QStringLiteral("beam_training_input_paths"))) value.beamTrainingInputPaths = obj.value(QStringLiteral("beam_training_input_paths")).toString(value.beamTrainingInputPaths);
    if (obj.contains(QStringLiteral("beam_training_reset_dataset_on_start"))) value.beamTrainingResetDatasetOnStart = obj.value(QStringLiteral("beam_training_reset_dataset_on_start")).toBool(value.beamTrainingResetDatasetOnStart);
    if (obj.contains(QStringLiteral("beam_training_output_checkpoint_path"))) value.beamTrainingOutputCheckpointPath = obj.value(QStringLiteral("beam_training_output_checkpoint_path")).toString(value.beamTrainingOutputCheckpointPath);
    if (obj.contains(QStringLiteral("beam_training_epochs"))) value.beamTrainingEpochs = obj.value(QStringLiteral("beam_training_epochs")).toInt(value.beamTrainingEpochs);
    if (obj.contains(QStringLiteral("beam_training_batch_size"))) value.beamTrainingBatchSize = obj.value(QStringLiteral("beam_training_batch_size")).toInt(value.beamTrainingBatchSize);
    if (obj.contains(QStringLiteral("beam_training_learning_rate"))) value.beamTrainingLearningRate = obj.value(QStringLiteral("beam_training_learning_rate")).toDouble(value.beamTrainingLearningRate);
    if (obj.contains(QStringLiteral("beam_training_validation_split"))) value.beamTrainingValidationSplit = obj.value(QStringLiteral("beam_training_validation_split")).toDouble(value.beamTrainingValidationSplit);
    if (obj.contains(QStringLiteral("beam_training_hidden_dim"))) value.beamTrainingHiddenDim = obj.value(QStringLiteral("beam_training_hidden_dim")).toInt(value.beamTrainingHiddenDim);

    if (obj.contains(QStringLiteral("los"))) value.los = obj.value(QStringLiteral("los")).toBool(value.los);
    if (obj.contains(QStringLiteral("specular_reflection"))) value.specularReflection = obj.value(QStringLiteral("specular_reflection")).toBool(value.specularReflection);
    if (obj.contains(QStringLiteral("diffuse_reflection"))) value.diffuseReflection = obj.value(QStringLiteral("diffuse_reflection")).toBool(value.diffuseReflection);
    if (obj.contains(QStringLiteral("refraction"))) value.refraction = obj.value(QStringLiteral("refraction")).toBool(value.refraction);
    if (obj.contains(QStringLiteral("diffraction"))) value.diffraction = obj.value(QStringLiteral("diffraction")).toBool(value.diffraction);
    if (obj.contains(QStringLiteral("edge_diffraction"))) value.edgeDiffraction = obj.value(QStringLiteral("edge_diffraction")).toBool(value.edgeDiffraction);
    if (obj.contains(QStringLiteral("diffraction_lit_region"))) value.diffractionLitRegion = obj.value(QStringLiteral("diffraction_lit_region")).toBool(value.diffractionLitRegion);

    if (obj.contains(QStringLiteral("sim_hz"))) value.simHz = obj.value(QStringLiteral("sim_hz")).toDouble(value.simHz);
    if (obj.contains(QStringLiteral("max_pose_staleness_s"))) value.maxPoseStalenessS = obj.value(QStringLiteral("max_pose_staleness_s")).toDouble(value.maxPoseStalenessS);
    if (obj.contains(QStringLiteral("rf_frame_id"))) value.rfFrameId = obj.value(QStringLiteral("rf_frame_id")).toString(value.rfFrameId);
    if (obj.contains(QStringLiteral("csv_enabled"))) value.csvEnabled = obj.value(QStringLiteral("csv_enabled")).toBool(value.csvEnabled);
    if (obj.contains(QStringLiteral("csv_path"))) value.csvPath = obj.value(QStringLiteral("csv_path")).toString(value.csvPath);
    if (obj.contains(QStringLiteral("python_executable"))) value.pythonExecutable = obj.value(QStringLiteral("python_executable")).toString(value.pythonExecutable);
    if (value.miVariant.trimmed().isEmpty()) value.miVariant = fallback.miVariant;
    if (value.txArrayPattern.trimmed().isEmpty()) value.txArrayPattern = fallback.txArrayPattern;
    if (value.txArrayPolarization.trimmed().isEmpty()) value.txArrayPolarization = fallback.txArrayPolarization;
    if (value.rxArrayPattern.trimmed().isEmpty()) value.rxArrayPattern = fallback.rxArrayPattern;
    if (value.rxArrayPolarization.trimmed().isEmpty()) value.rxArrayPolarization = fallback.rxArrayPolarization;
    if (value.rfFrameId.trimmed().isEmpty()) value.rfFrameId = fallback.rfFrameId;
    if (value.pythonExecutable.trimmed().isEmpty()) value.pythonExecutable = fallback.pythonExecutable;
    if (value.beamSelectionMode.trimmed().isEmpty()) value.beamSelectionMode = fallback.beamSelectionMode;
    if (value.beamCodebookType.trimmed().isEmpty()) value.beamCodebookType = fallback.beamCodebookType;
    if (value.beamCodebookNumBeams != 8 && value.beamCodebookNumBeams != 64 && value.beamCodebookNumBeams != 128) value.beamCodebookNumBeams = fallback.beamCodebookNumBeams;
    if (value.beamFeatureMode.trimmed().isEmpty()) value.beamFeatureMode = fallback.beamFeatureMode;
    if (value.beamTrainingDatasetPath.trimmed().isEmpty()) value.beamTrainingDatasetPath = fallback.beamTrainingDatasetPath;
    value.beamTrainingInputPaths = joinBeamTrainingDatasetPaths(splitBeamTrainingDatasetPaths(value.beamTrainingInputPaths));
    if (value.beamTrainingOutputCheckpointPath.trimmed().isEmpty()) value.beamTrainingOutputCheckpointPath = fallback.beamTrainingOutputCheckpointPath;
    if (value.beamOversamplingV < 1) value.beamOversamplingV = 1;
    if (value.beamOversamplingH < 1) value.beamOversamplingH = 1;
    return value;
}

QJsonObject coordinateTransformSettingsToJson(const CoordinateTransformSettings& value) {
    QJsonObject obj;
    obj[QStringLiteral("ros_to_scene_matrix_path")] = value.rosToSceneMatrixPath;
    obj[QStringLiteral("airsim_to_scene_matrix_path")] = value.airsimToSceneMatrixPath;
    obj[QStringLiteral("rx_array_facing_direction")] = normalizeRxArrayFacingDirectionKey(value.rxArrayFacingDirection);
    obj[QStringLiteral("uav_to_rx_array_tx")] = value.uavToRxArrayTx;
    obj[QStringLiteral("uav_to_rx_array_ty")] = value.uavToRxArrayTy;
    obj[QStringLiteral("uav_to_rx_array_tz")] = value.uavToRxArrayTz;
    obj[QStringLiteral("uav_to_rx_array_center_matrix_path")] = value.uavToRxArrayCenterMatrixPath;
    obj[QStringLiteral("rx_array_elements_json_path")] = value.rxArrayElementsJsonPath;
    obj[QStringLiteral("output_frame_key")] = coordinateOutputFrameToKey(coordinateOutputFrameFromKey(value.outputFrameKey));
    return obj;
}

CoordinateTransformSettings coordinateTransformSettingsFromJson(const QJsonObject& obj, const CoordinateTransformSettings& fallback) {
    CoordinateTransformSettings value = fallback;
    if (obj.contains(QStringLiteral("ros_to_scene_matrix_path"))) value.rosToSceneMatrixPath = obj.value(QStringLiteral("ros_to_scene_matrix_path")).toString();
    if (obj.contains(QStringLiteral("airsim_to_scene_matrix_path"))) value.airsimToSceneMatrixPath = obj.value(QStringLiteral("airsim_to_scene_matrix_path")).toString();
    if (obj.contains(QStringLiteral("rx_array_facing_direction"))) value.rxArrayFacingDirection = obj.value(QStringLiteral("rx_array_facing_direction")).toString(value.rxArrayFacingDirection);
    if (obj.contains(QStringLiteral("uav_to_rx_array_tx"))) value.uavToRxArrayTx = obj.value(QStringLiteral("uav_to_rx_array_tx")).toDouble(value.uavToRxArrayTx);
    if (obj.contains(QStringLiteral("uav_to_rx_array_ty"))) value.uavToRxArrayTy = obj.value(QStringLiteral("uav_to_rx_array_ty")).toDouble(value.uavToRxArrayTy);
    if (obj.contains(QStringLiteral("uav_to_rx_array_tz"))) value.uavToRxArrayTz = obj.value(QStringLiteral("uav_to_rx_array_tz")).toDouble(value.uavToRxArrayTz);
    if (obj.contains(QStringLiteral("uav_to_rx_array_center_matrix_path"))) value.uavToRxArrayCenterMatrixPath = obj.value(QStringLiteral("uav_to_rx_array_center_matrix_path")).toString();
    if (obj.contains(QStringLiteral("rx_array_elements_json_path"))) value.rxArrayElementsJsonPath = obj.value(QStringLiteral("rx_array_elements_json_path")).toString();
    if (obj.contains(QStringLiteral("output_frame_key"))) value.outputFrameKey = obj.value(QStringLiteral("output_frame_key")).toString(value.outputFrameKey);
    if (value.rosToSceneMatrixPath.trimmed().isEmpty()) value.rosToSceneMatrixPath = fallback.rosToSceneMatrixPath;
    if (value.airsimToSceneMatrixPath.trimmed().isEmpty()) value.airsimToSceneMatrixPath = fallback.airsimToSceneMatrixPath;
    if (!obj.contains(QStringLiteral("uav_to_rx_array_center_matrix_path")) && value.uavToRxArrayCenterMatrixPath.trimmed().isEmpty()) value.uavToRxArrayCenterMatrixPath = fallback.uavToRxArrayCenterMatrixPath;
    if (!obj.contains(QStringLiteral("rx_array_elements_json_path")) && value.rxArrayElementsJsonPath.trimmed().isEmpty()) value.rxArrayElementsJsonPath = fallback.rxArrayElementsJsonPath;
    value.rxArrayFacingDirection = normalizeRxArrayFacingDirectionKey(value.rxArrayFacingDirection);
    value.outputFrameKey = coordinateOutputFrameToKey(coordinateOutputFrameFromKey(value.outputFrameKey));
    return value;
}

QString processStateText(QProcess::ProcessState state) {
    switch (state) {
    case QProcess::NotRunning:
        return QStringLiteral("NotRunning");
    case QProcess::Starting:
        return QStringLiteral("Starting");
    case QProcess::Running:
        return QStringLiteral("Running");
    }
    return QStringLiteral("Unknown");
}

QString runAndCapture(const QString& program, const QStringList& args, int timeoutMs = 2000) {
    QProcess proc;
    proc.start(program, args);
    if (!proc.waitForStarted(timeoutMs)) {
        return QString();
    }
    proc.waitForFinished(timeoutMs);
    return QString::fromLocal8Bit(proc.readAllStandardOutput()).trimmed();
}

bool isExecutablePython(const QString& path) {
    const QFileInfo fi(path);
    return fi.exists() && fi.isFile() && fi.isExecutable();
}

QString detectPythonVersion(const QString& path) {
    if (!isExecutablePython(path)) {
        return QString();
    }
    QString out = runAndCapture(path, {"-c", "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}')"}, 2500);
    return out.trimmed();
}

QString detectPythonKind(const QString& path) {
    const QString lower = QFileInfo(path).absoluteFilePath().toLower();
    if (lower.contains("/envs/") || lower.contains("miniconda") || lower.contains("anaconda") || lower.contains("conda")) {
        return "Conda";
    }
    if (lower.contains("venv") || lower.contains(".venv") || lower.contains("virtualenv")) {
        return "VirtualEnv";
    }
    return "Global";
}

QString envDisplayName(const QString& path, const QString& kind) {
    const QString normalized = QFileInfo(path).absoluteFilePath();
    if (kind == "Conda") {
        const int envsIndex = normalized.indexOf("/envs/");
        if (envsIndex >= 0) {
            const QString tail = normalized.mid(envsIndex + 6);
            const int slash = tail.indexOf('/');
            if (slash > 0) return tail.left(slash);
        }
        const QString baseName = QFileInfo(QFileInfo(normalized).dir().dirName()).fileName();
        if (!baseName.isEmpty()) return baseName;
    }
    if (kind == "VirtualEnv") {
        const QDir binDir = QFileInfo(normalized).dir();
        const QString envName = QFileInfo(binDir.absolutePath() + "/..").fileName();
        if (!envName.isEmpty()) return envName;
    }
    return QFileInfo(normalized).fileName();
}

PythonEnvOption makePythonEnvOption(const QString& path) {
    PythonEnvOption option;
    option.path = QFileInfo(path).absoluteFilePath();
    option.kind = detectPythonKind(option.path);
    option.version = detectPythonVersion(option.path);
    option.name = envDisplayName(option.path, option.kind);
    return option;
}

QList<PythonEnvOption> discoverPythonEnvironments() {
    std::set<QString> unique;
    QList<PythonEnvOption> results;

    auto addCandidate = [&](const QString& candidate) {
        const QString normalized = QFileInfo(candidate).absoluteFilePath();
        if (!isExecutablePython(normalized) || unique.count(normalized)) {
            return;
        }
        unique.insert(normalized);
        results.push_back(makePythonEnvOption(normalized));
    };

    const QStringList directCandidates = {
        "/usr/bin/python3", "/usr/local/bin/python3", "/usr/bin/python", "/usr/local/bin/python",
        QDir::home().filePath("miniconda3/bin/python"),
        QDir::home().filePath("anaconda3/bin/python"),
        QDir::home().filePath("mambaforge/bin/python"),
        QDir::home().filePath("micromamba/bin/python")
    };
    for (const QString& path : directCandidates) addCandidate(path);

    for (const QString& root : {QDir::home().filePath("miniconda3/envs"), QDir::home().filePath("anaconda3/envs"), QDir::home().filePath("mambaforge/envs")}) {
        QDir dir(root);
        if (!dir.exists()) continue;
        for (const QFileInfo& fi : dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot)) {
            addCandidate(fi.absoluteFilePath() + "/bin/python");
        }
    }

    for (const QString& root : {QDir::homePath(), QDir::home().filePath(".virtualenvs")}) {
        QDir dir(root);
        if (!dir.exists()) continue;
        for (const QFileInfo& fi : dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot)) {
            const QString name = fi.fileName();
            if (name == ".venv" || name == "venv" || name.endsWith("_venv") || name.contains("venv")) {
                addCandidate(fi.absoluteFilePath() + "/bin/python");
            }
        }
    }

    const QString pathProbe = runAndCapture("bash", {"-lc", "which -a python3 python 2>/dev/null | awk '!seen[$0]++'"}, 2000);
    for (const QString& line : pathProbe.split(QStringLiteral("\n"), QString::SkipEmptyParts)) addCandidate(line.trimmed());

    std::sort(results.begin(), results.end(), [](const PythonEnvOption& a, const PythonEnvOption& b) {
        const auto rank = [](const QString& kind) {
            if (kind == "Conda") return 0;
            if (kind == "VirtualEnv") return 1;
            return 2;
        };
        if (rank(a.kind) != rank(b.kind)) return rank(a.kind) < rank(b.kind);
        if (a.name != b.name) return a.name.toLower() < b.name.toLower();
        return a.path.toLower() < b.path.toLower();
    });
    return results;
}

QString pythonSummary(const PythonEnvOption& option) {
    const QString version = option.version.isEmpty() ? QStringLiteral("Python ?") : QStringLiteral("Python %1").arg(option.version);
    return QString("%1  %2").arg(version, option.path);
}

QString testPythonEnvironment(const QString& pythonPath) {
    if (!isExecutablePython(pythonPath)) {
        return QStringLiteral("Python executable not found or not executable.");
    }
    QProcess proc;
    proc.start(pythonPath, {"-c", "import sys; print(sys.executable); import sionna, mitsuba, drjit, polyscope, omegaconf; print('OK')"});
    if (!proc.waitForStarted(2500)) {
        return QStringLiteral("Failed to start the selected Python interpreter.");
    }
    proc.waitForFinished(5000);
    const QString out = QString::fromLocal8Bit(proc.readAllStandardOutput()).trimmed();
    const QString err = QString::fromLocal8Bit(proc.readAllStandardError()).trimmed();
    if (proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0) {
        return err.isEmpty() ? out : (out + QStringLiteral("\n") + err).trimmed();
    }
    return out.isEmpty() ? QStringLiteral("OK") : out;
}

bool filterPythonTree(QTreeWidget* tree, const QString& keyword) {
    const QString needle = keyword.trimmed().toLower();
    bool anyVisible = false;
    for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        auto* group = tree->topLevelItem(i);
        bool groupVisible = false;
        for (int j = 0; j < group->childCount(); ++j) {
            auto* child = group->child(j);
            const QString hay = (child->text(0) + " " + child->text(1) + " " + child->text(2)).toLower();
            const bool visible = needle.isEmpty() || hay.contains(needle);
            child->setHidden(!visible);
            groupVisible = groupVisible || visible;
        }
        group->setHidden(!groupVisible);
        if (groupVisible) group->setExpanded(true);
        anyVisible = anyVisible || groupVisible;
    }
    return anyVisible;
}

QString selectPythonEnvironment(QWidget* parent, const QString& currentPath) {
    QDialog dialog(parent);
    dialog.setWindowTitle(QObject::tr("Select Python Environment"));
    dialog.resize(860, 520);

    auto* layout = new QVBoxLayout(&dialog);
    auto* search = new QLineEdit(&dialog);
    search->setPlaceholderText(QObject::tr("Search environments by name, version, or path"));
    layout->addWidget(search);

    auto* tree = new QTreeWidget(&dialog);
    tree->setColumnCount(3);
    tree->setHeaderLabels({QObject::tr("Environment"), QObject::tr("Interpreter"), QObject::tr("Type")});
    tree->header()->setStretchLastSection(false);
    tree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    tree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    tree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    tree->setRootIsDecorated(true);
    layout->addWidget(tree, 1);

    const QList<PythonEnvOption> envs = discoverPythonEnvironments();
    QMap<QString, QTreeWidgetItem*> groups;
    for (const QString& groupName : {QStringLiteral("Conda"), QStringLiteral("Virtual Environments"), QStringLiteral("Global")}) {
        auto* item = new QTreeWidgetItem(tree, QStringList{groupName});
        item->setFirstColumnSpanned(true);
        item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
        groups.insert(groupName, item);
    }

    auto kindLabel = [](const QString& kind) {
        if (kind == "Conda") return QStringLiteral("Conda");
        if (kind == "VirtualEnv") return QStringLiteral("Virtual Env");
        return QStringLiteral("Global");
    };
    auto groupKey = [](const QString& kind) {
        if (kind == "Conda") return QStringLiteral("Conda");
        if (kind == "VirtualEnv") return QStringLiteral("Virtual Environments");
        return QStringLiteral("Global");
    };

    QTreeWidgetItem* selected = nullptr;
    for (const auto& env : envs) {
        auto* parentItem = groups.value(groupKey(env.kind));
        const QString envLabel = env.version.isEmpty() ? env.name : QString("%1 (%2)").arg(env.name, env.version);
        auto* child = new QTreeWidgetItem(parentItem, QStringList{envLabel, env.path, kindLabel(env.kind)});
        child->setData(0, Qt::UserRole, env.path);
        if (QFileInfo(currentPath).absoluteFilePath() == QFileInfo(env.path).absoluteFilePath()) {
            selected = child;
        }
    }
    tree->expandAll();
    if (selected) tree->setCurrentItem(selected);

    auto* buttonsRow = new QHBoxLayout();
    auto* browse = new QPushButton(QObject::tr("Browse..."), &dialog);
    auto* createHint = new QLabel(QObject::tr("Tip: select the Python executable inside your Conda or virtual environment, for example envs/sionna_rt/bin/python."), &dialog);
    createHint->setWordWrap(true);
    buttonsRow->addWidget(browse);
    buttonsRow->addWidget(createHint, 1);
    layout->addLayout(buttonsRow);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    layout->addWidget(buttons);

    QObject::connect(search, &QLineEdit::textChanged, &dialog, [tree](const QString& text) {
        filterPythonTree(tree, text);
    });
    QObject::connect(browse, &QPushButton::clicked, &dialog, [&dialog, tree]() {
        const QString path = QFileDialog::getOpenFileName(&dialog, QObject::tr("Select Python executable"), QDir::homePath());
        if (path.isEmpty()) return;
        if (!isExecutablePython(path)) {
            QMessageBox::warning(&dialog, QObject::tr("Python"), QObject::tr("The selected file is not an executable Python interpreter."));
            return;
        }
        dialog.setProperty("selected_python_path", QFileInfo(path).absoluteFilePath());
        dialog.accept();
    });
    QObject::connect(tree, &QTreeWidget::itemDoubleClicked, &dialog, [&dialog](QTreeWidgetItem* item, int) {
        if (!item || item->childCount() > 0) return;
        dialog.setProperty("selected_python_path", item->data(0, Qt::UserRole).toString());
        dialog.accept();
    });
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&dialog, tree]() {
        auto* item = tree->currentItem();
        if (!item || item->childCount() > 0) {
            QMessageBox::information(&dialog, QObject::tr("Python"), QObject::tr("Please select one Python interpreter."));
            return;
        }
        dialog.setProperty("selected_python_path", item->data(0, Qt::UserRole).toString());
        dialog.accept();
    });
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted) return QString();
    return dialog.property("selected_python_path").toString();
}

QString quoted(const QString& s) {
    return s;
}

bool isIgnoredDeveloperRosTopic(const QString& topicName) {
    const QString topic = topicName.trimmed();
    return topic.isEmpty()
        || topic == QStringLiteral("/rosout")
        || topic == QStringLiteral("/rosout_agg")
        || topic == QStringLiteral("/statistics");
}

QStringList xmlRpcStringArrayToQStringList(const XmlRpc::XmlRpcValue& value) {
    QStringList out;
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        return out;
    }
    for (int i = 0; i < value.size(); ++i) {
        if (value[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
            out << QString::fromStdString(static_cast<std::string>(value[i]));
        }
    }
    out.removeDuplicates();
    return out;
}

QMap<QString, QString> rosPublishedTopicTypes() {
    QMap<QString, QString> topicTypes;
    if (!ros::isInitialized() || !ros::master::check()) {
        return topicTypes;
    }

    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics)) {
        return topicTypes;
    }
    for (const auto& topic : topics) {
        const QString name = QString::fromStdString(topic.name).trimmed();
        if (!isIgnoredDeveloperRosTopic(name)) {
            topicTypes.insert(name, QString::fromStdString(topic.datatype).trimmed());
        }
    }
    return topicTypes;
}

QMap<QString, QStringList> rosPublisherNodesByTopic() {
    QMap<QString, QStringList> publishersByTopic;
    if (!ros::isInitialized() || !ros::master::check()) {
        return publishersByTopic;
    }

    XmlRpc::XmlRpcValue request;
    request[0] = ros::this_node::getName();
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;
    if (!ros::master::execute("getSystemState", request, response, payload, false)) {
        return publishersByTopic;
    }
    if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 1) {
        return publishersByTopic;
    }

    const XmlRpc::XmlRpcValue& publishers = payload[0];
    if (publishers.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        return publishersByTopic;
    }
    for (int i = 0; i < publishers.size(); ++i) {
        const XmlRpc::XmlRpcValue& entry = publishers[i];
        if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 2
            || entry[0].getType() != XmlRpc::XmlRpcValue::TypeString) {
            continue;
        }
        const QString topic = QString::fromStdString(static_cast<std::string>(entry[0])).trimmed();
        if (isIgnoredDeveloperRosTopic(topic)) {
            continue;
        }
        publishersByTopic.insert(topic, xmlRpcStringArrayToQStringList(entry[1]));
    }
    return publishersByTopic;
}

QString rosNodeUri(const QString& nodeName) {
    const QString node = nodeName.trimmed();
    if (node.isEmpty() || !ros::isInitialized() || !ros::master::check()) {
        return QString();
    }

    XmlRpc::XmlRpcValue request;
    request[0] = ros::this_node::getName();
    request[1] = node.toStdString();
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;
    if (!ros::master::execute("lookupNode", request, response, payload, false)) {
        return QString();
    }
    if (payload.getType() != XmlRpc::XmlRpcValue::TypeString) {
        return QString();
    }
    return QString::fromStdString(static_cast<std::string>(payload)).trimmed();
}

bool rosTopicHasSubscribers(const QString& topicName, bool ignoreThisNode = false) {
    const QString topic = topicName.trimmed();
    if (topic.isEmpty() || !ros::isInitialized()) {
        return false;
    }
    if (!ros::master::check()) {
        return false;
    }

    XmlRpc::XmlRpcValue request;
    request[0] = ros::this_node::getName();
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;
    if (!ros::master::execute("getSystemState", request, response, payload, false)) {
        return false;
    }
    if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 2) {
        return false;
    }

    const XmlRpc::XmlRpcValue& subscribers = payload[1];
    if (subscribers.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        return false;
    }

    for (int i = 0; i < subscribers.size(); ++i) {
        const XmlRpc::XmlRpcValue& entry = subscribers[i];
        if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 2) {
            continue;
        }
        if (entry[0].getType() != XmlRpc::XmlRpcValue::TypeString) {
            continue;
        }

        const QString entryTopic = QString::fromStdString(static_cast<std::string>(entry[0]));
        if (entryTopic != topic) {
            continue;
        }

        const XmlRpc::XmlRpcValue& nodes = entry[1];
        if (nodes.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            return false;
        }
        const QString thisNode = ignoreThisNode
            ? QString::fromStdString(ros::this_node::getName())
            : QString();
        for (int nodeIndex = 0; nodeIndex < nodes.size(); ++nodeIndex) {
            if (nodes[nodeIndex].getType() != XmlRpc::XmlRpcValue::TypeString) {
                continue;
            }
            const QString nodeName = QString::fromStdString(static_cast<std::string>(nodes[nodeIndex]));
            if (ignoreThisNode && nodeName == thisNode) {
                continue;
            }
            return true;
        }
        return false;
    }
    return false;
}

QSet<QString> rosTopicsWithSubscribers(bool ignoreThisNode = false) {
    QSet<QString> topics;
    if (!ros::isInitialized() || !ros::master::check()) {
        return topics;
    }

    XmlRpc::XmlRpcValue request;
    request[0] = ros::this_node::getName();
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;
    if (!ros::master::execute("getSystemState", request, response, payload, false)) {
        return topics;
    }
    if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 2) {
        return topics;
    }

    const XmlRpc::XmlRpcValue& subscribers = payload[1];
    if (subscribers.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        return topics;
    }

    const QString thisNode = ignoreThisNode
        ? QString::fromStdString(ros::this_node::getName())
        : QString();
    for (int i = 0; i < subscribers.size(); ++i) {
        const XmlRpc::XmlRpcValue& entry = subscribers[i];
        if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 2) {
            continue;
        }
        if (entry[0].getType() != XmlRpc::XmlRpcValue::TypeString || entry[1].getType() != XmlRpc::XmlRpcValue::TypeArray) {
            continue;
        }

        bool hasSubscriber = false;
        const XmlRpc::XmlRpcValue& nodes = entry[1];
        for (int nodeIndex = 0; nodeIndex < nodes.size(); ++nodeIndex) {
            if (nodes[nodeIndex].getType() != XmlRpc::XmlRpcValue::TypeString) {
                continue;
            }
            const QString nodeName = QString::fromStdString(static_cast<std::string>(nodes[nodeIndex]));
            if (ignoreThisNode && nodeName == thisNode) {
                continue;
            }
            hasSubscriber = true;
            break;
        }
        if (hasSubscriber) {
            topics.insert(QString::fromStdString(static_cast<std::string>(entry[0])));
        }
    }
    return topics;
}


struct RosbagTopicInfo {
    QString topic;
    QString type;
};

QString unquoteRosbagYamlScalar(QString text) {
    text = text.trimmed();
    if ((text.startsWith('\'') && text.endsWith('\''))
        || (text.startsWith('"') && text.endsWith('"'))) {
        text = text.mid(1, text.size() - 2).trimmed();
    }
    return text;
}

QVector<RosbagTopicInfo> rosbagTopicInfoInBag(const QString& bagPath, QString* errorMessage = nullptr) {
    if (errorMessage) {
        errorMessage->clear();
    }
    const QString trimmed = bagPath.trimmed();
    if (trimmed.isEmpty()) {
        if (errorMessage) {
            *errorMessage = QObject::tr("Rosbag path is empty.");
        }
        return {};
    }

    QProcess proc;
    proc.setProcessChannelMode(QProcess::MergedChannels);
    proc.start(QStringLiteral("rosbag"), {QStringLiteral("info"), QStringLiteral("--yaml"), trimmed});
    if (!proc.waitForStarted(3000)) {
        if (errorMessage) {
            *errorMessage = QObject::tr("Failed to start rosbag info: %1").arg(proc.errorString());
        }
        return {};
    }
    if (!proc.waitForFinished(10000)) {
        proc.kill();
        proc.waitForFinished(1000);
        if (errorMessage) {
            *errorMessage = QObject::tr("Timed out while reading rosbag metadata.");
        }
        return {};
    }
    const QString output = QString::fromLocal8Bit(proc.readAllStandardOutput());
    if (proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0) {
        if (errorMessage) {
            *errorMessage = QObject::tr("rosbag info failed: %1").arg(output.trimmed().isEmpty() ? proc.errorString() : output.trimmed());
        }
        return {};
    }

    QVector<RosbagTopicInfo> topicInfos;
    RosbagTopicInfo current;
    auto flushCurrent = [&]() {
        if (!current.topic.trimmed().isEmpty()) {
            topicInfos.push_back(current);
        }
        current = RosbagTopicInfo{};
    };

    const QStringList lines = output.split(QRegularExpression(QStringLiteral("[\r\n]+")), QString::SkipEmptyParts);
    for (QString line : lines) {
        line = line.trimmed();
        if (line.startsWith(QStringLiteral("- "))) {
            line = line.mid(2).trimmed();
        }
        if (line.startsWith(QStringLiteral("topic:"))) {
            flushCurrent();
            current.topic = unquoteRosbagYamlScalar(line.mid(QStringLiteral("topic:").size()));
        } else if (line.startsWith(QStringLiteral("type:")) && !current.topic.trimmed().isEmpty()) {
            current.type = unquoteRosbagYamlScalar(line.mid(QStringLiteral("type:").size()));
        }
    }
    flushCurrent();

    if (topicInfos.isEmpty()) {
        static const QRegularExpression inlineTopicsRe(
            QStringLiteral("^\\s*topics:\\s*\\[(.*)\\]\\s*$"),
            QRegularExpression::MultilineOption);
        const QRegularExpressionMatch inlineMatch = inlineTopicsRe.match(output);
        if (inlineMatch.hasMatch()) {
            const QStringList rawTopics = inlineMatch.captured(1).split(',', QString::SkipEmptyParts);
            for (const QString& rawTopic : rawTopics) {
                const QString topic = unquoteRosbagYamlScalar(rawTopic);
                if (!topic.isEmpty()) {
                    topicInfos.push_back(RosbagTopicInfo{topic, QString()});
                }
            }
        }
    }

    QVector<RosbagTopicInfo> uniqueTopicInfos;
    QSet<QString> seenTopics;
    for (const RosbagTopicInfo& info : topicInfos) {
        const QString topic = info.topic.trimmed();
        if (topic.isEmpty() || seenTopics.contains(topic)) {
            continue;
        }
        seenTopics.insert(topic);
        uniqueTopicInfos.push_back(RosbagTopicInfo{topic, info.type.trimmed()});
    }
    if (uniqueTopicInfos.isEmpty() && errorMessage) {
        *errorMessage = QObject::tr("No topics found in rosbag metadata.");
    }
    return uniqueTopicInfos;
}

QStringList rosbagTopicsInBag(const QString& bagPath, QString* errorMessage = nullptr) {
    const QVector<RosbagTopicInfo> topicInfos = rosbagTopicInfoInBag(bagPath, errorMessage);
    QStringList topics;
    for (const RosbagTopicInfo& info : topicInfos) {
        topics << info.topic;
    }
    return topics;
}

QString rosbagExcludeRegex(const QStringList& topics) {
    QStringList escaped;
    for (const QString& topic : topics) {
        const QString trimmed = topic.trimmed();
        if (!trimmed.isEmpty()) {
            escaped << QRegularExpression::escape(trimmed);
        }
    }
    if (escaped.isEmpty()) {
        return QString();
    }
    return QStringLiteral("^(%1)$").arg(escaped.join(QStringLiteral("|")));
}

QString replaceRosbagTopicSuffix(const QString& topic, const QString& fromSuffix, const QString& toSuffix) {
    const QString trimmed = topic.trimmed();
    if (trimmed.isEmpty() || fromSuffix.isEmpty() || !trimmed.endsWith(fromSuffix)) {
        return trimmed;
    }
    return trimmed.left(trimmed.size() - fromSuffix.size()) + toSuffix;
}

QString defaultRosbagBeamTopicForSysTopic(const QString& sysTopic, const QString& fallbackBeamTopic) {
    const QString derived = replaceRosbagTopicSuffix(
        sysTopic, QStringLiteral("sys_observations"), QStringLiteral("beam_observations"));
    if (!derived.isEmpty() && derived != sysTopic.trimmed()) {
        return derived;
    }
    return fallbackBeamTopic.trimmed();
}

QString defaultRosbagBeamCodebookTopicForBeamTopic(const QString& beamTopic, const QString& fallbackCodebookTopic) {
    const QString derived = replaceRosbagTopicSuffix(
        beamTopic, QStringLiteral("beam_observations"), QStringLiteral("beam_codebook"));
    if (!derived.isEmpty() && derived != beamTopic.trimmed()) {
        return derived;
    }
    return fallbackCodebookTopic.trimmed();
}

QString firstRosbagTopicPresent(const QStringList& bagTopics, const QStringList& candidates) {
    QSet<QString> seen;
    for (const QString& candidate : candidates) {
        const QString trimmed = candidate.trimmed();
        if (trimmed.isEmpty() || seen.contains(trimmed)) {
            continue;
        }
        seen.insert(trimmed);
        if (bagTopics.contains(trimmed)) {
            return trimmed;
        }
    }
    return QString();
}

QString firstRosbagTopicWithSuffix(const QStringList& bagTopics, const QStringList& suffixes) {
    for (const QString& suffix : suffixes) {
        const QString trimmedSuffix = suffix.trimmed();
        if (trimmedSuffix.isEmpty()) {
            continue;
        }
        for (const QString& topic : bagTopics) {
            const QString trimmedTopic = topic.trimmed();
            if (trimmedTopic == trimmedSuffix || trimmedTopic.endsWith(QStringLiteral("/") + trimmedSuffix)) {
                return trimmedTopic;
            }
        }
    }
    return QString();
}

QString firstRosbagTopicWithType(const QVector<RosbagTopicInfo>& bagTopicInfos, const QStringList& types) {
    QSet<QString> wantedTypes;
    for (const QString& type : types) {
        const QString trimmed = type.trimmed();
        if (!trimmed.isEmpty()) {
            wantedTypes.insert(trimmed);
        }
    }
    if (wantedTypes.isEmpty()) {
        return QString();
    }
    for (const RosbagTopicInfo& info : bagTopicInfos) {
        if (wantedTypes.contains(info.type.trimmed())) {
            return info.topic.trimmed();
        }
    }
    return QString();
}

QStringList rosbagTopicNames(const QVector<RosbagTopicInfo>& bagTopicInfos) {
    QStringList topics;
    for (const RosbagTopicInfo& info : bagTopicInfos) {
        if (!info.topic.trimmed().isEmpty()) {
            topics << info.topic.trimmed();
        }
    }
    return topics;
}

struct RosbagWirelessTopicPresence {
    QString rfTopic;
    QString sysTopic;
    QString beamTopic;
    QString beamCodebookTopic;

    bool hasRf() const { return !rfTopic.trimmed().isEmpty(); }
    bool hasSys() const { return !sysTopic.trimmed().isEmpty(); }
    bool hasBeam() const { return !beamTopic.trimmed().isEmpty() || !beamCodebookTopic.trimmed().isEmpty(); }
    bool hasAny() const { return hasRf() || hasSys() || hasBeam(); }
};

RosbagWirelessTopicPresence rosbagWirelessTopicPresence(const QVector<RosbagTopicInfo>& bagTopicInfos,
                                                        const QString& rfTopic,
                                                        const QString& sysTopic,
                                                        const QString& beamTopic,
                                                        const QString& beamCodebookTopic,
                                                        const QString& internalRfTopic,
                                                        const QString& internalSysTopic,
                                                        const QString& internalBeamTopic,
                                                        const QString& internalBeamCodebookTopic) {
    const QStringList bagTopics = rosbagTopicNames(bagTopicInfos);
    RosbagWirelessTopicPresence presence;
    presence.rfTopic = firstRosbagTopicWithType(bagTopicInfos, QStringList{QStringLiteral("rf_msgs/RfObservationArray")});
    if (presence.rfTopic.isEmpty()) {
        presence.rfTopic = firstRosbagTopicPresent(bagTopics, QStringList{rfTopic, internalRfTopic});
    }
    if (presence.rfTopic.isEmpty()) {
        presence.rfTopic = firstRosbagTopicWithSuffix(bagTopics, QStringList{QStringLiteral("rf_observations")});
    }
    presence.sysTopic = firstRosbagTopicWithType(bagTopicInfos, QStringList{QStringLiteral("sionna_sys_msgs/SysObservationArray")});
    if (presence.sysTopic.isEmpty()) {
        presence.sysTopic = firstRosbagTopicPresent(bagTopics, QStringList{sysTopic, internalSysTopic});
    }
    if (presence.sysTopic.isEmpty()) {
        presence.sysTopic = firstRosbagTopicWithSuffix(bagTopics, QStringList{QStringLiteral("sys_observations")});
    }
    presence.beamTopic = firstRosbagTopicWithType(bagTopicInfos, QStringList{QStringLiteral("sionna_beam_msgs/BeamObservationArray")});
    if (presence.beamTopic.isEmpty()) {
        presence.beamTopic = firstRosbagTopicPresent(
            bagTopics,
            QStringList{beamTopic, defaultRosbagBeamTopicForSysTopic(sysTopic, internalBeamTopic), internalBeamTopic});
    }
    if (presence.beamTopic.isEmpty()) {
        presence.beamTopic = firstRosbagTopicWithSuffix(bagTopics, QStringList{QStringLiteral("beam_observations")});
    }
    presence.beamCodebookTopic = firstRosbagTopicWithType(bagTopicInfos, QStringList{QStringLiteral("sionna_beam_msgs/BeamCodebook")});
    if (presence.beamCodebookTopic.isEmpty()) {
        presence.beamCodebookTopic = firstRosbagTopicPresent(
            bagTopics,
            QStringList{beamCodebookTopic, defaultRosbagBeamCodebookTopicForBeamTopic(beamTopic, internalBeamCodebookTopic), internalBeamCodebookTopic});
    }
    if (presence.beamCodebookTopic.isEmpty()) {
        presence.beamCodebookTopic = firstRosbagTopicWithSuffix(bagTopics, QStringList{QStringLiteral("beam_codebook")});
    }
    return presence;
}

QString rosbagTopicPresenceLine(const QString& label, const QString& configuredTopic, const QString& detectedTopic) {
    const QString displayTopic = !detectedTopic.trimmed().isEmpty()
        ? detectedTopic.trimmed()
        : (configuredTopic.trimmed().isEmpty() ? QStringLiteral("<empty>") : configuredTopic.trimmed());
    return QStringLiteral("%1 %2 %3")
        .arg(label,
             displayTopic,
             detectedTopic.trimmed().isEmpty() ? QStringLiteral("×") : QStringLiteral("√"));
}

QString rosbagWirelessTopicPresenceText(const RosbagWirelessTopicPresence& presence,
                                        const QString& rfTopic,
                                        const QString& sysTopic,
                                        const QString& beamTopic,
                                        const QString& beamCodebookTopic) {
    return QStringList{
        rosbagTopicPresenceLine(QStringLiteral("RF"), rfTopic, presence.rfTopic),
        rosbagTopicPresenceLine(QStringLiteral("SYS"), sysTopic, presence.sysTopic),
        rosbagTopicPresenceLine(QStringLiteral("Beam"), beamTopic, presence.beamTopic),
        rosbagTopicPresenceLine(QStringLiteral("Codebook"), beamCodebookTopic, presence.beamCodebookTopic),
    }.join(QStringLiteral("    "));
}

}


namespace {

struct StationPathStats {
    int count{0};
    QString strongestId;
    double strongestPowerDb{-std::numeric_limits<double>::infinity()};
};

StationPathStats stationStatsFor(const airsim_gui::BaseStation& station, const airsim_gui::BeamPathList& paths) {
    StationPathStats stats;
    for (const auto& path : paths) {
        if (path.txId == station.name || path.txId == station.id) {
            ++stats.count;
            if (stats.strongestId.isEmpty() || path.powerDb > stats.strongestPowerDb) {
                stats.strongestId = path.id;
                stats.strongestPowerDb = path.powerDb;
            }
        }
    }
    return stats;
}


QString rfPathTypeLabel(int value) {
    switch (value) {
    case 1: return QStringLiteral("LOS");
    case 2: return QStringLiteral("NLOS");
    case 3: return QStringLiteral("NO_PATH");
    default: return QStringLiteral("UNKNOWN(%1)").arg(value);
    }
}

QString fmtMaybe(bool enabled, double value, int precision, const QString& suffix = QString()) {
    if (!enabled) {
        return QStringLiteral("n/a");
    }
    return QStringLiteral("%1%2").arg(value, 0, 'f', precision).arg(suffix);
}

QString fmtFinite(double value, int precision, const QString& suffix = QString()) {
    if (!std::isfinite(value)) {
        return QStringLiteral("n/a");
    }
    return QStringLiteral("%1%2").arg(value, 0, 'f', precision).arg(suffix);
}

QString fmtOptionalInt(int value, const QString& suffix = QString()) {
    if (value < 0) {
        return QStringLiteral("n/a");
    }
    return QStringLiteral("%1%2").arg(value).arg(suffix);
}

QString ackStateLabel(int tbOk) {
    if (tbOk == 1) {
        return QStringLiteral("ACK");
    }
    if (tbOk == 0) {
        return QStringLiteral("NACK");
    }
    return QStringLiteral("n/a");
}

}

QVector<int> MainWindow::parseBeamTopkIndices(const QString& text) {
    QVector<int> indices;
    QSet<int> seen;
    const QStringList tokens = text.split(QRegularExpression(QStringLiteral("[,;\\s]+")), QString::SkipEmptyParts);
    indices.reserve(tokens.size());
    for (const QString& token : tokens) {
        bool ok = false;
        const int value = token.trimmed().toInt(&ok);
        if (!ok || seen.contains(value)) {
            continue;
        }
        seen.insert(value);
        indices.push_back(value);
    }
    return indices;
}

bool MainWindow::sysCandidateMatchesIdentity(const SysCandidateView& candidate, int anchorId, const QString& bsName) {
    if (anchorId >= 0 && candidate.anchorId >= 0 && candidate.anchorId == anchorId) {
        return true;
    }
    const QString candidateName = candidate.bsName.trimmed();
    const QString targetName = bsName.trimmed();
    return !candidateName.isEmpty() && !targetName.isEmpty() && candidateName == targetName;
}

bool MainWindow::sysCandidateMatchesServing(const SysCandidateView& candidate, const SysSnapshotView& snapshot) {
    return sysCandidateMatchesIdentity(candidate, snapshot.servingAnchorId, snapshot.servingBsName);
}

bool MainWindow::sysCandidateHasUsablePath(const SysCandidateView& candidate) {
    if (candidate.numPaths <= 0) {
        return false;
    }
    const QString pathType = candidate.strongestPathType.trimmed();
    if (!pathType.isEmpty() && pathType.compare(QStringLiteral("NO_PATH"), Qt::CaseInsensitive) == 0) {
        return false;
    }
    return true;
}

bool MainWindow::sysCandidateHasBeamData(const SysCandidateView& candidate) {
    return candidate.beamEnabled
        || !candidate.beamSelectionMode.trimmed().isEmpty()
        || !candidate.beamCodebookType.trimmed().isEmpty()
        || candidate.beamNumBeams >= 0
        || candidate.beamSelectedIndex >= 0
        || std::isfinite(candidate.beamSelectedGainDb)
        || candidate.beamOracleIndex >= 0
        || std::isfinite(candidate.beamOracleGainDb)
        || !candidate.beamPredictorStatus.trimmed().isEmpty()
        || !candidate.beamFeatureMode.trimmed().isEmpty()
        || candidate.beamTopK >= 0
        || candidate.beamPredictedIndex >= 0
        || std::isfinite(candidate.beamPredictedConfidence)
        || candidate.beamOracleInTopK >= 0
        || candidate.beamHit >= 0
        || !candidate.beamSelectionSource.trimmed().isEmpty()
        || !candidate.beamTopkIndices.trimmed().isEmpty();
}

void MainWindow::normalizeSysCandidateBeamFields(SysCandidateView* candidate) {
    if (!candidate) {
        return;
    }

    if (!sysCandidateHasUsablePath(*candidate)) {
        candidate->beamSelectedIndex = -1;
        candidate->beamSelectedGainDb = std::numeric_limits<double>::quiet_NaN();
        candidate->beamOracleIndex = -1;
        candidate->beamOracleGainDb = std::numeric_limits<double>::quiet_NaN();
        candidate->beamPredictedIndex = -1;
        candidate->beamPredictedConfidence = std::numeric_limits<double>::quiet_NaN();
        candidate->beamOracleInTopK = -1;
        candidate->beamHit = -1;
        candidate->beamSelectionSource.clear();
        candidate->beamTopkIndices.clear();
        return;
    }

    const QVector<int> topkIndices = parseBeamTopkIndices(candidate->beamTopkIndices);
    if (topkIndices.isEmpty()) {
        candidate->beamTopkIndices.clear();
    } else {
        QStringList parts;
        parts.reserve(topkIndices.size());
        for (const int index : topkIndices) {
            parts << QString::number(index);
        }
        candidate->beamTopkIndices = parts.join(QStringLiteral(","));
    }

    if (candidate->beamSelectedIndex >= 0 && candidate->beamOracleIndex >= 0) {
        candidate->beamHit = candidate->beamSelectedIndex == candidate->beamOracleIndex ? 1 : 0;
    } else {
        candidate->beamHit = -1;
    }

    if (candidate->beamOracleIndex >= 0 && !topkIndices.isEmpty()) {
        candidate->beamOracleInTopK = topkIndices.contains(candidate->beamOracleIndex) ? 1 : 0;
    } else if (candidate->beamOracleIndex >= 0
               && candidate->beamPredictedIndex >= 0
               && candidate->beamTopK == 1) {
        candidate->beamOracleInTopK = candidate->beamPredictedIndex == candidate->beamOracleIndex ? 1 : 0;
    } else {
        candidate->beamOracleInTopK = -1;
    }
}

void MainWindow::normalizeSysSnapshotBeamFields(SysSnapshotView* snapshot) {
    if (!snapshot) {
        return;
    }

    const QVector<int> topkIndices = parseBeamTopkIndices(snapshot->beamTopkIndices);
    if (topkIndices.isEmpty()) {
        snapshot->beamTopkIndices.clear();
    } else {
        QStringList parts;
        parts.reserve(topkIndices.size());
        for (const int index : topkIndices) {
            parts << QString::number(index);
        }
        snapshot->beamTopkIndices = parts.join(QStringLiteral(","));
    }

    if (snapshot->beamSelectedIndex >= 0 && snapshot->beamOracleIndex >= 0) {
        snapshot->beamHit = snapshot->beamSelectedIndex == snapshot->beamOracleIndex ? 1 : 0;
    } else {
        snapshot->beamHit = -1;
    }

    if (snapshot->beamOracleIndex >= 0 && !topkIndices.isEmpty()) {
        snapshot->beamOracleInTopK = topkIndices.contains(snapshot->beamOracleIndex) ? 1 : 0;
    } else if (snapshot->beamOracleIndex >= 0
               && snapshot->beamPredictedIndex >= 0
               && snapshot->beamTopK == 1) {
        snapshot->beamOracleInTopK = snapshot->beamPredictedIndex == snapshot->beamOracleIndex ? 1 : 0;
    } else {
        snapshot->beamOracleInTopK = -1;
    }
}

void MainWindow::copySysCandidateBeamFields(SysCandidateView* target, const SysCandidateView& source) {
    if (!target) {
        return;
    }
    target->beamEnabled = source.beamEnabled;
    target->beamSelectionMode = source.beamSelectionMode;
    target->beamCodebookType = source.beamCodebookType;
    target->beamNumBeams = source.beamNumBeams;
    target->beamSelectedIndex = source.beamSelectedIndex;
    target->beamSelectedGainDb = source.beamSelectedGainDb;
    target->beamOracleIndex = source.beamOracleIndex;
    target->beamOracleGainDb = source.beamOracleGainDb;
    target->beamSelectedAnchor = source.beamSelectedAnchor;
    target->beamPredictorStatus = source.beamPredictorStatus;
    target->beamFeatureMode = source.beamFeatureMode;
    target->beamTopK = source.beamTopK;
    target->beamPredictedIndex = source.beamPredictedIndex;
    target->beamPredictedConfidence = source.beamPredictedConfidence;
    target->beamOracleInTopK = source.beamOracleInTopK;
    target->beamHit = source.beamHit;
    target->beamSelectionSource = source.beamSelectionSource;
    target->beamTopkIndices = source.beamTopkIndices;
}

void MainWindow::normalizeCurrentSysCandidates() {
    const bool hasServing = currentSysSnapshot_.servingAnchorId >= 0 || !currentSysSnapshot_.servingBsName.trimmed().isEmpty();
    const bool hasBeamReference = currentSysSnapshot_.beamReferenceAnchorId >= 0 || !currentSysSnapshot_.beamReferenceBsName.trimmed().isEmpty();
    for (auto& candidate : currentSysSnapshot_.candidates) {
        if (candidate.servingBsName.trimmed().isEmpty() && !currentSysSnapshot_.servingBsName.trimmed().isEmpty()) {
            candidate.servingBsName = currentSysSnapshot_.servingBsName;
        }
        if (candidate.servingAnchorId < 0 && currentSysSnapshot_.servingAnchorId >= 0) {
            candidate.servingAnchorId = currentSysSnapshot_.servingAnchorId;
        }
        if (hasServing) {
            candidate.isServingBs = sysCandidateMatchesServing(candidate, currentSysSnapshot_);
        }
        normalizeSysCandidateBeamFields(&candidate);
        if (hasBeamReference) {
            candidate.beamSelectedAnchor = sysCandidateMatchesIdentity(
                candidate, currentSysSnapshot_.beamReferenceAnchorId, currentSysSnapshot_.beamReferenceBsName);
        } else {
            candidate.beamSelectedAnchor = candidate.isServingBs && sysCandidateHasBeamData(candidate);
        }
    }
}

void MainWindow::syncCurrentBeamSummaryFromCandidate(const SysCandidateView& candidate, const QJsonObject* payload) {
    if (!sysCandidateHasBeamData(candidate)) {
        return;
    }

    if (!sysCandidateHasUsablePath(candidate)) {
        currentSysSnapshot_.beamEnabled = candidate.beamEnabled;
        currentSysSnapshot_.beamReferenceBsName = !candidate.bsName.trimmed().isEmpty() ? candidate.bsName : currentSysSnapshot_.servingBsName;
        currentSysSnapshot_.beamReferenceAnchorId = candidate.anchorId >= 0 ? candidate.anchorId : currentSysSnapshot_.servingAnchorId;
        currentSysSnapshot_.beamSelectedIndex = -1;
        currentSysSnapshot_.beamSelectedGainDb = std::numeric_limits<double>::quiet_NaN();
        currentSysSnapshot_.beamOracleIndex = -1;
        currentSysSnapshot_.beamOracleGainDb = std::numeric_limits<double>::quiet_NaN();
        currentSysSnapshot_.beamPredictedIndex = -1;
        currentSysSnapshot_.beamPredictedConfidence = std::numeric_limits<double>::quiet_NaN();
        currentSysSnapshot_.beamOracleInTopK = -1;
        currentSysSnapshot_.beamHit = -1;
        currentSysSnapshot_.beamSelectionSource.clear();
        currentSysSnapshot_.beamTopkIndices.clear();
        currentSysSnapshot_.beamPredictorStatus.clear();
        currentSysSnapshot_.beamPredictorError.clear();
        return;
    }

    currentSysSnapshot_.beamEnabled = candidate.beamEnabled;
    if (!candidate.beamSelectionMode.trimmed().isEmpty()) {
        currentSysSnapshot_.beamSelectionMode = candidate.beamSelectionMode;
    }
    if (!candidate.beamCodebookType.trimmed().isEmpty()) {
        currentSysSnapshot_.beamCodebookType = candidate.beamCodebookType;
    }
    if (candidate.beamNumBeams >= 0) {
        currentSysSnapshot_.beamNumBeams = candidate.beamNumBeams;
    }
    if (!candidate.bsName.trimmed().isEmpty()) {
        currentSysSnapshot_.beamReferenceBsName = candidate.bsName;
    } else if (!currentSysSnapshot_.servingBsName.trimmed().isEmpty()) {
        currentSysSnapshot_.beamReferenceBsName = currentSysSnapshot_.servingBsName;
    }
    if (candidate.anchorId >= 0) {
        currentSysSnapshot_.beamReferenceAnchorId = candidate.anchorId;
    } else if (currentSysSnapshot_.servingAnchorId >= 0) {
        currentSysSnapshot_.beamReferenceAnchorId = currentSysSnapshot_.servingAnchorId;
    }
    currentSysSnapshot_.beamSelectedIndex = candidate.beamSelectedIndex;
    currentSysSnapshot_.beamSelectedGainDb = candidate.beamSelectedGainDb;
    currentSysSnapshot_.beamOracleIndex = candidate.beamOracleIndex;
    currentSysSnapshot_.beamOracleGainDb = candidate.beamOracleGainDb;
    if (!candidate.beamPredictorStatus.trimmed().isEmpty()) {
        currentSysSnapshot_.beamPredictorStatus = candidate.beamPredictorStatus;
    }
    if (!candidate.beamFeatureMode.trimmed().isEmpty()) {
        currentSysSnapshot_.beamFeatureMode = candidate.beamFeatureMode;
    }
    if (candidate.beamTopK >= 0) {
        currentSysSnapshot_.beamTopK = candidate.beamTopK;
    }
    currentSysSnapshot_.beamPredictedIndex = candidate.beamPredictedIndex;
    currentSysSnapshot_.beamPredictedConfidence = candidate.beamPredictedConfidence;
    currentSysSnapshot_.beamOracleInTopK = candidate.beamOracleInTopK;
    currentSysSnapshot_.beamHit = candidate.beamHit;
    currentSysSnapshot_.beamSelectionSource = candidate.beamSelectionSource;
    currentSysSnapshot_.beamTopkIndices = candidate.beamTopkIndices;

    if (!payload) {
        normalizeSysSnapshotBeamFields(&currentSysSnapshot_);
        return;
    }
    if (payload->contains(QStringLiteral("tx_idx"))) {
        currentSysSnapshot_.beamReferenceTxIdx = payload->value(QStringLiteral("tx_idx")).toInt(currentSysSnapshot_.beamReferenceTxIdx);
    }
    if (payload->contains(QStringLiteral("beam_manual_index"))) {
        currentSysSnapshot_.beamManualIndex = payload->value(QStringLiteral("beam_manual_index")).toInt(currentSysSnapshot_.beamManualIndex);
    }
    if (payload->contains(QStringLiteral("beam_serving_source"))) {
        const QString servingSource = payload->value(QStringLiteral("beam_serving_source")).toString().trimmed();
        if (!servingSource.isEmpty()) {
            currentSysSnapshot_.beamServingSource = servingSource;
        }
    }
    if (payload->contains(QStringLiteral("beam_predictor_available"))) {
        currentSysSnapshot_.beamPredictorAvailable = payload->value(QStringLiteral("beam_predictor_available")).toBool(currentSysSnapshot_.beamPredictorAvailable);
    }
    if (payload->contains(QStringLiteral("beam_predictor_status"))) {
        const QString predictorStatus = payload->value(QStringLiteral("beam_predictor_status")).toString();
        if (!predictorStatus.trimmed().isEmpty()) {
            currentSysSnapshot_.beamPredictorStatus = predictorStatus;
        }
    }
    if (payload->contains(QStringLiteral("beam_predictor_error"))) {
        currentSysSnapshot_.beamPredictorError = payload->value(QStringLiteral("beam_predictor_error")).toString(currentSysSnapshot_.beamPredictorError);
    }
    normalizeSysSnapshotBeamFields(&currentSysSnapshot_);
}

namespace {

const RfAnchorObservationData* findAnchorForStation(const std::vector<RfAnchorObservationData>& anchors,
                                                    const BaseStation& station) {
    for (const auto& anchor : anchors) {
        if ((!station.id.isEmpty() && anchor.anchorName == station.id)
            || (!station.name.isEmpty() && anchor.anchorName == station.name)) {
            return &anchor;
        }
    }
    return nullptr;
}

QStringList candidateAirSimSettingsPaths() {
    QStringList candidates;
    const QByteArray envPath = qgetenv("AIRSIM_SETTINGS_FILE");
    if (!envPath.isEmpty()) {
        candidates << QFileInfo(QString::fromLocal8Bit(envPath)).absoluteFilePath();
    }
    const QString docs = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    if (!docs.isEmpty()) {
        candidates << QDir(docs).filePath(QStringLiteral("AirSim/settings.json"));
    }
    candidates << QDir::home().filePath(QStringLiteral("Documents/AirSim/settings.json"));
    candidates.removeDuplicates();
    return candidates;
}

QStringList parseAirSimCameraNames(const QString& settingsPath, const QString& preferredVehicle, QString* errorMessage = nullptr) {
    QFile file(settingsPath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = QObject::tr("Could not open %1").arg(settingsPath);
        }
        return {};
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage) {
            *errorMessage = QObject::tr("Failed to parse %1: %2").arg(settingsPath, parseError.errorString());
        }
        return {};
    }

    QStringList ordered;
    auto addName = [&ordered](const QString& name) {
        const QString trimmed = name.trimmed();
        if (!trimmed.isEmpty() && !ordered.contains(trimmed)) {
            ordered << trimmed;
        }
    };

    const QJsonObject root = doc.object();
    const QJsonObject external = root.value(QStringLiteral("ExternalCameras")).toObject();
    for (auto it = external.begin(); it != external.end(); ++it) {
        addName(it.key());
    }

    const QJsonObject vehicles = root.value(QStringLiteral("Vehicles")).toObject();
    auto addVehicleCameras = [&vehicles, &addName](const QString& vehicleName) {
        const QJsonObject vehicle = vehicles.value(vehicleName).toObject();
        const QJsonObject cameras = vehicle.value(QStringLiteral("Cameras")).toObject();
        for (auto it = cameras.begin(); it != cameras.end(); ++it) {
            addName(it.key());
        }
    };

    const QString preferred = preferredVehicle.trimmed();
    if (!preferred.isEmpty() && vehicles.contains(preferred)) {
        addVehicleCameras(preferred);
    }
    for (auto it = vehicles.begin(); it != vehicles.end(); ++it) {
        if (it.key() == preferred) {
            continue;
        }
        addVehicleCameras(it.key());
    }

    if (ordered.isEmpty()) {
        addName(QStringLiteral("front_center"));
    }
    return ordered;
}

struct CameraFpsControlItem {
    QString label;
    QString controlKey;
    QString rosTopic;
};

QString normalizedRosTopicName(QString topic) {
    topic = topic.trimmed();
    if (!topic.isEmpty() && !topic.startsWith(QChar('/'))) {
        topic.prepend(QChar('/'));
    }
    return topic;
}

QString encodedImageTopicFpsKey(const QString& rosTopic) {
    const QByteArray bytes = normalizedRosTopicName(rosTopic).toUtf8();
    QString key = QStringLiteral("t");
    for (const unsigned char c : bytes) {
        const bool alphaNumeric = (c >= '0' && c <= '9')
            || (c >= 'A' && c <= 'Z')
            || (c >= 'a' && c <= 'z');
        if (alphaNumeric) {
            key += QLatin1Char(static_cast<char>(c));
        } else {
            key += QStringLiteral("_%1").arg(static_cast<int>(c), 2, 16, QLatin1Char('0'));
        }
    }
    return key;
}

QString airSimImageTypeName(int imageType) {
    switch (imageType) {
    case 0:
        return QStringLiteral("Scene");
    case 1:
        return QStringLiteral("DepthPlanar");
    case 2:
        return QStringLiteral("DepthPerspective");
    case 3:
        return QStringLiteral("DepthVis");
    case 4:
        return QStringLiteral("DisparityNormalized");
    case 5:
        return QStringLiteral("Segmentation");
    case 6:
        return QStringLiteral("SurfaceNormals");
    case 7:
        return QStringLiteral("Infrared");
    default:
        return QString::number(imageType);
    }
}

QVector<int> captureImageTypesFromCameraObject(const QJsonObject& cameraObject) {
    QVector<int> imageTypes;
    QSet<int> seen;
    const QJsonArray captureSettings = cameraObject.value(QStringLiteral("CaptureSettings")).toArray();
    for (const QJsonValue& captureValue : captureSettings) {
        if (!captureValue.isObject()) {
            continue;
        }
        const int imageType = captureValue.toObject().value(QStringLiteral("ImageType")).toInt(0);
        if (seen.contains(imageType)) {
            continue;
        }
        seen.insert(imageType);
        imageTypes.push_back(imageType);
    }
    if (imageTypes.isEmpty()) {
        imageTypes.push_back(0);
    }
    return imageTypes;
}

QVector<CameraFpsControlItem> parseAirSimRosImageTopicItems(const QString& settingsPath,
                                                            const QString& preferredVehicle,
                                                            QString* errorMessage = nullptr) {
    QFile file(settingsPath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = QObject::tr("Could not open %1").arg(settingsPath);
        }
        return {};
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage) {
            *errorMessage = QObject::tr("Failed to parse %1: %2").arg(settingsPath, parseError.errorString());
        }
        return {};
    }

    QVector<CameraFpsControlItem> items;
    QSet<QString> seenTopics;
    const QJsonObject vehicles = doc.object().value(QStringLiteral("Vehicles")).toObject();
    QStringList vehicleNames;
    const QString preferred = preferredVehicle.trimmed();
    if (!preferred.isEmpty() && vehicles.contains(preferred)) {
        vehicleNames << preferred;
    }
    for (auto it = vehicles.begin(); it != vehicles.end(); ++it) {
        if (it.key() != preferred) {
            vehicleNames << it.key();
        }
    }

    for (const QString& vehicleName : vehicleNames) {
        const QJsonObject vehicleObject = vehicles.value(vehicleName).toObject();
        const QJsonObject cameras = vehicleObject.value(QStringLiteral("Cameras")).toObject();
        for (auto cameraIt = cameras.begin(); cameraIt != cameras.end(); ++cameraIt) {
            const QString cameraName = cameraIt.key();
            const QVector<int> imageTypes = captureImageTypesFromCameraObject(cameraIt.value().toObject());
            for (const int imageType : imageTypes) {
                const QString topic = normalizedRosTopicName(QStringLiteral("/airsim_node/%1/%2/%3")
                    .arg(vehicleName, cameraName, airSimImageTypeName(imageType)));
                if (seenTopics.contains(topic)) {
                    continue;
                }
                seenTopics.insert(topic);
                items.push_back({
                    QObject::tr("ROS: %1").arg(topic),
                    QString::fromLatin1(kCameraFpsRosTopicPrefix) + topic,
                    topic
                });
            }
        }
    }
    return items;
}

QVector<CameraImageLayerItem> parseDroneFrontCameraImageLayerItems(const QString& settingsPath,
                                                                   const QString& preferredVehicle,
                                                                   QString* errorMessage = nullptr) {
    QFile file(settingsPath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = QObject::tr("Could not open %1").arg(settingsPath);
        }
        return {};
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage) {
            *errorMessage = QObject::tr("Failed to parse %1: %2").arg(settingsPath, parseError.errorString());
        }
        return {};
    }

    const QJsonObject vehicles = doc.object().value(QStringLiteral("Vehicles")).toObject();
    if (vehicles.isEmpty()) {
        return {};
    }

    QString vehicleName = preferredVehicle.trimmed();
    if (vehicleName.isEmpty() || !vehicles.contains(vehicleName)) {
        vehicleName = vehicles.begin().key();
    }

    const QJsonObject vehicleObject = vehicles.value(vehicleName).toObject();
    const QJsonObject cameras = vehicleObject.value(QStringLiteral("Cameras")).toObject();
    if (cameras.isEmpty()) {
        return {};
    }

    QString cameraName;
    const QStringList preferredCameraNames{
        QStringLiteral("CameraImage"),
        QStringLiteral("front_center"),
        QStringLiteral("front_camera"),
        QStringLiteral("0")
    };
    for (const QString& candidate : preferredCameraNames) {
        if (cameras.contains(candidate)) {
            cameraName = candidate;
            break;
        }
    }
    if (cameraName.isEmpty()) {
        cameraName = cameras.begin().key();
    }

    const QVector<int> imageTypes = captureImageTypesFromCameraObject(cameras.value(cameraName).toObject());
    QVector<CameraImageLayerItem> items;
    QSet<QString> seenTopics;
    for (const int imageType : imageTypes) {
        const QString imageTypeName = airSimImageTypeName(imageType);
        const QString topic = normalizedRosTopicName(QStringLiteral("/airsim_node/%1/%2/%3")
            .arg(vehicleName, cameraName, imageTypeName));
        if (seenTopics.contains(topic)) {
            continue;
        }
        seenTopics.insert(topic);
        items.push_back({
            QString::fromLatin1(kDroneCameraImageLayerPrefix) + encodedImageTopicFpsKey(topic),
            QObject::tr("Drone Camera %1").arg(imageTypeName),
            topic,
            imageType
        });
    }
    return items;
}

struct InfoCardWidgets {
    QFrame* frame{nullptr};
    QLabel* titleLabel{nullptr};
    QLabel* bodyLabel{nullptr};
};

InfoCardWidgets createInfoCard(const QString& title, const QString& body, QWidget* parent) {
    auto* card = new QFrame(parent);
    card->setFrameShape(QFrame::StyledPanel);
    card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
    card->setStyleSheet(QStringLiteral("QFrame { background: #1f2430; border: 1px solid #394150; border-radius: 10px; } QLabel { color: #e8ecf1; }"));
    auto* layout = new QVBoxLayout(card);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(6);
    auto* titleLabel = new QLabel(title, card);
    titleLabel->setObjectName(QStringLiteral("cardTitleLabel"));
    titleLabel->setStyleSheet(QStringLiteral("font-weight: 700; font-size: 13px;"));
    titleLabel->setWordWrap(true);
    auto* bodyLabel = new QLabel(body, card);
    bodyLabel->setObjectName(QStringLiteral("cardBodyLabel"));
    bodyLabel->setWordWrap(true);
    bodyLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    bodyLabel->setStyleSheet(QStringLiteral("color: #c9d2dd;"));
    layout->addWidget(titleLabel);
    layout->addWidget(bodyLabel);
    return {card, titleLabel, bodyLabel};
}

QFrame* makeInfoCard(const QString& title, const QString& body, QWidget* parent) {
    return createInfoCard(title, body, parent).frame;
}

}

class CameraImageStripWidget : public QWidget {
public:
    explicit CameraImageStripWidget(QWidget* host)
        : QWidget(host), host_(host) {
        setObjectName(QStringLiteral("cameraImageStrip"));
        setAttribute(Qt::WA_StyledBackground, true);
        setMouseTracking(true);
        setStyleSheet(QStringLiteral(
            "QWidget#cameraImageStrip { background: rgba(10, 16, 28, 178); border: 1px solid rgba(148, 163, 184, 90); border-radius: 10px; }"
            "QFrame#cameraImageCard { background: rgba(17, 24, 39, 205); border: 2px solid #64748b; border-radius: 4px; }"
            "QLabel { color: #e5edf7; }"
            "QLabel#cameraImageTitle { font-weight: 700; font-size: 11px; }"
            "QLabel#cameraImageView { background: #0b1020; color: #94a3b8; }"));
        auto* layout = new QHBoxLayout(this);
        layout->setContentsMargins(10, 8, 10, 8);
        layout->setSpacing(10);
        if (host_) {
            host_->installEventFilter(this);
        }
        hide();
    }

    ~CameraImageStripWidget() override {
        shutdownSubscriptions();
    }

    void setStreams(const QVector<CameraImageLayerItem>& streams, bool active) {
        streams_ = streams;
        active_ = active;
        rebuildCards();
        restartSubscriptions();
        reposition();
    }

    void setActive(bool active) {
        if (active_ == active) {
            return;
        }
        active_ = active;
        restartSubscriptions();
        reposition();
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override {
        if (watched == host_ && (event->type() == QEvent::Resize || event->type() == QEvent::Show || event->type() == QEvent::Move)) {
            reposition();
        }
        return QWidget::eventFilter(watched, event);
    }

    void paintEvent(QPaintEvent* event) override {
        QWidget::paintEvent(event);
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(QColor(203, 213, 225, 170), 2));
        const QRect handle = resizeHandleRect();
        for (int offset = 4; offset <= 14; offset += 5) {
            painter.drawLine(handle.right() - offset, handle.bottom() - 2,
                             handle.right() - 2, handle.bottom() - offset);
        }
    }

    void mousePressEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton && resizeHandleRect().contains(event->pos())) {
            resizing_ = true;
            resizeStartGlobalPos_ = event->globalPos();
            resizeStartCardSize_ = cardSize_;
            setCursor(Qt::SizeFDiagCursor);
            event->accept();
            return;
        }
        QWidget::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (resizing_) {
            const QPoint delta = event->globalPos() - resizeStartGlobalPos_;
            const int count = std::max(1, cards_.size());
            cardSize_.setWidth(std::max(static_cast<int>(kMinCardWidth),
                                        std::min(static_cast<int>(kMaxCardWidth),
                                                 resizeStartCardSize_.width() + delta.x() / count)));
            cardSize_.setHeight(std::max(static_cast<int>(kMinCardHeight),
                                         std::min(static_cast<int>(kMaxCardHeight),
                                                  resizeStartCardSize_.height() + delta.y())));
            reposition();
            update();
            event->accept();
            return;
        }
        setCursor(resizeHandleRect().contains(event->pos()) ? Qt::SizeFDiagCursor : Qt::ArrowCursor);
        QWidget::mouseMoveEvent(event);
    }

    void mouseReleaseEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton && resizing_) {
            resizing_ = false;
            setCursor(resizeHandleRect().contains(event->pos()) ? Qt::SizeFDiagCursor : Qt::ArrowCursor);
            event->accept();
            return;
        }
        QWidget::mouseReleaseEvent(event);
    }

private:
    enum {
        kMinCardWidth = 320,
        kMaxCardWidth = 760,
        kMinCardHeight = 150,
        kMaxCardHeight = 420
    };

    struct Card {
        CameraImageLayerItem item;
        QFrame* frame{nullptr};
        QLabel* title{nullptr};
        QLabel* image{nullptr};
        ros::Subscriber subscriber;
        QImage lastImage;
    };

    static QImage imageFromRosMessage(const sensor_msgs::Image& msg) {
        if (msg.width == 0 || msg.height == 0 || msg.step == 0 || msg.data.empty()) {
            return QImage();
        }
        const size_t requiredBytes = static_cast<size_t>(msg.step) * static_cast<size_t>(msg.height);
        if (msg.data.size() < requiredBytes) {
            return QImage();
        }

        const auto* data = msg.data.data();
        const int width = static_cast<int>(msg.width);
        const int height = static_cast<int>(msg.height);
        const int bytesPerLine = static_cast<int>(msg.step);
        const std::string& encoding = msg.encoding;

        if (encoding == "rgb8") {
            QImage view(data, width, height, bytesPerLine, QImage::Format_RGB888);
            return view.copy();
        }
        if (encoding == "bgr8") {
            QImage view(data, width, height, bytesPerLine, QImage::Format_RGB888);
            return view.rgbSwapped().copy();
        }
        if (encoding == "rgba8") {
            QImage view(data, width, height, bytesPerLine, QImage::Format_RGBA8888);
            return view.convertToFormat(QImage::Format_RGB32);
        }
        if (encoding == "bgra8") {
            QImage view(data, width, height, bytesPerLine, QImage::Format_ARGB32);
            return view.convertToFormat(QImage::Format_RGB32);
        }
        if (encoding == "mono8" || encoding == "8UC1") {
            QImage view(data, width, height, bytesPerLine, QImage::Format_Grayscale8);
            return view.copy();
        }
        if (encoding == "mono16" || encoding == "16UC1") {
            return normalizedU16Image(msg);
        }
        if (encoding == "32FC1" || encoding == "32FC") {
            return normalizedFloatImage(msg);
        }
        return QImage();
    }

    static QImage normalizedFloatImage(const sensor_msgs::Image& msg) {
        const int width = static_cast<int>(msg.width);
        const int height = static_cast<int>(msg.height);
        QImage out(width, height, QImage::Format_Grayscale8);
        if (msg.step < static_cast<uint32_t>(width * static_cast<int>(sizeof(float)))) {
            return QImage();
        }

        double minValue = std::numeric_limits<double>::infinity();
        double maxValue = -std::numeric_limits<double>::infinity();
        for (int y = 0; y < height; ++y) {
            const auto* row = reinterpret_cast<const float*>(msg.data.data() + static_cast<size_t>(y) * msg.step);
            for (int x = 0; x < width; ++x) {
                const float value = row[x];
                if (!std::isfinite(value)) {
                    continue;
                }
                minValue = std::min(minValue, static_cast<double>(value));
                maxValue = std::max(maxValue, static_cast<double>(value));
            }
        }
        if (!std::isfinite(minValue) || !std::isfinite(maxValue) || maxValue <= minValue) {
            out.fill(0);
            return out;
        }

        const double scale = 255.0 / (maxValue - minValue);
        for (int y = 0; y < height; ++y) {
            const auto* row = reinterpret_cast<const float*>(msg.data.data() + static_cast<size_t>(y) * msg.step);
            auto* dst = out.scanLine(y);
            for (int x = 0; x < width; ++x) {
                const float value = row[x];
                dst[x] = std::isfinite(value)
                    ? static_cast<uchar>(std::max(0, std::min(255, static_cast<int>((static_cast<double>(value) - minValue) * scale))))
                    : static_cast<uchar>(0);
            }
        }
        return out;
    }

    static QImage normalizedU16Image(const sensor_msgs::Image& msg) {
        const int width = static_cast<int>(msg.width);
        const int height = static_cast<int>(msg.height);
        QImage out(width, height, QImage::Format_Grayscale8);
        if (msg.step < static_cast<uint32_t>(width * static_cast<int>(sizeof(uint16_t)))) {
            return QImage();
        }

        uint16_t minValue = std::numeric_limits<uint16_t>::max();
        uint16_t maxValue = 0;
        for (int y = 0; y < height; ++y) {
            const auto* row = reinterpret_cast<const uint16_t*>(msg.data.data() + static_cast<size_t>(y) * msg.step);
            for (int x = 0; x < width; ++x) {
                minValue = std::min(minValue, row[x]);
                maxValue = std::max(maxValue, row[x]);
            }
        }
        if (maxValue <= minValue) {
            out.fill(0);
            return out;
        }

        const double scale = 255.0 / static_cast<double>(maxValue - minValue);
        for (int y = 0; y < height; ++y) {
            const auto* row = reinterpret_cast<const uint16_t*>(msg.data.data() + static_cast<size_t>(y) * msg.step);
            auto* dst = out.scanLine(y);
            for (int x = 0; x < width; ++x) {
                dst[x] = static_cast<uchar>(std::max(0, std::min(255, static_cast<int>((row[x] - minValue) * scale))));
            }
        }
        return out;
    }

    QRect resizeHandleRect() const {
        return QRect(std::max(0, width() - 28), std::max(0, height() - 28), 24, 24);
    }

    QSize effectiveCardSize() const {
        const int count = std::max(1, cards_.isEmpty() ? streams_.size() : cards_.size());
        const int spacing = 10;
        const int maxVisibleWidth = host_ ? std::max(320, host_->width() - 80) : 1280;
        const int availableWidth = (maxVisibleWidth - 20 - std::max(0, count - 1) * spacing) / count;
        const int cardWidth = std::max(static_cast<int>(kMinCardWidth),
                                       std::min(cardSize_.width(),
                                                std::max(static_cast<int>(kMinCardWidth), availableWidth)));
        const int maxHeightFromHost = host_
            ? std::max(static_cast<int>(kMinCardHeight), host_->height() - 100)
            : static_cast<int>(kMaxCardHeight);
        const int cardHeight = std::max(static_cast<int>(kMinCardHeight),
                                        std::min(cardSize_.height(),
                                                 std::min(static_cast<int>(kMaxCardHeight), maxHeightFromHost)));
        return QSize(cardWidth, cardHeight);
    }

    void refreshCardPixmap(Card& card) {
        if (!card.image || card.lastImage.isNull()) {
            return;
        }
        const QPixmap pixmap = QPixmap::fromImage(card.lastImage).scaled(card.image->size(),
                                                                        Qt::KeepAspectRatio,
                                                                        Qt::FastTransformation);
        card.image->setPixmap(pixmap);
        card.image->setText(QString());
    }

    void applyCardSizes() {
        const QSize cardSize = effectiveCardSize();
        const QSize imageSize(std::max(1, cardSize.width() - 12), std::max(96, cardSize.height() - 38));
        for (Card& card : cards_) {
            if (!card.frame || !card.title || !card.image) {
                continue;
            }
            card.frame->setFixedSize(cardSize);
            card.title->setFixedHeight(20);
            card.title->setMaximumWidth(std::max(1, cardSize.width() - 12));
            card.image->setFixedSize(imageSize);
            refreshCardPixmap(card);
        }
    }

    void rebuildCards() {
        shutdownSubscriptions();
        auto* layout = qobject_cast<QHBoxLayout*>(this->layout());
        while (layout && layout->count() > 0) {
            QLayoutItem* item = layout->takeAt(0);
            if (item->widget()) {
                item->widget()->deleteLater();
            }
            delete item;
        }
        cards_.clear();

        for (const CameraImageLayerItem& stream : streams_) {
            auto* frame = new QFrame(this);
            frame->setObjectName(QStringLiteral("cameraImageCard"));
            frame->setAttribute(Qt::WA_TransparentForMouseEvents, true);
            auto* cardLayout = new QVBoxLayout(frame);
            cardLayout->setContentsMargins(6, 5, 6, 5);
            cardLayout->setSpacing(3);
            auto* title = new QLabel(QStringLiteral("%1  %2").arg(stream.label, stream.topic), frame);
            title->setObjectName(QStringLiteral("cameraImageTitle"));
            title->setTextFormat(Qt::PlainText);
            title->setAttribute(Qt::WA_TransparentForMouseEvents, true);
            title->setFixedHeight(20);
            title->setToolTip(stream.topic);
            auto* image = new QLabel(tr("Waiting for image..."), frame);
            image->setObjectName(QStringLiteral("cameraImageView"));
            image->setAttribute(Qt::WA_TransparentForMouseEvents, true);
            image->setAlignment(Qt::AlignCenter);
            image->setScaledContents(false);
            cardLayout->addWidget(title);
            cardLayout->addWidget(image);
            layout->addWidget(frame);
            cards_.push_back({stream, frame, title, image, ros::Subscriber()});
        }
        applyCardSizes();
    }

    void restartSubscriptions() {
        shutdownSubscriptions();
        if (!active_ || cards_.isEmpty()) {
            hide();
            return;
        }
        show();
        raise();
        if (!ros::isInitialized()) {
            setAllStatus(tr("ROS is not initialized"));
            return;
        }
        ros::start();
        if (!ros::master::check()) {
            setAllStatus(tr("Waiting for ROS master"));
            return;
        }

        nodeHandle_ = std::make_unique<ros::NodeHandle>();
        for (Card& card : cards_) {
            const QString key = card.item.key;
            const QString topic = card.item.topic;
            card.subscriber = nodeHandle_->subscribe<sensor_msgs::Image>(
                topic.toStdString(),
                1,
                [this, key, topic](const sensor_msgs::Image::ConstPtr& msg) {
                    if (!msg) {
                        return;
                    }
                    const QImage decoded = imageFromRosMessage(*msg);
                    QMetaObject::invokeMethod(this,
                        [this, key, topic, decoded]() {
                            updateImage(key, topic, decoded);
                        },
                        Qt::QueuedConnection);
                },
                ros::VoidConstPtr(),
                ros::TransportHints().tcpNoDelay());
        }
        spinner_ = std::make_unique<ros::AsyncSpinner>(1);
        spinner_->start();
    }

    void shutdownSubscriptions() {
        for (Card& card : cards_) {
            card.subscriber.shutdown();
        }
        if (spinner_) {
            spinner_->stop();
            spinner_.reset();
        }
        nodeHandle_.reset();
    }

    void updateImage(const QString& key, const QString& topic, const QImage& image) {
        for (Card& card : cards_) {
            if (card.item.key != key) {
                continue;
            }
            if (image.isNull()) {
                card.lastImage = QImage();
                card.image->setText(tr("Unsupported image encoding\n%1").arg(topic));
                card.image->setPixmap(QPixmap());
                return;
            }
            card.lastImage = image;
            refreshCardPixmap(card);
            return;
        }
    }

    void setAllStatus(const QString& text) {
        for (Card& card : cards_) {
            card.lastImage = QImage();
            card.image->setPixmap(QPixmap());
            card.image->setText(text);
        }
    }

    void reposition() {
        if (!host_ || !active_ || streams_.isEmpty()) {
            hide();
            return;
        }
        applyCardSizes();
        const QSize cardSize = effectiveCardSize();
        const int cardWidth = cardSize.width();
        const int spacing = 10;
        const int maxVisibleWidth = std::max(320, host_->width() - 80);
        const int desiredWidth = std::min(maxVisibleWidth, static_cast<int>(streams_.size()) * cardWidth + std::max(0, static_cast<int>(streams_.size()) - 1) * spacing + 20);
        const int height = cardSize.height() + 24;
        const int x = std::max(8, (host_->width() - desiredWidth) / 2);
        const int y = std::max(8, host_->height() - height - 18);
        setGeometry(x, y, desiredWidth, height);
        show();
        raise();
    }

    QWidget* host_{nullptr};
    QVector<CameraImageLayerItem> streams_;
    QVector<Card> cards_;
    bool active_{false};
    bool resizing_{false};
    QPoint resizeStartGlobalPos_;
    QSize resizeStartCardSize_;
    QSize cardSize_{420, 190};
    std::unique_ptr<ros::NodeHandle> nodeHandle_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
};

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    frameTransformManager_ = std::make_shared<FrameTransformManager>();
    rosBridge_ = new RosBridge(this);
    rosBridge_->setFrameTransformManager(frameTransformManager_);
    demoController_ = new DemoController(this);
    sceneManager_ = new SceneManager();
    simulatorProcess_ = new QProcess(this);
    ckmProcess_ = new QProcess(this);
    rosbagRecordProcess_ = new QProcess(this);
    rosbagPlayProcess_ = new QProcess(this);
    rosbagResimProcess_ = new QProcess(this);
    managedRoscoreProcess_ = new QProcess(this);
    simulatorProcess_->setProcessChannelMode(QProcess::MergedChannels);
    ckmProcess_->setProcessChannelMode(QProcess::MergedChannels);
    rosbagRecordProcess_->setProcessChannelMode(QProcess::MergedChannels);
    rosbagPlayProcess_->setProcessChannelMode(QProcess::MergedChannels);
    rosbagResimProcess_->setProcessChannelMode(QProcess::MergedChannels);
    managedRoscoreProcess_->setProcessChannelMode(QProcess::MergedChannels);

    connect(rosBridge_, &RosBridge::droneUpdated, this, &MainWindow::onDroneUpdated);
    connect(rosBridge_, &RosBridge::pathsUpdated, this, &MainWindow::onPathsUpdated);
    connect(rosBridge_, &RosBridge::rfDataUpdated, this, &MainWindow::onRfDataUpdated);
    connect(rosBridge_, &RosBridge::sysObservationUpdated, this, &MainWindow::onSysObservationUpdated);
    connect(rosBridge_, &RosBridge::beamObservationUpdated, this, &MainWindow::onBeamObservationUpdated);
    connect(rosBridge_, &RosBridge::statusMessage, this, &MainWindow::onStatusMessage);

    connect(demoController_, &DemoController::droneUpdated, this, &MainWindow::onDroneUpdated);
    connect(demoController_, &DemoController::pathsUpdated, this, &MainWindow::onPathsUpdated);
    connect(demoController_, &DemoController::statusMessage, this, &MainWindow::onStatusMessage);

    connect(simulatorProcess_, &QProcess::readyReadStandardOutput, this, &MainWindow::onSimulatorOutput);
    connect(simulatorProcess_, &QProcess::errorOccurred, this, &MainWindow::onSimulatorError);
    connect(simulatorProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onSimulatorFinished);

    connect(ckmProcess_, &QProcess::readyReadStandardOutput, this, &MainWindow::onCkmProcessOutput);
    connect(ckmProcess_, &QProcess::errorOccurred, this, &MainWindow::onCkmProcessError);
    connect(ckmProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onCkmProcessFinished);

    connect(rosbagRecordProcess_, &QProcess::readyReadStandardOutput, this, &MainWindow::onRosbagRecordOutput);
    connect(rosbagRecordProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onRosbagRecordFinished);
    connect(rosbagPlayProcess_, &QProcess::readyReadStandardOutput, this, &MainWindow::onRosbagPlaybackOutput);
    connect(rosbagPlayProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onRosbagPlaybackFinished);
    connect(rosbagResimProcess_, &QProcess::readyReadStandardOutput, this, &MainWindow::onRosbagResimOutput);
    connect(rosbagResimProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onRosbagResimFinished);

    sysDataRefreshTimer_ = new QTimer(this);
    sysDataRefreshTimer_->setSingleShot(true);
    sysDataRefreshTimer_->setInterval(120);
    connect(sysDataRefreshTimer_, &QTimer::timeout, this, &MainWindow::refreshSysDataWindowIfNeeded);

    buildUi();
    {
        QSettings settings(QStringLiteral("OpenAI"), QStringLiteral("airsim_gui_UErealtime"));
        const int savedBrightnessPercent = std::max(25, std::min(300, settings.value(QStringLiteral("liveView/displayBrightnessPercent"), 100).toInt()));
        if (airsimBrightnessSlider_) {
            airsimBrightnessSlider_->setValue(savedBrightnessPercent);
        }
    }
    updateRosbagUiState();
    connect(sceneWidget_, &Scene3DWidget::baseStationEditStarted, this, &MainWindow::onBaseStationEditStarted);
    connect(sceneWidget_, &Scene3DWidget::baseStationEditFinished, this, &MainWindow::onBaseStationEditFinished);
    connect(sceneWidget_, &Scene3DWidget::baseStationsEdited, this, &MainWindow::onBaseStationsEdited);
    connect(sceneWidget_, &Scene3DWidget::stationSelectionChanged, this, &MainWindow::onStationSelectionChanged);
    connect(sceneWidget_, &Scene3DWidget::statusMessage, this, &MainWindow::onStatusMessage);
    loadInitialData();
    setGuiConfigDirty(false);
    setBaseStationsDirty(false);
    clearBaseStationUndoHistory();
    refreshRosTopics();
    refreshAirSimCameraList();
    refreshInfoPanel();
    syncAirSimLiveViewSettings();

    stationCameraDemandTimer_ = new QTimer(this);
    stationCameraDemandTimer_->setInterval(1000);
    connect(stationCameraDemandTimer_, &QTimer::timeout, this, &MainWindow::refreshStationCameraStreamingState);
    stationCameraDemandTimer_->start();
    refreshStationCameraStreamingState();

    QTimer::singleShot(0, this, [this]() {
        promptStartupConfigChoice();
    });
}

MainWindow::~MainWindow() {
    disableAllStationCameraAirsimNodePublishing();
    restoreAllRosTopicPublisherRegistrations(false);
    stopAirSimLiveView();
    stopAllStationCameraWindows();
    stopDemo();
    stopInternalSimulator();
    stopDenseCkmGeneration();
    stopProcessGracefully(rosbagResimProcess_, 3000);
    stopProcessGracefully(rosbagPlayProcess_, 3000);
    stopProcessGracefully(rosbagRecordProcess_, 3000);
    stopProcessGracefully(managedRoscoreProcess_, 3000);
    delete sceneManager_;
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event) {
    if (watched == sionnaPreviewWindow_ && event && event->type() == QEvent::Close) {
        if (sionnaPreviewWidget_) {
            sionnaPreviewWidget_->setPreviewActive(false);
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (!event) {
        return;
    }
    if (!promptSaveBaseStationsJsonIfDirty(
            tr("The base-station JSON has unsaved changes. Save it before closing the GUI?"))) {
        event->ignore();
        return;
    }
    if (!promptSaveConfigOnClose()) {
        event->ignore();
        return;
    }
    event->accept();
}

void MainWindow::markGuiConfigDirty() {
    setGuiConfigDirty(true);
}

void MainWindow::markBaseStationsDirty() {
    const bool wasDirty = baseStationsDirty_;
    setBaseStationsDirty(true);
    setGuiConfigDirty(true);
    if (!wasDirty) {
        refreshInfoPanel();
    }
}

void MainWindow::setGuiConfigDirty(bool dirty) {
    if (suppressDirtyTracking_ && dirty) {
        return;
    }
    if (guiConfigDirty_ == dirty) {
        return;
    }
    guiConfigDirty_ = dirty;
    updateWindowModifiedState();
}

void MainWindow::setBaseStationsDirty(bool dirty) {
    if (suppressDirtyTracking_ && dirty) {
        return;
    }
    if (baseStationsDirty_ == dirty) {
        return;
    }
    baseStationsDirty_ = dirty;
    updateWindowModifiedState();
}

void MainWindow::updateWindowModifiedState() {
    setWindowModified(hasUnsavedConfigChanges());
}

void MainWindow::updateMainWindowTitle() {
    QString title = QStringLiteral("SimART");
    if (guiConfigSessionActive_) {
        QString configName;
        const QString path = currentGuiConfigPath_.trimmed();
        if (!path.isEmpty()) {
            configName = QFileInfo(path).completeBaseName().trimmed();
            if (configName.isEmpty()) {
                configName = QFileInfo(path).fileName().trimmed();
            }
        }
        if (configName.isEmpty()) {
            configName = QStringLiteral("untitled");
        }
        title += QStringLiteral("-") + configName;
    }
    title += QStringLiteral("[*]");
    setWindowTitle(title);
}

bool MainWindow::hasUnsavedConfigChanges() const {
    return guiConfigDirty_ || baseStationsDirty_;
}

bool MainWindow::saveGuiConfigForClose() {
    QString path = currentGuiConfigPath_.trimmed();
    if (path.isEmpty()) {
        path = QFileDialog::getSaveFileName(this,
                                            tr("Save SimART config"),
                                            QDir::home().filePath(QString::fromLatin1(kUntitledGuiConfigName) + QLatin1String(kGuiConfigExtension)),
                                            tr("SimART Config (*%1)").arg(QLatin1String(kGuiConfigExtension)));
        if (path.isEmpty()) {
            return false;
        }
        path = normalizedGuiConfigPath(path);
    }

    QString errorMessage;
    if (!saveGuiConfigToFile(path, &errorMessage)) {
        QMessageBox::warning(this, tr("Save SimART Config"), errorMessage);
        return false;
    }
    currentGuiConfigPath_ = path;
    guiConfigSessionActive_ = true;
    updateMainWindowTitle();
    setGuiConfigDirty(false);
    onStatusMessage(tr("Saved SimART config: %1").arg(QFileInfo(path).fileName()));
    return true;
}

bool MainWindow::promptSaveConfigOnClose() {
    const bool needsInitialSave = guiConfigSessionActive_ && currentGuiConfigPath_.trimmed().isEmpty();
    if (!guiConfigDirty_ && !needsInitialSave) {
        return true;
    }

    const QString message = needsInitialSave
        ? tr("This configuration has not been saved yet. Save it as a config file?")
        : tr("Save the current configuration file?");
    const QMessageBox::StandardButton choice = QMessageBox::question(
        this,
        tr("Save SimART Config"),
        message,
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
        QMessageBox::Save);
    if (choice == QMessageBox::Cancel) {
        return false;
    }
    if (choice == QMessageBox::Save) {
        return saveGuiConfigForClose();
    }
    return true;
}

void MainWindow::pushBaseStationUndoState(const std::vector<BaseStation>& stations) {
    if (undoingBaseStationEdit_) {
        return;
    }
    if (!baseStationUndoStack_.empty() && baseStationUndoStack_.back().size() == stations.size()) {
        bool same = true;
        for (size_t i = 0; i < stations.size(); ++i) {
            const BaseStation& a = baseStationUndoStack_.back()[i];
            const BaseStation& b = stations[i];
            same = same
                && a.id == b.id
                && a.name == b.name
                && std::abs(a.position.x - b.position.x) < 1e-9
                && std::abs(a.position.y - b.position.y) < 1e-9
                && std::abs(a.position.z - b.position.z) < 1e-9
                && std::abs(a.color.x - b.color.x) < 1e-9
                && std::abs(a.color.y - b.color.y) < 1e-9
                && std::abs(a.color.z - b.color.z) < 1e-9
                && a.previewCameraName == b.previewCameraName
                && a.previewRosTopic == b.previewRosTopic
                && std::abs(a.previewOffsetZ - b.previewOffsetZ) < 1e-9
                && std::abs(a.previewFps - b.previewFps) < 1e-9
                && a.previewCameraTargetEnabled == b.previewCameraTargetEnabled
                && nearlyEqualVec3(a.previewCameraTarget, b.previewCameraTarget);
            if (!same) {
                break;
            }
        }
        if (same) {
            updateUndoActionState();
            return;
        }
    }
    baseStationUndoStack_.push_back(stations);
    constexpr size_t kMaxUndoStates = 64;
    if (baseStationUndoStack_.size() > kMaxUndoStates) {
        baseStationUndoStack_.erase(baseStationUndoStack_.begin());
    }
    updateUndoActionState();
}

void MainWindow::clearBaseStationUndoHistory() {
    baseStationUndoStack_.clear();
    baseStationEditTransactionBaseline_.clear();
    baseStationEditTransactionActive_ = false;
    baseStationUndoCapturedForTransaction_ = false;
    updateUndoActionState();
}

void MainWindow::updateUndoActionState() {
    if (undoBaseStationEditAction_) {
        undoBaseStationEditAction_->setEnabled(!baseStationUndoStack_.empty());
    }
}

void MainWindow::buildUi() {
    updateMainWindowTitle();
    resize(1500, 920);

    sceneWidget_ = new Scene3DWidget(this);
    ueViewWidget_ = new UeViewWidget(this);
    ckmMapWidget_ = new CkmMapWidget(this);
    sionnaPreviewWindow_ = new QMainWindow(this, Qt::Window);
    sionnaPreviewWindow_->setAttribute(Qt::WA_DeleteOnClose, false);
    sionnaPreviewWindow_->setAttribute(Qt::WA_QuitOnClose, false);
    sionnaPreviewWindow_->setWindowTitle(tr("Sionna Preview"));
    sionnaPreviewWindow_->resize(1120, 820);
    sionnaPreviewWindow_->installEventFilter(this);
    sionnaPreviewWidget_ = new SionnaPreviewWidget(sionnaPreviewWindow_);
    sionnaPreviewWidget_->setLauncherScriptPath(findBundledScript(QStringLiteral("scripts/sionna_embed_launcher.py")));
    sionnaPreviewWindow_->setCentralWidget(sionnaPreviewWidget_);
    connect(sionnaPreviewWidget_, &SionnaPreviewWidget::statusMessage, this, &MainWindow::onStatusMessage);
    connect(ckmMapWidget_, &CkmMapWidget::generateRequested, this, &MainWindow::generateDenseCkm);
    connect(ckmMapWidget_, &CkmMapWidget::stopRequested, this, &MainWindow::stopDenseCkmFromUi);
    connect(ckmMapWidget_, &CkmMapWidget::loadLocalRequested, this, &MainWindow::loadDenseCkmFromDirectory);
    connect(ckmMapWidget_, &CkmMapWidget::pointSelectionChanged, this, &MainWindow::onCkmPointSelectionChanged);
    connect(ckmMapWidget_, &CkmMapWidget::pointSelectionCleared, this, &MainWindow::onCkmPointSelectionCleared);

    QString qmlError;
    const QString qmlPath = findConfigFile("qml/UeLiveView.qml");
    const QString helperScript;
    if (!ueViewWidget_->initialize(qmlPath, helperScript, &qmlError)) {
        QMessageBox::warning(this, tr("UE Live View"), qmlError);
    }
    airSimViewController_ = ueViewWidget_->controller();
    if (airSimViewController_) {
        airSimViewController_->setFrameTransformManager(frameTransformManager_);
    }
    sceneCameraImageStrip_ = new CameraImageStripWidget(sceneWidget_->overlayHostWidget());
    liveCameraImageStrip_ = new CameraImageStripWidget(ueViewWidget_->overlayHostWidget());

    centerViewTabs_ = new QTabWidget(this);
    centerViewTabs_->addTab(sceneWidget_, tr("3D View"));
    centerViewTabs_->addTab(ueViewWidget_, tr("UE Live View"));
    centerViewTabs_->addTab(ckmMapWidget_, tr("CKM Map"));
    connect(centerViewTabs_, &QTabWidget::currentChanged, this, &MainWindow::onCenterViewTabChanged);
    setCentralWidget(centerViewTabs_);

    buildMenuBar();
    buildToolBar();
    buildLeftDock();
    buildRightDock();
    buildDeveloperToolsDock();
    syncDroneCameraImageLayerItems();

    if (airSimViewController_) {
        connect(airSimViewController_, &AirSimViewController::helperMessage, this, &MainWindow::onAirSimLiveViewStatus);
        connect(airSimViewController_, &AirSimViewController::connectedChanged, this, &MainWindow::refreshAirSimCameraList);
        connect(airSimViewController_, &AirSimViewController::connectedChanged, this, &MainWindow::refreshStationCameraStreamingState);
        connect(airSimViewController_, &AirSimViewController::stationSelectionRequested, this,
                [this](int index) {
                    if (!sceneWidget_) {
                        return;
                    }
                    sceneWidget_->setSelectedBaseStationIndex(index);
                    if (index < 0) {
                        onStatusMessage(tr("Base-station selection cleared."));
                    }
                });
    }

    liveViewTabActive_ = (centerViewTabs_ && centerViewTabs_->currentWidget() == ueViewWidget_);
    syncSionnaPreviewSettings();
    onCenterViewTabChanged(centerViewTabs_ ? centerViewTabs_->currentIndex() : 0);
    refreshRightPanelMode();
    updateRosbagUiState();
    refreshDenseCkmControls();
    statusBar()->showMessage("Ready.");
}

void MainWindow::buildMenuBar() {
    auto* fileMenu = menuBar()->addMenu(tr("File"));
    openGuiConfigAction_ = fileMenu->addAction(tr("Open SimART Config..."), this, &MainWindow::openGuiConfig);
    saveGuiConfigAction_ = fileMenu->addAction(tr("Save SimART Config"), this, &MainWindow::saveGuiConfig);
    saveGuiConfigAction_->setShortcut(QKeySequence::Save);
    saveGuiConfigAction_->setShortcutContext(Qt::WindowShortcut);
    addAction(saveGuiConfigAction_);
    saveGuiConfigAsAction_ = fileMenu->addAction(tr("Save SimART Config As..."), this, &MainWindow::saveGuiConfigAs);
    fileMenu->addSeparator();
    loadSceneMeshAction_ = fileMenu->addAction(tr("Load Scene Mesh..."), this, &MainWindow::loadSceneMesh);
    importOsmSceneAction_ = fileMenu->addAction(tr("Search && Import OSM Area..."), this, &MainWindow::importOsmScene);
    clearSceneAction_ = fileMenu->addAction(tr("Clear Scene Geometry"), this, &MainWindow::clearSceneGeometry);
    fileMenu->addSeparator();
    reloadConfigAction_ = fileMenu->addAction(tr("Reload JSON Configs"), this, &MainWindow::reloadConfigs);
    fileMenu->addSeparator();
    fileMenu->addAction(tr("Quit"), qApp, &QApplication::quit);

    auto* editMenu = menuBar()->addMenu(tr("Edit"));
    undoBaseStationEditAction_ = editMenu->addAction(tr("Undo Base Station Edit"), this, &MainWindow::undoBaseStationEdit);
    undoBaseStationEditAction_->setShortcut(QKeySequence::Undo);
    undoBaseStationEditAction_->setEnabled(false);

    auto* viewMenu = menuBar()->addMenu(tr("View"));
    viewMenu->addAction(tr("Reset 3D Camera"), sceneWidget_, &Scene3DWidget::resetCamera);
    if (airSimViewController_) {
        viewMenu->addAction(tr("Reset Live Camera"), airSimViewController_, &AirSimViewController::resetCamera);
    }
    viewMenu->addSeparator();
    auto* modeMenu = viewMenu->addMenu(tr("Center View"));
    auto* modeGroup = new QActionGroup(this);
    modeGroup->setExclusive(true);
    editorViewAction_ = modeMenu->addAction(tr("3D View"), this, &MainWindow::switchToEditorView);
    editorViewAction_->setCheckable(true);
    editorViewAction_->setChecked(true);
    liveViewAction_ = modeMenu->addAction(tr("UE Live View"), this, &MainWindow::switchToLiveView);
    liveViewAction_->setCheckable(true);
    ckmMapViewAction_ = modeMenu->addAction(tr("CKM Map"), this, &MainWindow::switchToCkmMap);
    ckmMapViewAction_->setCheckable(true);
    modeGroup->addAction(editorViewAction_);
    modeGroup->addAction(liveViewAction_);
    modeGroup->addAction(ckmMapViewAction_);
    auto* liveMenu = viewMenu->addMenu(tr("AirSim Live View"));
    stopAirSimLiveViewAction_ = liveMenu->addAction(tr("Stop"), this, &MainWindow::stopAirSimLiveView);
    liveMenu->addAction(tr("Refresh Camera List"), this, &MainWindow::refreshAirSimCameraList);
    viewMenu->addSeparator();
    showRfDataWindowAction_ = viewMenu->addAction(tr("Wireless Data Monitor"), this, &MainWindow::showRfDataWindow);
    showSysDataWindowAction_ = viewMenu->addAction(tr("Sionna SYS Monitor"), this, &MainWindow::showSysDataWindow);
    showDeveloperToolsAction_ = viewMenu->addAction(tr("Developer Tools"), this, &MainWindow::showDeveloperToolsPanel);

    auto* stationMenu = menuBar()->addMenu(tr("Base Stations"));
    loadBaseStationsAction_ = stationMenu->addAction(tr("Load Base Stations JSON..."), this, &MainWindow::loadBaseStationsJson);
    saveBaseStationsAction_ = stationMenu->addAction(tr("Save Base Stations JSON"), this, &MainWindow::saveBaseStationsJson);
    saveBaseStationsAsAction_ = stationMenu->addAction(tr("Save Base Stations JSON As..."), this, &MainWindow::saveBaseStationsJsonAs);
    stationMenu->addSeparator();
    addBaseStationAction_ = stationMenu->addAction(tr("Add Base Station (next click in 3D)"), this, &MainWindow::enterAddBaseStationMode);
    deleteBaseStationAction_ = stationMenu->addAction(tr("Delete Selected Base Station"), this, &MainWindow::deleteSelectedBaseStation);

    auto* simMenu = menuBar()->addMenu(tr("Simulation"));
    simSettingsAction_ = simMenu->addAction(tr("Sionna Settings..."), this, &MainWindow::openSimulationSettings);
    coordinateTransformsAction_ = simMenu->addAction(tr("Coordinate Frames..."), this, &MainWindow::openCoordinateTransformSettings);
    showRosbagToolsAction_ = simMenu->addAction(tr("Rosbag Tools..."), this, &MainWindow::showRosbagToolsWindow);
}

void MainWindow::buildToolBar() {
    auto* toolbar = addToolBar(tr("Main"));
    toolbar->setMovable(false);
    toolbar->addAction(tr("Open Config"), this, &MainWindow::openGuiConfig);
    toolbar->addAction(tr("Save Config"), this, &MainWindow::saveGuiConfig);
    toolbar->addSeparator();
    toolbar->addAction(tr("Start Simulation"), this, &MainWindow::connectRos);
    toolbar->addAction(tr("Stop Simulation"), this, &MainWindow::disconnectRos);
    toolbar->addAction(tr("Simulation Settings"), this, &MainWindow::openSimulationSettings);
    toolbar->addAction(tr("Coordinate Frames"), this, &MainWindow::openCoordinateTransformSettings);
    toolbar->addSeparator();
    toolbar->addAction(tr("Load Scene Mesh"), this, &MainWindow::loadSceneMesh);
    toolbar->addAction(tr("Import OSM Area"), this, &MainWindow::importOsmScene);
    toolbar->addAction(tr("Clear Scene"), this, &MainWindow::clearSceneGeometry);
    toolbar->addSeparator();
    toolbar->addAction(tr("Wireless Data"), this, &MainWindow::showRfDataWindow);
    toolbar->addAction(tr("Sionna SYS"), this, &MainWindow::showSysDataWindow);
    toolbar->addAction(tr("Dev Tools"), this, &MainWindow::showDeveloperToolsPanel);
    toolbar->addAction(tr("Rosbag Tools"), this, &MainWindow::showRosbagToolsWindow);
    toolbar->addSeparator();
    toolbar->addAction(tr("Reset Camera"), sceneWidget_, &Scene3DWidget::resetCamera);
    if (airSimViewController_) {
        toolbar->addAction(tr("Reset Live Camera"), airSimViewController_, &AirSimViewController::resetCamera);
    }
    toolbar->addAction(tr("Reload Config"), this, &MainWindow::reloadConfigs);
}

void MainWindow::buildLeftDock() {
    layersDock_ = new QDockWidget(tr("Layers"), this);
    layersDock_->setObjectName(QStringLiteral("layersDock"));
    auto* layersPanel = new QWidget(layersDock_);
    auto* layersLayout = new QVBoxLayout(layersPanel);
    layersLayout->setContentsMargins(4, 4, 4, 4);
    layersLayout->setSpacing(6);

    layerTree_ = new QTreeWidget(layersPanel);
    layerTree_->setHeaderHidden(true);

    const std::vector<std::pair<QString, QString>> items = {
        {"axes", "3D Axes"},
        {"ground", "Ground"},
        {"buildings", "Buildings"},
        {"scene_mesh", "Imported Scene Mesh"},
        {"base_stations", "Base Stations"},
        {"drone", "Drone"},
        {"history", "Trajectory"},
        {"paths", "RF Paths"},
        {"view_gizmo", "View Drag Ball"},
    };

    for (const auto& kv : items) {
        auto* item = new QTreeWidgetItem(layerTree_);
        item->setText(0, kv.second);
        item->setData(0, Qt::UserRole, kv.first);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Checked);
    }

    connect(layerTree_, &QTreeWidget::itemChanged, this, &MainWindow::onLayerItemChanged);

    layersLayout->addWidget(layerTree_, 1);

    airsimSettingsSyncPanel_ = new QFrame(layersPanel);
    airsimSettingsSyncPanel_->setObjectName(QStringLiteral("airsimSettingsSyncPanel"));
    airsimSettingsSyncPanel_->setFrameShape(QFrame::StyledPanel);
    airsimSettingsSyncPanel_->setStyleSheet(QStringLiteral(
        "#airsimSettingsSyncPanel { background: #fff7ed; border: 1px solid #fdba74; border-radius: 4px; }"));
    auto* syncLayout = new QVBoxLayout(airsimSettingsSyncPanel_);
    syncLayout->setContentsMargins(8, 8, 8, 8);
    syncLayout->setSpacing(6);
    airsimSettingsSyncHintLabel_ = new QLabel(airsimSettingsSyncPanel_);
    airsimSettingsSyncHintLabel_->setWordWrap(true);
    syncAirSimSettingsButton_ = new QPushButton(tr("Sync AirSim settings.json"), airsimSettingsSyncPanel_);
    connect(syncAirSimSettingsButton_, &QPushButton::clicked, this, &MainWindow::syncAirSimSettingsExternalCameras);
    syncLayout->addWidget(airsimSettingsSyncHintLabel_);
    syncLayout->addWidget(syncAirSimSettingsButton_);
    airsimSettingsSyncPanel_->hide();
    layersLayout->addWidget(airsimSettingsSyncPanel_);

    layersDock_->setWidget(layersPanel);
    addDockWidget(Qt::LeftDockWidgetArea, layersDock_);
    lastLeftDock_ = layersDock_;
    refreshLayerPanelForCurrentView();
    refreshAirSimSettingsSyncStatus();
}

void MainWindow::buildRightDock() {
    auto* dock = new QDockWidget(tr("Controls / Live Info"), this);
    auto* container = new QWidget(dock);
    auto* rootLayout = new QVBoxLayout(container);
    rootLayout->setContentsMargins(4, 4, 4, 4);

    auto* controlsPane = new QWidget(container);
    auto* controlsLayout = new QVBoxLayout(controlsPane);

    auto* rosBox = new QGroupBox(tr("ROS1 Input Topics"), controlsPane);
    auto* rosLayout = new QFormLayout(rosBox);
    poseTopicCombo_ = new QComboBox(rosBox);
    trajectoryTopicCombo_ = new QComboBox(rosBox);
    poseTopicCombo_->setEditable(true);
    trajectoryTopicCombo_->setEditable(true);
    poseTopicCombo_->setInsertPolicy(QComboBox::NoInsert);
    trajectoryTopicCombo_->setInsertPolicy(QComboBox::NoInsert);
    poseTopicCombo_->setMinimumContentsLength(24);
    trajectoryTopicCombo_->setMinimumContentsLength(24);
    poseTopicCombo_->setEditText("/airsim_node/PX4/odom_local_ned");
    trajectoryTopicCombo_->setEditText(QString());
    if (trajectoryTopicCombo_->lineEdit()) {
        trajectoryTopicCombo_->lineEdit()->setPlaceholderText(tr("Leave empty to build trajectory internally from pose/odom"));
    }
    rosLayout->addRow(tr("Pose topic"), poseTopicCombo_);
    rosLayout->addRow(tr("Trajectory topic (optional)"), trajectoryTopicCombo_);
    poseTypeValue_ = new QLabel("<unknown>", rosBox);
    trajectoryTypeValue_ = new QLabel("<unknown>", rosBox);
    rfTopicValue_ = new QLabel(internalRfTopic_, rosBox);
    sysTopicValue_ = new QLabel(internalSysTopic_, rosBox);
    simulatorStatusValue_ = new QLabel("Stopped", rosBox);
    rosLayout->addRow(tr("Pose type"), poseTypeValue_);
    rosLayout->addRow(tr("Trajectory type"), trajectoryTypeValue_);
    rosLayout->addRow(tr("Internal RF topic"), rfTopicValue_);
    rosLayout->addRow(tr("Internal SYS topic"), sysTopicValue_);
    rosLayout->addRow(tr("Simulator"), simulatorStatusValue_);

    refreshTopicsButton_ = new QPushButton(tr("Refresh Topic List"), rosBox);
    connect(refreshTopicsButton_, &QPushButton::clicked, this, &MainWindow::refreshRosTopics);
    rosLayout->addRow(refreshTopicsButton_);

    connectButton_ = new QPushButton(tr("Start Simulation"), rosBox);
    disconnectButton_ = new QPushButton(tr("Stop Simulation"), rosBox);
    demoButton_ = new QPushButton(tr("Start Demo"), rosBox);
    stopDemoButton_ = new QPushButton(tr("Stop Demo"), rosBox);
    simSettingsButton_ = new QPushButton(tr("Sionna Settings"), rosBox);
    coordinateTransformsButton_ = new QPushButton(tr("Coordinate Frames"), rosBox);
    connect(connectButton_, &QPushButton::clicked, this, &MainWindow::connectRos);
    connect(disconnectButton_, &QPushButton::clicked, this, &MainWindow::disconnectRos);
    connect(demoButton_, &QPushButton::clicked, this, &MainWindow::startDemo);
    connect(stopDemoButton_, &QPushButton::clicked, this, &MainWindow::stopDemo);
    connect(simSettingsButton_, &QPushButton::clicked, this, &MainWindow::openSimulationSettings);
    connect(coordinateTransformsButton_, &QPushButton::clicked, this, &MainWindow::openCoordinateTransformSettings);
    connect(poseTopicCombo_, &QComboBox::editTextChanged, this, &MainWindow::updateSelectedTopicTypes);
    connect(trajectoryTopicCombo_, &QComboBox::editTextChanged, this, &MainWindow::updateSelectedTopicTypes);

    auto* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(connectButton_);
    buttonLayout->addWidget(disconnectButton_);
    rosLayout->addRow(buttonLayout);
    auto* buttonLayout2 = new QHBoxLayout();
    buttonLayout2->addWidget(demoButton_);
    buttonLayout2->addWidget(stopDemoButton_);
    rosLayout->addRow(buttonLayout2);
    auto* settingsRow = new QHBoxLayout();
    settingsRow->addWidget(simSettingsButton_);
    settingsRow->addWidget(coordinateTransformsButton_);
    rosLayout->addRow(settingsRow);

    rosbagRecordBox_ = new QGroupBox(tr("Rosbag Recording"), controlsPane);
    auto* rosbagRecordLayout = new QFormLayout(rosbagRecordBox_);
    rosbagRecordPathEdit_ = new QLineEdit(defaultRosbagOutputPath(QStringLiteral("session")), rosbagRecordBox_);
    auto* rosbagRecordBrowseButton = new QPushButton(tr("Browse..."), rosbagRecordBox_);
    connect(rosbagRecordBrowseButton, &QPushButton::clicked, this, &MainWindow::browseRosbagRecordOutput);
    auto* rosbagRecordPathRow = new QWidget(rosbagRecordBox_);
    auto* rosbagRecordPathRowLayout = new QHBoxLayout(rosbagRecordPathRow);
    rosbagRecordPathRowLayout->setContentsMargins(0, 0, 0, 0);
    rosbagRecordPathRowLayout->addWidget(rosbagRecordPathEdit_, 1);
    rosbagRecordPathRowLayout->addWidget(rosbagRecordBrowseButton);
    rosbagRecordStatusValue_ = new QLabel(tr("Idle"), rosbagRecordBox_);
    rosbagRecordStatusValue_->setWordWrap(true);
    rosbagRecordIncludeRfCheck_ = new QCheckBox(tr("Record RF topic"), rosbagRecordBox_);
    rosbagRecordIncludeSysCheck_ = new QCheckBox(tr("Record SYS topic"), rosbagRecordBox_);
    rosbagRecordIncludeBeamCheck_ = new QCheckBox(tr("Record Beam topics"), rosbagRecordBox_);
    rosbagRecordIncludeRfCheck_->setChecked(true);
    rosbagRecordIncludeSysCheck_->setChecked(true);
    rosbagRecordIncludeBeamCheck_->setChecked(true);
    rosbagRecordToggleButton_ = new QPushButton(tr("Start Recording"), rosbagRecordBox_);
    rosbagToolsButton_ = new QPushButton(tr("Open Rosbag Tools"), rosbagRecordBox_);
    connect(rosbagRecordToggleButton_, &QPushButton::clicked, this, &MainWindow::toggleRosbagRecording);
    connect(rosbagToolsButton_, &QPushButton::clicked, this, &MainWindow::showRosbagToolsWindow);
    auto* rosbagRecordButtonsRow = new QWidget(rosbagRecordBox_);
    auto* rosbagRecordButtonsLayout = new QHBoxLayout(rosbagRecordButtonsRow);
    rosbagRecordButtonsLayout->setContentsMargins(0, 0, 0, 0);
    rosbagRecordButtonsLayout->addWidget(rosbagRecordToggleButton_);
    rosbagRecordButtonsLayout->addWidget(rosbagToolsButton_);
    auto* rosbagRecordHint = new QLabel(tr("All non-wireless topics are always recorded. You can independently include or exclude the internal RF, SYS, and Beam topics for the output bag."), rosbagRecordBox_);
    rosbagRecordHint->setWordWrap(true);
    auto* rosbagRecordTopicRow = new QWidget(rosbagRecordBox_);
    auto* rosbagRecordTopicLayout = new QHBoxLayout(rosbagRecordTopicRow);
    rosbagRecordTopicLayout->setContentsMargins(0, 0, 0, 0);
    rosbagRecordTopicLayout->addWidget(rosbagRecordIncludeRfCheck_);
    rosbagRecordTopicLayout->addWidget(rosbagRecordIncludeSysCheck_);
    rosbagRecordTopicLayout->addWidget(rosbagRecordIncludeBeamCheck_);
    rosbagRecordTopicLayout->addStretch(1);
    rosbagRecordLayout->addRow(tr("Output bag"), rosbagRecordPathRow);
    rosbagRecordLayout->addRow(tr("Wireless topics"), rosbagRecordTopicRow);
    rosbagRecordLayout->addRow(tr("Status"), rosbagRecordStatusValue_);
    rosbagRecordLayout->addRow(rosbagRecordButtonsRow);
    rosbagRecordLayout->addRow(tr("Note"), rosbagRecordHint);

    auto* baseStationBox = new QGroupBox(tr("Base Stations"), controlsPane);
    auto* bsLayout = new QFormLayout(baseStationBox);
    baseStationFileValue_ = new QLabel(baseStationsConfigPath_, baseStationBox);
    baseStationFileValue_->setWordWrap(true);
    baseStationSelectionValue_ = new QLabel(tr("<none>"), baseStationBox);
    baseStationModeValue_ = new QLabel(tr("Select"), baseStationBox);
    bsLayout->addRow(tr("JSON file"), baseStationFileValue_);
    bsLayout->addRow(tr("Selected"), baseStationSelectionValue_);
    bsLayout->addRow(tr("Edit mode"), baseStationModeValue_);
    loadStationsButton_ = new QPushButton(tr("Load JSON"), baseStationBox);
    saveStationsButton_ = new QPushButton(tr("Save JSON"), baseStationBox);
    addStationButton_ = new QPushButton(tr("Add (next 3D click)"), baseStationBox);
    deleteStationButton_ = new QPushButton(tr("Delete selected"), baseStationBox);
    connect(loadStationsButton_, &QPushButton::clicked, this, &MainWindow::loadBaseStationsJson);
    connect(saveStationsButton_, &QPushButton::clicked, this, &MainWindow::saveBaseStationsJson);
    connect(addStationButton_, &QPushButton::clicked, this, &MainWindow::enterAddBaseStationMode);
    connect(deleteStationButton_, &QPushButton::clicked, this, &MainWindow::deleteSelectedBaseStation);

    stationIdEdit_ = new QLineEdit(baseStationBox);
    stationNameEdit_ = new QLineEdit(baseStationBox);
    stationPreviewCameraEdit_ = new QLineEdit(baseStationBox);
    stationPreviewRosTopicEdit_ = new QLineEdit(baseStationBox);
    for (auto* edit : {stationIdEdit_, stationNameEdit_, stationPreviewCameraEdit_, stationPreviewRosTopicEdit_}) {
        edit->setEnabled(false);
        connect(edit, &QLineEdit::editingFinished, this, &MainWindow::onStationDetailsEdited);
    }

    stationXSpin_ = new QDoubleSpinBox(baseStationBox);
    stationYSpin_ = new QDoubleSpinBox(baseStationBox);
    stationZSpin_ = new QDoubleSpinBox(baseStationBox);
    for (auto* spin : {stationXSpin_, stationYSpin_, stationZSpin_}) {
        spin->setRange(-1000000.0, 1000000.0);
        spin->setDecimals(3);
        spin->setSingleStep(1.0);
        spin->setKeyboardTracking(false);
        spin->setEnabled(false);
        spin->clear();
        QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onStationCoordinateEdited);
    }

    stationPreviewOffsetZSpin_ = new QDoubleSpinBox(baseStationBox);
    stationPreviewOffsetZSpin_->setRange(-1000.0, 1000.0);
    stationPreviewOffsetZSpin_->setDecimals(3);
    stationPreviewOffsetZSpin_->setSingleStep(0.5);
    stationPreviewOffsetZSpin_->setKeyboardTracking(false);
    stationPreviewOffsetZSpin_->setEnabled(false);
    connect(stationPreviewOffsetZSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onStationDetailsEdited);

    stationColorRSpin_ = new QDoubleSpinBox(baseStationBox);
    stationColorGSpin_ = new QDoubleSpinBox(baseStationBox);
    stationColorBSpin_ = new QDoubleSpinBox(baseStationBox);
    for (auto* spin : {stationColorRSpin_, stationColorGSpin_, stationColorBSpin_}) {
        spin->setRange(0.0, 1.0);
        spin->setDecimals(3);
        spin->setSingleStep(0.05);
        spin->setKeyboardTracking(false);
        spin->setEnabled(false);
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onStationDetailsEdited);
    }

    auto* bsRow1 = new QHBoxLayout();
    bsRow1->addWidget(loadStationsButton_);
    bsRow1->addWidget(saveStationsButton_);
    bsLayout->addRow(bsRow1);
    auto* bsRow2 = new QHBoxLayout();
    bsRow2->addWidget(addStationButton_);
    bsRow2->addWidget(deleteStationButton_);
    bsLayout->addRow(bsRow2);
    bsLayout->addRow(tr("ID"), stationIdEdit_);
    bsLayout->addRow(tr("Name"), stationNameEdit_);
    auto* coordRow = new QHBoxLayout();
    coordRow->addWidget(new QLabel(tr("X"), baseStationBox));
    coordRow->addWidget(stationXSpin_);
    coordRow->addWidget(new QLabel(tr("Y"), baseStationBox));
    coordRow->addWidget(stationYSpin_);
    coordRow->addWidget(new QLabel(tr("Z"), baseStationBox));
    coordRow->addWidget(stationZSpin_);
    bsLayout->addRow(tr("Coordinates"), coordRow);
    auto* colorRow = new QHBoxLayout();
    colorRow->addWidget(new QLabel(tr("R"), baseStationBox));
    colorRow->addWidget(stationColorRSpin_);
    colorRow->addWidget(new QLabel(tr("G"), baseStationBox));
    colorRow->addWidget(stationColorGSpin_);
    colorRow->addWidget(new QLabel(tr("B"), baseStationBox));
    colorRow->addWidget(stationColorBSpin_);
    bsLayout->addRow(tr("Color"), colorRow);
    bsLayout->addRow(tr("Preview camera"), stationPreviewCameraEdit_);
    bsLayout->addRow(tr("Preview ROS topic"), stationPreviewRosTopicEdit_);
    bsLayout->addRow(tr("Preview Z offset"), stationPreviewOffsetZSpin_);
    auto* stationHint = new QLabel(tr("Note: preview camera names here only write base_stations.json. Matching cameras still need to exist in AirSim settings.json."), baseStationBox);
    stationHint->setWordWrap(true);
    bsLayout->addRow(tr("AirSim note"), stationHint);

    airsimCacheCheck_ = nullptr;
    airsimCachePathEdit_ = nullptr;

    auto* osmBox = new QGroupBox(tr("OSM Search / Area Import"), controlsPane);
    auto* osmLayout = new QFormLayout(osmBox);
    osmSourceValue_ = new QLabel(tr("Search a place and drag a region on the map."), osmBox);
    osmSourceValue_->setWordWrap(true);
    osmSelectionValue_ = new QLabel(tr("No OSM area selected yet."), osmBox);
    osmSelectionValue_->setWordWrap(true);
    importOsmButton_ = new QPushButton(tr("Search && Import OSM Area"), osmBox);
    connect(importOsmButton_, &QPushButton::clicked, this, &MainWindow::importOsmScene);
    osmLayout->addRow(tr("Workflow"), osmSourceValue_);
    osmLayout->addRow(tr("Last selection"), osmSelectionValue_);
    osmLayout->addRow(importOsmButton_);

    auto* liveViewBox = new QGroupBox(tr("AirSim UE Live View (C++ RPC)"), controlsPane);
    auto* liveViewLayout = new QFormLayout(liveViewBox);
    airsimHostEdit_ = new QLineEdit("127.0.0.1", liveViewBox);
    airsimPortEdit_ = new QLineEdit("41451", liveViewBox);
    airsimLiveCameraCombo_ = new QComboBox(liveViewBox);
    airsimLiveCameraCombo_->setEditable(true);
    airsimLiveCameraCombo_->setInsertPolicy(QComboBox::NoInsert);
    airsimLiveCameraCombo_->setMinimumContentsLength(18);
    airsimLiveCameraCombo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    airsimLiveCameraCombo_->addItem(QStringLiteral("front_center"));
    airsimLiveVehicleEdit_ = new QLineEdit(QStringLiteral(""), liveViewBox);
    airsimLiveFollowCheck_ = new QCheckBox(tr("Follow vehicle focus"), liveViewBox);
    airsimLiveFollowCheck_->setChecked(true);
    airsimLiveFpsSpin_ = new QDoubleSpinBox(liveViewBox);
    airsimLiveFpsSpin_->setRange(1.0, static_cast<double>(kMaxGuiPreviewFps));
    airsimLiveFpsSpin_->setDecimals(1);
    airsimLiveFpsSpin_->setKeyboardTracking(false);
    airsimLiveFpsSpin_->setValue(static_cast<double>(kDefaultGuiPreviewFps));
    airsimLiveFpsSpin_->hide();
    auto* cameraRow = new QWidget(liveViewBox);
    auto* cameraRowLayout = new QHBoxLayout(cameraRow);
    cameraRowLayout->setContentsMargins(0, 0, 0, 0);
    cameraRowLayout->setSpacing(6);
    cameraRowLayout->addWidget(airsimLiveCameraCombo_, 1);
    auto* refreshCamerasButton = new QPushButton(tr("Refresh"), cameraRow);
    cameraRowLayout->addWidget(refreshCamerasButton);
    auto* settingsPathRow = new QWidget(liveViewBox);
    auto* settingsPathLayout = new QHBoxLayout(settingsPathRow);
    settingsPathLayout->setContentsMargins(0, 0, 0, 0);
    settingsPathLayout->setSpacing(6);
    airsimSettingsPathEdit_ = new QLineEdit(settingsPathRow);
    airsimSettingsPathEdit_->setPlaceholderText(tr("Auto-detect AirSim settings.json"));
    auto* browseSettingsButton = new QPushButton(tr("Browse..."), settingsPathRow);
    auto* autoSettingsButton = new QPushButton(tr("Auto"), settingsPathRow);
    settingsPathLayout->addWidget(airsimSettingsPathEdit_, 1);
    settingsPathLayout->addWidget(browseSettingsButton);
    settingsPathLayout->addWidget(autoSettingsButton);
    airsimSettingsPathValue_ = new QLabel(tr("No AirSim settings.json resolved yet."), liveViewBox);
    airsimSettingsPathValue_->setWordWrap(true);
    airsimSettingsPathValue_->setStyleSheet(QStringLiteral("color: #64748b;"));
    liveViewLayout->addRow(tr("Host"), airsimHostEdit_);
    liveViewLayout->addRow(tr("Port"), airsimPortEdit_);
    liveViewLayout->addRow(tr("Preview camera"), cameraRow);
    liveViewLayout->addRow(tr("Vehicle"), airsimLiveVehicleEdit_);

    auto* stationStreamBox = new QGroupBox(tr("Camera Streaming"), liveViewBox);
    auto* stationStreamLayout = new QFormLayout(stationStreamBox);
    stationCameraFpsTopicCombo_ = new QComboBox(stationStreamBox);
    stationCameraFpsTopicCombo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    stationCameraFpsTopicCombo_->setMinimumContentsLength(24);
    stationCameraFpsSpin_ = new QSpinBox(stationStreamBox);
    stationCameraFpsSpin_->setRange(1, kMaxCameraFps);
    stationCameraFpsSpin_->setKeyboardTracking(false);
    stationCameraFpsSpin_->setValue(kDefaultCameraFps);
    publishAllStationCameraTopicsButton_ = new QPushButton(tr("Publish All Topics"), stationStreamBox);
    stopStationCameraPublishingButton_ = new QPushButton(tr("Stop Publishing"), stationStreamBox);
    auto* stationPublishButtons = new QWidget(stationStreamBox);
    auto* stationPublishButtonsLayout = new QHBoxLayout(stationPublishButtons);
    stationPublishButtonsLayout->setContentsMargins(0, 0, 0, 0);
    stationPublishButtonsLayout->addWidget(publishAllStationCameraTopicsButton_);
    stationPublishButtonsLayout->addWidget(stopStationCameraPublishingButton_);
    auto* stationStreamHint = new QLabel(tr("Select GuiPreview, AirSim ROS image topics, or station camera topics here to set their FPS. Station topics still publish on demand: selected stations or topics with active ROS subscribers are enabled; Publish All Topics temporarily forces every station topic on."), stationStreamBox);
    stationStreamHint->setWordWrap(true);
    stationStreamLayout->addRow(tr("Camera/topic"), stationCameraFpsTopicCombo_);
    stationStreamLayout->addRow(tr("FPS"), stationCameraFpsSpin_);
    stationStreamLayout->addRow(tr("Manual"), stationPublishButtons);
    stationStreamLayout->addRow(tr("Behavior"), stationStreamHint);
    liveViewLayout->addRow(stationStreamBox);

    auto* brightnessRow = new QWidget(liveViewBox);
    auto* brightnessRowLayout = new QHBoxLayout(brightnessRow);
    brightnessRowLayout->setContentsMargins(0, 0, 0, 0);
    brightnessRowLayout->setSpacing(8);
    airsimBrightnessSlider_ = new QSlider(Qt::Horizontal, brightnessRow);
    airsimBrightnessSlider_->setRange(25, 300);
    airsimBrightnessSlider_->setSingleStep(5);
    airsimBrightnessSlider_->setPageStep(25);
    airsimBrightnessSlider_->setTickInterval(25);
    airsimBrightnessSlider_->setTickPosition(QSlider::TicksBelow);
    airsimBrightnessSlider_->setValue(100);
    airsimBrightnessValueLabel_ = new QLabel(tr("100%"), brightnessRow);
    airsimBrightnessValueLabel_->setMinimumWidth(56);
    airsimBrightnessValueLabel_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    brightnessRowLayout->addWidget(airsimBrightnessSlider_, 1);
    brightnessRowLayout->addWidget(airsimBrightnessValueLabel_, 0);
    liveViewLayout->addRow(tr("Brightness"), brightnessRow);
    liveViewLayout->addRow(QString(), airsimLiveFollowCheck_);
    liveViewLayout->addRow(tr("Settings file"), settingsPathRow);
    liveViewLayout->addRow(tr("Active settings"), airsimSettingsPathValue_);
    auto* liveButtonRow = new QHBoxLayout();
    auto* liveStartButton = new QPushButton(tr("Start Live View"), liveViewBox);
    auto* liveStopButton = new QPushButton(tr("Stop"), liveViewBox);
    auto* liveOpenButton = new QPushButton(tr("Open UE View"), liveViewBox);
    connect(airsimSettingsPathEdit_, &QLineEdit::editingFinished, this, &MainWindow::onAirSimSettingsPathEditingFinished);
    connect(browseSettingsButton, &QPushButton::clicked, this, &MainWindow::browseAirSimSettingsPath);
    connect(autoSettingsButton, &QPushButton::clicked, this, &MainWindow::useAutoAirSimSettingsPath);
    connect(liveStartButton, &QPushButton::clicked, this, &MainWindow::startAirSimLiveView);
    connect(liveStopButton, &QPushButton::clicked, this, &MainWindow::stopAirSimLiveView);
    connect(liveOpenButton, &QPushButton::clicked, this, &MainWindow::switchToLiveView);
    connect(publishAllStationCameraTopicsButton_, &QPushButton::clicked, this, &MainWindow::startPublishingAllStationCameraTopics);
    connect(stopStationCameraPublishingButton_, &QPushButton::clicked, this, &MainWindow::stopPublishingAllStationCameraTopics);
    liveButtonRow->addWidget(liveStartButton);
    liveButtonRow->addWidget(liveStopButton);
    liveButtonRow->addWidget(liveOpenButton);
    liveViewLayout->addRow(liveButtonRow);
    connect(refreshCamerasButton, &QPushButton::clicked, this, &MainWindow::refreshAirSimCameraList);
    connect(airsimHostEdit_, &QLineEdit::textChanged, this, &MainWindow::syncAirSimLiveViewSettings);
    connect(airsimPortEdit_, &QLineEdit::textChanged, this, &MainWindow::syncAirSimLiveViewSettings);
    connect(airsimLiveCameraCombo_, &QComboBox::editTextChanged, this, &MainWindow::syncAirSimLiveViewSettings);
    connect(airsimLiveVehicleEdit_, &QLineEdit::textChanged, this, &MainWindow::refreshAirSimCameraList);
    connect(airsimLiveVehicleEdit_, &QLineEdit::textChanged, this, &MainWindow::syncAirSimLiveViewSettings);
    connect(airsimLiveFollowCheck_, &QCheckBox::toggled, this, &MainWindow::syncAirSimLiveViewSettings);
    connect(airsimLiveFpsSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::syncAirSimLiveViewSettings);
    connect(stationCameraFpsTopicCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onStationCameraFpsTopicChanged);
    connect(stationCameraFpsSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onStationCameraFpsValueChanged);
    connect(airsimBrightnessSlider_, &QSlider::valueChanged, this, &MainWindow::onAirSimBrightnessSliderChanged);

    controlsLayout->addWidget(rosBox);
    controlsLayout->addWidget(rosbagRecordBox_);
    controlsLayout->addWidget(baseStationBox);
    controlsLayout->addWidget(osmBox);
    controlsLayout->addWidget(liveViewBox);
    controlsLayout->addStretch(1);

    auto* controlsScroll = new QScrollArea(container);
    controlsScroll->setWidgetResizable(true);
    controlsScroll->setWidget(controlsPane);

    liveStateBox_ = new QGroupBox(tr("Live State"), container);
    auto* liveLayout = new QVBoxLayout(liveStateBox_);
    auto* summaryBox = new QFrame(liveStateBox_);
    summaryBox->setFrameShape(QFrame::StyledPanel);
    auto* summaryLayout = new QFormLayout(summaryBox);
    summaryLayout->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
    modeValue_ = new QLabel("Idle", summaryBox);
    positionValue_ = new QLabel("-", summaryBox);
    attitudeValue_ = new QLabel("-", summaryBox);
    baseStationCountValue_ = new QLabel("0", summaryBox);
    pathCountValue_ = new QLabel("0", summaryBox);
    strongestPathValue_ = new QLabel("-", summaryBox);
    sysModeValue_ = new QLabel("Disabled", summaryBox);
    sysServingValue_ = new QLabel("-", summaryBox);
    sysLinkValue_ = new QLabel("-", summaryBox);
    sceneSourceValue_ = new QLabel("None", summaryBox);
    sceneSourceValue_->setWordWrap(true);
    sceneSourceValue_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    sceneSourceValue_->setMinimumWidth(0);
    summaryLayout->addRow(tr("Mode"), modeValue_);
    summaryLayout->addRow(tr("Scene source"), sceneSourceValue_);
    summaryLayout->addRow(tr("Drone position"), positionValue_);
    summaryLayout->addRow(tr("Attitude"), attitudeValue_);
    summaryLayout->addRow(tr("Base stations"), baseStationCountValue_);
    summaryLayout->addRow(tr("RF paths"), pathCountValue_);
    summaryLayout->addRow(tr("Strongest path"), strongestPathValue_);
    summaryLayout->addRow(tr("SYS over RT"), sysModeValue_);
    summaryLayout->addRow(tr("Serving BS"), sysServingValue_);
    summaryLayout->addRow(tr("Latest SYS link"), sysLinkValue_);
    liveLayout->addWidget(summaryBox);

    liveStateScroll_ = new QScrollArea(liveStateBox_);
    liveStateScroll_->setWidgetResizable(true);
    liveStateCardsContainer_ = new QWidget(liveStateScroll_);
    liveStateCardsLayout_ = new QVBoxLayout(liveStateCardsContainer_);
    liveStateCardsLayout_->setContentsMargins(4, 4, 4, 4);
    liveStateCardsLayout_->setSpacing(8);
    liveStateCardsLayout_->addStretch(1);
    liveStateScroll_->setWidget(liveStateCardsContainer_);
    liveLayout->addWidget(liveStateScroll_, 1);

    ckmDataBox_ = new QGroupBox(tr("CKM Data"), container);
    auto* ckmDataLayout = new QVBoxLayout(ckmDataBox_);
    ckmDataTitleValue_ = new QLabel(tr("No CKM point selected"), ckmDataBox_);
    ckmDataTitleValue_->setWordWrap(true);
    ckmDataTitleValue_->setStyleSheet(QStringLiteral("font-size: 16px; font-weight: 700;"));
    ckmDataSummaryValue_ = new QLabel(tr("Load a Dense CKM result, switch to the CKM Map page, then click the scene overlay to inspect the nearest offline sample."), ckmDataBox_);
    ckmDataSummaryValue_->setWordWrap(true);
    ckmDataSummaryValue_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ckmDataDetailsView_ = new QPlainTextEdit(ckmDataBox_);
    ckmDataDetailsView_->setReadOnly(true);
    ckmDataDetailsView_->setPlaceholderText(tr("Offline CKM values for the selected point will appear here."));
    ckmDataLayout->addWidget(ckmDataTitleValue_);
    ckmDataLayout->addWidget(ckmDataSummaryValue_);
    ckmDataLayout->addWidget(ckmDataDetailsView_, 1);
    ckmDataBox_->setVisible(false);

    auto* logBox = new QGroupBox(tr("Simulator Log"), container);
    auto* logLayout = new QVBoxLayout(logBox);
    simulatorLogView_ = new QPlainTextEdit(logBox);
    simulatorLogView_->setReadOnly(true);
    simulatorLogView_->setMaximumBlockCount(500);
    simulatorLogView_->setPlaceholderText(tr("Internal Sionna process output will appear here."));
    logLayout->addWidget(simulatorLogView_);

    rightPanelSplitter_ = new QSplitter(Qt::Vertical, container);
    rightPanelSplitter_->addWidget(controlsScroll);
    rightPanelSplitter_->addWidget(liveStateBox_);
    rightPanelSplitter_->addWidget(ckmDataBox_);
    rightPanelSplitter_->addWidget(logBox);
    rightPanelSplitter_->setStretchFactor(0, 0);
    rightPanelSplitter_->setStretchFactor(1, 1);
    rightPanelSplitter_->setStretchFactor(2, 1);
    rightPanelSplitter_->setStretchFactor(3, 1);
    rightPanelSplitter_->setSizes({340, 320, 320, 120});

    rootLayout->addWidget(rightPanelSplitter_);

    dock->setWidget(container);
    addDockWidget(Qt::RightDockWidgetArea, dock);
}

void MainWindow::buildDeveloperToolsDock() {
    if (developerToolsWindow_) {
        return;
    }

    developerToolsWindow_ = new QMainWindow(this, Qt::Window);
    developerToolsWindow_->setAttribute(Qt::WA_DeleteOnClose, false);
    developerToolsWindow_->setAttribute(Qt::WA_QuitOnClose, false);
    developerToolsWindow_->setWindowTitle(tr("Developer Tools"));
    developerToolsWindow_->resize(940, 620);

    auto* container = new QWidget(developerToolsWindow_);
    auto* layout = new QVBoxLayout(container);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);

    auto* title = new QLabel(tr("Developer Tools"), container);
    title->setStyleSheet(QStringLiteral("font-weight: 700; font-size: 15px;"));
    layout->addWidget(title);

    sionnaPreviewControlsBox_ = new QGroupBox(tr("Sionna Preview"), container);
    auto* sionnaPreviewLayout = new QFormLayout(sionnaPreviewControlsBox_);
    sionnaScenePathValue_ = new QLabel(tr("Use Simulation Settings -> Scene XML"), sionnaPreviewControlsBox_);
    sionnaScenePathValue_->setWordWrap(true);
    sionnaPreviewHintValue_ = new QLabel(tr("Opens the bundled Sionna RT preview in a separate window. Changes to the Scene XML or Python environment restart the preview process."), sionnaPreviewControlsBox_);
    sionnaPreviewHintValue_->setWordWrap(true);
    auto* sionnaPreviewButtons = new QWidget(sionnaPreviewControlsBox_);
    auto* sionnaPreviewButtonsLayout = new QHBoxLayout(sionnaPreviewButtons);
    sionnaPreviewButtonsLayout->setContentsMargins(0, 0, 0, 0);
    auto* openPreviewButton = new QPushButton(tr("Open Preview Window"), sionnaPreviewButtons);
    auto* restartPreviewButton = new QPushButton(tr("Restart Preview"), sionnaPreviewButtons);
    connect(openPreviewButton, &QPushButton::clicked, this, &MainWindow::switchToSionnaPreview);
    connect(restartPreviewButton, &QPushButton::clicked, this, &MainWindow::restartSionnaPreview);
    sionnaPreviewButtonsLayout->addWidget(openPreviewButton);
    sionnaPreviewButtonsLayout->addWidget(restartPreviewButton);
    sionnaPreviewButtonsLayout->addStretch(1);
    sionnaPreviewLayout->addRow(tr("Scene XML"), sionnaScenePathValue_);
    sionnaPreviewLayout->addRow(tr("Behavior"), sionnaPreviewHintValue_);
    sionnaPreviewLayout->addRow(sionnaPreviewButtons);
    layout->addWidget(sionnaPreviewControlsBox_);

    auto* rosTopicTitle = new QLabel(tr("ROS Topic Publishing Control"), container);
    rosTopicTitle->setStyleSheet(QStringLiteral("font-weight: 700;"));
    layout->addWidget(rosTopicTitle);

    auto* hint = new QLabel(tr("Choose which ROS topics should be allowed to publish. "
                               "The checkboxes do not take effect until you click Apply. "
                               "GUI-owned topics are stopped at the source; external topics such as AirSim/PX4 are gated through the ROS master publisher registry. "
                               "Already-open ROS connections may need a reconnect to fully observe external topic changes."), container);
    hint->setWordWrap(true);
    layout->addWidget(hint);

    rosTopicPublishModeValue_ = new QLabel(container);
    rosTopicPublishModeValue_->setWordWrap(true);
    layout->addWidget(rosTopicPublishModeValue_);

    rosTopicPublishTree_ = new QTreeWidget(container);
    rosTopicPublishTree_->setColumnCount(5);
    rosTopicPublishTree_->setHeaderLabels({tr("Publish"), tr("Topic"), tr("Type"), tr("Publishers"), tr("Control")});
    rosTopicPublishTree_->setRootIsDecorated(false);
    rosTopicPublishTree_->setAlternatingRowColors(true);
    rosTopicPublishTree_->setSelectionMode(QAbstractItemView::NoSelection);
    rosTopicPublishTree_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    rosTopicPublishTree_->setTextElideMode(Qt::ElideNone);
    auto* topicHeader = rosTopicPublishTree_->header();
    topicHeader->setStretchLastSection(false);
    topicHeader->setSectionsMovable(true);
    topicHeader->setMinimumSectionSize(60);
    topicHeader->setSectionResizeMode(QHeaderView::Interactive);
    topicHeader->resizeSection(0, 90);
    topicHeader->resizeSection(1, 640);
    topicHeader->resizeSection(2, 240);
    topicHeader->resizeSection(3, 220);
    topicHeader->resizeSection(4, 180);
    layout->addWidget(rosTopicPublishTree_, 1);

    auto* buttonRow = new QWidget(container);
    auto* buttonLayout = new QHBoxLayout(buttonRow);
    buttonLayout->setContentsMargins(0, 0, 0, 0);
    refreshRosTopicPublishingButton_ = new QPushButton(tr("Refresh Topics"), buttonRow);
    applyRosTopicPublishingButton_ = new QPushButton(tr("Apply Topic Publishing"), buttonRow);
    restoreDefaultTopicPublishingButton_ = new QPushButton(tr("Restore Default"), buttonRow);
    buttonLayout->addWidget(refreshRosTopicPublishingButton_);
    buttonLayout->addStretch(1);
    buttonLayout->addWidget(applyRosTopicPublishingButton_);
    buttonLayout->addWidget(restoreDefaultTopicPublishingButton_);
    layout->addWidget(buttonRow);

    connect(refreshRosTopicPublishingButton_, &QPushButton::clicked, this, &MainWindow::refreshDeveloperTopicList);
    connect(applyRosTopicPublishingButton_, &QPushButton::clicked, this, &MainWindow::applyDeveloperTopicPublishing);
    connect(restoreDefaultTopicPublishingButton_, &QPushButton::clicked, this, &MainWindow::restoreDefaultTopicPublishing);

    developerToolsWindow_->setCentralWidget(container);
    syncDeveloperTopicPublishingControls();
}

void MainWindow::showDeveloperToolsPanel() {
    if (!developerToolsWindow_) {
        buildDeveloperToolsDock();
    }
    syncDeveloperTopicPublishingControls();
    developerToolsWindow_->show();
    developerToolsWindow_->raise();
    developerToolsWindow_->activateWindow();
}

void MainWindow::refreshDeveloperTopicList() {
    refreshRosTopics();
    syncDeveloperTopicPublishingControls();
}

void MainWindow::syncDeveloperTopicPublishingControls() {
    if (!rosTopicPublishTree_) {
        return;
    }

    const QMap<QString, QString> topicTypes = rosPublishedTopicTypes();
    const QMap<QString, QStringList> publishersByTopic = rosPublisherNodesByTopic();

    struct TopicRow {
        QString topic;
        QString type;
        QString publishers;
        QString control;
        bool localGuiTopic{false};
        bool checked{true};
    };

    QMap<QString, TopicRow> rows;
    for (const auto& station : stations_) {
        const QString topic = station.previewRosTopic.trimmed().isEmpty()
            ? defaultPreviewRosTopicForStation(station)
            : station.previewRosTopic.trimmed();
        if (isIgnoredDeveloperRosTopic(topic)) {
            continue;
        }
        TopicRow row;
        row.topic = topic;
        row.type = topicTypes.value(topic, QStringLiteral("sensor_msgs/Image"));
        row.publishers = publishersByTopic.value(topic).join(QStringLiteral(", "));
        row.control = tr("GUI source");
        row.localGuiTopic = true;
        row.checked = manualRosTopicPublishingEnabled_
            ? manualRosTopicPublishing_.value(topic, false)
            : forceAllStationCameraPublishing_;
        rows.insert(topic, row);
    }

    for (auto it = topicTypes.constBegin(); it != topicTypes.constEnd(); ++it) {
        const QString topic = it.key();
        if (isIgnoredDeveloperRosTopic(topic)) {
            continue;
        }
        TopicRow row = rows.value(topic);
        row.topic = topic;
        row.type = it.value();
        row.publishers = publishersByTopic.value(topic).join(QStringLiteral(", "));
        if (!row.localGuiTopic) {
            row.control = tr("ROS master gate");
            row.checked = manualRosTopicPublishingEnabled_
                ? manualRosTopicPublishing_.value(topic, !suppressedRosPublisherRegistrations_.contains(topic))
                : !suppressedRosPublisherRegistrations_.contains(topic);
        }
        rows.insert(topic, row);
    }

    for (auto it = suppressedRosPublisherRegistrations_.constBegin(); it != suppressedRosPublisherRegistrations_.constEnd(); ++it) {
        const QString topic = it.key();
        if (isIgnoredDeveloperRosTopic(topic) || rows.contains(topic)) {
            continue;
        }
        TopicRow row;
        row.topic = topic;
        row.type = it.value().empty() ? QString() : it.value().front().topicType;
        QStringList publishers;
        for (const RosPublisherRegistration& registration : it.value()) {
            publishers << registration.nodeName;
        }
        publishers.removeDuplicates();
        row.publishers = publishers.join(QStringLiteral(", "));
        row.control = tr("ROS master gate (disabled)");
        row.localGuiTopic = false;
        row.checked = manualRosTopicPublishingEnabled_
            ? manualRosTopicPublishing_.value(topic, false)
            : false;
        rows.insert(topic, row);
    }

    QSet<QString> currentTopics;
    for (auto it = rows.constBegin(); it != rows.constEnd(); ++it) {
        currentTopics.insert(it.key());
    }
    for (auto it = manualRosTopicPublishing_.begin(); it != manualRosTopicPublishing_.end(); ) {
        if (!currentTopics.contains(it.key())) {
            it = manualRosTopicPublishing_.erase(it);
        } else {
            ++it;
        }
    }

    QSignalBlocker blocker(rosTopicPublishTree_);
    rosTopicPublishTree_->clear();

    int enabledCount = 0;
    for (auto it = rows.constBegin(); it != rows.constEnd(); ++it) {
        const TopicRow& row = it.value();
        if (row.checked) {
            ++enabledCount;
        }
        auto* item = new QTreeWidgetItem(rosTopicPublishTree_);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, row.checked ? Qt::Checked : Qt::Unchecked);
        item->setText(0, tr("Publish"));
        item->setText(1, row.topic);
        item->setText(2, row.type.isEmpty() ? tr("<unknown>") : row.type);
        item->setText(3, row.publishers.isEmpty() ? tr("<none>") : row.publishers);
        item->setText(4, row.control);
        item->setToolTip(1, row.topic);
        item->setToolTip(2, row.type);
        item->setToolTip(3, row.publishers);
        item->setData(0, Qt::UserRole, row.topic);
        item->setData(0, Qt::UserRole + 1, row.localGuiTopic);
    }

    if (rosTopicPublishModeValue_) {
        QString modeText;
        if (manualRosTopicPublishingEnabled_) {
            modeText = tr("Mode: Manual override active. %1/%2 topics are enabled.")
                .arg(enabledCount)
                .arg(rosTopicPublishTree_->topLevelItemCount());
        } else if (forceAllStationCameraPublishing_) {
            modeText = tr("Mode: Publish All station-camera topics is active; external topics are otherwise default.");
        } else {
            modeText = tr("Mode: Default. GUI station-camera topics publish on demand; external topics follow their own publisher nodes.");
        }
        if (!ros::isInitialized() || !ros::master::check()) {
            modeText += tr(" ROS master is not available, so only configured GUI topics are listed.");
        }
        rosTopicPublishModeValue_->setText(modeText);
    }

    const bool hasTopics = rosTopicPublishTree_->topLevelItemCount() > 0;
    if (applyRosTopicPublishingButton_) {
        applyRosTopicPublishingButton_->setEnabled(hasTopics);
    }
    if (restoreDefaultTopicPublishingButton_) {
        restoreDefaultTopicPublishingButton_->setEnabled(manualRosTopicPublishingEnabled_
                                                         || forceAllStationCameraPublishing_
                                                         || !suppressedRosPublisherRegistrations_.empty());
    }
}

void MainWindow::applyDeveloperTopicPublishing() {
    if (!rosTopicPublishTree_) {
        return;
    }

    QMap<QString, bool> requestedPublishing;
    QStringList checkedStationTopics;
    QStringList disabledExternalTopics;
    QStringList restoredExternalTopics;
    for (int i = 0; i < rosTopicPublishTree_->topLevelItemCount(); ++i) {
        QTreeWidgetItem* item = rosTopicPublishTree_->topLevelItem(i);
        if (!item) {
            continue;
        }
        const QString topic = item->data(0, Qt::UserRole).toString().trimmed();
        if (topic.isEmpty()) {
            continue;
        }
        const bool enabled = item->checkState(0) == Qt::Checked;
        requestedPublishing.insert(topic, enabled);
        if (item->data(0, Qt::UserRole + 1).toBool()) {
            if (enabled) {
                checkedStationTopics << topic;
            }
            continue;
        }
        if (enabled) {
            restoredExternalTopics << topic;
        } else {
            disabledExternalTopics << topic;
        }
    }

    const bool mainLiveViewConnected = airSimViewController_
        && airSimViewController_->isRunning()
        && airSimViewController_->isConnected();
    if (!checkedStationTopics.isEmpty() && !mainLiveViewConnected) {
        QMessageBox::information(this,
                                 tr("Developer Tools"),
                                 tr("Connect to AirSim first: click Start Live View and wait for the connection to succeed before applying manual publishing settings that include station camera topics."));
        return;
    }

    QStringList errors;
    for (const QString& topic : restoredExternalTopics) {
        QString error;
        if (!restoreRosTopicPublisherRegistrations(topic, &error) && !error.trimmed().isEmpty()) {
            errors << error;
        }
    }
    for (const QString& topic : disabledExternalTopics) {
        QString error;
        if (!suppressRosTopicPublisherRegistrations(topic, &error) && !error.trimmed().isEmpty()) {
            errors << error;
        }
    }

    manualRosTopicPublishing_ = requestedPublishing;
    manualRosTopicPublishingEnabled_ = true;
    forceAllStationCameraPublishing_ = false;
    refreshStationCameraStreamingState();
    syncDeveloperTopicPublishingControls();
    int enabledCount = 0;
    for (auto it = requestedPublishing.constBegin(); it != requestedPublishing.constEnd(); ++it) {
        if (it.value()) {
            ++enabledCount;
        }
    }
    onStatusMessage(tr("Applied manual ROS topic publishing: %1 topics enabled.").arg(enabledCount));
    if (!errors.isEmpty()) {
        QMessageBox::warning(this, tr("Developer Tools"),
                             tr("Some external topic operations could not be completed:\n\n%1")
                                 .arg(errors.join(QStringLiteral("\n"))));
    }
}

void MainWindow::restoreDefaultTopicPublishing() {
    manualRosTopicPublishingEnabled_ = false;
    manualRosTopicPublishing_.clear();
    forceAllStationCameraPublishing_ = false;
    restoreAllRosTopicPublisherRegistrations();
    refreshStationCameraStreamingState();
    syncDeveloperTopicPublishingControls();
    onStatusMessage(tr("Restored default ROS topic publishing behavior."));
}

bool MainWindow::suppressRosTopicPublisherRegistrations(const QString& topicName, QString* errorMessage) {
    if (errorMessage) {
        errorMessage->clear();
    }
    const QString topic = topicName.trimmed();
    if (topic.isEmpty() || isIgnoredDeveloperRosTopic(topic)) {
        return true;
    }
    if (suppressedRosPublisherRegistrations_.contains(topic)) {
        return true;
    }
    if (!ros::isInitialized() || !ros::master::check()) {
        if (errorMessage) {
            *errorMessage = tr("%1: ROS master is not available.").arg(topic);
        }
        return false;
    }

    const QMap<QString, QString> topicTypes = rosPublishedTopicTypes();
    const QMap<QString, QStringList> publishersByTopic = rosPublisherNodesByTopic();
    const QString topicType = topicTypes.value(topic).trimmed();
    const QStringList publishers = publishersByTopic.value(topic);
    if (publishers.isEmpty()) {
        return true;
    }

    std::vector<RosPublisherRegistration> registrations;
    QStringList errors;
    for (const QString& nodeNameRaw : publishers) {
        const QString nodeName = nodeNameRaw.trimmed();
        if (nodeName.isEmpty()) {
            continue;
        }
        const QString nodeUri = rosNodeUri(nodeName);
        if (nodeUri.isEmpty()) {
            errors << tr("%1: could not resolve node URI for %2.").arg(topic, nodeName);
            continue;
        }

        XmlRpc::XmlRpcValue request;
        request[0] = nodeName.toStdString();
        request[1] = topic.toStdString();
        request[2] = nodeUri.toStdString();
        XmlRpc::XmlRpcValue response;
        XmlRpc::XmlRpcValue payload;
        if (!ros::master::execute("unregisterPublisher", request, response, payload, false)) {
            errors << tr("%1: failed to unregister publisher %2.").arg(topic, nodeName);
            continue;
        }

        RosPublisherRegistration registration;
        registration.topicName = topic;
        registration.topicType = topicType;
        registration.nodeName = nodeName;
        registration.nodeUri = nodeUri;
        registrations.push_back(registration);
    }

    if (!registrations.empty()) {
        suppressedRosPublisherRegistrations_.insert(topic, registrations);
    }
    if (!errors.isEmpty() && errorMessage) {
        *errorMessage = errors.join(QStringLiteral(" "));
    }
    return errors.isEmpty();
}

bool MainWindow::restoreRosTopicPublisherRegistrations(const QString& topicName, QString* errorMessage) {
    if (errorMessage) {
        errorMessage->clear();
    }
    const QString topic = topicName.trimmed();
    if (topic.isEmpty() || !suppressedRosPublisherRegistrations_.contains(topic)) {
        return true;
    }
    if (!ros::isInitialized() || !ros::master::check()) {
        if (errorMessage) {
            *errorMessage = tr("%1: ROS master is not available.").arg(topic);
        }
        return false;
    }

    const std::vector<RosPublisherRegistration> registrations = suppressedRosPublisherRegistrations_.value(topic);
    QStringList errors;
    for (const RosPublisherRegistration& registration : registrations) {
        if (registration.nodeName.trimmed().isEmpty()
            || registration.nodeUri.trimmed().isEmpty()
            || registration.topicType.trimmed().isEmpty()) {
            errors << tr("%1: stored publisher registration is incomplete.").arg(topic);
            continue;
        }

        XmlRpc::XmlRpcValue request;
        request[0] = registration.nodeName.toStdString();
        request[1] = topic.toStdString();
        request[2] = registration.topicType.toStdString();
        request[3] = registration.nodeUri.toStdString();
        XmlRpc::XmlRpcValue response;
        XmlRpc::XmlRpcValue payload;
        if (!ros::master::execute("registerPublisher", request, response, payload, false)) {
            errors << tr("%1: failed to restore publisher %2.").arg(topic, registration.nodeName);
        }
    }

    if (errors.isEmpty()) {
        suppressedRosPublisherRegistrations_.remove(topic);
    }
    if (!errors.isEmpty() && errorMessage) {
        *errorMessage = errors.join(QStringLiteral(" "));
    }
    return errors.isEmpty();
}

void MainWindow::restoreAllRosTopicPublisherRegistrations(bool showWarnings) {
    const QStringList topics = suppressedRosPublisherRegistrations_.keys();
    QStringList errors;
    for (const QString& topic : topics) {
        QString error;
        if (!restoreRosTopicPublisherRegistrations(topic, &error) && !error.trimmed().isEmpty()) {
            errors << error;
        }
    }
    if (showWarnings && !errors.isEmpty()) {
        QMessageBox::warning(this, tr("Developer Tools"),
                             tr("Some external topic registrations could not be restored:\n\n%1")
                                 .arg(errors.join(QStringLiteral("\n"))));
    }
}



double MainWindow::currentAirSimBrightnessFactor() const {
    const int percent = airsimBrightnessSlider_ ? airsimBrightnessSlider_->value() : 100;
    return static_cast<double>(std::max(25, std::min(300, percent))) / 100.0;
}

void MainWindow::onAirSimBrightnessSliderChanged(int value) {
    const int percent = std::max(25, std::min(300, value));
    if (airsimBrightnessValueLabel_) {
        airsimBrightnessValueLabel_->setText(tr("%1%").arg(percent));
    }
    if (airSimViewController_) {
        airSimViewController_->setDisplayBrightness(static_cast<double>(percent) / 100.0);
    }
    for (auto& window : stationCameraWindows_) {
        if (window.view && window.view->controller()) {
            window.view->controller()->setDisplayBrightness(static_cast<double>(percent) / 100.0);
        }
    }
    QSettings settings(QStringLiteral("OpenAI"), QStringLiteral("airsim_gui_UErealtime"));
    settings.setValue(QStringLiteral("liveView/displayBrightnessPercent"), percent);
}

void MainWindow::syncAirSimLiveViewSettings() {
    if (!airSimViewController_) {
        return;
    }
    const QString host = airsimHostEdit_ ? airsimHostEdit_->text().trimmed() : QStringLiteral("127.0.0.1");
    const int port = airsimPortEdit_ ? airsimPortEdit_->text().trimmed().toInt() : 41451;
    bool stationEndpointChanged = false;
    for (const auto& window : stationCameraWindows_) {
        if (!window.view || !window.view->controller()) {
            continue;
        }
        AirSimViewController* stationController = window.view->controller();
        if (stationController->host() != host || stationController->port() != port) {
            stationEndpointChanged = true;
            break;
        }
    }

    airSimViewController_->setHost(host);
    airSimViewController_->setPort(port);
    airSimViewController_->setCameraName(airsimLiveCameraCombo_ ? airsimLiveCameraCombo_->currentText().trimmed() : QStringLiteral("front_center"));
    airSimViewController_->setVehicleName(airsimLiveVehicleEdit_ ? airsimLiveVehicleEdit_->text().trimmed() : QString());
    airSimViewController_->setFollowVehicle(airsimLiveFollowCheck_ ? airsimLiveFollowCheck_->isChecked() : true);
    airSimViewController_->setFramesPerSecond(airsimLiveFpsSpin_ ? airsimLiveFpsSpin_->value() : static_cast<double>(kDefaultGuiPreviewFps));
    airSimViewController_->setDepthFetchEnabled(false);
    airSimViewController_->setDisplayBrightness(currentAirSimBrightnessFactor());
    airSimViewController_->setPythonExecutable(simSettings_.pythonExecutable);
    if (stationEndpointChanged) {
        syncStationCameraWindows();
    }
    refreshAirSimSettingsSyncStatus();
}

void MainWindow::onStationCameraFpsTopicChanged(int index) {
    if (suppressStationCameraFpsControls_ || !stationCameraFpsTopicCombo_ || !stationCameraFpsSpin_) {
        return;
    }

    const QString controlKey = stationCameraFpsTopicCombo_->itemData(index, Qt::UserRole).toString().trimmed();
    const QString rosTopic = stationCameraFpsTopicCombo_->itemData(index, Qt::UserRole + 1).toString().trimmed();
    const int fpsValue = clampCameraFpsForControlKey(controlKey, cameraFpsForControl(controlKey, rosTopic));

    QSignalBlocker spinBlocker(stationCameraFpsSpin_);
    suppressStationCameraFpsControls_ = true;
    stationCameraFpsSpin_->setEnabled(!controlKey.isEmpty());
    stationCameraFpsSpin_->setRange(1, maxCameraFpsForControlKey(controlKey));
    stationCameraFpsSpin_->setValue(fpsValue);
    suppressStationCameraFpsControls_ = false;
    applyCameraFpsForControl(controlKey, rosTopic, fpsValue, false);
}

void MainWindow::onStationCameraFpsValueChanged(int value) {
    if (suppressStationCameraFpsControls_ || !stationCameraFpsTopicCombo_) {
        return;
    }

    const QString controlKey = stationCameraFpsTopicCombo_->currentData(Qt::UserRole).toString().trimmed();
    const QString rosTopic = stationCameraFpsTopicCombo_->currentData(Qt::UserRole + 1).toString().trimmed();
    applyCameraFpsForControl(controlKey, rosTopic, value, true);
}

void MainWindow::startPublishingAllStationCameraTopics() {
    if (!ros::isInitialized() || !ros::master::check()) {
        QMessageBox::information(this,
                                 tr("Station Camera Topics"),
                                 tr("Start ROS master and airsim_node_ex before publishing all station camera topics."));
        return;
    }

    if (stations_.empty()) {
        QMessageBox::information(this,
                                 tr("Station Camera Topics"),
                                 tr("No base stations are loaded, so there are no station camera topics to publish."));
        return;
    }

    forceAllStationCameraPublishing_ = true;
    manualRosTopicPublishingEnabled_ = false;
    manualRosTopicPublishing_.clear();
    refreshStationCameraStreamingState();
    syncDeveloperTopicPublishingControls();
    onStatusMessage(tr("Requesting all base-station camera topics for this session."));
}

void MainWindow::stopPublishingAllStationCameraTopics() {
    forceAllStationCameraPublishing_ = false;
    refreshStationCameraStreamingState();
    syncDeveloperTopicPublishingControls();
    onStatusMessage(tr("Stopped forced publishing of all base-station camera topics."));
}

void MainWindow::startAirSimLiveView() {
    reconnectAirSimLiveViewOnReturn_ = false;
    refreshAirSimCameraList();
    syncAirSimLiveViewSettings();
    if (airSimViewController_) {
        airSimViewController_->start();
        switchToLiveView();
    }
}

void MainWindow::stopAirSimLiveView() {
    reconnectAirSimLiveViewOnReturn_ = false;
    if (airSimViewController_) {
        airSimViewController_->stop();
    }
    refreshStationCameraStreamingState();
    syncDeveloperTopicPublishingControls();
}

void MainWindow::restartAirSimLiveView() {
    reconnectAirSimLiveViewOnReturn_ = false;
    refreshAirSimCameraList();
    syncAirSimLiveViewSettings();
    if (airSimViewController_) {
        airSimViewController_->restart();
        switchToLiveView();
    }
}

void MainWindow::switchToEditorView() {
    if (centerViewTabs_) {
        centerViewTabs_->setCurrentWidget(sceneWidget_);
    }
    if (editorViewAction_) editorViewAction_->setChecked(true);
}

void MainWindow::switchToLiveView() {
    if (centerViewTabs_) {
        centerViewTabs_->setCurrentWidget(ueViewWidget_);
    }
    if (liveViewAction_) liveViewAction_->setChecked(true);
}

void MainWindow::switchToSionnaPreview() {
    if (!sionnaPreviewWindow_ || !sionnaPreviewWidget_) {
        return;
    }
    syncSionnaPreviewSettings();
    sionnaPreviewWidget_->setPreviewActive(true);
    sionnaPreviewWindow_->show();
    sionnaPreviewWindow_->raise();
    sionnaPreviewWindow_->activateWindow();
}

void MainWindow::switchToCkmMap() {
    if (centerViewTabs_ && ckmMapWidget_) {
        centerViewTabs_->setCurrentWidget(ckmMapWidget_);
    }
    if (ckmMapViewAction_) ckmMapViewAction_->setChecked(true);
}

void MainWindow::onCenterViewTabChanged(int index) {
    Q_UNUSED(index);
    const QWidget* current = centerViewTabs_ ? centerViewTabs_->currentWidget() : nullptr;
    const bool isEditor = current == sceneWidget_;
    const bool isLive = current == ueViewWidget_;
    const bool isCkm = current == ckmMapWidget_;
    const bool wasLive = liveViewTabActive_;

    liveViewTabActive_ = isLive;

    if (editorViewAction_) editorViewAction_->setChecked(isEditor);
    if (liveViewAction_) liveViewAction_->setChecked(isLive);
    if (ckmMapViewAction_) ckmMapViewAction_->setChecked(isCkm);
    refreshRightPanelMode();
    refreshLayerPanelForCurrentView();
    refreshCameraImagePreviews();

    if (!airSimViewController_) {
        return;
    }

    if (wasLive && !isLive) {
        reconnectAirSimLiveViewOnReturn_ = airSimViewController_->isRunning() && airSimViewController_->isConnected();
        if (airSimViewController_->isRunning()) {
            airSimViewController_->stop();
        }
        return;
    }

    if (!wasLive && isLive && reconnectAirSimLiveViewOnReturn_) {
        reconnectAirSimLiveViewOnReturn_ = false;
        refreshAirSimCameraList();
        syncAirSimLiveViewSettings();
        airSimViewController_->start();
    }
}

void MainWindow::restartSionnaPreview() {
    if (sionnaPreviewWidget_) {
        sionnaPreviewWidget_->restartPreview();
        switchToSionnaPreview();
    }
}

void MainWindow::syncSionnaPreviewSettings() {
    if (sionnaScenePathValue_) {
        sionnaScenePathValue_->setText(simSettings_.scenePath.trimmed().isEmpty()
            ? tr("Use Simulation Settings -> Scene XML")
            : simSettings_.scenePath);
    }
    if (!sionnaPreviewWidget_) {
        return;
    }
    sionnaPreviewWidget_->setPythonExecutable(simSettings_.pythonExecutable);
    sionnaPreviewWidget_->setScenePath(simSettings_.scenePath);
}

QString MainWindow::selectedAirSimSettingsPath() const {
    const QString rawPath = airsimSettingsPathEdit_ ? airsimSettingsPathEdit_->text().trimmed() : QString();
    if (rawPath.isEmpty()) {
        return QString();
    }
    return configRelativePathToAbsolute(rawPath, currentGuiConfigPath_);
}

void MainWindow::setSelectedAirSimSettingsPath(const QString& path) {
    if (!airsimSettingsPathEdit_) {
        return;
    }
    const QString trimmed = path.trimmed();
    if (airsimSettingsPathEdit_->text() == trimmed) {
        airsimSettingsPathEdit_->setModified(false);
        return;
    }
    const QSignalBlocker blocker(airsimSettingsPathEdit_);
    airsimSettingsPathEdit_->setText(trimmed);
    airsimSettingsPathEdit_->setModified(false);
}

QStringList MainWindow::effectiveAirSimSettingsPaths() const {
    const QString selectedPath = selectedAirSimSettingsPath();
    if (!selectedPath.isEmpty()) {
        return {selectedPath};
    }
    return candidateAirSimSettingsPaths();
}

void MainWindow::browseAirSimSettingsPath() {
    const QString selectedPath = selectedAirSimSettingsPath();
    const QString startDir = selectedPath.isEmpty()
        ? QDir::home().filePath(QStringLiteral("Documents/AirSim"))
        : QFileInfo(selectedPath).absolutePath();
    const QString path = QFileDialog::getOpenFileName(this,
                                                      tr("Select AirSim settings.json"),
                                                      startDir,
                                                      tr("JSON (*.json)"));
    if (path.isEmpty()) {
        return;
    }
    setSelectedAirSimSettingsPath(QFileInfo(path).absoluteFilePath());
    markGuiConfigDirty();
    refreshAirSimCameraList();
    refreshStationCameraStreamingState();
    syncDeveloperTopicPublishingControls();
}

void MainWindow::useAutoAirSimSettingsPath() {
    if (!airsimSettingsPathEdit_ || airsimSettingsPathEdit_->text().trimmed().isEmpty()) {
        return;
    }
    setSelectedAirSimSettingsPath(QString());
    markGuiConfigDirty();
    refreshAirSimCameraList();
    refreshStationCameraStreamingState();
    syncDeveloperTopicPublishingControls();
}

void MainWindow::onAirSimSettingsPathEditingFinished() {
    if (!airsimSettingsPathEdit_) {
        return;
    }
    const bool changed = airsimSettingsPathEdit_->isModified();
    const QString trimmed = airsimSettingsPathEdit_->text().trimmed();
    if (airsimSettingsPathEdit_->text() != trimmed) {
        const QSignalBlocker blocker(airsimSettingsPathEdit_);
        airsimSettingsPathEdit_->setText(trimmed);
    }
    airsimSettingsPathEdit_->setModified(false);
    if (!changed) {
        return;
    }
    markGuiConfigDirty();
    refreshAirSimCameraList();
    refreshStationCameraStreamingState();
    syncDeveloperTopicPublishingControls();
}

void MainWindow::refreshAirSimCameraList() {
    if (!airsimLiveCameraCombo_) {
        return;
    }

    const QString currentSelection = airsimLiveCameraCombo_->currentText().trimmed();
    const QString preferredVehicle = airsimLiveVehicleEdit_ ? airsimLiveVehicleEdit_->text().trimmed() : QString();

    QString selectedPath;
    QStringList names;
    QString errorMessage;
    const QString selectedSettingsPath = selectedAirSimSettingsPath();
    const bool hasSelectedSettingsPath = !selectedSettingsPath.isEmpty();
    const QStringList candidates = effectiveAirSimSettingsPaths();
    for (const QString& candidate : candidates) {
        if (!QFileInfo::exists(candidate)) {
            if (hasSelectedSettingsPath && errorMessage.isEmpty()) {
                errorMessage = tr("Selected AirSim settings.json does not exist: %1").arg(candidate);
            }
            continue;
        }
        QString parseError;
        const QStringList parsed = parseAirSimCameraNames(candidate, preferredVehicle, &parseError);
        if (!parsed.isEmpty()) {
            selectedPath = QFileInfo(candidate).absoluteFilePath();
            names = parsed;
            break;
        }
        if (errorMessage.isEmpty()) {
            errorMessage = parseError;
        }
    }

    if (names.isEmpty() && !currentSelection.isEmpty()) {
        names << currentSelection;
    }
    if (names.isEmpty()) {
        names << QStringLiteral("front_center");
    }

    QSignalBlocker blocker(airsimLiveCameraCombo_);
    airsimLiveCameraCombo_->clear();
    airsimLiveCameraCombo_->addItems(names);
    const bool keepManualSelection = !currentSelection.isEmpty()
        && currentSelection != QStringLiteral("front_center")
        && currentSelection != QStringLiteral("front_camera");
    if (keepManualSelection) {
        airsimLiveCameraCombo_->setCurrentText(currentSelection);
    } else if (!names.isEmpty()) {
        airsimLiveCameraCombo_->setCurrentIndex(0);
    }

    if (airsimSettingsPathValue_) {
        if (!selectedPath.isEmpty()) {
            airsimSettingsPathValue_->setText(
                hasSelectedSettingsPath
                    ? tr("Using selected file: %1").arg(selectedPath)
                    : tr("Using auto-detected file: %1").arg(selectedPath));
        } else if (!errorMessage.isEmpty()) {
            airsimSettingsPathValue_->setText(errorMessage);
        } else {
            airsimSettingsPathValue_->setText(tr("No AirSim settings.json found. Select a file or use Documents/AirSim/settings.json."));
        }
    }
    syncDroneCameraImageLayerItems();
    syncStationCameraFpsControls();
    refreshAirSimSettingsSyncStatus();
}

QString MainWindow::currentAirSimSettingsPath(QString* errorMessage) const {
    if (errorMessage) {
        errorMessage->clear();
    }

    const QString selectedSettingsPath = selectedAirSimSettingsPath();
    if (!selectedSettingsPath.isEmpty()) {
        const QFileInfo info(selectedSettingsPath);
        if (info.exists() && info.isFile()) {
            return info.absoluteFilePath();
        }
        if (errorMessage) {
            *errorMessage = tr("Selected AirSim settings.json does not exist: %1").arg(selectedSettingsPath);
        }
        return QString();
    }

    const QStringList candidates = effectiveAirSimSettingsPaths();
    for (const QString& candidate : candidates) {
        const QFileInfo info(candidate);
        if (info.exists() && info.isFile()) {
            return info.absoluteFilePath();
        }
    }

    if (errorMessage) {
        *errorMessage = tr("No AirSim settings.json found. Select a file or use Documents/AirSim/settings.json.");
    }
    return QString();
}

QString MainWindow::currentAirSimPreviewCameraName() const {
    QString cameraName = airsimLiveCameraCombo_ ? airsimLiveCameraCombo_->currentText().trimmed() : QString();
    if (cameraName.isEmpty() && airSimViewController_) {
        cameraName = airSimViewController_->cameraName().trimmed();
    }
    return cameraName;
}

bool MainWindow::isStationExternalCameraName(const QString& cameraName) const {
    const QString trimmed = cameraName.trimmed();
    if (trimmed.isEmpty()) {
        return false;
    }
    for (const BaseStation& station : stations_) {
        if (stationExternalCameraName(station) == trimmed) {
            return true;
        }
    }
    return false;
}

bool MainWindow::hasCustomAirSimPreviewExternalCamera(const QJsonObject& externalCameras) const {
    const QString cameraName = currentAirSimPreviewCameraName();
    if (cameraName.isEmpty()
        || cameraName.compare(QLatin1String(kGuiPreviewCameraName), Qt::CaseInsensitive) == 0
        || isStationExternalCameraName(cameraName)) {
        return false;
    }
    return externalCameras.contains(cameraName) && externalCameras.value(cameraName).isObject();
}

bool MainWindow::missingAirSimGuiPreviewExternalCamera(const QString& settingsPath,
                                                       QString* errorMessage) const {
    if (errorMessage) {
        errorMessage->clear();
    }

    QFile file(settingsPath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = tr("Could not open %1").arg(settingsPath);
        }
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage) {
            *errorMessage = tr("Failed to parse %1: %2").arg(settingsPath, parseError.errorString());
        }
        return false;
    }

    const QJsonObject root = doc.object();
    const QJsonValue externalValue = root.value(QStringLiteral("ExternalCameras"));
    QJsonObject external;
    if (!externalValue.isUndefined()) {
        if (!externalValue.isObject()) {
            if (errorMessage) {
                *errorMessage = tr("ExternalCameras exists in %1 but is not a JSON object.").arg(settingsPath);
            }
            return false;
        }
        external = externalValue.toObject();
    }

    if (hasCustomAirSimPreviewExternalCamera(external)) {
        return false;
    }
    return !external.contains(QString::fromLatin1(kGuiPreviewCameraName));
}

QVector<int> MainWindow::missingAirSimExternalCameraStationIndices(const QString& settingsPath,
                                                                   QString* errorMessage) const {
    if (errorMessage) {
        errorMessage->clear();
    }

    QFile file(settingsPath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = tr("Could not open %1").arg(settingsPath);
        }
        return {};
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage) {
            *errorMessage = tr("Failed to parse %1: %2").arg(settingsPath, parseError.errorString());
        }
        return {};
    }

    const QJsonObject root = doc.object();
    const QJsonValue externalValue = root.value(QStringLiteral("ExternalCameras"));
    QJsonObject external;
    if (!externalValue.isUndefined()) {
        if (!externalValue.isObject()) {
            if (errorMessage) {
                *errorMessage = tr("ExternalCameras exists in %1 but is not a JSON object.").arg(settingsPath);
            }
            return {};
        }
        external = externalValue.toObject();
    }

    QVector<int> missing;
    QSet<QString> seenCameraNames;
    for (int i = 0; i < static_cast<int>(stations_.size()); ++i) {
        const QString cameraName = stationExternalCameraName(stations_[static_cast<size_t>(i)]);
        if (cameraName.isEmpty()
            || cameraName.compare(QStringLiteral("GuiPreview"), Qt::CaseInsensitive) == 0
            || seenCameraNames.contains(cameraName)) {
            continue;
        }
        seenCameraNames.insert(cameraName);
        if (!external.contains(cameraName)) {
            missing.push_back(i);
        }
    }
    return missing;
}

QJsonObject MainWindow::buildAirSimExternalCameraEntries(
    const QVector<int>& stationIndices,
    const MainWindow::AirSimCameraCaptureConfig& stationCaptureConfig,
    bool includeGuiPreview,
    const MainWindow::AirSimCameraCaptureConfig& guiPreviewCaptureConfig) const {
    const int stationWidth = std::max(1, stationCaptureConfig.width);
    const int stationHeight = std::max(1, stationCaptureConfig.height);
    const int stationFov = std::max(1, std::min(179, stationCaptureConfig.fovDeg));

    QJsonObject entries;
    if (includeGuiPreview) {
        entries[QString::fromLatin1(kGuiPreviewCameraName)] =
            buildAirSimGuiPreviewExternalCameraEntry(guiPreviewCaptureConfig);
    }

    QSet<QString> seenCameraNames;
    for (const int index : stationIndices) {
        if (index < 0 || index >= static_cast<int>(stations_.size())) {
            continue;
        }
        const BaseStation& station = stations_[static_cast<size_t>(index)];
        const QString cameraName = stationExternalCameraName(station);
        if (cameraName.isEmpty()
            || cameraName.compare(QStringLiteral("GuiPreview"), Qt::CaseInsensitive) == 0
            || seenCameraNames.contains(cameraName)) {
            continue;
        }
        seenCameraNames.insert(cameraName);

        const Vec3 airsimPosition = scenePointToAirSim(station.position);

        QJsonObject captureSettings;
        captureSettings[QStringLiteral("ImageType")] = 0;
        captureSettings[QStringLiteral("Width")] = stationWidth;
        captureSettings[QStringLiteral("Height")] = stationHeight;
        captureSettings[QStringLiteral("FOV_Degrees")] = stationFov;

        QJsonObject camera;
        camera[QStringLiteral("CaptureSettings")] = QJsonArray{captureSettings};
        camera[QStringLiteral("PublishToRos")] = 1;
        camera[QStringLiteral("X")] = airsimPosition.x;
        camera[QStringLiteral("Y")] = airsimPosition.y;
        camera[QStringLiteral("Z")] = airsimPosition.z + station.previewOffsetZ;
        camera[QStringLiteral("Pitch")] = 0;
        camera[QStringLiteral("Roll")] = 0;
        camera[QStringLiteral("Yaw")] = 0;
        entries[cameraName] = camera;
    }
    return entries;
}

QJsonObject MainWindow::buildAirSimGuiPreviewExternalCameraEntry(
    const MainWindow::AirSimCameraCaptureConfig& captureConfig) const {
    const int clampedWidth = std::max(1, captureConfig.width);
    const int clampedHeight = std::max(1, captureConfig.height);
    const int clampedFov = std::max(1, std::min(179, captureConfig.fovDeg));

    QJsonObject captureSettings;
    captureSettings[QStringLiteral("ImageType")] = 0;
    captureSettings[QStringLiteral("Width")] = clampedWidth;
    captureSettings[QStringLiteral("Height")] = clampedHeight;
    captureSettings[QStringLiteral("FOV_Degrees")] = clampedFov;

    QJsonObject camera;
    camera[QStringLiteral("CaptureSettings")] = QJsonArray{captureSettings};
    camera[QStringLiteral("PublishToRos")] = 0;
    camera[QStringLiteral("X")] = -8.0;
    camera[QStringLiteral("Y")] = 0.0;
    camera[QStringLiteral("Z")] = -2.0;
    camera[QStringLiteral("Pitch")] = 10;
    camera[QStringLiteral("Roll")] = 0;
    camera[QStringLiteral("Yaw")] = 0;
    return camera;
}

bool MainWindow::buildAirSimSettingsPreview(const QString& settingsPath,
                                            const QVector<int>& stationIndices,
                                            const MainWindow::AirSimCameraCaptureConfig& stationCaptureConfig,
                                            bool includeGuiPreview,
                                            const MainWindow::AirSimCameraCaptureConfig& guiPreviewCaptureConfig,
                                            QString* previewText,
                                            QVector<QPair<int, int>>* addedLineRanges,
                                            QString* errorMessage) const {
    if (previewText) {
        previewText->clear();
    }
    if (addedLineRanges) {
        addedLineRanges->clear();
    }
    if (errorMessage) {
        errorMessage->clear();
    }

    QFile file(settingsPath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = tr("Could not open %1").arg(settingsPath);
        }
        return false;
    }

    const QByteArray rawSettings = file.readAll();
    const QString originalText = QString::fromUtf8(rawSettings);
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(rawSettings, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage) {
            *errorMessage = tr("Failed to parse %1: %2").arg(settingsPath, parseError.errorString());
        }
        return false;
    }

    QJsonObject root = doc.object();
    const QJsonValue externalValue = root.value(QStringLiteral("ExternalCameras"));
    QJsonObject external;
    if (!externalValue.isUndefined()) {
        if (!externalValue.isObject()) {
            if (errorMessage) {
                *errorMessage = tr("ExternalCameras exists in %1 but is not a JSON object.").arg(settingsPath);
            }
            return false;
        }
        external = externalValue.toObject();
    }

    const QJsonObject entries = buildAirSimExternalCameraEntries(stationIndices,
                                                                 stationCaptureConfig,
                                                                 includeGuiPreview,
                                                                 guiPreviewCaptureConfig);
    QJsonObject addedEntries;
    QStringList addedNames;
    for (auto it = entries.constBegin(); it != entries.constEnd(); ++it) {
        if (external.contains(it.key())) {
            continue;
        }
        addedEntries[it.key()] = it.value();
        addedNames << it.key();
    }

    if (addedEntries.isEmpty()) {
        if (previewText) {
            *previewText = originalText;
        }
        return true;
    }

    const int rootStart = originalText.indexOf(QChar('{'));
    const int rootEnd = findJsonObjectEndInText(originalText, rootStart);
    if (rootStart < 0 || rootEnd < 0) {
        if (errorMessage) {
            *errorMessage = tr("Could not locate the root JSON object in %1.").arg(settingsPath);
        }
        return false;
    }

    QString text;
    int externalKeyStart = -1;
    const int externalStart = findTopLevelObjectPropertyObjectStart(originalText,
                                                                    rootStart,
                                                                    rootEnd,
                                                                    QStringLiteral("ExternalCameras"),
                                                                    &externalKeyStart);
    if (externalStart >= 0) {
        const int externalEnd = findJsonObjectEndInText(originalText, externalStart);
        if (externalEnd < 0) {
            if (errorMessage) {
                *errorMessage = tr("Could not locate the end of ExternalCameras in %1.").arg(settingsPath);
            }
            return false;
        }

        const QString objectIndent = lineIndentAtPosition(originalText, externalKeyStart);
        const QString entryIndent = objectIndent + QStringLiteral("  ");
        const QString entriesText = externalCameraEntriesInsertionText(addedEntries, entryIndent);
        const int insertionPos = jsonInsertionPointBeforeClosingBrace(originalText, externalStart, externalEnd);
        const bool objectHasEntries = textRangeHasNonWhitespace(originalText, externalStart + 1, externalEnd);
        const bool closingBraceAlreadyIndented = textRangeHasLineBreak(originalText, insertionPos, externalEnd);

        QString insertion = objectHasEntries ? QStringLiteral(",\n") : QStringLiteral("\n");
        insertion += entriesText;
        if (!closingBraceAlreadyIndented) {
            insertion += QChar('\n') + objectIndent;
        }
        text = originalText.left(insertionPos) + insertion + originalText.mid(insertionPos);
    } else {
        const QString rootIndent = lineIndentAtPosition(originalText, rootStart);
        const QString fieldIndent = rootIndent + QStringLiteral("  ");
        const QString entryIndent = fieldIndent + QStringLiteral("  ");
        const QString entriesText = externalCameraEntriesInsertionText(addedEntries, entryIndent);
        const int insertionPos = jsonInsertionPointBeforeClosingBrace(originalText, rootStart, rootEnd);
        const bool rootHasFields = textRangeHasNonWhitespace(originalText, rootStart + 1, rootEnd);
        const bool closingBraceAlreadyIndented = textRangeHasLineBreak(originalText, insertionPos, rootEnd);

        QString insertion = rootHasFields ? QStringLiteral(",\n") : QStringLiteral("\n");
        insertion += fieldIndent + QStringLiteral("\"ExternalCameras\": {\n");
        insertion += entriesText;
        insertion += QChar('\n') + fieldIndent + QChar('}');
        if (!closingBraceAlreadyIndented) {
            insertion += QChar('\n') + rootIndent;
        }
        text = originalText.left(insertionPos) + insertion + originalText.mid(insertionPos);
    }

    if (previewText) {
        *previewText = text;
    }
    if (addedLineRanges) {
        *addedLineRanges = highlightedJsonObjectRanges(text, addedNames);
    }
    return true;
}

bool MainWindow::writeMissingAirSimExternalCameras(const QString& settingsPath,
                                                   const QVector<int>& stationIndices,
                                                   const MainWindow::AirSimCameraCaptureConfig& stationCaptureConfig,
                                                   bool includeGuiPreview,
                                                   const MainWindow::AirSimCameraCaptureConfig& guiPreviewCaptureConfig,
                                                   QString* backupPath,
                                                   QString* errorMessage) {
    if (backupPath) {
        backupPath->clear();
    }
    if (errorMessage) {
        errorMessage->clear();
    }

    QString updatedText;
    QVector<QPair<int, int>> addedRanges;
    QString previewError;
    if (!buildAirSimSettingsPreview(settingsPath,
                                    stationIndices,
                                    stationCaptureConfig,
                                    includeGuiPreview,
                                    guiPreviewCaptureConfig,
                                    &updatedText,
                                    &addedRanges,
                                    &previewError)) {
        if (errorMessage) {
            *errorMessage = previewError;
        }
        return false;
    }

    QFile file(settingsPath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = tr("Could not open %1").arg(settingsPath);
        }
        return false;
    }
    const QString currentText = QString::fromUtf8(file.readAll());
    if (updatedText == currentText) {
        return true;
    }

    const QFileInfo settingsInfo(settingsPath);
    const QDir settingsDir = settingsInfo.absoluteDir();
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    QString backup = settingsDir.filePath(timestamp + QStringLiteral(".json"));
    for (int suffix = 2; QFileInfo::exists(backup); ++suffix) {
        backup = settingsDir.filePath(QStringLiteral("%1_%2.json").arg(timestamp).arg(suffix));
    }
    if (!QFile::copy(settingsPath, backup)) {
        if (errorMessage) {
            *errorMessage = tr("Could not create backup %1").arg(backup);
        }
        return false;
    }

    QSaveFile out(settingsPath);
    if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (errorMessage) {
            *errorMessage = tr("Could not write %1").arg(settingsPath);
        }
        return false;
    }
    if (out.write(updatedText.toUtf8()) < 0 || !out.commit()) {
        if (errorMessage) {
            *errorMessage = tr("Failed to update %1").arg(settingsPath);
        }
        return false;
    }

    if (backupPath) {
        *backupPath = backup;
    }
    return true;
}

void MainWindow::refreshAirSimSettingsSyncStatus() {
    if (!airsimSettingsSyncPanel_ || !airsimSettingsSyncHintLabel_ || !syncAirSimSettingsButton_) {
        return;
    }

    QString pathError;
    const QString settingsPath = currentAirSimSettingsPath(&pathError);
    if (settingsPath.isEmpty()) {
        const QString previewCameraName = currentAirSimPreviewCameraName();
        const bool shouldShow = !stations_.empty()
            || previewCameraName.compare(QLatin1String(kGuiPreviewCameraName), Qt::CaseInsensitive) == 0;
        airsimSettingsSyncPanel_->setVisible(shouldShow);
        airsimSettingsSyncHintLabel_->setText(pathError);
        syncAirSimSettingsButton_->setEnabled(false);
        return;
    }

    QString guiPreviewError;
    const bool missingGuiPreview = missingAirSimGuiPreviewExternalCamera(settingsPath, &guiPreviewError);
    if (!guiPreviewError.trimmed().isEmpty()) {
        airsimSettingsSyncPanel_->show();
        airsimSettingsSyncHintLabel_->setText(guiPreviewError);
        syncAirSimSettingsButton_->setEnabled(false);
        return;
    }

    QString missingError;
    const QVector<int> missing = missingAirSimExternalCameraStationIndices(settingsPath, &missingError);
    if (!missingError.trimmed().isEmpty()) {
        airsimSettingsSyncPanel_->show();
        airsimSettingsSyncHintLabel_->setText(missingError);
        syncAirSimSettingsButton_->setEnabled(false);
        return;
    }

    if (!missingGuiPreview && missing.isEmpty()) {
        airsimSettingsSyncPanel_->hide();
        return;
    }

    QStringList missingParts;
    if (missingGuiPreview) {
        missingParts << QString::fromLatin1(kGuiPreviewCameraName);
    }
    QStringList names;
    for (const int index : missing) {
        if (index >= 0 && index < static_cast<int>(stations_.size())) {
            names << stationExternalCameraName(stations_[static_cast<size_t>(index)]);
        }
    }
    if (!names.isEmpty()) {
        missingParts << tr("%1 station external camera(s): %2")
                            .arg(missing.size())
                            .arg(names.join(QStringLiteral(", ")));
    }
    airsimSettingsSyncPanel_->show();
    airsimSettingsSyncHintLabel_->setText(
        tr("AirSim settings.json is missing %1. Sync settings.json before using these UE preview camera(s).")
            .arg(missingParts.join(QStringLiteral("; "))));
    syncAirSimSettingsButton_->setEnabled(true);
}

void MainWindow::syncAirSimSettingsExternalCameras() {
    QString pathError;
    const QString settingsPath = currentAirSimSettingsPath(&pathError);
    if (settingsPath.isEmpty()) {
        QMessageBox::warning(this, tr("Sync AirSim Settings"), pathError);
        refreshAirSimSettingsSyncStatus();
        return;
    }

    QString guiPreviewError;
    const bool missingGuiPreview = missingAirSimGuiPreviewExternalCamera(settingsPath, &guiPreviewError);
    if (!guiPreviewError.trimmed().isEmpty()) {
        QMessageBox::warning(this, tr("Sync AirSim Settings"), guiPreviewError);
        refreshAirSimSettingsSyncStatus();
        return;
    }

    QString missingError;
    const QVector<int> missing = missingAirSimExternalCameraStationIndices(settingsPath, &missingError);
    if (!missingError.trimmed().isEmpty()) {
        QMessageBox::warning(this, tr("Sync AirSim Settings"), missingError);
        refreshAirSimSettingsSyncStatus();
        return;
    }
    if (!missingGuiPreview && missing.isEmpty()) {
        QMessageBox::information(this, tr("Sync AirSim Settings"),
                                 tr("AirSim settings.json already contains all required UE preview cameras."));
        refreshAirSimSettingsSyncStatus();
        return;
    }

    QDialog dialog(this);
    dialog.setWindowTitle(tr("Sync AirSim settings.json"));
    dialog.resize(760, 620);
    auto* layout = new QVBoxLayout(&dialog);

    auto* intro = new QLabel(
        tr("Only missing entries under ExternalCameras will be added, including GuiPreview when the UE live view needs it. Existing settings and unused cameras will not be deleted."),
        &dialog);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    auto* pathLabel = new QLabel(tr("Target: %1").arg(settingsPath), &dialog);
    pathLabel->setWordWrap(true);
    layout->addWidget(pathLabel);

    auto* preview = new QPlainTextEdit(&dialog);
    preview->setReadOnly(true);
    preview->setLineWrapMode(QPlainTextEdit::NoWrap);
    preview->setStyleSheet(QStringLiteral(
        "QPlainTextEdit { background: #ffffff; color: #111827; border: 1px solid #cbd5e1; font-family: monospace; }"));
    layout->addWidget(preview, 1);

    auto* controls = new QWidget(&dialog);
    auto* controlsLayout = new QHBoxLayout(controls);
    controlsLayout->setContentsMargins(0, 0, 0, 0);
    controlsLayout->setSpacing(8);
    QSpinBox* guiWidthSpin = nullptr;
    QSpinBox* guiHeightSpin = nullptr;
    QSpinBox* guiFovSpin = nullptr;
    QSpinBox* stationWidthSpin = nullptr;
    QSpinBox* stationHeightSpin = nullptr;
    QSpinBox* stationFovSpin = nullptr;
    auto createCaptureGroup = [&](const QString& title,
                                  QSpinBox*& widthSpin,
                                  QSpinBox*& heightSpin,
                                  QSpinBox*& fovSpin) {
        auto* group = new QGroupBox(title, controls);
        auto* groupLayout = new QHBoxLayout(group);
        groupLayout->setContentsMargins(8, 6, 8, 6);
        groupLayout->setSpacing(6);
        widthSpin = new QSpinBox(group);
        widthSpin->setRange(1, 8192);
        widthSpin->setKeyboardTracking(false);
        widthSpin->setValue(1280);
        heightSpin = new QSpinBox(group);
        heightSpin->setRange(1, 8192);
        heightSpin->setKeyboardTracking(false);
        heightSpin->setValue(720);
        fovSpin = new QSpinBox(group);
        fovSpin->setRange(1, 179);
        fovSpin->setKeyboardTracking(false);
        fovSpin->setValue(90);
        groupLayout->addWidget(new QLabel(tr("Width"), group));
        groupLayout->addWidget(widthSpin);
        groupLayout->addWidget(new QLabel(tr("Height"), group));
        groupLayout->addWidget(heightSpin);
        groupLayout->addWidget(new QLabel(tr("FOV"), group));
        groupLayout->addWidget(fovSpin);
        controlsLayout->addWidget(group);
    };
    if (missingGuiPreview) {
        createCaptureGroup(tr("GuiPreview image settings"), guiWidthSpin, guiHeightSpin, guiFovSpin);
    }
    if (!missing.isEmpty()) {
        createCaptureGroup(tr("Base-station image settings"), stationWidthSpin, stationHeightSpin, stationFovSpin);
    }
    controlsLayout->addStretch(1);
    layout->addWidget(controls);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    buttons->button(QDialogButtonBox::Ok)->setText(tr("Confirm Sync"));
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);

    auto captureConfigFromSpins = [](QSpinBox* widthSpin,
                                     QSpinBox* heightSpin,
                                     QSpinBox* fovSpin) {
        AirSimCameraCaptureConfig config;
        if (widthSpin) {
            config.width = widthSpin->value();
        }
        if (heightSpin) {
            config.height = heightSpin->value();
        }
        if (fovSpin) {
            config.fovDeg = fovSpin->value();
        }
        return config;
    };

    auto updatePreview = [&]() {
        QString previewText;
        QVector<QPair<int, int>> addedRanges;
        QString previewError;
        const AirSimCameraCaptureConfig guiPreviewConfig =
            captureConfigFromSpins(guiWidthSpin, guiHeightSpin, guiFovSpin);
        const AirSimCameraCaptureConfig stationConfig =
            captureConfigFromSpins(stationWidthSpin, stationHeightSpin, stationFovSpin);
        const bool ok = buildAirSimSettingsPreview(settingsPath,
                                                   missing,
                                                   stationConfig,
                                                   missingGuiPreview,
                                                   guiPreviewConfig,
                                                   &previewText,
                                                   &addedRanges,
                                                   &previewError);
        buttons->button(QDialogButtonBox::Ok)->setEnabled(ok);
        preview->setPlainText(ok ? previewText : previewError);

        QList<QTextEdit::ExtraSelection> selections;
        QTextCharFormat addedFormat;
        addedFormat.setBackground(QColor(QStringLiteral("#bbf7d0")));
        addedFormat.setForeground(QColor(QStringLiteral("#14532d")));
        addedFormat.setProperty(QTextFormat::FullWidthSelection, true);
        for (const auto& range : addedRanges) {
            for (int line = range.first; line <= range.second; ++line) {
                QTextEdit::ExtraSelection selection;
                QTextCursor cursor(preview->document());
                cursor.movePosition(QTextCursor::Start);
                cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, line);
                selection.cursor = cursor;
                selection.format = addedFormat;
                selections << selection;
            }
        }
        preview->setExtraSelections(selections);

        if (ok && !addedRanges.isEmpty()) {
            const int targetLine = std::max(0, addedRanges.first().first - 3);
            QTextCursor cursor(preview->document());
            cursor.movePosition(QTextCursor::Start);
            cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, targetLine);
            preview->setTextCursor(cursor);
            preview->centerCursor();
            QTimer::singleShot(0, preview, [preview, targetLine]() {
                QTextCursor queuedCursor(preview->document());
                queuedCursor.movePosition(QTextCursor::Start);
                queuedCursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, targetLine);
                preview->setTextCursor(queuedCursor);
                preview->centerCursor();
            });
        }
    };
    auto connectCaptureSpin = [&](QSpinBox* spin) {
        if (spin) {
            connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), &dialog, [&](int) { updatePreview(); });
        }
    };
    connectCaptureSpin(guiWidthSpin);
    connectCaptureSpin(guiHeightSpin);
    connectCaptureSpin(guiFovSpin);
    connectCaptureSpin(stationWidthSpin);
    connectCaptureSpin(stationHeightSpin);
    connectCaptureSpin(stationFovSpin);
    updatePreview();

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    auto interpretSpinText = [](QSpinBox* spin) {
        if (spin) {
            spin->interpretText();
        }
    };
    interpretSpinText(guiWidthSpin);
    interpretSpinText(guiHeightSpin);
    interpretSpinText(guiFovSpin);
    interpretSpinText(stationWidthSpin);
    interpretSpinText(stationHeightSpin);
    interpretSpinText(stationFovSpin);

    const AirSimCameraCaptureConfig guiPreviewConfig =
        captureConfigFromSpins(guiWidthSpin, guiHeightSpin, guiFovSpin);
    const AirSimCameraCaptureConfig stationConfig =
        captureConfigFromSpins(stationWidthSpin, stationHeightSpin, stationFovSpin);

    QString backupPath;
    QString writeError;
    if (!writeMissingAirSimExternalCameras(settingsPath,
                                           missing,
                                           stationConfig,
                                           missingGuiPreview,
                                           guiPreviewConfig,
                                           &backupPath,
                                           &writeError)) {
        QMessageBox::warning(this, tr("Sync AirSim Settings"), writeError);
        refreshAirSimSettingsSyncStatus();
        return;
    }

    refreshAirSimCameraList();
    refreshAirSimSettingsSyncStatus();

    QMessageBox::information(
        this,
        tr("Sync AirSim Settings"),
        backupPath.isEmpty()
            ? tr("No missing UE preview cameras remained to add.")
            : tr("Added missing UE preview camera definitions to settings.json.\nBackup: %1").arg(backupPath));
}

void MainWindow::ensureRfDataWindow() {
    if (rfDataWindow_) {
        return;
    }

    rfDataWindow_ = new QMainWindow(this, Qt::Window);
    rfDataWindow_->setWindowTitle(tr("Wireless Data Monitor"));
    rfDataWindow_->resize(720, 820);

    auto* central = new QWidget(rfDataWindow_);
    auto* layout = new QVBoxLayout(central);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);

    auto* hint = new QLabel(tr("Each card shows one base station and all RF observation fields currently available for that anchor."), central);
    hint->setWordWrap(true);
    hint->setStyleSheet(QStringLiteral("color: #586574;"));
    layout->addWidget(hint);

    rfDataScroll_ = new QScrollArea(central);
    rfDataScroll_->setWidgetResizable(true);
    rfDataCardsContainer_ = new QWidget(rfDataScroll_);
    rfDataCardsLayout_ = new QGridLayout(rfDataCardsContainer_);
    rfDataCardsLayout_->setContentsMargins(4, 4, 4, 4);
    rfDataCardsLayout_->setHorizontalSpacing(10);
    rfDataCardsLayout_->setVerticalSpacing(10);
    rfDataCardsLayout_->setColumnStretch(0, 1);
    rfDataCardsLayout_->setColumnStretch(1, 1);
    rfDataScroll_->setWidget(rfDataCardsContainer_);
    layout->addWidget(rfDataScroll_, 1);

    rfDataWindow_->setCentralWidget(central);
}

void MainWindow::showRfDataWindow() {
    ensureRfDataWindow();
    rebuildRfDataCards();
    rfDataWindow_->show();
    rfDataWindow_->raise();
    rfDataWindow_->activateWindow();
}

void MainWindow::ensureSysDataWindow() {
    if (sysDataWindow_) {
        return;
    }

    sysDataWindow_ = new QMainWindow(this, Qt::Window);
    sysDataTableLayoutInitialized_ = false;
    sysDataLastRawJson_.clear();
    sysDataWindow_->setWindowTitle(tr("Sionna SYS Monitor"));
    sysDataWindow_->resize(1180, 860);

    auto* central = new QWidget(sysDataWindow_);
    auto* layout = new QVBoxLayout(central);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(10);

    auto* hint = new QLabel(tr("This page shows the latest SYS-over-RT snapshot plus the V1 beam-codebook sweep output. It includes the live serving-link decision, per-candidate beam metrics, and a raw JSON panel so every emitted field remains visible."), central);
    hint->setWordWrap(true);
    hint->setStyleSheet(QStringLiteral("color: #586574;"));
    layout->addWidget(hint);

    auto* cards = new QWidget(central);
    auto* cardsLayout = new QGridLayout(cards);
    cardsLayout->setContentsMargins(0, 0, 0, 0);
    cardsLayout->setHorizontalSpacing(10);
    cardsLayout->setVerticalSpacing(10);

    const auto statusCard = createInfoCard(tr("SYS Status"), tr("Waiting for SYS output..."), cards);
    sysDataStatusBodyLabel_ = statusCard.bodyLabel;
    cardsLayout->addWidget(statusCard.frame, 0, 0);

    const auto servingCard = createInfoCard(tr("Serving Link"), tr("No serving link yet."), cards);
    sysDataServingBodyLabel_ = servingCard.bodyLabel;
    cardsLayout->addWidget(servingCard.frame, 0, 1);

    const auto configCard = createInfoCard(tr("Configured SYS + Beam Parameters"), tr("Open Simulation Settings to configure SYS over RT and beamforming."), cards);
    sysDataConfigBodyLabel_ = configCard.bodyLabel;
    cardsLayout->addWidget(configCard.frame, 1, 0, 1, 2);

    cardsLayout->setColumnStretch(0, 1);
    cardsLayout->setColumnStretch(1, 1);
    layout->addWidget(cards);

    auto* candidatesBox = new QGroupBox(tr("Candidate Base Stations"), central);
    auto* candidatesLayout = new QVBoxLayout(candidatesBox);
    sysDataTable_ = new QTableWidget(candidatesBox);
    sysDataTable_->setColumnCount(29);
    sysDataTable_->setHorizontalHeaderLabels(QStringList{
        tr("Serving"),
        tr("Beam Ref"),
        tr("BS"),
        tr("Anchor ID"),
        tr("Candidate SINR (dB)"),
        tr("Candidate Rate (bps/Hz)"),
        tr("MCS"),
        tr("Decoded Bits"),
        tr("TB Result"),
        tr("Spectral Efficiency"),
        tr("BLER Target"),
        tr("Beam Mode"),
        tr("Beam Grid"),
        tr("Beams"),
        tr("Selected Beam"),
        tr("Selected Gain (dB)"),
        tr("Oracle Beam"),
        tr("Oracle Gain (dB)"),
        tr("Predictor"),
        tr("Pred Beam"),
        tr("Pred Conf"),
        tr("Oracle∈TopK"),
        tr("Hit"),
        tr("TopK"),
        tr("Source"),
        tr("Paths"),
        tr("Strongest Path (dB)"),
        tr("Path Type"),
        tr("Path Order / sim_idx")
    });
    sysDataTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    sysDataTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    sysDataTable_->setSelectionMode(QAbstractItemView::SingleSelection);
    sysDataTable_->setAlternatingRowColors(true);
    sysDataTable_->setWordWrap(false);
    sysDataTable_->verticalHeader()->setVisible(false);
    sysDataTable_->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    sysDataTable_->verticalHeader()->setDefaultSectionSize(sysDataTable_->fontMetrics().height() + 10);
    sysDataTable_->horizontalHeader()->setStretchLastSection(false);
    sysDataTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    sysDataTable_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    candidatesLayout->addWidget(sysDataTable_);
    layout->addWidget(candidatesBox, 1);

    auto* rawBox = new QGroupBox(tr("Raw SYS Snapshot (JSON)"), central);
    auto* rawLayout = new QVBoxLayout(rawBox);
    sysDataRawView_ = new QPlainTextEdit(rawBox);
    sysDataRawView_->setReadOnly(true);
    sysDataRawView_->setMaximumBlockCount(500);
    sysDataRawView_->setPlaceholderText(tr("Raw SYS JSON will appear here after the first snapshot."));
    rawLayout->addWidget(sysDataRawView_);
    layout->addWidget(rawBox, 1);

    sysDataWindow_->setCentralWidget(central);
}

void MainWindow::scheduleSysDataWindowRefresh(bool immediate) {
    sysDataRefreshPending_ = true;
    if (immediate) {
        if (sysDataRefreshTimer_) {
            sysDataRefreshTimer_->stop();
        }
        refreshSysDataWindowIfNeeded();
        return;
    }
    if (sysDataWindow_ && sysDataWindow_->isVisible() && sysDataRefreshTimer_ && !sysDataRefreshTimer_->isActive()) {
        sysDataRefreshTimer_->start();
    }
}

void MainWindow::refreshSysDataWindowIfNeeded() {
    if (!sysDataRefreshPending_) {
        return;
    }
    if (!sysDataWindow_ || !sysDataWindow_->isVisible()) {
        return;
    }
    sysDataRefreshPending_ = false;
    rebuildSysDataWindow();
}

void MainWindow::rebuildSysDataWindow() {
    if (!sysDataWindow_) {
        return;
    }

    QStringList statusLines;
    statusLines << QStringLiteral("Mode: %1").arg(currentSysSnapshot_.enabled ? QStringLiteral("Enabled") : QStringLiteral("Disabled"));
    statusLines << QStringLiteral("Scheduler policy: %1").arg(currentSysSnapshot_.schedulerPolicy.trimmed().isEmpty() ? QStringLiteral("n/a") : currentSysSnapshot_.schedulerPolicy);
    statusLines << QStringLiteral("Current sim_idx: %1").arg(fmtOptionalInt(currentSysSnapshot_.simIdx));
    statusLines << QStringLiteral("Candidates in snapshot: %1").arg(static_cast<int>(currentSysSnapshot_.candidates.size()));
    statusLines << QStringLiteral("RF anchors loaded in GUI: %1").arg(static_cast<int>(currentRfSnapshot_.anchors.size()));
    if (sysDataStatusBodyLabel_) {
        sysDataStatusBodyLabel_->setText(statusLines.join(QStringLiteral("\n")));
    }

    QStringList servingLines;
    servingLines << QStringLiteral("Serving BS: %1").arg(currentSysSnapshot_.servingBsName.trimmed().isEmpty() ? QStringLiteral("n/a") : currentSysSnapshot_.servingBsName);
    servingLines << QStringLiteral("Serving anchor ID: %1").arg(fmtOptionalInt(currentSysSnapshot_.servingAnchorId));
    servingLines << QStringLiteral("Serving index: %1").arg(fmtOptionalInt(currentSysSnapshot_.servingIdx));
    servingLines << QStringLiteral("MCS index: %1").arg(fmtOptionalInt(currentSysSnapshot_.servingMcsIndex));
    servingLines << QStringLiteral("Effective SINR: %1").arg(fmtFinite(currentSysSnapshot_.servingSinrEffDb, 2, QStringLiteral(" dB")));
    servingLines << QStringLiteral("Spectral efficiency: %1").arg(fmtFinite(currentSysSnapshot_.servingSpectralEfficiencyBpsHz, 3, QStringLiteral(" bps/Hz")));
    servingLines << QStringLiteral("Transport block result: %1").arg(ackStateLabel(currentSysSnapshot_.servingTbOk));
    servingLines << QStringLiteral("Beam mode: %1").arg(currentSysSnapshot_.beamEnabled ? (currentSysSnapshot_.beamSelectionMode.trimmed().isEmpty() ? QStringLiteral("enabled") : beamSelectionModeToUi(currentSysSnapshot_.beamSelectionMode)) : QStringLiteral("disabled"));
    servingLines << QStringLiteral("Beam reference BS: %1").arg(currentSysSnapshot_.beamReferenceBsName.trimmed().isEmpty() ? QStringLiteral("n/a") : currentSysSnapshot_.beamReferenceBsName);
    servingLines << QStringLiteral("Selected beam / gain: %1 / %2").arg(fmtOptionalInt(currentSysSnapshot_.beamSelectedIndex), fmtFinite(currentSysSnapshot_.beamSelectedGainDb, 2, QStringLiteral(" dB")));
    servingLines << QStringLiteral("Oracle beam / gain: %1 / %2").arg(fmtOptionalInt(currentSysSnapshot_.beamOracleIndex), fmtFinite(currentSysSnapshot_.beamOracleGainDb, 2, QStringLiteral(" dB")));
    servingLines << QStringLiteral("Predictor status: %1").arg(currentSysSnapshot_.beamPredictorStatus.trimmed().isEmpty() ? QStringLiteral("n/a") : currentSysSnapshot_.beamPredictorStatus);
    servingLines << QStringLiteral("Pred beam / conf: %1 / %2").arg(fmtOptionalInt(currentSysSnapshot_.beamPredictedIndex), fmtFinite(currentSysSnapshot_.beamPredictedConfidence, 3));
    servingLines << QStringLiteral("Oracle in TopK / Hit: %1 / %2").arg(ackStateLabel(currentSysSnapshot_.beamOracleInTopK), ackStateLabel(currentSysSnapshot_.beamHit));
    servingLines << QStringLiteral("TopK / source: %1 / %2").arg(currentSysSnapshot_.beamTopkIndices.trimmed().isEmpty() ? QStringLiteral("n/a") : currentSysSnapshot_.beamTopkIndices, currentSysSnapshot_.beamSelectionSource.trimmed().isEmpty() ? QStringLiteral("n/a") : beamSelectionModeToUi(currentSysSnapshot_.beamSelectionSource));
    if (sysDataServingBodyLabel_) {
        sysDataServingBodyLabel_->setText(servingLines.join(QStringLiteral("\n")));
    }

    QStringList configLines;
    configLines << QStringLiteral("Target BLER: %1").arg(QString::number(simSettings_.sysBlerTarget, 'g', 5));
    configLines << QStringLiteral("MCS table index: %1").arg(simSettings_.sysMcsTableIndex);
    configLines << QStringLiteral("BS TX power: %1 dBm").arg(QString::number(simSettings_.sysBsTxPowerDbm, 'f', 1));
    configLines << QStringLiteral("Subcarriers: %1").arg(simSettings_.sysNumSubcarriers);
    configLines << QStringLiteral("Subcarrier spacing: %1 Hz").arg(QString::number(simSettings_.sysSubcarrierSpacingHz, 'g', 10));
    configLines << QStringLiteral("OFDM symbols per slot: %1").arg(simSettings_.sysNumOfdmSymbols);
    configLines << QStringLiteral("Noise temperature: %1 K").arg(QString::number(simSettings_.sysTemperatureK, 'f', 1));
    configLines << QStringLiteral("RF carrier: %1 Hz").arg(QString::number(simSettings_.fcHz, 'g', 12));
    configLines << QStringLiteral("Beamforming: %1").arg(simSettings_.enableBeamforming ? QStringLiteral("Enabled") : QStringLiteral("Disabled"));
    configLines << QStringLiteral("Beam mode / codebook: %1 / %2").arg(beamSelectionModeToUi(simSettings_.beamSelectionMode), simSettings_.beamCodebookType);
    configLines << QStringLiteral("Generated beam count: %1").arg(simSettings_.beamCodebookNumBeams);
    configLines << QStringLiteral("Beam oversampling V/H: %1 / %2").arg(simSettings_.beamOversamplingV).arg(simSettings_.beamOversamplingH);
    configLines << QStringLiteral("Beam manual index: %1").arg(simSettings_.beamManualIndex);
    configLines << QStringLiteral("Beam custom codebook file: %1").arg(simSettings_.beamCodebookFilePath.trimmed().isEmpty() ? QStringLiteral("<sionna-generated>") : simSettings_.beamCodebookFilePath);
    configLines << QStringLiteral("Beam selection checkpoint: %1").arg(simSettings_.beamModelCheckpointPath.trimmed().isEmpty() ? QStringLiteral("<empty>") : simSettings_.beamModelCheckpointPath);
    configLines << QStringLiteral("Beam selection feature mode / TopK: %1 / %2").arg(beamFeatureModeToUi(simSettings_.beamFeatureMode), QString::number(simSettings_.beamTopK));
    configLines << QStringLiteral("Beam dataset export: %1").arg(simSettings_.beamExportTrainingDataset ? QStringLiteral("Enabled") : QStringLiteral("Disabled"));
    configLines << QStringLiteral("Beam dataset export path: %1").arg(simSettings_.beamTrainingDatasetPath.trimmed().isEmpty() ? QStringLiteral("<empty>") : simSettings_.beamTrainingDatasetPath);
    configLines << QStringLiteral("Beam training CSVs: %1").arg(effectiveBeamTrainingInputText(simSettings_.beamTrainingInputPaths, simSettings_.beamTrainingDatasetPath).trimmed().isEmpty() ? QStringLiteral("<empty>") : effectiveBeamTrainingInputText(simSettings_.beamTrainingInputPaths, simSettings_.beamTrainingDatasetPath).replace(QStringLiteral("\n"), QStringLiteral("; ")));
    configLines << QStringLiteral("Beam train out / epochs / batch: %1 / %2 / %3")
                       .arg(simSettings_.beamTrainingOutputCheckpointPath.trimmed().isEmpty() ? QStringLiteral("<empty>") : simSettings_.beamTrainingOutputCheckpointPath,
                            QString::number(simSettings_.beamTrainingEpochs),
                            QString::number(simSettings_.beamTrainingBatchSize));
    if (sysDataConfigBodyLabel_) {
        sysDataConfigBodyLabel_->setText(configLines.join(QStringLiteral("\n")));
    }

    if (sysDataTable_) {
        const int rowCount = static_cast<int>(currentSysSnapshot_.candidates.size());
        sysDataTable_->setUpdatesEnabled(false);
        sysDataTable_->setRowCount(rowCount);
        int row = 0;
        for (const auto& candidate : currentSysSnapshot_.candidates) {
            const bool hasUsablePath = sysCandidateHasUsablePath(candidate);
            auto setItem = [this, row](int column, const QString& text, const QString& tooltip = QString()) {
                QTableWidgetItem* item = sysDataTable_->item(row, column);
                if (!item) {
                    item = new QTableWidgetItem();
                    sysDataTable_->setItem(row, column, item);
                }
                if (item->text() != text) {
                    item->setText(text);
                }
                if (item->toolTip() != tooltip) {
                    item->setToolTip(tooltip);
                }
            };
            setItem(0, candidate.isServingBs ? QStringLiteral("YES") : QStringLiteral("-"));
            setItem(1, (hasUsablePath && candidate.beamSelectedAnchor) ? QStringLiteral("YES") : QStringLiteral("-"));
            setItem(2, candidate.bsName.isEmpty() ? QStringLiteral("<unnamed>") : candidate.bsName);
            setItem(3, fmtOptionalInt(candidate.anchorId));
            setItem(4, hasUsablePath ? fmtFinite(candidate.candidateSinrEffDb, 2) : QStringLiteral("n/a"));
            setItem(5, hasUsablePath ? fmtFinite(candidate.candidateRateBpsHz, 3) : QStringLiteral("n/a"));
            setItem(6, hasUsablePath ? fmtOptionalInt(candidate.mcsIndex) : QStringLiteral("n/a"));
            setItem(7, hasUsablePath ? fmtOptionalInt(candidate.numDecodedBits) : QStringLiteral("n/a"));
            setItem(8, hasUsablePath ? ackStateLabel(candidate.tbOk) : QStringLiteral("n/a"));
            setItem(9, hasUsablePath ? fmtFinite(candidate.spectralEfficiencyBpsHz, 3) : QStringLiteral("n/a"));
            setItem(10, fmtFinite(candidate.blerTarget, 3));
            setItem(11, hasUsablePath ? (candidate.beamEnabled ? (candidate.beamSelectionMode.trimmed().isEmpty() ? QStringLiteral("enabled") : beamSelectionModeToUi(candidate.beamSelectionMode)) : QStringLiteral("-")) : QStringLiteral("-"));
            setItem(12, hasUsablePath ? (candidate.beamCodebookType.trimmed().isEmpty() ? QStringLiteral("-") : candidate.beamCodebookType) : QStringLiteral("-"));
            setItem(13, hasUsablePath ? fmtOptionalInt(candidate.beamNumBeams) : QStringLiteral("n/a"));
            setItem(14, hasUsablePath ? fmtOptionalInt(candidate.beamSelectedIndex) : QStringLiteral("n/a"));
            setItem(15, hasUsablePath ? fmtFinite(candidate.beamSelectedGainDb, 2) : QStringLiteral("n/a"));
            setItem(16, hasUsablePath ? fmtOptionalInt(candidate.beamOracleIndex) : QStringLiteral("n/a"));
            setItem(17, hasUsablePath ? fmtFinite(candidate.beamOracleGainDb, 2) : QStringLiteral("n/a"));
            setItem(18, hasUsablePath ? (candidate.beamPredictorStatus.trimmed().isEmpty() ? QStringLiteral("-") : candidate.beamPredictorStatus) : QStringLiteral("-"));
            setItem(19, hasUsablePath ? fmtOptionalInt(candidate.beamPredictedIndex) : QStringLiteral("n/a"));
            setItem(20, hasUsablePath ? fmtFinite(candidate.beamPredictedConfidence, 3) : QStringLiteral("n/a"));
            setItem(21, hasUsablePath ? ackStateLabel(candidate.beamOracleInTopK) : QStringLiteral("n/a"));
            setItem(22, hasUsablePath ? ackStateLabel(candidate.beamHit) : QStringLiteral("n/a"));
            setItem(23, hasUsablePath ? (candidate.beamTopkIndices.trimmed().isEmpty() ? QStringLiteral("-") : candidate.beamTopkIndices) : QStringLiteral("-"));
            setItem(24, hasUsablePath ? (candidate.beamSelectionSource.trimmed().isEmpty() ? QStringLiteral("-") : beamSelectionModeToUi(candidate.beamSelectionSource)) : QStringLiteral("-"));
            setItem(25, fmtOptionalInt(candidate.numPaths));
            setItem(26, fmtFinite(candidate.strongestPathPowerDb, 2));
            setItem(27, candidate.strongestPathType.isEmpty() ? QStringLiteral("n/a") : candidate.strongestPathType);
            setItem(28, QStringLiteral("%1 / %2").arg(fmtOptionalInt(candidate.strongestPathOrder), fmtOptionalInt(candidate.simIdx)));
            ++row;
        }
        if (!sysDataTableLayoutInitialized_ && rowCount > 0) {
            sysDataTable_->resizeColumnsToContents();
            sysDataTable_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
            sysDataTableLayoutInitialized_ = true;
        }
        sysDataTable_->setUpdatesEnabled(true);
        sysDataTable_->viewport()->update();
    }

    if (sysDataRawView_) {
        QJsonObject root;
        root.insert(QStringLiteral("enabled"), currentSysSnapshot_.enabled);
        root.insert(QStringLiteral("valid"), currentSysSnapshot_.valid);
        root.insert(QStringLiteral("scheduler_policy"), currentSysSnapshot_.schedulerPolicy);
        root.insert(QStringLiteral("sim_idx"), currentSysSnapshot_.simIdx);
        root.insert(QStringLiteral("serving_bs_name"), currentSysSnapshot_.servingBsName);
        root.insert(QStringLiteral("serving_anchor_id"), currentSysSnapshot_.servingAnchorId);
        root.insert(QStringLiteral("serving_idx"), currentSysSnapshot_.servingIdx);
        root.insert(QStringLiteral("serving_mcs_index"), currentSysSnapshot_.servingMcsIndex);
        if (std::isfinite(currentSysSnapshot_.servingSinrEffDb)) root.insert(QStringLiteral("serving_sinr_eff_db"), currentSysSnapshot_.servingSinrEffDb);
        if (std::isfinite(currentSysSnapshot_.servingSpectralEfficiencyBpsHz)) root.insert(QStringLiteral("serving_spectral_efficiency_bpshz"), currentSysSnapshot_.servingSpectralEfficiencyBpsHz);
        if (currentSysSnapshot_.servingTbOk >= 0) root.insert(QStringLiteral("serving_tb_ok"), currentSysSnapshot_.servingTbOk == 1);

        QJsonObject config;
        config.insert(QStringLiteral("sys_num_subcarriers"), simSettings_.sysNumSubcarriers);
        config.insert(QStringLiteral("sys_subcarrier_spacing_hz"), simSettings_.sysSubcarrierSpacingHz);
        config.insert(QStringLiteral("sys_num_ofdm_symbols"), simSettings_.sysNumOfdmSymbols);
        config.insert(QStringLiteral("sys_temperature_k"), simSettings_.sysTemperatureK);
        config.insert(QStringLiteral("sys_bler_target"), simSettings_.sysBlerTarget);
        config.insert(QStringLiteral("sys_mcs_table_index"), simSettings_.sysMcsTableIndex);
        config.insert(QStringLiteral("sys_bs_tx_power_dbm"), simSettings_.sysBsTxPowerDbm);
        config.insert(QStringLiteral("enable_beamforming"), simSettings_.enableBeamforming);
        config.insert(QStringLiteral("beam_selection_mode"), simSettings_.beamSelectionMode);
        config.insert(QStringLiteral("beam_codebook_type"), simSettings_.beamCodebookType);
        config.insert(QStringLiteral("beam_codebook_num_beams"), simSettings_.beamCodebookNumBeams);
        config.insert(QStringLiteral("beam_oversampling_v"), simSettings_.beamOversamplingV);
        config.insert(QStringLiteral("beam_oversampling_h"), simSettings_.beamOversamplingH);
        config.insert(QStringLiteral("beam_manual_index"), simSettings_.beamManualIndex);
        config.insert(QStringLiteral("beam_normalize_power"), simSettings_.beamNormalizePower);
        config.insert(QStringLiteral("beam_model_checkpoint_path"), simSettings_.beamModelCheckpointPath);
        config.insert(QStringLiteral("beam_feature_mode"), simSettings_.beamFeatureMode);
        config.insert(QStringLiteral("beam_top_k"), simSettings_.beamTopK);
        config.insert(QStringLiteral("beam_export_training_dataset"), simSettings_.beamExportTrainingDataset);
        config.insert(QStringLiteral("beam_training_dataset_path"), simSettings_.beamTrainingDatasetPath);
        config.insert(QStringLiteral("beam_training_input_paths"), effectiveBeamTrainingInputText(simSettings_.beamTrainingInputPaths, simSettings_.beamTrainingDatasetPath));
        config.insert(QStringLiteral("beam_training_reset_dataset_on_start"), simSettings_.beamTrainingResetDatasetOnStart);
        config.insert(QStringLiteral("beam_training_output_checkpoint_path"), simSettings_.beamTrainingOutputCheckpointPath);
        config.insert(QStringLiteral("beam_training_epochs"), simSettings_.beamTrainingEpochs);
        config.insert(QStringLiteral("beam_training_batch_size"), simSettings_.beamTrainingBatchSize);
        config.insert(QStringLiteral("beam_training_learning_rate"), simSettings_.beamTrainingLearningRate);
        config.insert(QStringLiteral("beam_training_validation_split"), simSettings_.beamTrainingValidationSplit);
        config.insert(QStringLiteral("beam_training_hidden_dim"), simSettings_.beamTrainingHiddenDim);
        root.insert(QStringLiteral("configured_sys"), config);
        root.insert(QStringLiteral("beam_enabled"), currentSysSnapshot_.beamEnabled);
        root.insert(QStringLiteral("beam_selection_mode"), currentSysSnapshot_.beamSelectionMode);
        root.insert(QStringLiteral("beam_codebook_type"), currentSysSnapshot_.beamCodebookType);
        root.insert(QStringLiteral("beam_num_beams"), currentSysSnapshot_.beamNumBeams);
        root.insert(QStringLiteral("beam_reference_bs_name"), currentSysSnapshot_.beamReferenceBsName);
        root.insert(QStringLiteral("beam_reference_anchor_id"), currentSysSnapshot_.beamReferenceAnchorId);
        root.insert(QStringLiteral("beam_reference_tx_idx"), currentSysSnapshot_.beamReferenceTxIdx);
        root.insert(QStringLiteral("beam_selected_index"), currentSysSnapshot_.beamSelectedIndex);
        if (std::isfinite(currentSysSnapshot_.beamSelectedGainDb)) root.insert(QStringLiteral("beam_selected_gain_db"), currentSysSnapshot_.beamSelectedGainDb);
        root.insert(QStringLiteral("beam_oracle_index"), currentSysSnapshot_.beamOracleIndex);
        if (std::isfinite(currentSysSnapshot_.beamOracleGainDb)) root.insert(QStringLiteral("beam_oracle_gain_db"), currentSysSnapshot_.beamOracleGainDb);
        root.insert(QStringLiteral("beam_manual_index"), currentSysSnapshot_.beamManualIndex);
        root.insert(QStringLiteral("beam_serving_source"), currentSysSnapshot_.beamServingSource);
        root.insert(QStringLiteral("beam_predictor_available"), currentSysSnapshot_.beamPredictorAvailable);
        root.insert(QStringLiteral("beam_predictor_status"), currentSysSnapshot_.beamPredictorStatus);
        root.insert(QStringLiteral("beam_predictor_error"), currentSysSnapshot_.beamPredictorError);
        root.insert(QStringLiteral("beam_feature_mode"), currentSysSnapshot_.beamFeatureMode);
        root.insert(QStringLiteral("beam_top_k"), currentSysSnapshot_.beamTopK);
        root.insert(QStringLiteral("beam_predicted_index"), currentSysSnapshot_.beamPredictedIndex);
        if (std::isfinite(currentSysSnapshot_.beamPredictedConfidence)) root.insert(QStringLiteral("beam_predicted_confidence"), currentSysSnapshot_.beamPredictedConfidence);
        root.insert(QStringLiteral("beam_oracle_in_topk"), currentSysSnapshot_.beamOracleInTopK);
        root.insert(QStringLiteral("beam_hit"), currentSysSnapshot_.beamHit);
        root.insert(QStringLiteral("beam_selection_source"), currentSysSnapshot_.beamSelectionSource);
        root.insert(QStringLiteral("beam_topk_indices"), currentSysSnapshot_.beamTopkIndices);

        QJsonArray candidates;
        for (const auto& candidate : currentSysSnapshot_.candidates) {
            QJsonObject obj;
            obj.insert(QStringLiteral("sim_idx"), candidate.simIdx);
            if (std::isfinite(candidate.odomStampS)) obj.insert(QStringLiteral("odom_stamp_s"), candidate.odomStampS);
            obj.insert(QStringLiteral("bs_name"), candidate.bsName);
            obj.insert(QStringLiteral("anchor_id"), candidate.anchorId);
            obj.insert(QStringLiteral("num_paths"), candidate.numPaths);
            if (std::isfinite(candidate.strongestPathPowerDb)) obj.insert(QStringLiteral("power_db"), candidate.strongestPathPowerDb);
            obj.insert(QStringLiteral("path_type"), candidate.strongestPathType);
            obj.insert(QStringLiteral("path_order"), candidate.strongestPathOrder);
            obj.insert(QStringLiteral("sys_enabled"), candidate.sysEnabled);
            if (std::isfinite(candidate.candidateRateBpsHz)) obj.insert(QStringLiteral("sys_candidate_rate_bpshz"), candidate.candidateRateBpsHz);
            if (std::isfinite(candidate.candidateSinrEffDb)) obj.insert(QStringLiteral("sys_candidate_sinr_eff_db"), candidate.candidateSinrEffDb);
            if (candidate.mcsIndex >= 0) obj.insert(QStringLiteral("sys_mcs_index"), candidate.mcsIndex);
            if (candidate.numDecodedBits >= 0) obj.insert(QStringLiteral("sys_num_decoded_bits"), candidate.numDecodedBits);
            if (candidate.tbOk >= 0) obj.insert(QStringLiteral("sys_tb_ok"), candidate.tbOk);
            if (std::isfinite(candidate.spectralEfficiencyBpsHz)) obj.insert(QStringLiteral("sys_spectral_efficiency_bpshz"), candidate.spectralEfficiencyBpsHz);
            obj.insert(QStringLiteral("sys_is_serving_bs"), candidate.isServingBs);
            obj.insert(QStringLiteral("sys_serving_bs_name"), candidate.servingBsName);
            obj.insert(QStringLiteral("sys_serving_anchor_id"), candidate.servingAnchorId);
            if (std::isfinite(candidate.blerTarget)) obj.insert(QStringLiteral("sys_bler_target"), candidate.blerTarget);
            obj.insert(QStringLiteral("beam_enabled"), candidate.beamEnabled);
            obj.insert(QStringLiteral("beam_selection_mode"), candidate.beamSelectionMode);
            obj.insert(QStringLiteral("beam_codebook_type"), candidate.beamCodebookType);
            obj.insert(QStringLiteral("beam_num_beams"), candidate.beamNumBeams);
            obj.insert(QStringLiteral("beam_selected_index"), candidate.beamSelectedIndex);
            if (std::isfinite(candidate.beamSelectedGainDb)) obj.insert(QStringLiteral("beam_selected_gain_db"), candidate.beamSelectedGainDb);
            obj.insert(QStringLiteral("beam_oracle_index"), candidate.beamOracleIndex);
            if (std::isfinite(candidate.beamOracleGainDb)) obj.insert(QStringLiteral("beam_oracle_gain_db"), candidate.beamOracleGainDb);
            obj.insert(QStringLiteral("beam_selected_anchor"), candidate.beamSelectedAnchor);
            obj.insert(QStringLiteral("beam_predictor_status"), candidate.beamPredictorStatus);
            obj.insert(QStringLiteral("beam_feature_mode"), candidate.beamFeatureMode);
            obj.insert(QStringLiteral("beam_top_k"), candidate.beamTopK);
            obj.insert(QStringLiteral("beam_predicted_index"), candidate.beamPredictedIndex);
            if (std::isfinite(candidate.beamPredictedConfidence)) obj.insert(QStringLiteral("beam_predicted_confidence"), candidate.beamPredictedConfidence);
            obj.insert(QStringLiteral("beam_oracle_in_topk"), candidate.beamOracleInTopK);
            obj.insert(QStringLiteral("beam_hit"), candidate.beamHit);
            obj.insert(QStringLiteral("beam_selection_source"), candidate.beamSelectionSource);
            obj.insert(QStringLiteral("beam_topk_indices"), candidate.beamTopkIndices);
            candidates.push_back(obj);
        }
        root.insert(QStringLiteral("candidates"), candidates);
        const QString rawJson = QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Indented));
        if (sysDataLastRawJson_ != rawJson) {
            sysDataLastRawJson_ = rawJson;
            sysDataRawView_->setPlainText(rawJson);
        }
    }

    if (sysDataWindow_) {
        const QString suffix = currentSysSnapshot_.servingBsName.trimmed().isEmpty()
            ? tr("no serving BS")
            : tr("serving %1").arg(currentSysSnapshot_.servingBsName);
        sysDataWindow_->setWindowTitle(tr("Sionna SYS Monitor · %1").arg(suffix));
    }
}

void MainWindow::showSysDataWindow() {
    ensureSysDataWindow();
    sysDataRefreshPending_ = false;
    rebuildSysDataWindow();
    sysDataWindow_->show();
    sysDataWindow_->raise();
    sysDataWindow_->activateWindow();
}


QString MainWindow::defaultRosbagOutputPath(const QString& baseNameHint) const {
    QString dir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    if (dir.trimmed().isEmpty()) {
        dir = QDir::homePath();
    }
    dir = QDir(dir).filePath(QStringLiteral("airsim_gui_rosbags"));
    QDir().mkpath(dir);

    QString stem = baseNameHint.trimmed();
    if (stem.isEmpty()) {
        stem = QStringLiteral("session");
    }
    stem.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9_-]+")), QStringLiteral("_"));
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    return QDir(dir).filePath(QStringLiteral("%1_%2.bag").arg(stem, timestamp));
}

void MainWindow::stopProcessGracefully(QProcess* process, int waitMs) {
    if (!process || process->state() == QProcess::NotRunning) {
        return;
    }

    const qint64 pid = process->processId();
    bool sentSignal = false;
#ifdef Q_OS_UNIX
    if (pid > 0) {
        sentSignal = (::kill(static_cast<pid_t>(pid), SIGINT) == 0);
    }
#endif
    if (!sentSignal) {
        process->terminate();
    }
    if (!process->waitForFinished(waitMs)) {
        process->kill();
        process->waitForFinished(1500);
    }
}

bool MainWindow::ensureRoscoreRunning(QString* errorMessage, int timeoutMs) {
    if (errorMessage) {
        errorMessage->clear();
    }
    if (ros::master::check()) {
        return true;
    }
    if (!managedRoscoreProcess_) {
        if (errorMessage) {
            *errorMessage = tr("roscore process is not available.");
        }
        return false;
    }

    if (managedRoscoreProcess_->state() == QProcess::NotRunning) {
        appendRosbagLog(QStringLiteral("ROS master not detected. Starting roscore..."));
        managedRoscoreProcess_->start(QStringLiteral("roscore"));
        if (!managedRoscoreProcess_->waitForStarted(3000)) {
            if (errorMessage) {
                *errorMessage = tr("Failed to start roscore: %1").arg(managedRoscoreProcess_->errorString());
            }
            return false;
        }
    }

    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents();
        if (ros::master::check()) {
            appendRosbagLog(QStringLiteral("ROS master is ready."));
            return true;
        }
        QThread::msleep(100);
    }

    if (errorMessage) {
        const QString output = QString::fromLocal8Bit(managedRoscoreProcess_->readAllStandardOutput()).trimmed();
        *errorMessage = output.isEmpty()
            ? tr("Timed out waiting for roscore to become available.")
            : tr("Timed out waiting for roscore. Last output: %1").arg(output);
    }
    return false;
}

void MainWindow::appendRosbagLog(const QString& text) {
    const QString trimmed = sanitizeBeamTerminologyForUi(text.trimmed());
    if (trimmed.isEmpty()) {
        return;
    }
    appendRuntimeLogLine(QStringLiteral("rosbag"), trimmed);
    rosbagToolsLog_ += trimmed + QStringLiteral("\n");
    if (rosbagToolsLogView_) {
        rosbagToolsLogView_->appendPlainText(trimmed);
    }
}

void MainWindow::updateRosbagPlaybackTopicPresence() {
    if (!rosbagPlaybackWirelessTopicsValue_) {
        return;
    }
    const QString bagPath = rosbagPlaybackInputEdit_ ? rosbagPlaybackInputEdit_->text().trimmed() : QString();
    if (bagPath.isEmpty()) {
        rosbagPlaybackWirelessTopicsValue_->setText(tr("Select a rosbag to inspect RF/SYS/Beam topics."));
        return;
    }
    if (!QFileInfo::exists(bagPath)) {
        rosbagPlaybackWirelessTopicsValue_->setText(tr("Bag file does not exist: %1").arg(bagPath));
        return;
    }

    QString errorMessage;
    const QVector<RosbagTopicInfo> bagTopicInfos = rosbagTopicInfoInBag(bagPath, &errorMessage);
    if (bagTopicInfos.isEmpty()) {
        rosbagPlaybackWirelessTopicsValue_->setText(errorMessage.isEmpty() ? tr("Failed to inspect rosbag topics.") : errorMessage);
        return;
    }

    const QString rfTopic = rosbagPlaybackRfTopicEdit_ ? rosbagPlaybackRfTopicEdit_->text().trimmed() : internalRfTopic_;
    const QString sysTopic = rosbagPlaybackSysTopicEdit_ ? rosbagPlaybackSysTopicEdit_->text().trimmed() : internalSysTopic_;
    const QString beamTopic = (rosbagPlaybackBeamTopicEdit_ && !rosbagPlaybackBeamTopicEdit_->text().trimmed().isEmpty())
        ? rosbagPlaybackBeamTopicEdit_->text().trimmed()
        : defaultRosbagBeamTopicForSysTopic(sysTopic, internalBeamTopic_);
    const QString beamCodebookTopic = (rosbagPlaybackBeamCodebookTopicEdit_ && !rosbagPlaybackBeamCodebookTopicEdit_->text().trimmed().isEmpty())
        ? rosbagPlaybackBeamCodebookTopicEdit_->text().trimmed()
        : defaultRosbagBeamCodebookTopicForBeamTopic(beamTopic, internalBeamCodebookTopic_);
    const RosbagWirelessTopicPresence presence = rosbagWirelessTopicPresence(
        bagTopicInfos, rfTopic, sysTopic, beamTopic, beamCodebookTopic,
        internalRfTopic_, internalSysTopic_, internalBeamTopic_, internalBeamCodebookTopic_);
    const QString actionText = presence.hasAny()
        ? tr("Recorded wireless topics are present; Start Simulation is disabled during playback.")
        : tr("No recorded RF/SYS/Beam topics; Start Simulation can run internal Sionna simulation from playback poses.");
    rosbagPlaybackWirelessTopicsValue_->setText(QStringLiteral("%1\n%2")
                                                    .arg(rosbagWirelessTopicPresenceText(presence, rfTopic, sysTopic, beamTopic, beamCodebookTopic),
                                                         actionText));
}

void MainWindow::updateRosbagResimTopicPresence() {
    if (!rosbagResimWirelessTopicsValue_) {
        return;
    }
    const QString bagPath = rosbagResimInputEdit_ ? rosbagResimInputEdit_->text().trimmed() : QString();
    if (bagPath.isEmpty()) {
        rosbagResimWirelessTopicsValue_->setText(tr("Select a rosbag to inspect RF/SYS/Beam topics."));
        return;
    }
    if (!QFileInfo::exists(bagPath)) {
        rosbagResimWirelessTopicsValue_->setText(tr("Bag file does not exist: %1").arg(bagPath));
        return;
    }

    QString errorMessage;
    const QVector<RosbagTopicInfo> bagTopicInfos = rosbagTopicInfoInBag(bagPath, &errorMessage);
    if (bagTopicInfos.isEmpty()) {
        rosbagResimWirelessTopicsValue_->setText(errorMessage.isEmpty() ? tr("Failed to inspect rosbag topics.") : errorMessage);
        return;
    }

    const QString rfTopic = rosbagResimRfTopicEdit_ ? rosbagResimRfTopicEdit_->text().trimmed() : internalRfTopic_;
    const QString sysTopic = rosbagResimSysTopicEdit_ ? rosbagResimSysTopicEdit_->text().trimmed() : internalSysTopic_;
    const QString beamTopic = (rosbagResimBeamTopicEdit_ && !rosbagResimBeamTopicEdit_->text().trimmed().isEmpty())
        ? rosbagResimBeamTopicEdit_->text().trimmed()
        : defaultRosbagBeamTopicForSysTopic(sysTopic, internalBeamTopic_);
    const QString beamCodebookTopic = (rosbagResimBeamCodebookTopicEdit_ && !rosbagResimBeamCodebookTopicEdit_->text().trimmed().isEmpty())
        ? rosbagResimBeamCodebookTopicEdit_->text().trimmed()
        : defaultRosbagBeamCodebookTopicForBeamTopic(beamTopic, internalBeamCodebookTopic_);
    const RosbagWirelessTopicPresence presence = rosbagWirelessTopicPresence(
        bagTopicInfos, rfTopic, sysTopic, beamTopic, beamCodebookTopic,
        internalRfTopic_, internalSysTopic_, internalBeamTopic_, internalBeamCodebookTopic_);
    const QString actionText = presence.hasAny()
        ? tr("Recorded wireless topics are present; re-simulation can replace or preserve them according to the generated-topic options.")
        : tr("No recorded RF/SYS/Beam topics; re-simulation can insert generated wireless topics into a new bag.");
    rosbagResimWirelessTopicsValue_->setText(QStringLiteral("%1\n%2")
                                                 .arg(rosbagWirelessTopicPresenceText(presence, rfTopic, sysTopic, beamTopic, beamCodebookTopic),
                                                      actionText));
}

void MainWindow::ensureRosbagToolsWindow() {
    if (rosbagToolsWindow_) {
        if (rosbagPlaybackRfTopicEdit_ && rosbagPlaybackRfTopicEdit_->text().trimmed().isEmpty()) {
            rosbagPlaybackRfTopicEdit_->setText(internalRfTopic_);
        }
        if (rosbagPlaybackSysTopicEdit_ && rosbagPlaybackSysTopicEdit_->text().trimmed().isEmpty()) {
            rosbagPlaybackSysTopicEdit_->setText(internalSysTopic_);
        }
        if (rosbagPlaybackBeamTopicEdit_ && rosbagPlaybackBeamTopicEdit_->text().trimmed().isEmpty()) {
            const QString playbackSysTopic = rosbagPlaybackSysTopicEdit_ ? rosbagPlaybackSysTopicEdit_->text().trimmed() : internalSysTopic_;
            rosbagPlaybackBeamTopicEdit_->setText(defaultRosbagBeamTopicForSysTopic(playbackSysTopic, internalBeamTopic_));
        }
        if (rosbagPlaybackBeamCodebookTopicEdit_ && rosbagPlaybackBeamCodebookTopicEdit_->text().trimmed().isEmpty()) {
            const QString playbackBeamTopic = rosbagPlaybackBeamTopicEdit_ ? rosbagPlaybackBeamTopicEdit_->text().trimmed() : internalBeamTopic_;
            rosbagPlaybackBeamCodebookTopicEdit_->setText(defaultRosbagBeamCodebookTopicForBeamTopic(playbackBeamTopic, internalBeamCodebookTopic_));
        }
        if (rosbagResimPoseTopicEdit_ && rosbagResimPoseTopicEdit_->text().trimmed().isEmpty() && poseTopicCombo_) {
            rosbagResimPoseTopicEdit_->setText(poseTopicCombo_->currentText().trimmed());
        }
        if (rosbagResimRfTopicEdit_ && rosbagResimRfTopicEdit_->text().trimmed().isEmpty()) {
            rosbagResimRfTopicEdit_->setText(internalRfTopic_);
        }
        if (rosbagResimSysTopicEdit_ && rosbagResimSysTopicEdit_->text().trimmed().isEmpty()) {
            rosbagResimSysTopicEdit_->setText(internalSysTopic_);
        }
        if (rosbagResimBeamTopicEdit_ && rosbagResimBeamTopicEdit_->text().trimmed().isEmpty()) {
            const QString resimSysTopic = rosbagResimSysTopicEdit_ ? rosbagResimSysTopicEdit_->text().trimmed() : internalSysTopic_;
            rosbagResimBeamTopicEdit_->setText(defaultRosbagBeamTopicForSysTopic(resimSysTopic, internalBeamTopic_));
        }
        if (rosbagResimBeamCodebookTopicEdit_ && rosbagResimBeamCodebookTopicEdit_->text().trimmed().isEmpty()) {
            const QString resimBeamTopic = rosbagResimBeamTopicEdit_ ? rosbagResimBeamTopicEdit_->text().trimmed() : internalBeamTopic_;
            rosbagResimBeamCodebookTopicEdit_->setText(defaultRosbagBeamCodebookTopicForBeamTopic(resimBeamTopic, internalBeamCodebookTopic_));
        }
        return;
    }

    rosbagToolsWindow_ = new QMainWindow(this, Qt::Window);
    rosbagToolsWindow_->setWindowTitle(tr("Rosbag Tools"));
    rosbagToolsWindow_->resize(900, 760);

    auto* central = new QWidget(rosbagToolsWindow_);
    auto* rootLayout = new QVBoxLayout(central);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    auto* hint = new QLabel(tr("Playback restores recorded flight topics through rosbag play. Re-sim creates a copied bag whose wireless topics are regenerated offline by the current Sionna pipeline. Non-wireless topics are always preserved."), central);
    hint->setWordWrap(true);
    rootLayout->addWidget(hint);

    rosbagToolsTabs_ = new QTabWidget(central);
    rootLayout->addWidget(rosbagToolsTabs_, 1);

    auto* playbackPage = new QWidget(rosbagToolsTabs_);
    auto* playbackLayout = new QFormLayout(playbackPage);
    rosbagPlaybackInputEdit_ = new QLineEdit(playbackPage);
    auto* playbackBrowseButton = new QPushButton(tr("Browse..."), playbackPage);
    connect(playbackBrowseButton, &QPushButton::clicked, this, &MainWindow::browseRosbagPlaybackInput);
    auto* playbackPathRow = new QWidget(playbackPage);
    auto* playbackPathRowLayout = new QHBoxLayout(playbackPathRow);
    playbackPathRowLayout->setContentsMargins(0, 0, 0, 0);
    playbackPathRowLayout->addWidget(rosbagPlaybackInputEdit_, 1);
    playbackPathRowLayout->addWidget(playbackBrowseButton);
    rosbagPlaybackRfTopicEdit_ = new QLineEdit(internalRfTopic_, playbackPage);
    rosbagPlaybackSysTopicEdit_ = new QLineEdit(internalSysTopic_, playbackPage);
    rosbagPlaybackBeamTopicEdit_ = new QLineEdit(defaultRosbagBeamTopicForSysTopic(internalSysTopic_, internalBeamTopic_), playbackPage);
    rosbagPlaybackBeamCodebookTopicEdit_ = new QLineEdit(defaultRosbagBeamCodebookTopicForBeamTopic(rosbagPlaybackBeamTopicEdit_->text(), internalBeamCodebookTopic_), playbackPage);
    rosbagPlaybackIncludeRfCheck_ = new QCheckBox(tr("Play RF topic"), playbackPage);
    rosbagPlaybackIncludeSysCheck_ = new QCheckBox(tr("Play SYS topic"), playbackPage);
    rosbagPlaybackIncludeBeamCheck_ = new QCheckBox(tr("Play beam topics"), playbackPage);
    rosbagPlaybackIncludeRfCheck_->setChecked(true);
    rosbagPlaybackIncludeSysCheck_->setChecked(true);
    rosbagPlaybackIncludeBeamCheck_->setChecked(true);
    auto* playbackTopicRow = new QWidget(playbackPage);
    auto* playbackTopicLayout = new QHBoxLayout(playbackTopicRow);
    playbackTopicLayout->setContentsMargins(0, 0, 0, 0);
    playbackTopicLayout->addWidget(rosbagPlaybackIncludeRfCheck_);
    playbackTopicLayout->addWidget(rosbagPlaybackIncludeSysCheck_);
    playbackTopicLayout->addWidget(rosbagPlaybackIncludeBeamCheck_);
    playbackTopicLayout->addStretch(1);
    rosbagPlaybackClockCheck_ = new QCheckBox(tr("Play clock topic (--clock)"), playbackPage);
    rosbagPlaybackClockCheck_->setChecked(true);
    rosbagPlaybackRateSpin_ = new QDoubleSpinBox(playbackPage);
    rosbagPlaybackRateSpin_->setRange(0.05, 10.0);
    rosbagPlaybackRateSpin_->setDecimals(2);
    rosbagPlaybackRateSpin_->setSingleStep(0.05);
    rosbagPlaybackRateSpin_->setKeyboardTracking(false);
    rosbagPlaybackRateSpin_->setValue(1.0);
    rosbagPlaybackStatusValue_ = new QLabel(tr("Idle"), playbackPage);
    rosbagPlaybackStatusValue_->setWordWrap(true);
    rosbagPlaybackWirelessTopicsValue_ = new QLabel(tr("Select a rosbag to inspect RF/SYS/Beam topics."), playbackPage);
    rosbagPlaybackWirelessTopicsValue_->setWordWrap(true);
    rosbagPlaybackToggleButton_ = new QPushButton(tr("Start Playback"), playbackPage);
    connect(rosbagPlaybackToggleButton_, &QPushButton::clicked, this, &MainWindow::toggleRosbagPlayback);
    connect(rosbagPlaybackInputEdit_, &QLineEdit::editingFinished, this, &MainWindow::updateRosbagPlaybackTopicPresence);
    connect(rosbagPlaybackRfTopicEdit_, &QLineEdit::editingFinished, this, &MainWindow::updateRosbagPlaybackTopicPresence);
    connect(rosbagPlaybackSysTopicEdit_, &QLineEdit::editingFinished, this, &MainWindow::updateRosbagPlaybackTopicPresence);
    connect(rosbagPlaybackBeamTopicEdit_, &QLineEdit::editingFinished, this, &MainWindow::updateRosbagPlaybackTopicPresence);
    connect(rosbagPlaybackBeamCodebookTopicEdit_, &QLineEdit::editingFinished, this, &MainWindow::updateRosbagPlaybackTopicPresence);
    auto* playbackNote = new QLabel(tr("Other bag topics are always played. If the bag contains any RF/SYS/beam topics, playback is treated as recorded wireless data and Start Simulation is disabled until playback stops. If the bag has no RF/SYS/beam topics, playback stays as simple data playback unless you click Start Simulation, which then uses the bag pose stream to drive the internal Sionna simulation."), playbackPage);
    playbackNote->setWordWrap(true);
    playbackLayout->addRow(tr("Input bag"), playbackPathRow);
    playbackLayout->addRow(tr("RF topic"), rosbagPlaybackRfTopicEdit_);
    playbackLayout->addRow(tr("SYS topic"), rosbagPlaybackSysTopicEdit_);
    playbackLayout->addRow(tr("Beam topic"), rosbagPlaybackBeamTopicEdit_);
    playbackLayout->addRow(tr("Beam codebook topic"), rosbagPlaybackBeamCodebookTopicEdit_);
    playbackLayout->addRow(tr("Wireless topics"), playbackTopicRow);
    playbackLayout->addRow(QString(), rosbagPlaybackClockCheck_);
    playbackLayout->addRow(tr("Play rate"), rosbagPlaybackRateSpin_);
    playbackLayout->addRow(tr("Bag wireless topics"), rosbagPlaybackWirelessTopicsValue_);
    playbackLayout->addRow(tr("Status"), rosbagPlaybackStatusValue_);
    playbackLayout->addRow(rosbagPlaybackToggleButton_);
    playbackLayout->addRow(tr("Note"), playbackNote);
    rosbagToolsTabs_->addTab(playbackPage, tr("Replay"));

    auto* resimPage = new QWidget(rosbagToolsTabs_);
    auto* resimLayout = new QFormLayout(resimPage);
    rosbagResimInputEdit_ = new QLineEdit(resimPage);
    auto* resimInputBrowseButton = new QPushButton(tr("Browse..."), resimPage);
    connect(resimInputBrowseButton, &QPushButton::clicked, this, &MainWindow::browseRosbagResimInput);
    auto* resimInputRow = new QWidget(resimPage);
    auto* resimInputRowLayout = new QHBoxLayout(resimInputRow);
    resimInputRowLayout->setContentsMargins(0, 0, 0, 0);
    resimInputRowLayout->addWidget(rosbagResimInputEdit_, 1);
    resimInputRowLayout->addWidget(resimInputBrowseButton);

    rosbagResimOutputEdit_ = new QLineEdit(defaultRosbagOutputPath(QStringLiteral("session_wireless_resim")), resimPage);
    auto* resimOutputBrowseButton = new QPushButton(tr("Browse..."), resimPage);
    connect(resimOutputBrowseButton, &QPushButton::clicked, this, &MainWindow::browseRosbagResimOutput);
    auto* resimOutputRow = new QWidget(resimPage);
    auto* resimOutputRowLayout = new QHBoxLayout(resimOutputRow);
    resimOutputRowLayout->setContentsMargins(0, 0, 0, 0);
    resimOutputRowLayout->addWidget(rosbagResimOutputEdit_, 1);
    resimOutputRowLayout->addWidget(resimOutputBrowseButton);

    rosbagResimPoseTopicEdit_ = new QLineEdit(poseTopicCombo_ ? poseTopicCombo_->currentText().trimmed() : QString(), resimPage);
    rosbagResimRfTopicEdit_ = new QLineEdit(internalRfTopic_, resimPage);
    rosbagResimSysTopicEdit_ = new QLineEdit(internalSysTopic_, resimPage);
    rosbagResimBeamTopicEdit_ = new QLineEdit(defaultRosbagBeamTopicForSysTopic(internalSysTopic_, internalBeamTopic_), resimPage);
    rosbagResimBeamCodebookTopicEdit_ = new QLineEdit(defaultRosbagBeamCodebookTopicForBeamTopic(rosbagResimBeamTopicEdit_->text(), internalBeamCodebookTopic_), resimPage);
    rosbagResimWriteRfCheck_ = new QCheckBox(tr("Write RF topic"), resimPage);
    rosbagResimWriteSysCheck_ = new QCheckBox(tr("Write SYS topic"), resimPage);
    rosbagResimWriteBeamCheck_ = new QCheckBox(tr("Write beam topics"), resimPage);
    rosbagResimWriteRfCheck_->setChecked(true);
    rosbagResimWriteSysCheck_->setChecked(true);
    rosbagResimWriteBeamCheck_->setChecked(true);
    auto* resimTopicRow = new QWidget(resimPage);
    auto* resimTopicLayout = new QHBoxLayout(resimTopicRow);
    resimTopicLayout->setContentsMargins(0, 0, 0, 0);
    resimTopicLayout->addWidget(rosbagResimWriteRfCheck_);
    resimTopicLayout->addWidget(rosbagResimWriteSysCheck_);
    resimTopicLayout->addWidget(rosbagResimWriteBeamCheck_);
    resimTopicLayout->addStretch(1);
    rosbagResimMessageFrequencySpin_ = new QDoubleSpinBox(resimPage);
    rosbagResimMessageFrequencySpin_->setRange(0.01, 1000.0);
    rosbagResimMessageFrequencySpin_->setDecimals(3);
    rosbagResimMessageFrequencySpin_->setSingleStep(1.0);
    rosbagResimMessageFrequencySpin_->setSuffix(tr(" Hz"));
    rosbagResimMessageFrequencySpin_->setKeyboardTracking(false);
    rosbagResimMessageFrequencySpin_->setValue(10.0);
    rosbagResimStatusValue_ = new QLabel(tr("Idle"), resimPage);
    rosbagResimStatusValue_->setWordWrap(true);
    rosbagResimWirelessTopicsValue_ = new QLabel(tr("Select a rosbag to inspect RF/SYS/Beam topics."), resimPage);
    rosbagResimWirelessTopicsValue_->setWordWrap(true);
    rosbagResimProgressBar_ = new QProgressBar(resimPage);
    rosbagResimProgressBar_->setRange(0, 100);
    rosbagResimProgressBar_->setValue(0);
    rosbagResimToggleButton_ = new QPushButton(tr("Start Re-simulation"), resimPage);
    connect(rosbagResimToggleButton_, &QPushButton::clicked, this, &MainWindow::toggleRosbagResim);
    connect(rosbagResimInputEdit_, &QLineEdit::editingFinished, this, &MainWindow::updateRosbagResimTopicPresence);
    connect(rosbagResimRfTopicEdit_, &QLineEdit::editingFinished, this, &MainWindow::updateRosbagResimTopicPresence);
    connect(rosbagResimSysTopicEdit_, &QLineEdit::editingFinished, this, &MainWindow::updateRosbagResimTopicPresence);
    connect(rosbagResimBeamTopicEdit_, &QLineEdit::editingFinished, this, &MainWindow::updateRosbagResimTopicPresence);
    connect(rosbagResimBeamCodebookTopicEdit_, &QLineEdit::editingFinished, this, &MainWindow::updateRosbagResimTopicPresence);
    auto* resimNote = new QLabel(tr("The original bag is kept untouched. A new bag is written by copying all original topics and optionally replacing or inserting the RF, SYS, and beam topics with offline Sionna results sampled at the target message frequency. This also works when the input bag has no wireless topics yet."), resimPage);
    resimNote->setWordWrap(true);
    resimLayout->addRow(tr("Input bag"), resimInputRow);
    resimLayout->addRow(tr("Output bag"), resimOutputRow);
    resimLayout->addRow(tr("Pose topic"), rosbagResimPoseTopicEdit_);
    resimLayout->addRow(tr("RF topic"), rosbagResimRfTopicEdit_);
    resimLayout->addRow(tr("SYS topic"), rosbagResimSysTopicEdit_);
    resimLayout->addRow(tr("Beam topic"), rosbagResimBeamTopicEdit_);
    resimLayout->addRow(tr("Beam codebook topic"), rosbagResimBeamCodebookTopicEdit_);
    resimLayout->addRow(tr("Generated topics"), resimTopicRow);
    resimLayout->addRow(tr("Message frequency"), rosbagResimMessageFrequencySpin_);
    resimLayout->addRow(tr("Bag wireless topics"), rosbagResimWirelessTopicsValue_);
    resimLayout->addRow(tr("Progress"), rosbagResimProgressBar_);
    resimLayout->addRow(tr("Status"), rosbagResimStatusValue_);
    resimLayout->addRow(rosbagResimToggleButton_);
    resimLayout->addRow(tr("Note"), resimNote);
    rosbagToolsTabs_->addTab(resimPage, tr("Re-sim Wireless"));

    auto* logPage = new QWidget(rosbagToolsTabs_);
    auto* logLayout = new QVBoxLayout(logPage);
    rosbagToolsLogView_ = new QPlainTextEdit(logPage);
    rosbagToolsLogView_->setReadOnly(true);
    rosbagToolsLogView_->setMaximumBlockCount(2000);
    rosbagToolsLogView_->setPlaceholderText(tr("Rosbag playback / processing logs will appear here."));
    if (!rosbagToolsLog_.trimmed().isEmpty()) {
        rosbagToolsLogView_->setPlainText(rosbagToolsLog_.trimmed());
    }
    logLayout->addWidget(rosbagToolsLogView_);
    rosbagToolsTabs_->addTab(logPage, tr("Log"));

    rosbagToolsWindow_->setCentralWidget(central);
    updateRosbagPlaybackTopicPresence();
    updateRosbagResimTopicPresence();
    updateRosbagUiState();
}

void MainWindow::updateRosbagUiState() {
    if (rosbagRecordToggleButton_) {
        rosbagRecordToggleButton_->setText(rosbagRecordProcess_ && rosbagRecordProcess_->state() != QProcess::NotRunning
            ? tr("Stop Recording")
            : tr("Start Recording"));
    }
    if (rosbagRecordStatusValue_) {
        if (rosbagRecordProcess_ && rosbagRecordProcess_->state() != QProcess::NotRunning) {
            rosbagRecordStatusValue_->setText(tr("Recording to %1").arg(rosbagRecordPathEdit_ ? rosbagRecordPathEdit_->text().trimmed() : QString()));
        } else if (rosbagRecordStatusValue_->text().trimmed().isEmpty()) {
            rosbagRecordStatusValue_->setText(tr("Idle"));
        }
    }

    if (rosbagPlaybackToggleButton_) {
        rosbagPlaybackToggleButton_->setText(rosbagPlayProcess_ && rosbagPlayProcess_->state() != QProcess::NotRunning
            ? tr("Stop Playback")
            : tr("Start Playback"));
    }
    if (rosbagPlaybackStatusValue_ && rosbagPlaybackStatusValue_->text().trimmed().isEmpty()) {
        rosbagPlaybackStatusValue_->setText(tr("Idle"));
    }

    if (rosbagResimToggleButton_) {
        rosbagResimToggleButton_->setText(rosbagResimProcess_ && rosbagResimProcess_->state() != QProcess::NotRunning
            ? tr("Stop Re-simulation")
            : tr("Start Re-simulation"));
    }
    if (rosbagResimStatusValue_ && rosbagResimStatusValue_->text().trimmed().isEmpty()) {
        rosbagResimStatusValue_->setText(tr("Idle"));
    }
}

void MainWindow::browseRosbagRecordOutput() {
    const QString current = rosbagRecordPathEdit_ ? rosbagRecordPathEdit_->text().trimmed() : defaultRosbagOutputPath(QStringLiteral("session"));
    const QString path = QFileDialog::getSaveFileName(this, tr("Select rosbag output"), current, tr("ROS bag (*.bag)"));
    if (!path.isEmpty() && rosbagRecordPathEdit_) {
        rosbagRecordPathEdit_->setText(path.endsWith(QStringLiteral(".bag")) ? path : path + QStringLiteral(".bag"));
    }
}

void MainWindow::toggleRosbagRecording() {
    if (rosbagRecordProcess_ && rosbagRecordProcess_->state() != QProcess::NotRunning) {
        if (rosbagRecordStatusValue_) {
            rosbagRecordStatusValue_->setText(tr("Stopping recorder..."));
        }
        appendRosbagLog(QStringLiteral("Stopping rosbag record."));
        stopProcessGracefully(rosbagRecordProcess_, 5000);
        updateRosbagUiState();
        return;
    }

    QString outputPath = rosbagRecordPathEdit_ ? rosbagRecordPathEdit_->text().trimmed() : QString();
    if (outputPath.isEmpty()) {
        outputPath = defaultRosbagOutputPath(QStringLiteral("session"));
        if (rosbagRecordPathEdit_) {
            rosbagRecordPathEdit_->setText(outputPath);
        }
    }
    if (!outputPath.endsWith(QStringLiteral(".bag"), Qt::CaseInsensitive)) {
        outputPath += QStringLiteral(".bag");
        if (rosbagRecordPathEdit_) {
            rosbagRecordPathEdit_->setText(outputPath);
        }
    }
    QDir().mkpath(QFileInfo(outputPath).absolutePath());

    QStringList args = {QStringLiteral("record"), QStringLiteral("-a")};
    QStringList excludedTopics;
    if (!(rosbagRecordIncludeRfCheck_ && rosbagRecordIncludeRfCheck_->isChecked())) {
        excludedTopics << internalRfTopic_;
    }
    if (!(rosbagRecordIncludeSysCheck_ && rosbagRecordIncludeSysCheck_->isChecked())) {
        excludedTopics << internalSysTopic_;
    }
    if (!(rosbagRecordIncludeBeamCheck_ && rosbagRecordIncludeBeamCheck_->isChecked())) {
        excludedTopics << internalBeamTopic_ << internalBeamCodebookTopic_;
    }
    const QString excludeRegex = rosbagExcludeRegex(excludedTopics);
    if (!excludeRegex.isEmpty()) {
        args << QStringLiteral("-x") << excludeRegex;
    }
    args << QStringLiteral("-O") << outputPath;
    appendRosbagLog(QStringLiteral("Starting rosbag record: rosbag %1").arg(args.join(QStringLiteral(" "))));
    rosbagRecordProcess_->start(QStringLiteral("rosbag"), args);
    if (!rosbagRecordProcess_->waitForStarted(3000)) {
        const QString message = tr("Failed to start rosbag record: %1").arg(rosbagRecordProcess_->errorString());
        if (rosbagRecordStatusValue_) {
            rosbagRecordStatusValue_->setText(message);
        }
        appendRosbagLog(message);
        updateRosbagUiState();
        return;
    }
    if (rosbagRecordStatusValue_) {
        rosbagRecordStatusValue_->setText(tr("Recording to %1").arg(outputPath));
    }
    onStatusMessage(tr("Rosbag recording started: %1").arg(outputPath));
    updateRosbagUiState();
}

void MainWindow::showRosbagToolsWindow() {
    ensureRosbagToolsWindow();
    if (rosbagPlaybackRfTopicEdit_ && rosbagPlaybackRfTopicEdit_->text().trimmed().isEmpty()) {
        rosbagPlaybackRfTopicEdit_->setText(internalRfTopic_);
    }
    if (rosbagPlaybackSysTopicEdit_ && rosbagPlaybackSysTopicEdit_->text().trimmed().isEmpty()) {
        rosbagPlaybackSysTopicEdit_->setText(internalSysTopic_);
    }
    if (rosbagPlaybackBeamTopicEdit_ && rosbagPlaybackBeamTopicEdit_->text().trimmed().isEmpty()) {
        const QString playbackSysTopic = rosbagPlaybackSysTopicEdit_ ? rosbagPlaybackSysTopicEdit_->text().trimmed() : internalSysTopic_;
        rosbagPlaybackBeamTopicEdit_->setText(defaultRosbagBeamTopicForSysTopic(playbackSysTopic, internalBeamTopic_));
    }
    if (rosbagPlaybackBeamCodebookTopicEdit_ && rosbagPlaybackBeamCodebookTopicEdit_->text().trimmed().isEmpty()) {
        const QString playbackBeamTopic = rosbagPlaybackBeamTopicEdit_ ? rosbagPlaybackBeamTopicEdit_->text().trimmed() : internalBeamTopic_;
        rosbagPlaybackBeamCodebookTopicEdit_->setText(defaultRosbagBeamCodebookTopicForBeamTopic(playbackBeamTopic, internalBeamCodebookTopic_));
    }
    if (rosbagResimPoseTopicEdit_ && rosbagResimPoseTopicEdit_->text().trimmed().isEmpty() && poseTopicCombo_) {
        rosbagResimPoseTopicEdit_->setText(poseTopicCombo_->currentText().trimmed());
    }
    if (rosbagResimRfTopicEdit_ && rosbagResimRfTopicEdit_->text().trimmed().isEmpty()) {
        rosbagResimRfTopicEdit_->setText(internalRfTopic_);
    }
    if (rosbagResimSysTopicEdit_ && rosbagResimSysTopicEdit_->text().trimmed().isEmpty()) {
        rosbagResimSysTopicEdit_->setText(internalSysTopic_);
    }
    if (rosbagResimBeamTopicEdit_ && rosbagResimBeamTopicEdit_->text().trimmed().isEmpty()) {
        const QString resimSysTopic = rosbagResimSysTopicEdit_ ? rosbagResimSysTopicEdit_->text().trimmed() : internalSysTopic_;
        rosbagResimBeamTopicEdit_->setText(defaultRosbagBeamTopicForSysTopic(resimSysTopic, internalBeamTopic_));
    }
    if (rosbagResimBeamCodebookTopicEdit_ && rosbagResimBeamCodebookTopicEdit_->text().trimmed().isEmpty()) {
        const QString resimBeamTopic = rosbagResimBeamTopicEdit_ ? rosbagResimBeamTopicEdit_->text().trimmed() : internalBeamTopic_;
        rosbagResimBeamCodebookTopicEdit_->setText(defaultRosbagBeamCodebookTopicForBeamTopic(resimBeamTopic, internalBeamCodebookTopic_));
    }
    updateRosbagPlaybackTopicPresence();
    updateRosbagResimTopicPresence();
    rosbagToolsWindow_->show();
    rosbagToolsWindow_->raise();
    rosbagToolsWindow_->activateWindow();
}

void MainWindow::browseRosbagPlaybackInput() {
    ensureRosbagToolsWindow();
    const QString current = rosbagPlaybackInputEdit_ ? rosbagPlaybackInputEdit_->text().trimmed() : QString();
    const QString path = QFileDialog::getOpenFileName(rosbagToolsWindow_ ? rosbagToolsWindow_ : this, tr("Select rosbag for playback"), current, tr("ROS bag (*.bag)"));
    if (!path.isEmpty() && rosbagPlaybackInputEdit_) {
        rosbagPlaybackInputEdit_->setText(path);
        updateRosbagPlaybackTopicPresence();
    }
}

bool MainWindow::connectRosBridgeOnly(const QString& poseTopic, const QString& trajectoryTopic, const QString& rfTopic,
                                      const QString& sysTopic, const QString& beamTopic, QString* errorMessage, int timeoutMs) {
    auto waitForType = [timeoutMs](const QString& topicName, const std::function<bool(const QString&)>& checker, QString* resolvedType) -> bool {
        if (resolvedType) {
            resolvedType->clear();
        }
        QElapsedTimer timer;
        timer.start();
        while (timer.elapsed() < timeoutMs) {
            QCoreApplication::processEvents();
            const QString type = RosBridge::topicType(topicName);
            if (!type.isEmpty() && checker(type)) {
                if (resolvedType) {
                    *resolvedType = type;
                }
                return true;
            }
            QThread::msleep(100);
        }
        const QString type = RosBridge::topicType(topicName);
        if (resolvedType) {
            *resolvedType = type;
        }
        return !type.isEmpty() && checker(type);
    };

    QString poseType;
    QString trajectoryType;
    QString rfType;
    QString sysType;
    if (!waitForType(poseTopic, [](const QString& type) { return RosBridge::isSupportedPoseType(type); }, &poseType)) {
        if (errorMessage) {
            *errorMessage = tr("Pose topic %1 did not appear with a supported type.").arg(poseTopic);
        }
        return false;
    }
    QString trajectoryTopicResolved = trajectoryTopic.trimmed();
    if (!trajectoryTopicResolved.isEmpty()) {
        if (!waitForType(trajectoryTopicResolved, [](const QString& type) { return RosBridge::isSupportedTrajectoryType(type); }, &trajectoryType)) {
            if (errorMessage) {
                *errorMessage = tr("Trajectory topic %1 did not appear with a supported type.").arg(trajectoryTopicResolved);
            }
            return false;
        }
    }
    QString rfTopicResolved = rfTopic.trimmed();
    if (!rfTopicResolved.isEmpty()) {
        if (!waitForType(rfTopicResolved, [](const QString& type) { return RosBridge::isSupportedRfType(type); }, &rfType)) {
            if (errorMessage) {
                *errorMessage = tr("RF topic %1 did not appear with a supported type.").arg(rfTopicResolved);
            }
            return false;
        }
    }

    QString sysTopicResolved = sysTopic.trimmed();
    if (!sysTopicResolved.isEmpty()) {
        if (!waitForType(sysTopicResolved, [](const QString& type) { return RosBridge::isSupportedSysType(type); }, &sysType)) {
            if (errorMessage) {
                *errorMessage = tr("SYS topic %1 did not appear with a supported type.").arg(sysTopicResolved);
            }
            return false;
        }
    }

    QString beamTopicResolved = beamTopic.trimmed();
    QString beamType;
    if (!beamTopicResolved.isEmpty()) {
        waitForType(beamTopicResolved, [](const QString& type) { return RosBridge::isSupportedBeamType(type); }, &beamType);
        if (!RosBridge::isSupportedBeamType(beamType)) {
            beamTopicResolved.clear();
        }
    }

    rosBridge_->stop();
    if (!rosBridge_->start(poseTopic, trajectoryTopicResolved, rfTopicResolved, sysTopicResolved, beamTopicResolved)) {
        if (errorMessage) {
            *errorMessage = tr("Failed to connect GUI to playback topics.");
        }
        return false;
    }
    if (rfTopicResolved.isEmpty()) {
        currentPaths_.clear();
        currentRfSnapshot_ = RfObservationSnapshot{};
        sceneWidget_->setBeamPaths(currentPaths_);
        if (airSimViewController_) {
            airSimViewController_->setBeamPaths(currentPaths_);
        }
        rebuildRfDataCards();
    }
    if (sysTopicResolved.isEmpty()) {
        sysModeSummary_ = QStringLiteral("Disabled");
        sysServingSummary_ = QStringLiteral("-");
        sysLinkSummary_ = QStringLiteral("-");
        currentSysSnapshot_ = SysSnapshotView{};
        rebuildSysDataWindow();
    }
    refreshInfoPanel();
    return true;
}

void MainWindow::toggleRosbagPlayback() {
    ensureRosbagToolsWindow();
    if (rosbagPlayProcess_ && rosbagPlayProcess_->state() != QProcess::NotRunning) {
        if (rosbagPlaybackStatusValue_) {
            rosbagPlaybackStatusValue_->setText(tr("Stopping playback..."));
        }
        appendRosbagLog(QStringLiteral("Stopping rosbag playback."));
        stopProcessGracefully(rosbagPlayProcess_, 5000);
        updateRosbagUiState();
        return;
    }

    const QString bagPath = rosbagPlaybackInputEdit_ ? rosbagPlaybackInputEdit_->text().trimmed() : QString();
    const QString rfTopic = rosbagPlaybackRfTopicEdit_ ? rosbagPlaybackRfTopicEdit_->text().trimmed() : internalRfTopic_;
    const QString sysTopic = rosbagPlaybackSysTopicEdit_ ? rosbagPlaybackSysTopicEdit_->text().trimmed() : internalSysTopic_;
    const QString beamTopic = (rosbagPlaybackBeamTopicEdit_ && !rosbagPlaybackBeamTopicEdit_->text().trimmed().isEmpty())
        ? rosbagPlaybackBeamTopicEdit_->text().trimmed()
        : defaultRosbagBeamTopicForSysTopic(sysTopic, internalBeamTopic_);
    const QString beamCodebookTopic = (rosbagPlaybackBeamCodebookTopicEdit_ && !rosbagPlaybackBeamCodebookTopicEdit_->text().trimmed().isEmpty())
        ? rosbagPlaybackBeamCodebookTopicEdit_->text().trimmed()
        : defaultRosbagBeamCodebookTopicForBeamTopic(beamTopic, internalBeamCodebookTopic_);
    const bool includeRf = !rosbagPlaybackIncludeRfCheck_ || rosbagPlaybackIncludeRfCheck_->isChecked();
    const bool includeSys = !rosbagPlaybackIncludeSysCheck_ || rosbagPlaybackIncludeSysCheck_->isChecked();
    const bool includeBeam = !rosbagPlaybackIncludeBeamCheck_ || rosbagPlaybackIncludeBeamCheck_->isChecked();
    const QString poseTopic = poseTopicCombo_ ? poseTopicCombo_->currentText().trimmed() : QString();
    const QString trajectoryTopic = trajectoryTopicCombo_ ? trajectoryTopicCombo_->currentText().trimmed() : QString();
    if (bagPath.isEmpty() || !QFileInfo::exists(bagPath)) {
        if (rosbagPlaybackStatusValue_) {
            rosbagPlaybackStatusValue_->setText(tr("Select an existing input bag first."));
        }
        return;
    }
    if (poseTopic.isEmpty()) {
        if (rosbagPlaybackStatusValue_) {
            rosbagPlaybackStatusValue_->setText(tr("Pose topic must not be empty."));
        }
        return;
    }

    QString roscoreError;
    if (!ensureRoscoreRunning(&roscoreError, 10000)) {
        if (rosbagPlaybackStatusValue_) {
            rosbagPlaybackStatusValue_->setText(roscoreError);
        }
        appendRosbagLog(roscoreError);
        return;
    }

    QString bagTopicError;
    const QVector<RosbagTopicInfo> bagTopicInfos = rosbagTopicInfoInBag(bagPath, &bagTopicError);
    const QStringList bagTopics = rosbagTopicNames(bagTopicInfos);
    if (bagTopicInfos.isEmpty()) {
        if (rosbagPlaybackStatusValue_) {
            rosbagPlaybackStatusValue_->setText(bagTopicError.isEmpty() ? tr("Failed to inspect rosbag topics.") : bagTopicError);
        }
        if (!bagTopicError.isEmpty()) {
            appendRosbagLog(bagTopicError);
        }
        return;
    }

    const RosbagWirelessTopicPresence wirelessPresence = rosbagWirelessTopicPresence(
        bagTopicInfos, rfTopic, sysTopic, beamTopic, beamCodebookTopic,
        internalRfTopic_, internalSysTopic_, internalBeamTopic_, internalBeamCodebookTopic_);
    const QString rfTopicInBag = wirelessPresence.rfTopic;
    const QString sysTopicInBag = wirelessPresence.sysTopic;
    const QString beamTopicInBag = wirelessPresence.beamTopic;
    const QString beamCodebookTopicInBag = wirelessPresence.beamCodebookTopic;
    const bool bagHasWirelessTopics = wirelessPresence.hasAny();

    QStringList selectedTopics = bagTopics;
    if (!includeRf) {
        const QStringList rfTopicsToFilter = {rfTopicInBag, rfTopic, internalRfTopic_};
        for (const QString& topic : rfTopicsToFilter) {
            const QString trimmed = topic.trimmed();
            if (!trimmed.isEmpty()) {
                selectedTopics.removeAll(trimmed);
            }
        }
    }
    if (!includeSys) {
        const QStringList sysTopicsToFilter = {sysTopicInBag, sysTopic, internalSysTopic_};
        for (const QString& topic : sysTopicsToFilter) {
            const QString trimmed = topic.trimmed();
            if (!trimmed.isEmpty()) {
                selectedTopics.removeAll(trimmed);
            }
        }
    }
    if (!includeBeam) {
        const QStringList beamTopicsToFilter = {
            beamTopicInBag,
            beamCodebookTopicInBag,
            beamTopic,
            beamCodebookTopic,
            defaultRosbagBeamTopicForSysTopic(sysTopic, internalBeamTopic_),
            defaultRosbagBeamCodebookTopicForBeamTopic(beamTopic, internalBeamCodebookTopic_),
            internalBeamTopic_,
            internalBeamCodebookTopic_,
        };
        for (const QString& topic : beamTopicsToFilter) {
            const QString trimmed = topic.trimmed();
            if (!trimmed.isEmpty()) {
                selectedTopics.removeAll(trimmed);
            }
        }
    }
    if (selectedTopics.isEmpty()) {
        if (rosbagPlaybackStatusValue_) {
            rosbagPlaybackStatusValue_->setText(tr("All topics were filtered out. Enable at least one topic set to play."));
        }
        return;
    }

    stopDemo();
    rosBridge_->stop();
    stopInternalSimulator();
    resetLiveSessionState(true);
    rosbagPlaybackActive_ = false;
    rosbagPlaybackBagHasWirelessTopics_ = bagHasWirelessTopics;
    rosbagPlaybackUsingRfTopic_ = false;
    rosbagPlaybackUsingSysTopic_ = false;
    rosbagPlaybackUsingBeamTopic_ = false;
    rosbagPlaybackPoseTopic_.clear();
    rosbagPlaybackTrajectoryTopic_.clear();
    QStringList args = {QStringLiteral("play"), QStringLiteral("--delay=2.0")};
    if (rosbagPlaybackClockCheck_ && rosbagPlaybackClockCheck_->isChecked()) {
        args << QStringLiteral("--clock");
    }
    args << QStringLiteral("--rate=%1").arg(QString::number(rosbagPlaybackRateSpin_ ? rosbagPlaybackRateSpin_->value() : 1.0, 'f', 2))
         << bagPath;
    if (selectedTopics != bagTopics) {
        args << selectedTopics;
    }
    appendRosbagLog(QStringLiteral("Starting rosbag play: rosbag %1").arg(args.join(QStringLiteral(" "))));
    rosbagPlayProcess_->start(QStringLiteral("rosbag"), args);
    if (!rosbagPlayProcess_->waitForStarted(3000)) {
        const QString message = tr("Failed to start rosbag play: %1").arg(rosbagPlayProcess_->errorString());
        if (rosbagPlaybackStatusValue_) {
            rosbagPlaybackStatusValue_->setText(message);
        }
        appendRosbagLog(message);
        rosbagPlaybackBagHasWirelessTopics_ = false;
        rosbagPlaybackPoseTopic_.clear();
        rosbagPlaybackTrajectoryTopic_.clear();
        updateRosbagUiState();
        return;
    }

    if (bagHasWirelessTopics) {
        QStringList wirelessTopics;
        if (!rfTopicInBag.isEmpty()) {
            wirelessTopics << QStringLiteral("RF=%1").arg(rfTopicInBag);
        }
        if (!sysTopicInBag.isEmpty()) {
            wirelessTopics << QStringLiteral("SYS=%1").arg(sysTopicInBag);
        }
        if (!beamTopicInBag.isEmpty()) {
            wirelessTopics << QStringLiteral("Beam=%1").arg(beamTopicInBag);
        }
        if (!beamCodebookTopicInBag.isEmpty()) {
            wirelessTopics << QStringLiteral("Codebook=%1").arg(beamCodebookTopicInBag);
        }
        appendRosbagLog(QStringLiteral("Detected recorded wireless topics in bag: %1. Start Simulation is disabled during this playback.")
                            .arg(wirelessTopics.join(QStringLiteral("; "))));
    } else {
        appendRosbagLog(QStringLiteral("No recorded RF/SYS/Beam topics detected in this bag. You may click Start Simulation during playback to run internal Sionna simulation from the bag pose stream."));
    }

    const QString rfTopicForGui = (includeRf && !rfTopicInBag.isEmpty()) ? rfTopicInBag : QString();
    const QString sysTopicForGui = (includeSys && !sysTopicInBag.isEmpty()) ? sysTopicInBag : QString();
    const QString beamTopicForGui = (includeBeam && !beamTopicInBag.isEmpty()) ? beamTopicInBag : QString();
    const QString trajectoryTopicForGui = (!trajectoryTopic.isEmpty() && bagTopics.contains(trajectoryTopic)) ? trajectoryTopic : QString();
    QString errorMessage;
    if (!connectRosBridgeOnly(poseTopic, trajectoryTopicForGui, rfTopicForGui, sysTopicForGui, beamTopicForGui, &errorMessage, 8000)) {
        stopProcessGracefully(rosbagPlayProcess_, 2000);
        if (rosbagPlaybackStatusValue_) {
            rosbagPlaybackStatusValue_->setText(errorMessage);
        }
        appendRosbagLog(errorMessage);
        rosbagPlaybackBagHasWirelessTopics_ = false;
        rosbagPlaybackPoseTopic_.clear();
        rosbagPlaybackTrajectoryTopic_.clear();
        updateRosbagUiState();
        return;
    }

    rosbagPlaybackActive_ = true;
    rosbagPlaybackUsingRfTopic_ = !rfTopicForGui.isEmpty();
    rosbagPlaybackUsingSysTopic_ = !sysTopicForGui.isEmpty();
    rosbagPlaybackUsingBeamTopic_ = !beamTopicForGui.isEmpty();
    rosbagPlaybackPoseTopic_ = poseTopic;
    rosbagPlaybackTrajectoryTopic_ = trajectoryTopicForGui;
    refreshInfoPanel();

    if (rosbagPlaybackStatusValue_) {
        rosbagPlaybackStatusValue_->setText(bagHasWirelessTopics
            ? tr("Playing %1. Recorded wireless topics are present; Start Simulation is disabled until playback stops.").arg(QFileInfo(bagPath).fileName())
            : tr("Playing %1. No recorded RF/SYS/Beam topics; click Start Simulation to run internal Sionna simulation from playback poses.").arg(QFileInfo(bagPath).fileName()));
    }
    onStatusMessage(tr("Rosbag playback started: %1").arg(QFileInfo(bagPath).fileName()));
    updateRosbagUiState();
}

void MainWindow::browseRosbagResimInput() {
    ensureRosbagToolsWindow();
    const QString current = rosbagResimInputEdit_ ? rosbagResimInputEdit_->text().trimmed() : QString();
    const QString path = QFileDialog::getOpenFileName(rosbagToolsWindow_ ? rosbagToolsWindow_ : this, tr("Select input rosbag"), current, tr("ROS bag (*.bag)"));
    if (!path.isEmpty() && rosbagResimInputEdit_) {
        rosbagResimInputEdit_->setText(path);
        if (rosbagResimOutputEdit_ && rosbagResimOutputEdit_->text().trimmed().isEmpty()) {
            rosbagResimOutputEdit_->setText(defaultRosbagOutputPath(QFileInfo(path).completeBaseName() + QStringLiteral("_wireless_resim")));
        }
        updateRosbagResimTopicPresence();
    }
}

void MainWindow::browseRosbagResimOutput() {
    ensureRosbagToolsWindow();
    const QString current = rosbagResimOutputEdit_ ? rosbagResimOutputEdit_->text().trimmed() : defaultRosbagOutputPath(QStringLiteral("session_wireless_resim"));
    const QString path = QFileDialog::getSaveFileName(rosbagToolsWindow_ ? rosbagToolsWindow_ : this, tr("Select output rosbag"), current, tr("ROS bag (*.bag)"));
    if (!path.isEmpty() && rosbagResimOutputEdit_) {
        rosbagResimOutputEdit_->setText(path.endsWith(QStringLiteral(".bag")) ? path : path + QStringLiteral(".bag"));
    }
}

void MainWindow::toggleRosbagResim() {
    ensureRosbagToolsWindow();
    if (rosbagResimProcess_ && rosbagResimProcess_->state() != QProcess::NotRunning) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("Stopping after current snapshot..."));
        }
        appendRosbagLog(QStringLiteral("Stopping rosbag re-simulation."));
        stopProcessGracefully(rosbagResimProcess_, 5000);
        updateRosbagUiState();
        return;
    }

    const QString inputBag = rosbagResimInputEdit_ ? rosbagResimInputEdit_->text().trimmed() : QString();
    QString outputBag = rosbagResimOutputEdit_ ? rosbagResimOutputEdit_->text().trimmed() : QString();
    const QString poseTopic = rosbagResimPoseTopicEdit_ ? rosbagResimPoseTopicEdit_->text().trimmed() : QString();
    const QString rfTopic = rosbagResimRfTopicEdit_ ? rosbagResimRfTopicEdit_->text().trimmed() : internalRfTopic_;
    const QString sysTopic = rosbagResimSysTopicEdit_ ? rosbagResimSysTopicEdit_->text().trimmed() : internalSysTopic_;
    const QString beamTopic = (rosbagResimBeamTopicEdit_ && !rosbagResimBeamTopicEdit_->text().trimmed().isEmpty())
        ? rosbagResimBeamTopicEdit_->text().trimmed()
        : defaultRosbagBeamTopicForSysTopic(sysTopic, internalBeamTopic_);
    const QString beamCodebookTopic = (rosbagResimBeamCodebookTopicEdit_ && !rosbagResimBeamCodebookTopicEdit_->text().trimmed().isEmpty())
        ? rosbagResimBeamCodebookTopicEdit_->text().trimmed()
        : defaultRosbagBeamCodebookTopicForBeamTopic(beamTopic, internalBeamCodebookTopic_);
    const bool writeRf = !rosbagResimWriteRfCheck_ || rosbagResimWriteRfCheck_->isChecked();
    const bool writeSys = !rosbagResimWriteSysCheck_ || rosbagResimWriteSysCheck_->isChecked();
    const bool writeBeam = !rosbagResimWriteBeamCheck_ || rosbagResimWriteBeamCheck_->isChecked();
    if (inputBag.isEmpty() || !QFileInfo::exists(inputBag)) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("Select an existing input bag first."));
        }
        return;
    }
    if (outputBag.isEmpty()) {
        outputBag = defaultRosbagOutputPath(QFileInfo(inputBag).completeBaseName() + QStringLiteral("_wireless_resim"));
        if (rosbagResimOutputEdit_) {
            rosbagResimOutputEdit_->setText(outputBag);
        }
    }
    if (!outputBag.endsWith(QStringLiteral(".bag"), Qt::CaseInsensitive)) {
        outputBag += QStringLiteral(".bag");
        if (rosbagResimOutputEdit_) {
            rosbagResimOutputEdit_->setText(outputBag);
        }
    }
    if (poseTopic.isEmpty()) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("Pose topic must not be empty."));
        }
        return;
    }
    if (writeRf && rfTopic.isEmpty()) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("RF topic must not be empty when RF output is enabled."));
        }
        return;
    }
    if (writeSys && sysTopic.isEmpty()) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("SYS topic must not be empty when SYS output is enabled."));
        }
        return;
    }
    if (writeBeam && beamTopic.isEmpty()) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("Beam topic must not be empty when beam output is enabled."));
        }
        return;
    }
    if (writeBeam && beamCodebookTopic.isEmpty()) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("Beam codebook topic must not be empty when beam output is enabled."));
        }
        return;
    }
    if (simSettings_.scenePath.trimmed().isEmpty()) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("Scene XML is empty. Configure Sionna Settings first."));
        }
        return;
    }
    if (!simSettings_.beamCodebookFilePath.trimmed().isEmpty() && !QFileInfo::exists(simSettings_.beamCodebookFilePath.trimmed())) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("Custom beam codebook file does not exist: %1").arg(simSettings_.beamCodebookFilePath.trimmed()));
        }
        return;
    }

    QDir().mkpath(QFileInfo(outputBag).absolutePath());
    lastExportedBsJsonPath_ = exportBaseStationsForSimulator();
    const QString scriptPath = findBundledScript(QStringLiteral("scripts/reprocess_rosbag_with_sionna.py"));
    const QString workerScriptPath = findBundledScript(QStringLiteral("scripts/sionna_resim_socket_worker.py"));
    if (!QFileInfo::exists(scriptPath)) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("Offline re-simulation script not found: %1").arg(scriptPath));
        }
        return;
    }
    if (!QFileInfo::exists(workerScriptPath)) {
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(tr("Socket Sionna worker script not found: %1").arg(workerScriptPath));
        }
        return;
    }
    const QString sionnaPythonProgram = simSettings_.pythonExecutable.trimmed().isEmpty() ? QStringLiteral("python3") : simSettings_.pythonExecutable.trimmed();
    const QString rosPythonProgram = QFileInfo(QStringLiteral("/usr/bin/python3")).exists()
                                        ? QStringLiteral("/usr/bin/python3")
                                        : QStringLiteral("python3");
    const QString pythonProgram = rosPythonProgram;
    QString fcText = QString::number(simSettings_.fcHz, 'g', 16);
    if (!fcText.contains(QChar('.')) && !fcText.contains(QChar('e')) && !fcText.contains(QChar('E'))) {
        fcText += QStringLiteral(".0");
    }
    const double messageFrequencyHz = rosbagResimMessageFrequencySpin_ ? rosbagResimMessageFrequencySpin_->value() : 10.0;
    QStringList args;
    args << scriptPath
         << QStringLiteral("--input-bag") << inputBag
         << QStringLiteral("--output-bag") << outputBag
         << QStringLiteral("--pose-topic") << poseTopic
         << QStringLiteral("--rf-topic") << (rfTopic.isEmpty() ? internalRfTopic_ : rfTopic)
         << QStringLiteral("--sys-topic") << (sysTopic.isEmpty() ? internalSysTopic_ : sysTopic)
         << QStringLiteral("--beam-topic") << beamTopic
         << QStringLiteral("--beam-codebook-topic") << beamCodebookTopic
         << QStringLiteral("--write-rf") << (writeRf ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--write-sys") << (writeSys ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--write-beam") << (writeBeam ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--ros-to-map-matrix-json") << coordinateTransforms_.rosToSceneMatrixPath
         << QStringLiteral("--airsim-to-scene-matrix-json") << coordinateTransforms_.airsimToSceneMatrixPath
         << QStringLiteral("--rx-array-facing-direction") << coordinateTransforms_.rxArrayFacingDirection
         << QStringLiteral("--uav-to-rx-array-tx") << QString::number(coordinateTransforms_.uavToRxArrayTx, 'g', 12)
         << QStringLiteral("--uav-to-rx-array-ty") << QString::number(coordinateTransforms_.uavToRxArrayTy, 'g', 12)
         << QStringLiteral("--uav-to-rx-array-tz") << QString::number(coordinateTransforms_.uavToRxArrayTz, 'g', 12)
         << QStringLiteral("--uav-to-rx-array-center-matrix-json") << coordinateTransforms_.uavToRxArrayCenterMatrixPath
         << QStringLiteral("--rx-array-elements-json") << coordinateTransforms_.rxArrayElementsJsonPath
         << QStringLiteral("--output-frame-key") << coordinateTransforms_.outputFrameKey
         << QStringLiteral("--scene-path") << simSettings_.scenePath
         << QStringLiteral("--bs-list-json") << lastExportedBsJsonPath_
         << QStringLiteral("--message-frequency-hz") << QString::number(messageFrequencyHz, 'g', 12)
         << QStringLiteral("--rf-frame-id") << simSettings_.rfFrameId
         << QStringLiteral("--fc-hz") << fcText
         << QStringLiteral("--mi-variant") << simSettings_.miVariant
         << QStringLiteral("--tx-array-num-rows") << QString::number(simSettings_.txArrayNumRows)
         << QStringLiteral("--tx-array-num-cols") << QString::number(simSettings_.txArrayNumCols)
         << QStringLiteral("--tx-array-vertical-spacing") << QString::number(simSettings_.txArrayVerticalSpacing, 'g', 8)
         << QStringLiteral("--tx-array-horizontal-spacing") << QString::number(simSettings_.txArrayHorizontalSpacing, 'g', 8)
         << QStringLiteral("--tx-array-pattern") << simSettings_.txArrayPattern
         << QStringLiteral("--tx-array-polarization") << simSettings_.txArrayPolarization
         << QStringLiteral("--rx-array-num-rows") << QString::number(simSettings_.rxArrayNumRows)
         << QStringLiteral("--rx-array-num-cols") << QString::number(simSettings_.rxArrayNumCols)
         << QStringLiteral("--rx-array-vertical-spacing") << QString::number(simSettings_.rxArrayVerticalSpacing, 'g', 8)
         << QStringLiteral("--rx-array-horizontal-spacing") << QString::number(simSettings_.rxArrayHorizontalSpacing, 'g', 8)
         << QStringLiteral("--rx-array-pattern") << simSettings_.rxArrayPattern
         << QStringLiteral("--rx-array-polarization") << simSettings_.rxArrayPolarization
         << QStringLiteral("--max-depth") << QString::number(simSettings_.maxDepth)
         << QStringLiteral("--samples-per-src") << QString::number(simSettings_.samplesPerSrc)
         << QStringLiteral("--max-num-paths-per-src") << QString::number(simSettings_.maxNumPathsPerSrc)
         << QStringLiteral("--synthetic-array") << (simSettings_.syntheticArray ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--merge-shapes") << (simSettings_.mergeShapes ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--enable-sys-integration") << (simSettings_.enableSysIntegration ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--enable-beamforming") << (simSettings_.enableBeamforming ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--beam-selection-mode") << simSettings_.beamSelectionMode
         << QStringLiteral("--beam-codebook-type") << simSettings_.beamCodebookType
         << QStringLiteral("--beam-codebook-num-beams") << QString::number(simSettings_.beamCodebookNumBeams)
         << QStringLiteral("--beam-oversampling-v") << QString::number(simSettings_.beamOversamplingV)
         << QStringLiteral("--beam-oversampling-h") << QString::number(simSettings_.beamOversamplingH)
         << QStringLiteral("--beam-manual-index") << QString::number(simSettings_.beamManualIndex)
         << QStringLiteral("--beam-normalize-power") << (simSettings_.beamNormalizePower ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--beam-codebook-file") << simSettings_.beamCodebookFilePath
         << QStringLiteral("--beam-model-checkpoint-path") << simSettings_.beamModelCheckpointPath
         << QStringLiteral("--beam-feature-mode") << simSettings_.beamFeatureMode
         << QStringLiteral("--beam-top-k") << QString::number(simSettings_.beamTopK)
         << QStringLiteral("--sys-num-subcarriers") << QString::number(simSettings_.sysNumSubcarriers)
         << QStringLiteral("--sys-subcarrier-spacing-hz") << QString::number(simSettings_.sysSubcarrierSpacingHz, 'g', 12)
         << QStringLiteral("--sys-num-ofdm-symbols") << QString::number(simSettings_.sysNumOfdmSymbols)
         << QStringLiteral("--sys-temperature-k") << QString::number(simSettings_.sysTemperatureK, 'g', 8)
         << QStringLiteral("--sys-bler-target") << QString::number(simSettings_.sysBlerTarget, 'g', 8)
         << QStringLiteral("--sys-mcs-table-index") << QString::number(simSettings_.sysMcsTableIndex)
         << QStringLiteral("--sys-bs-tx-power-dbm") << QString::number(simSettings_.sysBsTxPowerDbm, 'g', 8)
         << QStringLiteral("--los") << (simSettings_.los ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--specular-reflection") << (simSettings_.specularReflection ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--diffuse-reflection") << (simSettings_.diffuseReflection ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--refraction") << (simSettings_.refraction ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--diffraction") << (simSettings_.diffraction ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--edge-diffraction") << (simSettings_.edgeDiffraction ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--diffraction-lit-region") << (simSettings_.diffractionLitRegion ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--sionna-python") << sionnaPythonProgram
         << QStringLiteral("--worker-script") << workerScriptPath;

    appendRosbagLog(QStringLiteral("Starting rosbag re-simulation (ROS driver): %1 %2").arg(pythonProgram, args.join(QStringLiteral(" "))));
    appendRosbagLog(QStringLiteral("Using Sionna worker Python: %1").arg(sionnaPythonProgram));
    if (rosbagResimProgressBar_) {
        rosbagResimProgressBar_->setRange(0, 100);
        rosbagResimProgressBar_->setValue(0);
        rosbagResimProgressBar_->setFormat(QStringLiteral("%p%"));
    }
    if (rosbagResimStatusValue_) {
        rosbagResimStatusValue_->setText(tr("Launching offline Sionna re-simulation..."));
    }
    rosbagResimProcess_->start(pythonProgram, args);
    if (!rosbagResimProcess_->waitForStarted(3000)) {
        const QString message = tr("Failed to start rosbag re-simulation: %1").arg(rosbagResimProcess_->errorString());
        if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(message);
        }
        appendRosbagLog(message);
        updateRosbagUiState();
        return;
    }
    updateRosbagUiState();
}


void MainWindow::onRosbagRecordOutput() {
    const QString text = QString::fromLocal8Bit(rosbagRecordProcess_->readAllStandardOutput());
    if (text.trimmed().isEmpty()) {
        return;
    }
    const QStringList lines = text.split(QRegularExpression(QStringLiteral("[\r\n]+")), QString::SkipEmptyParts);
    for (const QString& line : lines) {
        appendRosbagLog(QStringLiteral("[record] %1").arg(line));
    }
}

void MainWindow::onRosbagRecordFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    Q_UNUSED(exitStatus)
    const QString path = rosbagRecordPathEdit_ ? rosbagRecordPathEdit_->text().trimmed() : QString();
    const bool hasBag = !path.isEmpty() && QFileInfo::exists(path);
    if (rosbagRecordStatusValue_) {
        rosbagRecordStatusValue_->setText((exitCode == 0 || hasBag) ? tr("Saved: %1").arg(path) : tr("Recorder stopped with exit code %1").arg(exitCode));
    }
    appendRosbagLog(QStringLiteral("rosbag record finished with exitCode=%1").arg(exitCode));
    updateRosbagUiState();
}

void MainWindow::onRosbagPlaybackOutput() {
    const QString text = QString::fromLocal8Bit(rosbagPlayProcess_->readAllStandardOutput());
    if (text.trimmed().isEmpty()) {
        return;
    }
    const QStringList lines = text.split(QRegularExpression(QStringLiteral("[\r\n]+")), QString::SkipEmptyParts);
    for (const QString& line : lines) {
        appendRosbagLog(QStringLiteral("[play] %1").arg(line));
    }
}

void MainWindow::onRosbagPlaybackFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    Q_UNUSED(exitStatus)
    const bool stopPoseDrivenSimulation = rosbagPlaybackActive_
        && !rosbagPlaybackBagHasWirelessTopics_
        && simulatorProcess_
        && simulatorProcess_->state() != QProcess::NotRunning;
    rosbagPlaybackActive_ = false;
    rosbagPlaybackBagHasWirelessTopics_ = false;
    rosbagPlaybackUsingRfTopic_ = false;
    rosbagPlaybackUsingSysTopic_ = false;
    rosbagPlaybackUsingBeamTopic_ = false;
    rosbagPlaybackPoseTopic_.clear();
    rosbagPlaybackTrajectoryTopic_.clear();
    rosBridge_->stop();
    if (stopPoseDrivenSimulation) {
        stopInternalSimulator();
    }
    if (rosbagPlaybackStatusValue_) {
        rosbagPlaybackStatusValue_->setText(exitCode == 0 ? tr("Playback finished.") : tr("Playback stopped with exit code %1").arg(exitCode));
    }
    appendRosbagLog(QStringLiteral("rosbag play finished with exitCode=%1").arg(exitCode));
    refreshInfoPanel();
    updateRosbagUiState();
}

void MainWindow::onRosbagResimOutput() {
    const QString text = QString::fromLocal8Bit(rosbagResimProcess_->readAllStandardOutput());
    if (text.trimmed().isEmpty()) {
        return;
    }
    const QStringList lines = text.split(QRegularExpression(QStringLiteral("[\r\n]+")), QString::SkipEmptyParts);
    static const QRegularExpression progressRe(QStringLiteral("^\\[progress\\]\\s+([0-9]+(?:\\.[0-9]+)?)\\s*(.*)$"));
    static const QRegularExpression progressJsonRe(QStringLiteral("^\\[progress-json\\]\\s+(\\{.*\\})\\s*$"));
    for (const QString& line : lines) {
        appendRosbagLog(QStringLiteral("[resim] %1").arg(line));
        const QString trimmed = line.trimmed();
        bool handledProgress = false;

        const QRegularExpressionMatch jsonMatch = progressJsonRe.match(trimmed);
        if (jsonMatch.hasMatch()) {
            QJsonParseError parseError;
            const QJsonDocument doc = QJsonDocument::fromJson(jsonMatch.captured(1).toUtf8(), &parseError);
            if (parseError.error == QJsonParseError::NoError && doc.isObject()) {
                const QJsonObject obj = doc.object();
                const int current = obj.value(QStringLiteral("current")).toInt(-1);
                const int total = obj.value(QStringLiteral("total")).toInt(-1);
                const double percent = obj.value(QStringLiteral("percent")).toDouble(std::numeric_limits<double>::quiet_NaN());
                const QString message = obj.value(QStringLiteral("message")).toString().trimmed();
                if (rosbagResimProgressBar_) {
                    if (current >= 0 && total > 0) {
                        rosbagResimProgressBar_->setRange(0, total);
                        rosbagResimProgressBar_->setValue(qBound(0, current, total));
                        rosbagResimProgressBar_->setFormat(tr("%v/%m (%p%)"));
                    } else if (std::isfinite(percent)) {
                        rosbagResimProgressBar_->setRange(0, 100);
                        rosbagResimProgressBar_->setValue(qBound(0, static_cast<int>(std::lround(percent)), 100));
                        rosbagResimProgressBar_->setFormat(QStringLiteral("%p%"));
                    }
                }
                if (rosbagResimStatusValue_) {
                    rosbagResimStatusValue_->setText(message.isEmpty() ? tr("Processing...") : message);
                }
                handledProgress = true;
            }
        }

        if (handledProgress) {
            continue;
        }

        const QRegularExpressionMatch match = progressRe.match(trimmed);
        if (match.hasMatch()) {
            const int value = qBound(0, static_cast<int>(std::lround(match.captured(1).toDouble())), 100);
            if (rosbagResimProgressBar_) {
                rosbagResimProgressBar_->setRange(0, 100);
                rosbagResimProgressBar_->setValue(value);
                rosbagResimProgressBar_->setFormat(QStringLiteral("%p%"));
            }
            if (rosbagResimStatusValue_) {
                const QString message = match.captured(2).trimmed();
                rosbagResimStatusValue_->setText(message.isEmpty() ? tr("Processing...") : message);
            }
        } else if (rosbagResimStatusValue_) {
            rosbagResimStatusValue_->setText(trimmed);
        }
    }
}

void MainWindow::onRosbagResimFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    Q_UNUSED(exitStatus)
    const QString outputPath = rosbagResimOutputEdit_ ? rosbagResimOutputEdit_->text().trimmed() : QString();
    const bool hasOutput = !outputPath.isEmpty() && QFileInfo::exists(outputPath);
    if (rosbagResimProgressBar_ && (exitCode == 0 || hasOutput)) {
        const int maximum = rosbagResimProgressBar_->maximum();
        const int minimum = rosbagResimProgressBar_->minimum();
        if (maximum > minimum) {
            rosbagResimProgressBar_->setValue(exitCode == 0 ? maximum : std::max(rosbagResimProgressBar_->value(), minimum + 1));
        } else {
            rosbagResimProgressBar_->setRange(0, 100);
            rosbagResimProgressBar_->setValue(exitCode == 0 ? 100 : std::max(rosbagResimProgressBar_->value(), 1));
            rosbagResimProgressBar_->setFormat(QStringLiteral("%p%"));
        }
    }
    if (rosbagResimStatusValue_) {
        rosbagResimStatusValue_->setText((exitCode == 0)
            ? tr("Re-simulation finished. Output: %1").arg(outputPath)
            : (hasOutput
                ? tr("Re-simulation stopped early. Partial output kept at %1").arg(outputPath)
                : tr("Re-simulation stopped with exit code %1. Partial output is kept if already written.").arg(exitCode)));
    }
    appendRosbagLog(QStringLiteral("rosbag re-simulation finished with exitCode=%1").arg(exitCode));
    updateRosbagUiState();
}

void MainWindow::rebuildRfDataCards() {
    if (!rfDataCardsLayout_ || !rfDataCardsContainer_) {
        return;
    }

    struct CardSpec {
        QString key;
        QString title;
        QString body;
        int row{0};
        int column{0};
        int rowSpan{1};
        int columnSpan{1};
    };

    QVector<CardSpec> specs;
    specs.reserve(static_cast<int>(stations_.size()) + static_cast<int>(currentRfSnapshot_.anchors.size()) + 1);

    QString summaryBody;
    if (currentRfSnapshot_.valid) {
        summaryBody = QStringLiteral("Frame: %1\nRX position: %2\nRX velocity: %3\nAnchors in message: %4")
            .arg(currentRfSnapshot_.frameId.isEmpty() ? QStringLiteral("<none>") : currentRfSnapshot_.frameId,
                 vecToString(currentRfSnapshot_.rxPosition),
                 vecToString(currentRfSnapshot_.rxVelocity),
                 QString::number(static_cast<int>(currentRfSnapshot_.anchors.size())));
    } else {
        summaryBody = tr("No rf_observations data received yet. Start Simulation and the internal RF simulator first.");
    }
    specs.push_back({QStringLiteral("summary"), tr("RX / Message Summary"), summaryBody, 0, 0, 1, 2});

    int cardIndex = 0;
    auto addCard = [&specs, &cardIndex](const QString& key, const QString& title, const QString& body) {
        const int row = 1 + (cardIndex / 2);
        const int column = cardIndex % 2;
        specs.push_back({key, title, body, row, column, 1, 1});
        ++cardIndex;
    };

    QSet<QString> usedAnchors;
    for (const auto& station : stations_) {
        const RfAnchorObservationData* anchor = findAnchorForStation(currentRfSnapshot_.anchors, station);
        QStringList lines;
        lines << QStringLiteral("Station name: %1").arg(station.name);
        lines << QStringLiteral("Station id: %1").arg(station.id);
        lines << QStringLiteral("Station position: %1").arg(vecToString(station.position));
        if (!anchor) {
            lines << tr("No matching anchor data in the latest rf_observations message.");
        } else {
            usedAnchors.insert(anchor->anchorName);
            lines << QStringLiteral("Anchor id: %1").arg(anchor->anchorId);
            lines << QStringLiteral("Anchor name: %1").arg(anchor->anchorName);
            lines << QStringLiteral("Anchor position: %1").arg(vecToString(anchor->anchorPosition));
            lines << QStringLiteral("Paths: %1").arg(static_cast<int>(anchor->paths.size()));
            for (size_t i = 0; i < anchor->paths.size(); ++i) {
                const auto& path = anchor->paths[i];
                lines << QString();
                lines << QStringLiteral("Path %1 · %2").arg(static_cast<int>(i + 1)).arg(path.pathId.isEmpty() ? QStringLiteral("<unnamed>") : path.pathId);
                lines << QStringLiteral("  type=%1  order=%2  index=%3  valid=%4")
                             .arg(rfPathTypeLabel(path.pathType))
                             .arg(path.pathOrder)
                             .arg(path.pathIndex)
                             .arg(path.isValid ? QStringLiteral("true") : QStringLiteral("false"));
                lines << QStringLiteral("  amplitude_abs=%1").arg(fmtMaybe(path.hasAmplitude, path.amplitudeAbs, 4));
                lines << QStringLiteral("  path_gain_linear=%1").arg(fmtMaybe(path.hasPathGain, path.pathGainLinear, 6));
                lines << QStringLiteral("  power_db=%1").arg(fmtMaybe(path.hasPowerDb, path.powerDb, 2, QStringLiteral(" dB")));
                lines << QStringLiteral("  tau_s=%1").arg(fmtMaybe(path.hasTau, path.tauS, 9, QStringLiteral(" s")));
                lines << QStringLiteral("  tau_std_s=%1").arg(fmtMaybe(path.hasTau, path.tauStdS, 9, QStringLiteral(" s")));
                lines << QStringLiteral("  doppler_hz=%1").arg(fmtMaybe(path.hasDoppler, path.dopplerHz, 3, QStringLiteral(" Hz")));
                lines << QStringLiteral("  doppler_std_hz=%1").arg(fmtMaybe(path.hasDoppler, path.dopplerStdHz, 3, QStringLiteral(" Hz")));
                lines << QStringLiteral("  aoa_az_rad=%1").arg(fmtMaybe(path.hasAoa, path.aoaAzRad, 4));
                lines << QStringLiteral("  aoa_el_rad=%1").arg(fmtMaybe(path.hasAoa, path.aoaElRad, 4));
                lines << QStringLiteral("  aoa_az_std_rad=%1").arg(fmtMaybe(path.hasAoa, path.aoaAzStdRad, 4));
                lines << QStringLiteral("  aoa_el_std_rad=%1").arg(fmtMaybe(path.hasAoa, path.aoaElStdRad, 4));
                lines << QStringLiteral("  tx_position=%1").arg(vecToString(path.txPosition));
                lines << QStringLiteral("  rx_position=%1").arg(vecToString(path.rxPosition));
                if (path.pathPoints.empty()) {
                    lines << QStringLiteral("  path_points=[]");
                } else {
                    lines << QStringLiteral("  path_points (%1):").arg(static_cast<int>(path.pathPoints.size()));
                    for (size_t ptIndex = 0; ptIndex < path.pathPoints.size(); ++ptIndex) {
                        lines << QStringLiteral("    [%1] %2")
                                     .arg(static_cast<int>(ptIndex))
                                     .arg(vecToString(path.pathPoints[ptIndex]));
                    }
                }
            }
        }

        addCard(QStringLiteral("station:%1:%2").arg(station.id, station.name),
                QStringLiteral("%1 (%2)").arg(station.name, station.id),
                lines.join(QStringLiteral("\n")));
    }

    for (const auto& anchor : currentRfSnapshot_.anchors) {
        if (usedAnchors.contains(anchor.anchorName)) {
            continue;
        }
        QStringList lines;
        lines << QStringLiteral("Anchor id: %1").arg(anchor.anchorId);
        lines << QStringLiteral("Anchor position: %1").arg(vecToString(anchor.anchorPosition));
        lines << QStringLiteral("Paths: %1").arg(static_cast<int>(anchor.paths.size()));
        addCard(QStringLiteral("anchor:%1:%2").arg(anchor.anchorId).arg(anchor.anchorName),
                QStringLiteral("Unmatched anchor: %1").arg(anchor.anchorName),
                lines.join(QStringLiteral("\n")));
    }

    QSet<QString> desiredKeys;
    for (const CardSpec& spec : specs) {
        desiredKeys.insert(spec.key);
        auto it = rfDataCardViews_.find(spec.key);
        if (it == rfDataCardViews_.end()) {
            const auto widgets = createInfoCard(spec.title, spec.body, rfDataCardsContainer_);
            RfCardView view;
            view.frame = widgets.frame;
            view.titleLabel = widgets.titleLabel;
            view.bodyLabel = widgets.bodyLabel;
            view.row = spec.row;
            view.column = spec.column;
            view.rowSpan = spec.rowSpan;
            view.columnSpan = spec.columnSpan;
            rfDataCardsLayout_->addWidget(view.frame, spec.row, spec.column, spec.rowSpan, spec.columnSpan);
            it = rfDataCardViews_.insert(spec.key, view);
        }

        RfCardView& view = it.value();
        if (view.titleLabel) {
            view.titleLabel->setText(spec.title);
        }
        if (view.bodyLabel) {
            view.bodyLabel->setText(spec.body);
        }
        if (view.frame && (view.row != spec.row || view.column != spec.column
                || view.rowSpan != spec.rowSpan || view.columnSpan != spec.columnSpan)) {
            rfDataCardsLayout_->removeWidget(view.frame);
            rfDataCardsLayout_->addWidget(view.frame, spec.row, spec.column, spec.rowSpan, spec.columnSpan);
            view.row = spec.row;
            view.column = spec.column;
            view.rowSpan = spec.rowSpan;
            view.columnSpan = spec.columnSpan;
        }
        if (view.frame) {
            view.frame->show();
        }
    }

    for (auto it = rfDataCardViews_.begin(); it != rfDataCardViews_.end(); ) {
        if (desiredKeys.contains(it.key())) {
            ++it;
            continue;
        }
        if (it.value().frame) {
            rfDataCardsLayout_->removeWidget(it.value().frame);
            it.value().frame->deleteLater();
        }
        it = rfDataCardViews_.erase(it);
    }

    if (rfDataSpacerRow_ >= 0) {
        rfDataCardsLayout_->setRowStretch(rfDataSpacerRow_, 0);
    }
    const int spacerRow = 1 + ((cardIndex + 1) / 2);
    rfDataCardsLayout_->setRowStretch(spacerRow, 1);
    rfDataSpacerRow_ = spacerRow;

    if (rfDataWindow_) {
        rfDataWindow_->setWindowTitle(currentRfSnapshot_.valid
            ? tr("Wireless Data Monitor · %1 anchors").arg(static_cast<int>(currentRfSnapshot_.anchors.size()))
            : tr("Wireless Data Monitor"));
    }
}

QString MainWindow::stationCameraKey(const BaseStation& station) const {
    return stationTokenForResources(station);
}

int MainWindow::stationIndexForKey(const QString& key) const {
    for (int i = 0; i < static_cast<int>(stations_.size()); ++i) {
        if (stationCameraKey(stations_[static_cast<size_t>(i)]) == key) {
            return i;
        }
    }
    return -1;
}

MainWindow::StationCameraWindow* MainWindow::findStationCameraWindow(const QString& key) {
    for (auto& window : stationCameraWindows_) {
        if (window.stationKey == key) {
            return &window;
        }
    }
    return nullptr;
}

const MainWindow::StationCameraWindow* MainWindow::findStationCameraWindow(const QString& key) const {
    for (const auto& window : stationCameraWindows_) {
        if (window.stationKey == key) {
            return &window;
        }
    }
    return nullptr;
}

void MainWindow::updateStationCameraWindowTitle(int index) {
    if (index < 0 || index >= static_cast<int>(stations_.size())) {
        return;
    }
    const QString key = stationCameraKey(stations_[static_cast<size_t>(index)]);
    for (auto& window : stationCameraWindows_) {
        if (window.stationKey != key || !window.dock) {
            continue;
        }
        QString title = tr("BS Camera · %1").arg(stations_[static_cast<size_t>(index)].name);
        if (index == selectedBaseStationIndex_) {
            title += tr(" [selected]");
        }
        if (window.pinned) {
            title += tr(" [pinned]");
        }
        window.dock->setWindowTitle(title);
        break;
    }
}

void MainWindow::ensureStationCameraWindow(int index) {
    if (index < 0 || index >= static_cast<int>(stations_.size())) {
        return;
    }

    const BaseStation& station = stations_[static_cast<size_t>(index)];
    const QString key = stationCameraKey(station);
    if (key.isEmpty()) {
        return;
    }

    for (auto& window : stationCameraWindows_) {
        if (window.stationKey == key) {
            syncStationCameraWindow(index);
            updateStationCameraWindowTitle(index);
            return;
        }
    }

    StationCameraWindow window;
    window.stationKey = key;
    window.dock = new QDockWidget(tr("BS Camera · %1").arg(station.name), this);
    window.dock->setObjectName(QStringLiteral("stationCameraDock_%1").arg(key));
    window.dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    window.dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetClosable);
    window.dock->setWindowTitle(tr("BS Camera · %1").arg(station.name));

    auto* container = new QWidget(window.dock);
    auto* layout = new QVBoxLayout(container);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(4);

    auto* topRow = new QHBoxLayout();
    window.pinButton = new QToolButton(container);
    window.pinButton->setText(tr("Pin"));
    window.pinButton->setCheckable(true);
    window.pinButton->setChecked(false);
    window.saveViewButton = new QToolButton(container);
    window.saveViewButton->setText(tr("Save View"));
    auto* resetButton = new QToolButton(container);
    resetButton->setText(tr("Reset"));
    auto* topicLabel = new QLabel(container);
    topicLabel->setText(tr("ROS: %1").arg(station.previewRosTopic.trimmed().isEmpty()
        ? defaultPreviewRosTopicForStation(station)
        : station.previewRosTopic.trimmed()));
    topicLabel->setWordWrap(true);
    window.topicLabel = topicLabel;
    topRow->addWidget(window.pinButton, 0);
    topRow->addWidget(window.saveViewButton, 0);
    topRow->addWidget(resetButton, 0);
    topRow->addWidget(topicLabel, 1);
    layout->addLayout(topRow);

    window.view = new UeViewWidget(container);
    window.view->setMinimumSize(460, 320);
    container->setMinimumSize(500, 420);
    window.dock->setMinimumSize(520, 460);
    QString qmlError;
    const QString stationQmlPath = findConfigFile("qml/UeStationCameraView.qml");
    if (!window.view->initialize(stationQmlPath, QString(), &qmlError)) {
        QMessageBox::warning(this, tr("Station Camera View"), qmlError);
    }
    layout->addWidget(window.view, 1);

    window.statusLabel = new QLabel(container);
    window.statusLabel->setWordWrap(true);
    window.statusLabel->setText(tr("Idle"));
    layout->addWidget(window.statusLabel);

    window.dock->setWidget(container);
    addDockWidget(Qt::LeftDockWidgetArea, window.dock);
    if (lastLeftDock_ && lastLeftDock_ != window.dock) {
        splitDockWidget(lastLeftDock_, window.dock, Qt::Vertical);
    }
    lastLeftDock_ = window.dock;
    window.dock->resize(560, 480);
    window.dock->hide();

    auto* controller = window.view->controller();
    controller->setFrameTransformManager(frameTransformManager_);
    controller->setAirsimToGuiTransformPath(coordinateTransforms_.airsimToSceneMatrixPath);
    controller->setViewLabel(tr("%1 Camera").arg(station.name));
    connect(controller, &AirSimViewController::helperMessage, this, [this, key](const QString& text) {
        for (auto& item : stationCameraWindows_) {
            if (item.stationKey == key && item.statusLabel) {
                item.statusLabel->setText(text);
                break;
            }
        }
        statusBar()->showMessage(text, 2500);
    });
    connect(resetButton, &QToolButton::clicked, controller, &AirSimViewController::resetCamera);
    connect(window.pinButton, &QToolButton::toggled, this, [this, key](bool checked) {
        for (auto& item : stationCameraWindows_) {
            if (item.stationKey != key) {
                continue;
            }
            item.pinned = checked;
            const int index = stationIndexForKey(key);
            updateStationCameraWindowTitle(index);
            break;
        }
        refreshStationCameraStreamingState();
    });
    connect(window.saveViewButton, &QToolButton::clicked, this, [this, key]() {
        saveStationCameraDefaultView(key);
    });
    connect(window.dock, &QDockWidget::visibilityChanged, this, [this, key](bool visible) {
        Q_UNUSED(visible);
        const int index = stationIndexForKey(key);
        updateStationCameraWindowTitle(index);
        refreshStationCameraStreamingState();
    });

    stationCameraWindows_.push_back(window);
    syncStationCameraWindow(index);
    updateStationCameraWindowTitle(index);
}

void MainWindow::syncStationCameraWindow(int index) {
    if (index < 0 || index >= static_cast<int>(stations_.size())) {
        return;
    }
    const BaseStation& station = stations_[static_cast<size_t>(index)];
    const QString key = stationCameraKey(station);
    for (auto& window : stationCameraWindows_) {
        if (window.stationKey != key || !window.view || !window.view->controller()) {
            continue;
        }
        auto* controller = window.view->controller();
        controller->setBaseStations(stations_);
        controller->setBeamPaths(currentPaths_);
        controller->setDroneState(currentDrone_);
        applyCurrentLayerStatesToAirSimController(controller);
        controller->setHost(airsimHostEdit_ ? airsimHostEdit_->text().trimmed() : QStringLiteral("127.0.0.1"));
        controller->setPort(airsimPortEdit_ ? airsimPortEdit_->text().trimmed().toInt() : 41451);
        const QString previewCameraName = station.previewCameraName.trimmed().isEmpty()
            ? defaultPreviewCameraNameForStation(station)
            : station.previewCameraName.trimmed();
        controller->setCameraName(previewCameraName);
        controller->setViewLabel(tr("%1 Camera").arg(station.name));
        controller->setVehicleName(QString());
        controller->setFollowVehicle(false);
        controller->setDepthFetchEnabled(false);
        controller->setDisplayBrightness(currentAirSimBrightnessFactor());
        controller->setAnchoredCamera(true);
        const Vec3 transformedAnchor = scenePointToAirSim(station.position);
        const Vec3 anchorPoint{transformedAnchor.x, transformedAnchor.y, transformedAnchor.z + station.previewOffsetZ};
        controller->setAnchorPoint(anchorPoint);
        const bool previewTargetChanged = !window.previewTargetAppliedInitialized
            || window.appliedPreviewCameraTargetEnabled != station.previewCameraTargetEnabled
            || !nearlyEqualVec3(window.appliedAnchorPoint, anchorPoint)
            || (station.previewCameraTargetEnabled
                && !nearlyEqualVec3(window.appliedPreviewCameraTarget, station.previewCameraTarget));
        if (previewTargetChanged) {
            if (station.previewCameraTargetEnabled) {
                const Vec3 targetPoint = scenePointToAirSim(station.previewCameraTarget);
                const Vec3 direction{
                    targetPoint.x - anchorPoint.x,
                    targetPoint.y - anchorPoint.y,
                    targetPoint.z - anchorPoint.z,
                };
                double yawDeg = 0.0;
                double pitchDeg = 0.0;
                if (yawPitchFromDirectionNed(direction, &yawDeg, &pitchDeg)) {
                    controller->setViewAngles(yawDeg, pitchDeg);
                } else {
                    controller->setViewAngles(0.0, 0.0);
                }
            } else {
                controller->setViewAngles(0.0, 0.0);
            }
            window.previewTargetAppliedInitialized = true;
            window.appliedPreviewCameraTargetEnabled = station.previewCameraTargetEnabled;
            window.appliedPreviewCameraTarget = station.previewCameraTarget;
            window.appliedAnchorPoint = anchorPoint;
        }
        const QString rosTopic = station.previewRosTopic.trimmed().isEmpty()
            ? defaultPreviewRosTopicForStation(station)
            : station.previewRosTopic.trimmed();
        controller->setFramesPerSecond(static_cast<double>(cameraFpsForControl(
            QString::fromLatin1(kCameraFpsStationPrefix) + key,
            rosTopic)));
        controller->setRosPublishingEnabled(false);
        controller->setRosImageTopic(rosTopic);
        controller->setRosImageSubscriptionTopic(rosTopic);
        if (window.topicLabel) {
            window.topicLabel->setText(tr("ROS: %1").arg(rosTopic));
        }
        controller->setRosFrameId(QStringLiteral("%1_camera").arg(key));
        controller->setPythonExecutable(simSettings_.pythonExecutable);
        updateStationCameraWindowTitle(index);
        break;
    }
}

void MainWindow::saveStationCameraDefaultView(const QString& key) {
    const int index = stationIndexForKey(key);
    if (index < 0 || index >= static_cast<int>(stations_.size())) {
        return;
    }
    StationCameraWindow* window = findStationCameraWindow(key);
    if (!window || !window->view || !window->view->controller()) {
        return;
    }

    AirSimViewController* controller = window->view->controller();
    const BaseStation& station = stations_[static_cast<size_t>(index)];
    Vec3 anchorPoint = scenePointToAirSim(station.position);
    anchorPoint.z += station.previewOffsetZ;
    const Vec3 forward = directionFromYawPitchNedDeg(controller->yawDeg(), controller->pitchDeg());
    const Vec3 targetPointAirsim{
        anchorPoint.x + forward.x * kPreviewCameraTargetDistanceMeters,
        anchorPoint.y + forward.y * kPreviewCameraTargetDistanceMeters,
        anchorPoint.z + forward.z * kPreviewCameraTargetDistanceMeters,
    };
    const Vec3 targetPointScene = airSimPointToScene(targetPointAirsim);

    BaseStation updated = station;
    updated.previewCameraTargetEnabled = true;
    updated.previewCameraTarget = targetPointScene;
    if (updated.previewCameraTargetEnabled == station.previewCameraTargetEnabled
        && nearlyEqualVec3(updated.previewCameraTarget, station.previewCameraTarget)) {
        onStatusMessage(tr("Base station %1 preview view is already saved.").arg(station.name));
        return;
    }

    pushBaseStationUndoState(stations_);
    stations_[static_cast<size_t>(index)] = updated;
    applyBaseStations(stations_);
    markBaseStationsDirty();
    if (baseStationsConfigPath_.trimmed().isEmpty()) {
        onStatusMessage(tr("Saved default BS camera view for %1 in memory. Use Save Base Stations JSON to persist it.")
                            .arg(updated.name));
    } else {
        onStatusMessage(tr("Updated default BS camera view for %1 in memory. Use Save Base Stations JSON to write %2.")
                            .arg(updated.name, QFileInfo(baseStationsConfigPath_).fileName()));
    }
}

void MainWindow::syncStationCameraWindows() {
    for (int i = 0; i < static_cast<int>(stations_.size()); ++i) {
        ensureStationCameraWindow(i);
        syncStationCameraWindow(i);
    }
}

void MainWindow::syncStationCameraFpsControls() {
    if (!stationCameraFpsTopicCombo_ || !stationCameraFpsSpin_) {
        return;
    }

    const QString selectedStationControlKey = (selectedBaseStationIndex_ >= 0 && selectedBaseStationIndex_ < static_cast<int>(stations_.size()))
        ? QString::fromLatin1(kCameraFpsStationPrefix) + stationCameraKey(stations_[static_cast<size_t>(selectedBaseStationIndex_)])
        : QString();
    QString preferredKey = stationCameraFpsTopicCombo_->currentData(Qt::UserRole).toString().trimmed();
    const QString guiPreviewKey = QString::fromLatin1(kCameraFpsGuiPreviewKey);
    if (!selectedStationControlKey.isEmpty() && (preferredKey.isEmpty() || preferredKey == guiPreviewKey)) {
        preferredKey = selectedStationControlKey;
    } else if (preferredKey.isEmpty()) {
        preferredKey = guiPreviewKey;
    }

    QSignalBlocker comboBlocker(stationCameraFpsTopicCombo_);
    QSignalBlocker spinBlocker(stationCameraFpsSpin_);
    suppressStationCameraFpsControls_ = true;

    stationCameraFpsTopicCombo_->clear();
    auto addCameraFpsItem = [this](const QString& label, const QString& controlKey, const QString& rosTopic) {
        const int row = stationCameraFpsTopicCombo_->count();
        stationCameraFpsTopicCombo_->addItem(label, controlKey);
        stationCameraFpsTopicCombo_->setItemData(row, normalizedRosTopicName(rosTopic), Qt::UserRole + 1);
        stationCameraFpsTopicCombo_->setItemData(row, label, Qt::ToolTipRole);
    };

    addCameraFpsItem(tr("GuiPreview (GUI live view)"),
                     QString::fromLatin1(kCameraFpsGuiPreviewKey),
                     QString());

    QString settingsParseError;
    const QString preferredVehicle = airsimLiveVehicleEdit_ ? airsimLiveVehicleEdit_->text().trimmed() : QString();
    const QStringList candidates = effectiveAirSimSettingsPaths();
    for (const QString& candidate : candidates) {
        if (!QFileInfo::exists(candidate)) {
            continue;
        }
        const QVector<CameraFpsControlItem> rosImageItems =
            parseAirSimRosImageTopicItems(candidate, preferredVehicle, &settingsParseError);
        for (const CameraFpsControlItem& item : rosImageItems) {
            addCameraFpsItem(item.label, item.controlKey, item.rosTopic);
        }
        if (!rosImageItems.isEmpty()) {
            break;
        }
    }

    for (const auto& station : stations_) {
        const QString topic = normalizedRosTopicName(station.previewRosTopic.trimmed().isEmpty()
            ? defaultPreviewRosTopicForStation(station)
            : station.previewRosTopic.trimmed());
        const QString stationKey = stationCameraKey(station);
        addCameraFpsItem(tr("Station: %1  %2").arg(station.name, topic),
                         QString::fromLatin1(kCameraFpsStationPrefix) + stationKey,
                         topic);
    }

    int comboIndex = -1;
    if (!preferredKey.isEmpty()) {
        for (int i = 0; i < stationCameraFpsTopicCombo_->count(); ++i) {
            if (stationCameraFpsTopicCombo_->itemData(i).toString() == preferredKey) {
                comboIndex = i;
                break;
            }
        }
    }
    if (comboIndex < 0 && stationCameraFpsTopicCombo_->count() > 0) {
        comboIndex = 0;
    }

    stationCameraFpsTopicCombo_->setEnabled(stationCameraFpsTopicCombo_->count() > 0);
    stationCameraFpsSpin_->setEnabled(comboIndex >= 0);

    QString activeControlKey;
    QString activeRosTopic;
    int fpsValue = kDefaultCameraFps;
    if (comboIndex >= 0) {
        stationCameraFpsTopicCombo_->setCurrentIndex(comboIndex);
        activeControlKey = stationCameraFpsTopicCombo_->itemData(comboIndex, Qt::UserRole).toString().trimmed();
        activeRosTopic = stationCameraFpsTopicCombo_->itemData(comboIndex, Qt::UserRole + 1).toString().trimmed();
        fpsValue = clampCameraFpsForControlKey(activeControlKey, cameraFpsForControl(activeControlKey, activeRosTopic));
        stationCameraFpsSpin_->setRange(1, maxCameraFpsForControlKey(activeControlKey));
        stationCameraFpsSpin_->setValue(fpsValue);
    } else {
        stationCameraFpsSpin_->setRange(1, kMaxCameraFps);
        stationCameraFpsSpin_->setValue(kDefaultCameraFps);
    }

    suppressStationCameraFpsControls_ = false;
    applyCameraFpsForControl(activeControlKey, activeRosTopic, fpsValue, false);
}

int MainWindow::cameraFpsForControl(const QString& controlKey, const QString& rosTopic) const {
    Q_UNUSED(rosTopic);
    const QString key = controlKey.trimmed();
    if (isGuiPreviewFpsKey(key)) {
        if (cameraFpsOverrides_.contains(key)) {
            return clampCameraFpsForControlKey(key, cameraFpsOverrides_.value(key, kDefaultGuiPreviewFps));
        }
        const double fps = airsimLiveFpsSpin_ ? airsimLiveFpsSpin_->value() : static_cast<double>(kDefaultGuiPreviewFps);
        return clampCameraFpsForControlKey(key, static_cast<int>(std::lround(fps)));
    }
    const QString stationPrefix = QString::fromLatin1(kCameraFpsStationPrefix);
    if (key.startsWith(stationPrefix)) {
        const QString stationKey = key.mid(stationPrefix.size());
        const int stationIndex = stationIndexForKey(stationKey);
        if (stationIndex >= 0 && stationIndex < static_cast<int>(stations_.size())) {
            const int fps = static_cast<int>(std::lround(stations_[static_cast<size_t>(stationIndex)].previewFps));
            return clampCameraFpsForControlKey(key, fps > 0 ? fps : kDefaultCameraFps);
        }
        return kDefaultCameraFps;
    }
    if (!key.isEmpty() && cameraFpsOverrides_.contains(key)) {
        return clampCameraFpsForControlKey(key, cameraFpsOverrides_.value(key, defaultCameraFpsForControlKey(key)));
    }
    if (!key.isEmpty()) {
        return kDefaultCameraFps;
    }
    return kDefaultCameraFps;
}

QString MainWindow::imageTopicFpsParamName(const QString& rosTopic) const {
    return QStringLiteral("/airsim_node/image_topic_fps/%1").arg(encodedImageTopicFpsKey(rosTopic));
}

void MainWindow::setAirsimImageTopicFpsLimit(const QString& rosTopic, int fps) {
    const QString topic = normalizedRosTopicName(rosTopic);
    if (topic.isEmpty()) {
        return;
    }
    const int clampedFps = std::max(1, std::min(kMaxCameraFps, fps));
    const QString paramName = imageTopicFpsParamName(topic);
    if (airsimImageTopicFpsLastApplied_.value(paramName, -1) == clampedFps) {
        return;
    }
    if (!ros::isInitialized() || !ros::master::check()) {
        return;
    }
    ros::param::set(paramName.toStdString(), static_cast<double>(clampedFps));
    airsimImageTopicFpsLastApplied_[paramName] = clampedFps;
}

void MainWindow::applyCameraFpsForControl(const QString& controlKey,
                                          const QString& rosTopic,
                                          int fps,
                                          bool showStatus) {
    const QString key = controlKey.trimmed();
    if (key.isEmpty()) {
        return;
    }
    const int clampedFps = clampCameraFpsForControlKey(key, fps);
    const QString stationPrefix = QString::fromLatin1(kCameraFpsStationPrefix);

    if (isGuiPreviewFpsKey(key)) {
        const bool overrideChanged = !cameraFpsOverrides_.contains(key)
            || cameraFpsOverrides_.value(key) != clampedFps;
        if (showStatus) {
            cameraFpsOverrides_[key] = clampedFps;
        }
        if (airsimLiveFpsSpin_) {
            QSignalBlocker blocker(airsimLiveFpsSpin_);
            airsimLiveFpsSpin_->setValue(static_cast<double>(clampedFps));
        }
        if (airSimViewController_) {
            airSimViewController_->setFramesPerSecond(static_cast<double>(clampedFps));
        }
        if (showStatus) {
            onStatusMessage(tr("Updated GuiPreview FPS to %1 fps.").arg(clampedFps));
        }
        if (showStatus && overrideChanged) {
            markGuiConfigDirty();
        }
        return;
    }

    if (!rosTopic.trimmed().isEmpty()) {
        setAirsimImageTopicFpsLimit(rosTopic, clampedFps);
    }

    if (key.startsWith(stationPrefix)) {
        const QString stationKey = key.mid(stationPrefix.size());
        const int stationIndex = stationIndexForKey(stationKey);
        if (stationIndex < 0 || stationIndex >= static_cast<int>(stations_.size())) {
            return;
        }
        const double normalizedFps = static_cast<double>(clampedFps);
        StationCameraWindow* window = findStationCameraWindow(stationKey);
        if (window && window->view && window->view->controller()) {
            window->view->controller()->setFramesPerSecond(normalizedFps);
        }
        const bool stationFpsChanged =
            std::abs(stations_[static_cast<size_t>(stationIndex)].previewFps - normalizedFps) > 1e-9;
        if (showStatus && stationFpsChanged) {
            stations_[static_cast<size_t>(stationIndex)].previewFps = normalizedFps;
            markBaseStationsDirty();
            refreshInfoPanel();
        }
        if (showStatus) {
            onStatusMessage(tr("Updated station camera FPS for %1 to %2 fps.")
                                .arg(stations_[static_cast<size_t>(stationIndex)].name)
                                .arg(clampedFps));
        }
        return;
    }

    const bool overrideChanged = !cameraFpsOverrides_.contains(key)
        || cameraFpsOverrides_.value(key) != clampedFps;
    if (showStatus) {
        cameraFpsOverrides_[key] = clampedFps;
    }
    if (showStatus && !rosTopic.trimmed().isEmpty()) {
        onStatusMessage(tr("Updated ROS camera FPS for %1 to %2 fps.")
                            .arg(normalizedRosTopicName(rosTopic))
                            .arg(clampedFps));
    }
    if (showStatus && overrideChanged) {
        markGuiConfigDirty();
    }
}

void MainWindow::applyKnownCameraFpsLimits() {
    for (const BaseStation& station : stations_) {
        const QString stationKey = stationCameraKey(station);
        if (stationKey.isEmpty()) {
            continue;
        }
        const QString controlKey = QString::fromLatin1(kCameraFpsStationPrefix) + stationKey;
        const QString rosTopic = normalizedRosTopicName(station.previewRosTopic.trimmed().isEmpty()
            ? defaultPreviewRosTopicForStation(station)
            : station.previewRosTopic.trimmed());
        setAirsimImageTopicFpsLimit(rosTopic, cameraFpsForControl(controlKey, rosTopic));
    }

    const QString rosPrefix = QString::fromLatin1(kCameraFpsRosTopicPrefix);
    for (auto it = cameraFpsOverrides_.cbegin(); it != cameraFpsOverrides_.cend(); ++it) {
        const QString key = it.key();
        if (key.startsWith(rosPrefix)) {
            setAirsimImageTopicFpsLimit(key.mid(rosPrefix.size()), it.value());
        }
    }

    if (stationCameraFpsTopicCombo_ && stationCameraFpsSpin_) {
        const QString currentKey = stationCameraFpsTopicCombo_->currentData(Qt::UserRole).toString().trimmed();
        const QString currentTopic = stationCameraFpsTopicCombo_->currentData(Qt::UserRole + 1).toString().trimmed();
        if (!currentTopic.isEmpty()) {
            setAirsimImageTopicFpsLimit(currentTopic, cameraFpsForControl(currentKey, currentTopic));
        }
    }
}

void MainWindow::syncStationCameraSelection() {
    for (int i = 0; i < static_cast<int>(stations_.size()); ++i) {
        updateStationCameraWindowTitle(i);
    }

    if (selectedBaseStationIndex_ >= 0 && selectedBaseStationIndex_ < static_cast<int>(stations_.size())) {
        ensureStationCameraWindow(selectedBaseStationIndex_);
    }

    syncStationCameraFpsControls();
}

bool MainWindow::stationCameraHasRosSubscribers(const BaseStation& station) const {
    const QString rosTopic = station.previewRosTopic.trimmed().isEmpty()
        ? defaultPreviewRosTopicForStation(station)
        : station.previewRosTopic.trimmed();
    return rosTopicHasSubscribers(rosTopic, true);
}

QString MainWindow::stationCameraPublishControlService(const BaseStation& station) const {
    const QString cameraName = station.previewCameraName.trimmed().isEmpty()
        ? defaultPreviewCameraNameForStation(station)
        : station.previewCameraName.trimmed();
    if (cameraName.isEmpty()) {
        return QString();
    }
    return QStringLiteral("/airsim_node/external_cameras/%1/set_publishing").arg(cameraName);
}

bool MainWindow::setStationCameraAirsimNodePublishing(const BaseStation& station,
                                                      bool enabled,
                                                      QString* errorMessage) {
    if (errorMessage) {
        errorMessage->clear();
    }
    const QString serviceName = stationCameraPublishControlService(station);
    if (serviceName.isEmpty()) {
        if (errorMessage) {
            *errorMessage = tr("Station camera control service name is empty.");
        }
        return false;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    const bool knownEnabled = stationCameraAirsimNodePublishingState_.value(serviceName, false);
    const qint64 lastRequestMs = stationCameraAirsimNodePublishingLastRequestMs_.value(serviceName, 0);
    if (enabled) {
        if (knownEnabled && nowMs - lastRequestMs < kStationCameraPublishLeaseRefreshMs) {
            return true;
        }
        if (!knownEnabled && lastRequestMs > 0 && nowMs - lastRequestMs < kStationCameraPublishRetryMs) {
            if (errorMessage) {
                *errorMessage = tr("Waiting before retrying %1.").arg(serviceName);
            }
            return false;
        }
    } else {
        if (!knownEnabled) {
            return true;
        }
    }

    stationCameraAirsimNodePublishingLastRequestMs_[serviceName] = nowMs;

    if (!ros::isInitialized() || !ros::master::check()) {
        if (enabled) {
            stationCameraAirsimNodePublishingState_[serviceName] = false;
        }
        if (errorMessage) {
            *errorMessage = tr("ROS master is not available.");
        }
        return false;
    }

    const std::string serviceNameStd = serviceName.toStdString();
    if (!ros::service::exists(serviceNameStd, false)) {
        if (enabled) {
            stationCameraAirsimNodePublishingState_[serviceName] = false;
        }
        if (errorMessage) {
            *errorMessage = tr("Waiting for %1.").arg(serviceName);
        }
        return false;
    }

    std_srvs::SetBool srv;
    srv.request.data = enabled;
    if (!ros::service::call(serviceNameStd, srv)) {
        if (enabled) {
            stationCameraAirsimNodePublishingState_[serviceName] = false;
        }
        if (errorMessage) {
            *errorMessage = tr("Failed to call %1.").arg(serviceName);
        }
        return false;
    }
    if (!srv.response.success) {
        if (enabled) {
            stationCameraAirsimNodePublishingState_[serviceName] = false;
        }
        if (errorMessage) {
            const QString responseMessage = QString::fromStdString(srv.response.message).trimmed();
            *errorMessage = responseMessage.isEmpty()
                ? tr("%1 rejected the request.").arg(serviceName)
                : responseMessage;
        }
        return false;
    }
    stationCameraAirsimNodePublishingState_[serviceName] = enabled;
    return true;
}

void MainWindow::disableAllStationCameraAirsimNodePublishing() {
    if (!ros::isInitialized() || !ros::master::check()) {
        return;
    }
    for (const BaseStation& station : stations_) {
        const QString serviceName = stationCameraPublishControlService(station);
        if (!stationCameraAirsimNodePublishingState_.value(serviceName, false)) {
            continue;
        }
        setStationCameraAirsimNodePublishing(station, false);
    }
}

void MainWindow::refreshStationCameraStreamingState() {
    const bool rosMasterAvailable = ros::isInitialized() && ros::master::check();
    if (rosMasterAvailable) {
        applyKnownCameraFpsLimits();
    } else {
        airsimImageTopicFpsLastApplied_.clear();
    }
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    const bool mainLiveViewRunning = airSimViewController_ && airSimViewController_->isRunning();
    const bool mainLiveViewConnectedNow = mainLiveViewRunning && airSimViewController_->isConnected();
    if (mainLiveViewConnectedNow) {
        stationCameraLastAirSimConnectedMs_ = nowMs;
    } else if (!mainLiveViewRunning) {
        stationCameraLastAirSimConnectedMs_ = 0;
    }
    const bool mainLiveViewConnected = mainLiveViewRunning
        && (mainLiveViewConnectedNow
            || (stationCameraLastAirSimConnectedMs_ > 0
                && nowMs - stationCameraLastAirSimConnectedMs_ <= kStationCameraAirSimDisconnectGraceMs));
    const QSet<QString> subscribedTopics = rosMasterAvailable
        ? rosTopicsWithSubscribers(true)
        : QSet<QString>();

    for (int i = 0; i < static_cast<int>(stations_.size()); ++i) {
        const BaseStation& station = stations_[static_cast<size_t>(i)];
        const QString key = stationCameraKey(station);
        if (key.isEmpty()) {
            continue;
        }

        ensureStationCameraWindow(i);
        StationCameraWindow* window = findStationCameraWindow(key);
        if (!window || !window->view || !window->view->controller()) {
            continue;
        }

        auto* controller = window->view->controller();

        const bool pinned = window->pinned;
        const bool selected = (i == selectedBaseStationIndex_);
        const bool previewDemand = selected || pinned;
        const QString rosTopic = station.previewRosTopic.trimmed().isEmpty()
            ? defaultPreviewRosTopicForStation(station)
            : station.previewRosTopic.trimmed();
        const bool subscriberDemand = rosMasterAvailable && subscribedTopics.contains(rosTopic);
        const bool manualPublishDemand = manualRosTopicPublishingEnabled_
            && manualRosTopicPublishing_.value(rosTopic, false);
        const bool forcePublishDemand = !manualRosTopicPublishingEnabled_ && forceAllStationCameraPublishing_;
        const bool publishDemand = rosMasterAvailable
            && mainLiveViewConnected
            && (manualRosTopicPublishingEnabled_
                ? manualPublishDemand
                : (previewDemand || subscriberDemand || forcePublishDemand));
        const bool shouldSubscribe = rosMasterAvailable
            && mainLiveViewConnected
            && (manualRosTopicPublishingEnabled_
                ? manualPublishDemand
                : (previewDemand || forcePublishDemand));
        controller->setRosPublishingEnabled(false);

        QString publishControlError;
        bool publishControlOk = true;
        if (rosMasterAvailable) {
            publishControlOk = setStationCameraAirsimNodePublishing(station, publishDemand, &publishControlError);
            if (!publishDemand) {
                publishControlOk = true;
                publishControlError.clear();
            }
        } else if (publishDemand) {
            publishControlOk = false;
        }

        if (window->statusLabel) {
            if (!mainLiveViewConnected && (previewDemand || subscriberDemand || manualPublishDemand || forcePublishDemand)) {
                window->statusLabel->setText(tr("Connect AirSim Live View before displaying station cameras."));
            } else if (!rosMasterAvailable && (previewDemand || subscriberDemand || manualPublishDemand || forcePublishDemand)) {
                window->statusLabel->setText(tr("Waiting for ROS master / airsim_node_ex."));
            } else if (publishDemand && !publishControlOk) {
                window->statusLabel->setText(publishControlError.isEmpty()
                    ? tr("Waiting for airsim_node_ex station camera control service.")
                    : publishControlError);
            } else if (manualRosTopicPublishingEnabled_ && shouldSubscribe) {
                window->statusLabel->setText(tr("Subscribing to ROS topic by Developer Tools manual override."));
            } else if (manualRosTopicPublishingEnabled_ && previewDemand && !manualPublishDemand) {
                window->statusLabel->setText(tr("Station preview disabled by Developer Tools topic override."));
            } else if (forcePublishDemand && shouldSubscribe) {
                window->statusLabel->setText(tr("Subscribing to force this station ROS topic on."));
            } else if (previewDemand && shouldSubscribe) {
                window->statusLabel->setText(tr("Displaying ROS topic; airsim_node_ex publishes while subscribed."));
            } else if (subscriberDemand) {
                window->statusLabel->setText(tr("ROS topic requested by external subscriber; GUI preview idle."));
            } else if (!controller->isRunning()) {
                window->statusLabel->setText(tr("Idle"));
            }
        }

        if (window->dock) {
            const bool shouldShow = previewDemand && mainLiveViewConnected;
            if (shouldShow) {
                if (!window->dock->isVisible()) {
                    window->dock->show();
                }
                if (!window->dock->isFloating()) {
                    resizeDocks(QList<QDockWidget*>() << window->dock, QList<int>() << 420, Qt::Vertical);
                }
            } else if (window->dock->isVisible()) {
                window->dock->hide();
            }
        }

        syncStationCameraWindow(i);
        controller->setExternalPoseControlEnabled(previewDemand);
        controller->setRosImageSubscriptionEnabled(shouldSubscribe);

        updateStationCameraWindowTitle(i);
    }
}

void MainWindow::removeMissingStationCameraWindows() {
    std::vector<StationCameraWindow> kept;
    kept.reserve(stationCameraWindows_.size());
    for (auto& window : stationCameraWindows_) {
        if (stationIndexForKey(window.stationKey) >= 0) {
            kept.push_back(window);
            continue;
        }
        if (window.view && window.view->controller()) {
            window.view->controller()->stop();
        }
        if (window.dock) {
            window.dock->deleteLater();
        }
    }
    stationCameraWindows_.swap(kept);
    lastLeftDock_ = layersDock_;
    for (const auto& window : stationCameraWindows_) {
        if (window.dock) {
            lastLeftDock_ = window.dock;
        }
    }
}

void MainWindow::stopAllStationCameraWindows() {
    for (auto& window : stationCameraWindows_) {
        if (window.view && window.view->controller()) {
            window.view->controller()->stop();
        }
    }
}

void MainWindow::onAirSimLiveViewStatus(const QString& text) {
    statusBar()->showMessage(text, 3000);
}

CoordinateTransformSettings MainWindow::defaultCoordinateTransformSettings() const {
    CoordinateTransformSettings settings;
    const auto firstExistingConfigPath = [this](const QStringList& relativePaths) -> QString {
        for (const QString& relativePath : relativePaths) {
            const QString candidate = findConfigFile(relativePath);
            if (QFileInfo::exists(candidate)) {
                return candidate;
            }
        }
        return QString();
    };
    const auto optionalConfigPath = [&firstExistingConfigPath](const QString& relativePath) -> QString {
        const QString candidate = firstExistingConfigPath({relativePath});
        return QFileInfo::exists(candidate) ? candidate : QString();
    };
    settings.rosToSceneMatrixPath = firstExistingConfigPath({
        QStringLiteral("SimART_sample_maps/BigCitySample/config/ROS_to_3D_matrix.json")
    });
    settings.airsimToSceneMatrixPath = firstExistingConfigPath({
        QStringLiteral("SimART_sample_maps/BigCitySample/config/airsim_to_3D_matrix.json")
    });
    settings.rxArrayFacingDirection = QStringLiteral("front");
    settings.uavToRxArrayTx = 0.0;
    settings.uavToRxArrayTy = 0.0;
    settings.uavToRxArrayTz = 0.0;
    settings.uavToRxArrayCenterMatrixPath = optionalConfigPath(QStringLiteral("config/uav_to_rx_array_center_matrix.json"));
    settings.rxArrayElementsJsonPath = optionalConfigPath(QStringLiteral("config/rx_array_elements.json"));
    settings.outputFrameKey = coordinateOutputFrameToKey(CoordinateOutputFrame::Scene3D);
    return settings;
}

void MainWindow::loadCoordinateTransformSettings() {
    const CoordinateTransformSettings defaults = defaultCoordinateTransformSettings();
    QSettings settings(QStringLiteral("OpenAI"), QStringLiteral("airsim_gui_UErealtime"));
    coordinateTransforms_.rosToSceneMatrixPath = settings.value(
        QStringLiteral("coordinateFrames/rosToSceneMatrixPath"), defaults.rosToSceneMatrixPath).toString().trimmed();
    coordinateTransforms_.airsimToSceneMatrixPath = settings.value(
        QStringLiteral("coordinateFrames/airsimToSceneMatrixPath"), defaults.airsimToSceneMatrixPath).toString().trimmed();
    coordinateTransforms_.rxArrayFacingDirection = settings.value(
        QStringLiteral("coordinateFrames/rxArrayFacingDirection"), defaults.rxArrayFacingDirection).toString().trimmed();
    coordinateTransforms_.uavToRxArrayTx = settings.value(
        QStringLiteral("coordinateFrames/uavToRxArrayTx"), defaults.uavToRxArrayTx).toDouble();
    coordinateTransforms_.uavToRxArrayTy = settings.value(
        QStringLiteral("coordinateFrames/uavToRxArrayTy"), defaults.uavToRxArrayTy).toDouble();
    coordinateTransforms_.uavToRxArrayTz = settings.value(
        QStringLiteral("coordinateFrames/uavToRxArrayTz"), defaults.uavToRxArrayTz).toDouble();
    if (settings.contains(QStringLiteral("coordinateFrames/uavToRxArrayCenterMatrixPath"))) {
        coordinateTransforms_.uavToRxArrayCenterMatrixPath = settings.value(
            QStringLiteral("coordinateFrames/uavToRxArrayCenterMatrixPath")).toString().trimmed();
    } else {
        coordinateTransforms_.uavToRxArrayCenterMatrixPath = defaults.uavToRxArrayCenterMatrixPath;
    }
    if (settings.contains(QStringLiteral("coordinateFrames/rxArrayElementsJsonPath"))) {
        coordinateTransforms_.rxArrayElementsJsonPath = settings.value(
            QStringLiteral("coordinateFrames/rxArrayElementsJsonPath")).toString().trimmed();
    } else {
        coordinateTransforms_.rxArrayElementsJsonPath = defaults.rxArrayElementsJsonPath;
    }
    coordinateTransforms_.outputFrameKey = settings.value(
        QStringLiteral("coordinateFrames/outputFrameKey"), defaults.outputFrameKey).toString().trimmed();
    if (coordinateTransforms_.rosToSceneMatrixPath.isEmpty()) {
        coordinateTransforms_.rosToSceneMatrixPath = defaults.rosToSceneMatrixPath;
    }
    if (coordinateTransforms_.airsimToSceneMatrixPath.isEmpty()) {
        coordinateTransforms_.airsimToSceneMatrixPath = defaults.airsimToSceneMatrixPath;
    }
    coordinateTransforms_.rxArrayFacingDirection = normalizeRxArrayFacingDirectionKey(coordinateTransforms_.rxArrayFacingDirection);
    coordinateTransforms_.outputFrameKey = coordinateOutputFrameToKey(coordinateOutputFrameFromKey(coordinateTransforms_.outputFrameKey));
}

void MainWindow::applyCoordinateTransformSettings(bool restartRosPipeline) {
    if (frameTransformManager_) {
        frameTransformManager_->setRosToSceneTransformPath(coordinateTransforms_.rosToSceneMatrixPath);
        frameTransformManager_->setAirSimToSceneTransformPath(coordinateTransforms_.airsimToSceneMatrixPath);
        frameTransformManager_->setRxArrayFacingDirection(coordinateTransforms_.rxArrayFacingDirection);
        frameTransformManager_->setUavToRxArrayOffset(
            coordinateTransforms_.uavToRxArrayTx,
            coordinateTransforms_.uavToRxArrayTy,
            coordinateTransforms_.uavToRxArrayTz);
        frameTransformManager_->setUavToRxArrayCenterTransformPath(coordinateTransforms_.uavToRxArrayCenterMatrixPath);
        frameTransformManager_->setRxArrayElementsTransformPath(coordinateTransforms_.rxArrayElementsJsonPath);
        frameTransformManager_->setRxArrayGeometry(
            simSettings_.rxArrayNumRows,
            simSettings_.rxArrayNumCols,
            simSettings_.rxArrayVerticalSpacing,
            simSettings_.rxArrayHorizontalSpacing,
            simSettings_.fcHz);
        if (!currentDrone_.sourceFrameId.trimmed().isEmpty()) {
            frameTransformManager_->setRosFrameId(currentDrone_.sourceFrameId);
        }
        if (currentDrone_.valid) {
            frameTransformManager_->setCurrentDronePoseInScene(
                currentDrone_.position,
                currentDrone_.rollDeg,
                currentDrone_.pitchDeg,
                currentDrone_.yawDeg);
        } else {
            frameTransformManager_->clearCurrentDronePoseInScene();
        }
    }
    if (rosBridge_) {
        rosBridge_->setFrameTransformManager(frameTransformManager_);
        rosBridge_->setRosToSceneTransformPath(coordinateTransforms_.rosToSceneMatrixPath);
    }
    if (airSimViewController_) {
        airSimViewController_->setFrameTransformManager(frameTransformManager_);
        airSimViewController_->setAirsimToGuiTransformPath(coordinateTransforms_.airsimToSceneMatrixPath);
    }
    for (auto& window : stationCameraWindows_) {
        if (window.view && window.view->controller()) {
            window.view->controller()->setFrameTransformManager(frameTransformManager_);
            window.view->controller()->setAirsimToGuiTransformPath(coordinateTransforms_.airsimToSceneMatrixPath);
        }
    }
    syncStationCameraWindows();
    refreshInfoPanel();

    if (!restartRosPipeline) {
        return;
    }

    const bool rosWasRunning = rosBridge_ && rosBridge_->isRunning();
    const bool simWasRunning = simulatorProcess_ && simulatorProcess_->state() != QProcess::NotRunning;
    if (rosWasRunning || simWasRunning) {
        disconnectRos();
        connectRos();
    }
}

Vec3 MainWindow::scenePointToAirSim(const Vec3& scenePoint) const {
    if (frameTransformManager_) {
        Vec3 transformed = scenePoint;
        if (frameTransformManager_->transformPoint(scenePoint,
                                                   FrameTransformManager::sceneFrameId(),
                                                   FrameTransformManager::airsimFrameId(),
                                                   &transformed)) {
            return transformed;
        }
    }
    QString error;
    const TransformMatrix4x4 airsimToScene = loadTransformMatrixFromJson(coordinateTransforms_.airsimToSceneMatrixPath, &error);
    if (!error.trimmed().isEmpty()) {
        return scenePoint;
    }
    TransformMatrix4x4 sceneToAirsim = identityTransformMatrix4x4();
    if (!invertTransformMatrix4x4(airsimToScene, &sceneToAirsim)) {
        return scenePoint;
    }
    return transformPointByMatrix(sceneToAirsim, scenePoint);
}

Vec3 MainWindow::airSimPointToScene(const Vec3& airsimPoint) const {
    if (frameTransformManager_) {
        Vec3 transformed = airsimPoint;
        if (frameTransformManager_->transformPoint(airsimPoint,
                                                   FrameTransformManager::airsimFrameId(),
                                                   FrameTransformManager::sceneFrameId(),
                                                   &transformed)) {
            return transformed;
        }
    }
    QString error;
    const TransformMatrix4x4 airsimToScene = loadTransformMatrixFromJson(coordinateTransforms_.airsimToSceneMatrixPath, &error);
    if (!error.trimmed().isEmpty()) {
        return airsimPoint;
    }
    return transformPointByMatrix(airsimToScene, airsimPoint);
}

QString MainWindow::normalizedGuiConfigPath(const QString& path) const {
    QString normalized = path.trimmed();
    if (normalized.isEmpty()) {
        return normalized;
    }
    if (!normalized.endsWith(QLatin1String(kGuiConfigExtension), Qt::CaseInsensitive)) {
        normalized += QLatin1String(kGuiConfigExtension);
    }
    return normalized;
}

bool MainWindow::saveGuiConfigToFile(const QString& filePath, QString* errorMessage) {
    if (errorMessage) {
        errorMessage->clear();
    }

    const QString configFilePath = normalizedGuiConfigPath(filePath);
    ensureRosbagToolsWindow();
    if (stationCameraFpsTopicCombo_ && stationCameraFpsSpin_) {
        stationCameraFpsSpin_->interpretText();
        const QString key = stationCameraFpsTopicCombo_->currentData(Qt::UserRole).toString().trimmed();
        if (key.startsWith(QLatin1String(kCameraFpsRosTopicPrefix))) {
            cameraFpsOverrides_[key] = clampCameraFpsForControlKey(key, stationCameraFpsSpin_->value());
        }
    }

    QJsonObject root;
    root[QStringLiteral("format")] = QStringLiteral("airsim_gui_UErealtime_config");
    root[QStringLiteral("version")] = kGuiConfigVersion;
    root[QStringLiteral("saved_at")] = QDateTime::currentDateTime().toString(Qt::ISODate);
    root[QStringLiteral("base_stations_file_path")] = baseStationsConfigPath_;
    root[QStringLiteral("internal_rf_topic")] = internalRfTopic_;
    root[QStringLiteral("internal_sys_topic")] = internalSysTopic_;
    root[QStringLiteral("pose_topic")] = poseTopicCombo_ ? poseTopicCombo_->currentText().trimmed() : QString();
    root[QStringLiteral("trajectory_topic")] = trajectoryTopicCombo_ ? trajectoryTopicCombo_->currentText().trimmed() : QString();
    root[QStringLiteral("imported_scene_mesh_path")] = currentSceneMeshPath_;

    QJsonObject layerStates;
    if (layerTree_) {
        for (QTreeWidgetItemIterator it(layerTree_); *it; ++it) {
            QTreeWidgetItem* item = *it;
            const QString key = item->data(0, Qt::UserRole).toString().trimmed();
            if (!key.isEmpty()) {
                layerStates[key] = (item->checkState(0) == Qt::Checked);
            }
        }
    }
    root[QStringLiteral("layer_states")] = layerStates;

    root[QStringLiteral("simulation_settings")] = simulationSettingsToJson(simSettings_);
    root[QStringLiteral("ckm_settings")] = ckmSettingsToJson(ckmSettings_);
    root[QStringLiteral("coordinate_transforms")] = coordinateTransformSettingsToJson(coordinateTransforms_);

    QJsonObject liveView;
    liveView[QStringLiteral("airsim_host")] = airsimHostEdit_ ? airsimHostEdit_->text().trimmed() : QString();
    liveView[QStringLiteral("airsim_port")] = airsimPortEdit_ ? airsimPortEdit_->text().trimmed() : QString();
    liveView[QStringLiteral("airsim_settings_path")] = airsimSettingsPathEdit_ ? airsimSettingsPathEdit_->text().trimmed() : QString();
    liveView[QStringLiteral("camera_name")] = airsimLiveCameraCombo_ ? airsimLiveCameraCombo_->currentText().trimmed() : QString();
    liveView[QStringLiteral("vehicle_name")] = airsimLiveVehicleEdit_ ? airsimLiveVehicleEdit_->text().trimmed() : QString();
    liveView[QStringLiteral("follow_vehicle_focus")] = airsimLiveFollowCheck_ ? airsimLiveFollowCheck_->isChecked() : true;
    liveView[QStringLiteral("live_fps")] = airsimLiveFpsSpin_
        ? static_cast<double>(clampCameraFpsForControlKey(QString::fromLatin1(kCameraFpsGuiPreviewKey),
                                                          static_cast<int>(std::lround(airsimLiveFpsSpin_->value()))))
        : static_cast<double>(kDefaultGuiPreviewFps);
    liveView[QStringLiteral("brightness_percent")] = airsimBrightnessSlider_ ? airsimBrightnessSlider_->value() : 100;
    root[QStringLiteral("live_view")] = liveView;

    QJsonObject cameraFpsOverrides;
    const QString stationPrefix = QString::fromLatin1(kCameraFpsStationPrefix);
    const QString guiPreviewKey = QString::fromLatin1(kCameraFpsGuiPreviewKey);
    const QString rosPrefix = QString::fromLatin1(kCameraFpsRosTopicPrefix);
    QSet<QString> serializedCameraFpsKeys;
    bool serializedCurrentRosImageItems = false;
    QString settingsParseError;
    const QString preferredVehicle = airsimLiveVehicleEdit_ ? airsimLiveVehicleEdit_->text().trimmed() : QString();
    for (const QString& candidate : effectiveAirSimSettingsPaths()) {
        if (!QFileInfo::exists(candidate)) {
            continue;
        }
        const QVector<CameraFpsControlItem> rosImageItems =
            parseAirSimRosImageTopicItems(candidate, preferredVehicle, &settingsParseError);
        for (const CameraFpsControlItem& item : rosImageItems) {
            const QString key = item.controlKey.trimmed();
            if (key.isEmpty()) {
                continue;
            }
            cameraFpsOverrides[key] = cameraFpsForControl(key, item.rosTopic);
            serializedCameraFpsKeys.insert(key);
        }
        if (!rosImageItems.isEmpty()) {
            serializedCurrentRosImageItems = true;
            break;
        }
    }
    for (auto it = cameraFpsOverrides_.cbegin(); it != cameraFpsOverrides_.cend(); ++it) {
        const QString key = it.key().trimmed();
        if (!key.isEmpty()
            && !serializedCameraFpsKeys.contains(key)
            && !key.startsWith(stationPrefix)
            && !(serializedCurrentRosImageItems && key.startsWith(rosPrefix))
            && key != guiPreviewKey) {
            cameraFpsOverrides[key] = clampCameraFpsForControlKey(key, it.value());
        }
    }
    root[QStringLiteral("camera_fps_overrides")] = cameraFpsOverrides;

    QJsonObject rosbagRecord;
    rosbagRecord[QStringLiteral("output_path")] = rosbagRecordPathEdit_ ? rosbagRecordPathEdit_->text().trimmed() : QString();
    rosbagRecord[QStringLiteral("include_rf")] = rosbagRecordIncludeRfCheck_ ? rosbagRecordIncludeRfCheck_->isChecked() : true;
    rosbagRecord[QStringLiteral("include_sys")] = rosbagRecordIncludeSysCheck_ ? rosbagRecordIncludeSysCheck_->isChecked() : true;
    rosbagRecord[QStringLiteral("include_beam")] = rosbagRecordIncludeBeamCheck_ ? rosbagRecordIncludeBeamCheck_->isChecked() : true;
    root[QStringLiteral("rosbag_record")] = rosbagRecord;

    QJsonObject rosbagPlayback;
    rosbagPlayback[QStringLiteral("input_bag")] = rosbagPlaybackInputEdit_ ? rosbagPlaybackInputEdit_->text().trimmed() : QString();
    rosbagPlayback[QStringLiteral("rf_topic")] = rosbagPlaybackRfTopicEdit_ ? rosbagPlaybackRfTopicEdit_->text().trimmed() : QString();
    rosbagPlayback[QStringLiteral("sys_topic")] = rosbagPlaybackSysTopicEdit_ ? rosbagPlaybackSysTopicEdit_->text().trimmed() : QString();
    rosbagPlayback[QStringLiteral("beam_topic")] = rosbagPlaybackBeamTopicEdit_ ? rosbagPlaybackBeamTopicEdit_->text().trimmed() : QString();
    rosbagPlayback[QStringLiteral("beam_codebook_topic")] = rosbagPlaybackBeamCodebookTopicEdit_ ? rosbagPlaybackBeamCodebookTopicEdit_->text().trimmed() : QString();
    rosbagPlayback[QStringLiteral("play_rf")] = rosbagPlaybackIncludeRfCheck_ ? rosbagPlaybackIncludeRfCheck_->isChecked() : true;
    rosbagPlayback[QStringLiteral("play_sys")] = rosbagPlaybackIncludeSysCheck_ ? rosbagPlaybackIncludeSysCheck_->isChecked() : true;
    rosbagPlayback[QStringLiteral("play_beam")] = rosbagPlaybackIncludeBeamCheck_ ? rosbagPlaybackIncludeBeamCheck_->isChecked() : true;
    rosbagPlayback[QStringLiteral("publish_clock")] = rosbagPlaybackClockCheck_ ? rosbagPlaybackClockCheck_->isChecked() : true;
    rosbagPlayback[QStringLiteral("play_rate")] = rosbagPlaybackRateSpin_ ? rosbagPlaybackRateSpin_->value() : 1.0;
    root[QStringLiteral("rosbag_playback")] = rosbagPlayback;

    QJsonObject rosbagResim;
    rosbagResim[QStringLiteral("input_bag")] = rosbagResimInputEdit_ ? rosbagResimInputEdit_->text().trimmed() : QString();
    rosbagResim[QStringLiteral("output_bag")] = rosbagResimOutputEdit_ ? rosbagResimOutputEdit_->text().trimmed() : QString();
    rosbagResim[QStringLiteral("pose_topic")] = rosbagResimPoseTopicEdit_ ? rosbagResimPoseTopicEdit_->text().trimmed() : QString();
    rosbagResim[QStringLiteral("rf_topic")] = rosbagResimRfTopicEdit_ ? rosbagResimRfTopicEdit_->text().trimmed() : QString();
    rosbagResim[QStringLiteral("sys_topic")] = rosbagResimSysTopicEdit_ ? rosbagResimSysTopicEdit_->text().trimmed() : QString();
    rosbagResim[QStringLiteral("beam_topic")] = rosbagResimBeamTopicEdit_ ? rosbagResimBeamTopicEdit_->text().trimmed() : QString();
    rosbagResim[QStringLiteral("beam_codebook_topic")] = rosbagResimBeamCodebookTopicEdit_ ? rosbagResimBeamCodebookTopicEdit_->text().trimmed() : QString();
    rosbagResim[QStringLiteral("write_rf")] = rosbagResimWriteRfCheck_ ? rosbagResimWriteRfCheck_->isChecked() : true;
    rosbagResim[QStringLiteral("write_sys")] = rosbagResimWriteSysCheck_ ? rosbagResimWriteSysCheck_->isChecked() : true;
    rosbagResim[QStringLiteral("write_beam")] = rosbagResimWriteBeamCheck_ ? rosbagResimWriteBeamCheck_->isChecked() : true;
    rosbagResim[QStringLiteral("message_frequency_hz")] = rosbagResimMessageFrequencySpin_ ? rosbagResimMessageFrequencySpin_->value() : 10.0;
    root[QStringLiteral("rosbag_resim")] = rosbagResim;

    makeGuiConfigPathsPortable(root, configFilePath);

    QFile file(configFilePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (errorMessage) {
            *errorMessage = tr("Could not write SimART config: %1").arg(configFilePath);
        }
        return false;
    }
    if (file.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) < 0) {
        if (errorMessage) {
            *errorMessage = tr("Failed to serialize SimART config: %1").arg(configFilePath);
        }
        return false;
    }
    file.close();
    return true;
}

bool MainWindow::loadGuiConfigFromFile(const QString& filePath, QString* errorMessage) {
    if (errorMessage) {
        errorMessage->clear();
    }

    const bool previousDirtySuppression = suppressDirtyTracking_;
    suppressDirtyTracking_ = true;
    auto restoreDirtyTracking = qScopeGuard([&]() {
        suppressDirtyTracking_ = previousDirtySuppression;
    });

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = tr("Could not open SimART config: %1").arg(filePath);
        }
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage) {
            *errorMessage = tr("SimART config parse failed: %1").arg(parseError.errorString());
        }
        return false;
    }

    QJsonObject root = doc.object();
    const QString format = root.value(QStringLiteral("format")).toString();
    if (!format.isEmpty() && format != QStringLiteral("airsim_gui_UErealtime_config")) {
        if (errorMessage) {
            *errorMessage = tr("Unsupported SimART config format: %1").arg(format);
        }
        return false;
    }

    resolveGuiConfigPaths(root, filePath);
    ensureRosbagToolsWindow();

    QString importedSceneMeshPath;
    bool importedSceneMeshPathProvided = false;

    if (root.contains(QStringLiteral("simulation_settings")) && root.value(QStringLiteral("simulation_settings")).isObject()) {
        simSettings_ = simulationSettingsFromJson(root.value(QStringLiteral("simulation_settings")).toObject(), simSettings_);
    }
    if (root.contains(QStringLiteral("ckm_settings")) && root.value(QStringLiteral("ckm_settings")).isObject()) {
        ckmSettings_ = ckmSettingsFromJson(root.value(QStringLiteral("ckm_settings")).toObject(), ckmSettings_);
    }
    if (ckmSettings_.outputDir.trimmed().isEmpty()) {
        ckmSettings_.outputDir = defaultDenseCkmOutputDir();
    }
    if (root.contains(QStringLiteral("coordinate_transforms")) && root.value(QStringLiteral("coordinate_transforms")).isObject()) {
        coordinateTransforms_ = coordinateTransformSettingsFromJson(root.value(QStringLiteral("coordinate_transforms")).toObject(), coordinateTransforms_);
    }
    if (root.contains(QStringLiteral("base_stations_file_path"))) {
        baseStationsConfigPath_ = root.value(QStringLiteral("base_stations_file_path")).toString().trimmed();
    }
    if (root.contains(QStringLiteral("internal_rf_topic"))) {
        internalRfTopic_ = root.value(QStringLiteral("internal_rf_topic")).toString(internalRfTopic_).trimmed();
        if (internalRfTopic_.isEmpty()) {
            internalRfTopic_ = QStringLiteral("/airsim_gui_UErealtime/rf_observations");
        }
    }
    if (root.contains(QStringLiteral("internal_sys_topic"))) {
        internalSysTopic_ = root.value(QStringLiteral("internal_sys_topic")).toString(internalSysTopic_).trimmed();
        if (internalSysTopic_.isEmpty()) {
            internalSysTopic_ = QStringLiteral("/airsim_gui_UErealtime/sys_observations");
        }
    }
    if (poseTopicCombo_ && root.contains(QStringLiteral("pose_topic"))) {
        poseTopicCombo_->setEditText(root.value(QStringLiteral("pose_topic")).toString().trimmed());
    }
    if (trajectoryTopicCombo_ && root.contains(QStringLiteral("trajectory_topic"))) {
        trajectoryTopicCombo_->setEditText(root.value(QStringLiteral("trajectory_topic")).toString().trimmed());
    }
    if (root.contains(QStringLiteral("imported_scene_mesh_path"))) {
        importedSceneMeshPath = root.value(QStringLiteral("imported_scene_mesh_path")).toString().trimmed();
        importedSceneMeshPathProvided = true;
    }

    bool baseStationsLoaded = false;
    if (!baseStationsConfigPath_.trimmed().isEmpty()) {
        std::vector<BaseStation> loadedStations;
        QString stationError;
        if (!JsonConfig::loadBaseStations(baseStationsConfigPath_, loadedStations, &stationError)) {
            if (errorMessage) {
                *errorMessage = tr("Failed to load base-station JSON %1: %2")
                                    .arg(baseStationsConfigPath_, stationError);
            }
            return false;
        }
        applyBaseStations(loadedStations);
        baseStationsLoaded = true;
    }

    if (!baseStationsLoaded
        && root.contains(QStringLiteral("base_stations"))
        && root.value(QStringLiteral("base_stations")).isArray()) {
        std::vector<BaseStation> loadedStations;
        const QJsonArray array = root.value(QStringLiteral("base_stations")).toArray();
        loadedStations.reserve(static_cast<size_t>(array.size()));
        for (const QJsonValue& value : array) {
            if (value.isObject()) {
                loadedStations.push_back(baseStationFromJson(value.toObject()));
            }
        }
        applyBaseStations(loadedStations);
        baseStationsLoaded = true;
    }

    if (root.contains(QStringLiteral("layer_states")) && root.value(QStringLiteral("layer_states")).isObject() && layerTree_) {
        const QJsonObject layerStates = root.value(QStringLiteral("layer_states")).toObject();
        const QSignalBlocker blocker(layerTree_);
        for (QTreeWidgetItemIterator it(layerTree_); *it; ++it) {
            QTreeWidgetItem* item = *it;
            const QString key = item->data(0, Qt::UserRole).toString().trimmed();
            if (!key.isEmpty() && layerStates.contains(key)) {
                item->setCheckState(0, layerStates.value(key).toBool(true) ? Qt::Checked : Qt::Unchecked);
                applyLayerState(key, item->checkState(0) == Qt::Checked);
            }
        }
    }

    if (root.contains(QStringLiteral("live_view")) && root.value(QStringLiteral("live_view")).isObject()) {
        const QJsonObject liveView = root.value(QStringLiteral("live_view")).toObject();
        if (airsimHostEdit_ && liveView.contains(QStringLiteral("airsim_host"))) airsimHostEdit_->setText(liveView.value(QStringLiteral("airsim_host")).toString());
        if (airsimPortEdit_ && liveView.contains(QStringLiteral("airsim_port"))) airsimPortEdit_->setText(liveView.value(QStringLiteral("airsim_port")).toString());
        if (liveView.contains(QStringLiteral("airsim_settings_path"))) {
            setSelectedAirSimSettingsPath(liveView.value(QStringLiteral("airsim_settings_path")).toString());
        } else {
            setSelectedAirSimSettingsPath(QString());
        }
        if (airsimLiveCameraCombo_ && liveView.contains(QStringLiteral("camera_name"))) airsimLiveCameraCombo_->setEditText(liveView.value(QStringLiteral("camera_name")).toString());
        if (airsimLiveVehicleEdit_ && liveView.contains(QStringLiteral("vehicle_name"))) airsimLiveVehicleEdit_->setText(liveView.value(QStringLiteral("vehicle_name")).toString());
        if (airsimLiveFollowCheck_ && liveView.contains(QStringLiteral("follow_vehicle_focus"))) airsimLiveFollowCheck_->setChecked(liveView.value(QStringLiteral("follow_vehicle_focus")).toBool(true));
        if (airsimLiveFpsSpin_ && liveView.contains(QStringLiteral("live_fps"))) {
            const int fps = clampCameraFpsForControlKey(QString::fromLatin1(kCameraFpsGuiPreviewKey),
                                                        static_cast<int>(std::lround(liveView.value(QStringLiteral("live_fps")).toDouble(static_cast<double>(kDefaultGuiPreviewFps)))));
            airsimLiveFpsSpin_->setValue(static_cast<double>(fps));
        }
        if (airsimBrightnessSlider_ && liveView.contains(QStringLiteral("brightness_percent"))) airsimBrightnessSlider_->setValue(liveView.value(QStringLiteral("brightness_percent")).toInt(100));
    } else {
        setSelectedAirSimSettingsPath(QString());
    }

    cameraFpsOverrides_.clear();
    if (root.contains(QStringLiteral("camera_fps_overrides")) && root.value(QStringLiteral("camera_fps_overrides")).isObject()) {
        const QJsonObject cameraFpsOverrides = root.value(QStringLiteral("camera_fps_overrides")).toObject();
        const QString stationPrefix = QString::fromLatin1(kCameraFpsStationPrefix);
        for (auto it = cameraFpsOverrides.begin(); it != cameraFpsOverrides.end(); ++it) {
            const QString key = it.key().trimmed();
            if (key.isEmpty() || key.startsWith(stationPrefix)) {
                continue;
            }
            const int fps = clampCameraFpsForControlKey(key, it.value().toInt(defaultCameraFpsForControlKey(key)));
            cameraFpsOverrides_[key] = fps;
        }
    }
    const QString guiPreviewFpsKey = QString::fromLatin1(kCameraFpsGuiPreviewKey);
    if (airsimLiveFpsSpin_ && cameraFpsOverrides_.contains(guiPreviewFpsKey)) {
        airsimLiveFpsSpin_->setValue(static_cast<double>(cameraFpsOverrides_.value(guiPreviewFpsKey, kDefaultGuiPreviewFps)));
    }

    if (root.contains(QStringLiteral("rosbag_record")) && root.value(QStringLiteral("rosbag_record")).isObject()) {
        const QJsonObject obj = root.value(QStringLiteral("rosbag_record")).toObject();
        if (rosbagRecordPathEdit_ && obj.contains(QStringLiteral("output_path"))) {
            rosbagRecordPathEdit_->setText(obj.value(QStringLiteral("output_path")).toString());
        }
        if (rosbagRecordIncludeRfCheck_ && obj.contains(QStringLiteral("include_rf"))) rosbagRecordIncludeRfCheck_->setChecked(obj.value(QStringLiteral("include_rf")).toBool(true));
        if (rosbagRecordIncludeSysCheck_ && obj.contains(QStringLiteral("include_sys"))) rosbagRecordIncludeSysCheck_->setChecked(obj.value(QStringLiteral("include_sys")).toBool(true));
        if (rosbagRecordIncludeBeamCheck_ && obj.contains(QStringLiteral("include_beam"))) rosbagRecordIncludeBeamCheck_->setChecked(obj.value(QStringLiteral("include_beam")).toBool(true));
    }
    if (root.contains(QStringLiteral("rosbag_playback")) && root.value(QStringLiteral("rosbag_playback")).isObject()) {
        const QJsonObject obj = root.value(QStringLiteral("rosbag_playback")).toObject();
        if (rosbagPlaybackInputEdit_ && obj.contains(QStringLiteral("input_bag"))) rosbagPlaybackInputEdit_->setText(obj.value(QStringLiteral("input_bag")).toString());
        if (rosbagPlaybackRfTopicEdit_ && obj.contains(QStringLiteral("rf_topic"))) rosbagPlaybackRfTopicEdit_->setText(obj.value(QStringLiteral("rf_topic")).toString());
        if (rosbagPlaybackSysTopicEdit_ && obj.contains(QStringLiteral("sys_topic"))) rosbagPlaybackSysTopicEdit_->setText(obj.value(QStringLiteral("sys_topic")).toString());
        if (rosbagPlaybackBeamTopicEdit_) {
            if (obj.contains(QStringLiteral("beam_topic"))) {
                rosbagPlaybackBeamTopicEdit_->setText(obj.value(QStringLiteral("beam_topic")).toString());
            } else {
                const QString playbackSysTopic = rosbagPlaybackSysTopicEdit_ ? rosbagPlaybackSysTopicEdit_->text().trimmed() : internalSysTopic_;
                rosbagPlaybackBeamTopicEdit_->setText(defaultRosbagBeamTopicForSysTopic(playbackSysTopic, internalBeamTopic_));
            }
        }
        if (rosbagPlaybackBeamCodebookTopicEdit_) {
            if (obj.contains(QStringLiteral("beam_codebook_topic"))) {
                rosbagPlaybackBeamCodebookTopicEdit_->setText(obj.value(QStringLiteral("beam_codebook_topic")).toString());
            } else {
                const QString playbackBeamTopic = rosbagPlaybackBeamTopicEdit_ ? rosbagPlaybackBeamTopicEdit_->text().trimmed() : internalBeamTopic_;
                rosbagPlaybackBeamCodebookTopicEdit_->setText(defaultRosbagBeamCodebookTopicForBeamTopic(playbackBeamTopic, internalBeamCodebookTopic_));
            }
        }
        if (rosbagPlaybackIncludeRfCheck_ && obj.contains(QStringLiteral("play_rf"))) rosbagPlaybackIncludeRfCheck_->setChecked(obj.value(QStringLiteral("play_rf")).toBool(true));
        if (rosbagPlaybackIncludeSysCheck_ && obj.contains(QStringLiteral("play_sys"))) rosbagPlaybackIncludeSysCheck_->setChecked(obj.value(QStringLiteral("play_sys")).toBool(true));
        if (rosbagPlaybackIncludeBeamCheck_ && obj.contains(QStringLiteral("play_beam"))) rosbagPlaybackIncludeBeamCheck_->setChecked(obj.value(QStringLiteral("play_beam")).toBool(true));
        if (rosbagPlaybackClockCheck_ && obj.contains(QStringLiteral("publish_clock"))) rosbagPlaybackClockCheck_->setChecked(obj.value(QStringLiteral("publish_clock")).toBool(true));
        if (rosbagPlaybackRateSpin_ && obj.contains(QStringLiteral("play_rate"))) rosbagPlaybackRateSpin_->setValue(obj.value(QStringLiteral("play_rate")).toDouble(1.0));
    } else {
        if (rosbagPlaybackBeamTopicEdit_) {
            const QString playbackSysTopic = rosbagPlaybackSysTopicEdit_ ? rosbagPlaybackSysTopicEdit_->text().trimmed() : internalSysTopic_;
            rosbagPlaybackBeamTopicEdit_->setText(defaultRosbagBeamTopicForSysTopic(playbackSysTopic, internalBeamTopic_));
        }
        if (rosbagPlaybackBeamCodebookTopicEdit_) {
            const QString playbackBeamTopic = rosbagPlaybackBeamTopicEdit_ ? rosbagPlaybackBeamTopicEdit_->text().trimmed() : internalBeamTopic_;
            rosbagPlaybackBeamCodebookTopicEdit_->setText(defaultRosbagBeamCodebookTopicForBeamTopic(playbackBeamTopic, internalBeamCodebookTopic_));
        }
    }
    if (root.contains(QStringLiteral("rosbag_resim")) && root.value(QStringLiteral("rosbag_resim")).isObject()) {
        const QJsonObject obj = root.value(QStringLiteral("rosbag_resim")).toObject();
        if (rosbagResimInputEdit_ && obj.contains(QStringLiteral("input_bag"))) rosbagResimInputEdit_->setText(obj.value(QStringLiteral("input_bag")).toString());
        if (rosbagResimOutputEdit_ && obj.contains(QStringLiteral("output_bag"))) rosbagResimOutputEdit_->setText(obj.value(QStringLiteral("output_bag")).toString());
        if (rosbagResimPoseTopicEdit_ && obj.contains(QStringLiteral("pose_topic"))) rosbagResimPoseTopicEdit_->setText(obj.value(QStringLiteral("pose_topic")).toString());
        if (rosbagResimRfTopicEdit_ && obj.contains(QStringLiteral("rf_topic"))) rosbagResimRfTopicEdit_->setText(obj.value(QStringLiteral("rf_topic")).toString());
        if (rosbagResimSysTopicEdit_ && obj.contains(QStringLiteral("sys_topic"))) rosbagResimSysTopicEdit_->setText(obj.value(QStringLiteral("sys_topic")).toString());
        if (rosbagResimBeamTopicEdit_) {
            if (obj.contains(QStringLiteral("beam_topic"))) {
                rosbagResimBeamTopicEdit_->setText(obj.value(QStringLiteral("beam_topic")).toString());
            } else {
                const QString resimSysTopic = rosbagResimSysTopicEdit_ ? rosbagResimSysTopicEdit_->text().trimmed() : internalSysTopic_;
                rosbagResimBeamTopicEdit_->setText(defaultRosbagBeamTopicForSysTopic(resimSysTopic, internalBeamTopic_));
            }
        }
        if (rosbagResimBeamCodebookTopicEdit_) {
            if (obj.contains(QStringLiteral("beam_codebook_topic"))) {
                rosbagResimBeamCodebookTopicEdit_->setText(obj.value(QStringLiteral("beam_codebook_topic")).toString());
            } else {
                const QString resimBeamTopic = rosbagResimBeamTopicEdit_ ? rosbagResimBeamTopicEdit_->text().trimmed() : internalBeamTopic_;
                rosbagResimBeamCodebookTopicEdit_->setText(defaultRosbagBeamCodebookTopicForBeamTopic(resimBeamTopic, internalBeamCodebookTopic_));
            }
        }
        if (rosbagResimWriteRfCheck_ && obj.contains(QStringLiteral("write_rf"))) rosbagResimWriteRfCheck_->setChecked(obj.value(QStringLiteral("write_rf")).toBool(true));
        if (rosbagResimWriteSysCheck_ && obj.contains(QStringLiteral("write_sys"))) rosbagResimWriteSysCheck_->setChecked(obj.value(QStringLiteral("write_sys")).toBool(true));
        if (rosbagResimWriteBeamCheck_ && obj.contains(QStringLiteral("write_beam"))) rosbagResimWriteBeamCheck_->setChecked(obj.value(QStringLiteral("write_beam")).toBool(true));
        if (rosbagResimMessageFrequencySpin_) {
            if (obj.contains(QStringLiteral("message_frequency_hz"))) {
                rosbagResimMessageFrequencySpin_->setValue(obj.value(QStringLiteral("message_frequency_hz")).toDouble(10.0));
            } else if (obj.contains(QStringLiteral("interval_s"))) {
                const double oldIntervalS = obj.value(QStringLiteral("interval_s")).toDouble(0.1);
                rosbagResimMessageFrequencySpin_->setValue(oldIntervalS > 0.0 ? 1.0 / oldIntervalS : 10.0);
            }
        }
    } else {
        if (rosbagResimBeamTopicEdit_) {
            const QString resimSysTopic = rosbagResimSysTopicEdit_ ? rosbagResimSysTopicEdit_->text().trimmed() : internalSysTopic_;
            rosbagResimBeamTopicEdit_->setText(defaultRosbagBeamTopicForSysTopic(resimSysTopic, internalBeamTopic_));
        }
        if (rosbagResimBeamCodebookTopicEdit_) {
            const QString resimBeamTopic = rosbagResimBeamTopicEdit_ ? rosbagResimBeamTopicEdit_->text().trimmed() : internalBeamTopic_;
            rosbagResimBeamCodebookTopicEdit_->setText(defaultRosbagBeamCodebookTopicForBeamTopic(resimBeamTopic, internalBeamCodebookTopic_));
        }
    }

    const bool restartRosPipeline = (rosBridge_ && rosBridge_->isRunning()) || (simulatorProcess_ && simulatorProcess_->state() != QProcess::NotRunning);
    applyCoordinateTransformSettings(restartRosPipeline);
    updateSelectedTopicTypes();
    updateRosbagPlaybackTopicPresence();
    updateRosbagResimTopicPresence();
    refreshAirSimCameraList();
    syncStationCameraSelection();
    syncAirSimLiveViewSettings();
    syncSionnaPreviewSettings();
    if (importedSceneMeshPathProvided) {
        if (importedSceneMeshPath.isEmpty()) {
            clearSceneGeometry();
        } else {
            const QFileInfo importedSceneInfo(importedSceneMeshPath);
            const QString resolvedScenePath = importedSceneInfo.isAbsolute()
                ? importedSceneInfo.absoluteFilePath()
                : QFileInfo(QFileInfo(filePath).dir(), importedSceneMeshPath).absoluteFilePath();
            if (isSupportedSceneMeshFilePath(resolvedScenePath)) {
                const SceneGeometryBundle bundle = sceneManager_->loadFromFile(resolvedScenePath);
                if (bundle.valid) {
                    applySceneBundle(bundle, QStringLiteral("File mesh: %1").arg(resolvedScenePath));
                } else {
                    clearSceneGeometry();
                    QMessageBox::warning(this, tr("Open SimART Config"),
                                         tr("Loaded the SimART config, but failed to restore the saved 3D scene.\n\n%1")
                                             .arg(bundle.errorMessage.trimmed().isEmpty()
                                                      ? resolvedScenePath
                                                      : bundle.errorMessage));
                }
            } else {
                clearSceneGeometry();
                QMessageBox::warning(this, tr("Open SimART Config"),
                                     tr("Loaded the SimART config, but the saved 3D scene file is missing or unsupported.\n\n%1")
                                         .arg(resolvedScenePath));
            }
        }
    }
    refreshInfoPanel();
    updateRosbagUiState();
    clearBaseStationUndoHistory();
    setGuiConfigDirty(false);
    setBaseStationsDirty(false);
    onStatusMessage(tr("Loaded SimART config: %1").arg(QFileInfo(filePath).fileName()));
    return true;
}

void MainWindow::openGuiConfig() {
    const QString startPath = currentGuiConfigPath_.trimmed().isEmpty()
        ? QDir::home().filePath(QStringLiteral("airsim_gui_config") + QLatin1String(kGuiConfigExtension))
        : currentGuiConfigPath_;
    const QString path = QFileDialog::getOpenFileName(this, tr("Open SimART config"), startPath,
        tr("SimART Config (*%1)").arg(QLatin1String(kGuiConfigExtension)));
    if (path.isEmpty()) {
        return;
    }
    if (!promptSaveBaseStationsJsonIfDirty(
            tr("The current base-station JSON has unsaved changes. Save it before opening another SimART config?"))) {
        return;
    }

    QString errorMessage;
    if (!loadGuiConfigFromFile(path, &errorMessage)) {
        QMessageBox::warning(this, tr("Open SimART Config"), errorMessage);
        return;
    }
    currentGuiConfigPath_ = normalizedGuiConfigPath(path);
    guiConfigSessionActive_ = true;
    updateMainWindowTitle();
    setGuiConfigDirty(false);
    setBaseStationsDirty(false);
}

void MainWindow::saveGuiConfig() {
    if (currentGuiConfigPath_.trimmed().isEmpty()) {
        saveGuiConfigAs();
        return;
    }
    if (!promptSaveBaseStationsJsonIfDirty(
            tr("The base-station JSON has unsaved changes. Save it before saving the SimART config?"))) {
        return;
    }

    QString errorMessage;
    if (!saveGuiConfigToFile(currentGuiConfigPath_, &errorMessage)) {
        QMessageBox::warning(this, tr("Save SimART Config"), errorMessage);
        return;
    }
    setGuiConfigDirty(false);
    onStatusMessage(tr("Saved SimART config: %1").arg(QFileInfo(currentGuiConfigPath_).fileName()));
}

void MainWindow::saveGuiConfigAs() {
    const QString startPath = currentGuiConfigPath_.trimmed().isEmpty()
        ? QDir::home().filePath(QString::fromLatin1(kUntitledGuiConfigName) + QLatin1String(kGuiConfigExtension))
        : currentGuiConfigPath_;
    QString path = QFileDialog::getSaveFileName(this, tr("Save SimART config as"), startPath,
        tr("SimART Config (*%1)").arg(QLatin1String(kGuiConfigExtension)));
    if (path.isEmpty()) {
        return;
    }
    path = normalizedGuiConfigPath(path);
    if (!promptSaveBaseStationsJsonIfDirty(
            tr("The base-station JSON has unsaved changes. Save it before saving the SimART config?"))) {
        return;
    }

    QString errorMessage;
    if (!saveGuiConfigToFile(path, &errorMessage)) {
        QMessageBox::warning(this, tr("Save SimART Config"), errorMessage);
        return;
    }
    currentGuiConfigPath_ = path;
    guiConfigSessionActive_ = true;
    updateMainWindowTitle();
    setGuiConfigDirty(false);
    onStatusMessage(tr("Saved SimART config: %1").arg(QFileInfo(path).fileName()));
}

void MainWindow::promptStartupConfigChoice() {
    QMessageBox box(this);
    box.setWindowTitle(tr("SimART Config"));
    box.setIcon(QMessageBox::Question);
    box.setText(tr("Choose how to start."));
    box.setInformativeText(tr("New Config uses the default settings first. You will choose the .agcfg location when you save or close the GUI. Open Existing Config loads an existing configuration immediately."));
    QPushButton* newButton = box.addButton(tr("New Config"), QMessageBox::AcceptRole);
    QPushButton* openButton = box.addButton(tr("Open Existing Config"), QMessageBox::AcceptRole);
    box.setDefaultButton(newButton);
    box.exec();

    if (box.clickedButton() == openButton) {
        openGuiConfig();
        return;
    }

    currentGuiConfigPath_.clear();
    guiConfigSessionActive_ = true;
    updateMainWindowTitle();
    setGuiConfigDirty(false);
    setBaseStationsDirty(false);
    onStatusMessage(tr("Started a new unsaved SimART config."));
}

void MainWindow::loadInitialData() {
    QString error;
    baseStationsConfigPath_ = findConfigFile("config/base_stations.json");
    if (!JsonConfig::loadBaseStations(baseStationsConfigPath_, stations_, &error)) {
        QMessageBox::warning(this, tr("Config Error"), error);
    }
    const QString worldFile = findConfigFile("config/world_scene.json");
    if (!JsonConfig::loadWorldConfig(worldFile, world_, &error)) {
        QMessageBox::warning(this, tr("Config Error"), error);
    }

    const QStringList sceneCandidates = {
        QStringLiteral("SimART_sample_maps/BigCitySample/BigCitySample_simptest_sionna/BigCitySample_simptest.xml")
    };
    for (const QString& sceneCandidate : sceneCandidates) {
        simSettings_.scenePath = findConfigFile(sceneCandidate);
        if (QFileInfo::exists(simSettings_.scenePath)) {
            break;
        }
    }
    if (!QFileInfo::exists(simSettings_.scenePath)) {
        simSettings_.scenePath.clear();
    }
    if (ckmSettings_.outputDir.trimmed().isEmpty()) {
        ckmSettings_.outputDir = defaultDenseCkmOutputDir();
    }
    enableDenseCkmAutoBoundsIfSceneAvailable();

    loadCoordinateTransformSettings();
    applyCoordinateTransformSettings(false);

    sceneWidget_->setWorldConfig(world_);
    applyBaseStations(stations_);
}


void MainWindow::applyBaseStations(const std::vector<BaseStation>& stations) {
    stations_ = stations;
    if (selectedBaseStationIndex_ >= static_cast<int>(stations_.size())) {
        selectedBaseStationIndex_ = stations_.empty() ? -1 : static_cast<int>(stations_.size()) - 1;
    }
    sceneWidget_->setBaseStations(stations_);
    demoController_->setBaseStations(stations_);
    if (airSimViewController_) {
        airSimViewController_->setBaseStations(stations_);
        airSimViewController_->setSelectedBaseStationIndex(selectedBaseStationIndex_);
    }
    removeMissingStationCameraWindows();
    syncStationCameraWindows();
    syncStationCameraSelection();
    syncDeveloperTopicPublishingControls();
    refreshAirSimSettingsSyncStatus();
    refreshInfoPanel();
}

bool MainWindow::persistBaseStationsToFile(const QString& filePath) {
    QString error;
    if (!JsonConfig::saveBaseStations(filePath, stations_, &error)) {
        QMessageBox::warning(this, tr("Save Base Stations"), error);
        return false;
    }
    if (baseStationFileValue_) {
        baseStationFileValue_->setText(filePath);
    }
    statusBar()->showMessage(tr("Saved base stations to %1").arg(filePath), 4000);
    return true;
}

bool MainWindow::saveBaseStationsJsonForPrompt() {
    QString path = baseStationsConfigPath_.trimmed();
    if (path.isEmpty()) {
        path = QFileDialog::getSaveFileName(this,
                                            tr("Save Base Stations JSON"),
                                            findConfigFile("config/base_stations.json"),
                                            tr("JSON (*.json)"));
        if (path.isEmpty()) {
            return false;
        }
    }

    const QString previousPath = baseStationsConfigPath_;
    const QString absolutePath = QFileInfo(path).absoluteFilePath();
    baseStationsConfigPath_ = absolutePath;
    if (!persistBaseStationsToFile(baseStationsConfigPath_)) {
        baseStationsConfigPath_ = previousPath;
        refreshInfoPanel();
        return false;
    }

    setBaseStationsDirty(false);
    if (baseStationsConfigPath_ != previousPath) {
        markGuiConfigDirty();
    }
    refreshInfoPanel();
    return true;
}

bool MainWindow::promptSaveBaseStationsJsonIfDirty(const QString& message) {
    if (!baseStationsDirty_) {
        return true;
    }

    const QMessageBox::StandardButton choice = QMessageBox::question(
        this,
        tr("Save Base Stations JSON"),
        message,
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
        QMessageBox::Save);
    if (choice == QMessageBox::Cancel) {
        return false;
    }
    if (choice == QMessageBox::Save) {
        return saveBaseStationsJsonForPrompt();
    }
    return true;
}

void MainWindow::loadBaseStationsJson() {
    const QString path = QFileDialog::getOpenFileName(this, tr("Load Base Stations JSON"), QFileInfo(baseStationsConfigPath_).absolutePath(), tr("JSON (*.json)"));
    if (path.isEmpty()) {
        return;
    }
    if (!promptSaveBaseStationsJsonIfDirty(
            tr("The current base-station JSON has unsaved changes. Save it before loading another base-station JSON file?"))) {
        return;
    }
    std::vector<BaseStation> loaded;
    QString error;
    if (!JsonConfig::loadBaseStations(path, loaded, &error)) {
        QMessageBox::warning(this, tr("Load Base Stations"), error);
        return;
    }
    baseStationsConfigPath_ = QFileInfo(path).absoluteFilePath();
    applyBaseStations(loaded);
    clearBaseStationUndoHistory();
    setBaseStationsDirty(false);
    markGuiConfigDirty();
    onStatusMessage(tr("Loaded base stations from %1").arg(baseStationsConfigPath_));
}

void MainWindow::saveBaseStationsJson() {
    if (baseStationsConfigPath_.trimmed().isEmpty()) {
        saveBaseStationsJsonAs();
        return;
    }
    saveBaseStationsJsonForPrompt();
}

void MainWindow::saveBaseStationsJsonAs() {
    const QString path = QFileDialog::getSaveFileName(this, tr("Save Base Stations JSON"), baseStationsConfigPath_.isEmpty() ? findConfigFile("config/base_stations.json") : baseStationsConfigPath_, tr("JSON (*.json)"));
    if (path.isEmpty()) {
        return;
    }
    const QString previousPath = baseStationsConfigPath_;
    baseStationsConfigPath_ = QFileInfo(path).absoluteFilePath();
    if (persistBaseStationsToFile(baseStationsConfigPath_)) {
        setBaseStationsDirty(false);
        if (baseStationsConfigPath_ != previousPath) {
            markGuiConfigDirty();
        }
        refreshInfoPanel();
    } else {
        baseStationsConfigPath_ = previousPath;
        refreshInfoPanel();
    }
}

void MainWindow::enterAddBaseStationMode() {
    sceneWidget_->addBaseStation();
    if (baseStationModeValue_) {
        baseStationModeValue_->setText(tr("Add (next empty 3D click only)"));
    }
}

void MainWindow::onStationCoordinateEdited() {
    if (suppressStationCoordinateUpdates_) {
        return;
    }
    if (selectedBaseStationIndex_ < 0 || selectedBaseStationIndex_ >= static_cast<int>(stations_.size())) {
        return;
    }
    Vec3 position{stationXSpin_ ? stationXSpin_->value() : 0.0,
                  stationYSpin_ ? stationYSpin_->value() : 0.0,
                  stationZSpin_ ? stationZSpin_->value() : 27.0};
    if (sceneWidget_->updateSelectedBaseStationPosition(position)) {
        onStatusMessage(tr("Moved selected base station to %1").arg(vecToString(position)));
    }
}

void MainWindow::onStationDetailsEdited() {
    if (suppressStationCoordinateUpdates_) {
        return;
    }
    if (selectedBaseStationIndex_ < 0 || selectedBaseStationIndex_ >= static_cast<int>(stations_.size())) {
        return;
    }

    BaseStation current = stations_[static_cast<size_t>(selectedBaseStationIndex_)];
    const QString oldDefaultCameraName = defaultPreviewCameraNameForStation(current);
    const QString oldDefaultRosTopic = defaultPreviewRosTopicForStation(current);

    BaseStation updated = current;
    if (stationIdEdit_) {
        updated.id = stationIdEdit_->text().trimmed();
    }
    if (updated.id.isEmpty()) {
        updated.id = QStringLiteral("bs_%1").arg(selectedBaseStationIndex_ + 1);
    }
    if (stationNameEdit_) {
        updated.name = stationNameEdit_->text().trimmed();
    }
    if (updated.name.isEmpty()) {
        updated.name = updated.id;
    }
    if (stationColorRSpin_) {
        updated.color.x = clamp01(stationColorRSpin_->value());
    }
    if (stationColorGSpin_) {
        updated.color.y = clamp01(stationColorGSpin_->value());
    }
    if (stationColorBSpin_) {
        updated.color.z = clamp01(stationColorBSpin_->value());
    }
    if (stationPreviewOffsetZSpin_) {
        updated.previewOffsetZ = stationPreviewOffsetZSpin_->value();
    }
    const QString requestedCameraName = stationPreviewCameraEdit_ ? stationPreviewCameraEdit_->text().trimmed() : QString();
    updated.previewCameraName = (requestedCameraName.isEmpty() || requestedCameraName == oldDefaultCameraName)
        ? defaultPreviewCameraNameForStation(updated)
        : sanitizedCameraToken(requestedCameraName);
    const QString requestedRosTopic = stationPreviewRosTopicEdit_ ? stationPreviewRosTopicEdit_->text().trimmed() : QString();
    updated.previewRosTopic = (requestedRosTopic.isEmpty() || requestedRosTopic == oldDefaultRosTopic)
        ? defaultPreviewRosTopicForStation(updated)
        : requestedRosTopic;

    if (baseStationEquivalent(current, updated)) {
        return;
    }

    pushBaseStationUndoState(stations_);
    stations_[static_cast<size_t>(selectedBaseStationIndex_)] = updated;
    applyBaseStations(stations_);
    markBaseStationsDirty();
    onStatusMessage(tr("Updated base station %1 in memory. Use Save Base Stations JSON to write the JSON file.").arg(updated.name));
}

void MainWindow::deleteSelectedBaseStation() {
    if (sceneWidget_->deleteSelectedBaseStation()) {
        if (baseStationModeValue_) {
            baseStationModeValue_->setText(tr("Select"));
        }
    }
}

void MainWindow::onBaseStationEditStarted() {
    if (undoingBaseStationEdit_) {
        return;
    }
    baseStationEditTransactionBaseline_ = stations_;
    baseStationEditTransactionActive_ = true;
    baseStationUndoCapturedForTransaction_ = false;
}

void MainWindow::onBaseStationEditFinished() {
    baseStationEditTransactionActive_ = false;
    baseStationUndoCapturedForTransaction_ = false;
    baseStationEditTransactionBaseline_.clear();
}

void MainWindow::undoBaseStationEdit() {
    if (baseStationUndoStack_.empty()) {
        onStatusMessage(tr("Nothing to undo."));
        updateUndoActionState();
        return;
    }
    const std::vector<BaseStation> previous = baseStationUndoStack_.back();
    baseStationUndoStack_.pop_back();
    undoingBaseStationEdit_ = true;
    applyBaseStations(previous);
    undoingBaseStationEdit_ = false;
    markBaseStationsDirty();
    updateUndoActionState();
    onStatusMessage(tr("Undid base station edit."));
}

void MainWindow::onBaseStationsEdited(const std::vector<airsim_gui::BaseStation>& stations) {
    if (!undoingBaseStationEdit_) {
        if (baseStationEditTransactionActive_) {
            if (!baseStationUndoCapturedForTransaction_) {
                pushBaseStationUndoState(baseStationEditTransactionBaseline_);
                baseStationUndoCapturedForTransaction_ = true;
            }
        } else {
            pushBaseStationUndoState(stations_);
        }
    }
    stations_ = stations;
    demoController_->setBaseStations(stations_);
    if (airSimViewController_) {
        airSimViewController_->setBaseStations(stations_);
        airSimViewController_->setSelectedBaseStationIndex(selectedBaseStationIndex_);
    }
    removeMissingStationCameraWindows();
    syncStationCameraWindows();
    syncStationCameraSelection();
    markBaseStationsDirty();
    refreshAirSimSettingsSyncStatus();
    refreshInfoPanel();
}

void MainWindow::onStationSelectionChanged(int index, const airsim_gui::BaseStation& station) {
    selectedBaseStationIndex_ = index;
    suppressStationCoordinateUpdates_ = true;
    if (baseStationSelectionValue_) {
        if (index >= 0) {
            baseStationSelectionValue_->setText(QStringLiteral("%1 (%2)").arg(station.name, vecToString(station.position)));
            if (stationIdEdit_) {
                stationIdEdit_->setEnabled(true);
                stationIdEdit_->setText(station.id);
            }
            if (stationNameEdit_) {
                stationNameEdit_->setEnabled(true);
                stationNameEdit_->setText(station.name);
            }
            if (stationXSpin_) {
                stationXSpin_->setEnabled(true);
                stationXSpin_->setValue(station.position.x);
            }
            if (stationYSpin_) {
                stationYSpin_->setEnabled(true);
                stationYSpin_->setValue(station.position.y);
            }
            if (stationZSpin_) {
                stationZSpin_->setEnabled(true);
                stationZSpin_->setValue(station.position.z);
            }
            if (stationColorRSpin_) {
                stationColorRSpin_->setEnabled(true);
                stationColorRSpin_->setValue(station.color.x);
            }
            if (stationColorGSpin_) {
                stationColorGSpin_->setEnabled(true);
                stationColorGSpin_->setValue(station.color.y);
            }
            if (stationColorBSpin_) {
                stationColorBSpin_->setEnabled(true);
                stationColorBSpin_->setValue(station.color.z);
            }
            if (stationPreviewCameraEdit_) {
                stationPreviewCameraEdit_->setEnabled(true);
                stationPreviewCameraEdit_->setText(station.previewCameraName.trimmed().isEmpty() ? defaultPreviewCameraNameForStation(station) : station.previewCameraName.trimmed());
            }
            if (stationPreviewRosTopicEdit_) {
                stationPreviewRosTopicEdit_->setEnabled(true);
                stationPreviewRosTopicEdit_->setText(station.previewRosTopic.trimmed().isEmpty() ? defaultPreviewRosTopicForStation(station) : station.previewRosTopic.trimmed());
            }
            if (stationPreviewOffsetZSpin_) {
                stationPreviewOffsetZSpin_->setEnabled(true);
                stationPreviewOffsetZSpin_->setValue(station.previewOffsetZ);
            }
        } else {
            baseStationSelectionValue_->setText(tr("<none>"));
            if (stationIdEdit_) {
                stationIdEdit_->setEnabled(false);
                stationIdEdit_->clear();
            }
            if (stationNameEdit_) {
                stationNameEdit_->setEnabled(false);
                stationNameEdit_->clear();
            }
            if (stationXSpin_) {
                stationXSpin_->setEnabled(false);
                stationXSpin_->clear();
            }
            if (stationYSpin_) {
                stationYSpin_->setEnabled(false);
                stationYSpin_->clear();
            }
            if (stationZSpin_) {
                stationZSpin_->setEnabled(false);
                stationZSpin_->clear();
            }
            if (stationColorRSpin_) {
                stationColorRSpin_->setEnabled(false);
                stationColorRSpin_->setValue(0.0);
            }
            if (stationColorGSpin_) {
                stationColorGSpin_->setEnabled(false);
                stationColorGSpin_->setValue(0.0);
            }
            if (stationColorBSpin_) {
                stationColorBSpin_->setEnabled(false);
                stationColorBSpin_->setValue(0.0);
            }
            if (stationPreviewCameraEdit_) {
                stationPreviewCameraEdit_->setEnabled(false);
                stationPreviewCameraEdit_->clear();
            }
            if (stationPreviewRosTopicEdit_) {
                stationPreviewRosTopicEdit_->setEnabled(false);
                stationPreviewRosTopicEdit_->clear();
            }
            if (stationPreviewOffsetZSpin_) {
                stationPreviewOffsetZSpin_->setEnabled(false);
                stationPreviewOffsetZSpin_->setValue(0.0);
            }
        }
    }
    suppressStationCoordinateUpdates_ = false;
    if (airSimViewController_) {
        airSimViewController_->setSelectedBaseStationIndex(selectedBaseStationIndex_);
    }
    if (index >= 0 && (!airSimViewController_ || !airSimViewController_->isRunning() || !airSimViewController_->isConnected())) {
        statusBar()->showMessage(tr("Connect AirSim Live View before displaying base-station camera previews."), 5000);
    }
    syncStationCameraSelection();
    refreshInfoPanel();
}

QString MainWindow::findConfigFile(const QString& relativePath) const {
    return findPackagedConfigFile(relativePath);
}

QString MainWindow::findBundledScript(const QString& relativePath) const {
    return findConfigFile(relativePath);
}

QString MainWindow::effectiveDenseCkmBoundsScenePath() const {
    const QString primaryScenePath = simSettings_.scenePath.trimmed();
    if (!primaryScenePath.isEmpty() && QFileInfo::exists(primaryScenePath)) {
        return primaryScenePath;
    }
    const QString renderScenePath = ckmSettings_.renderScenePath.trimmed();
    if (!renderScenePath.isEmpty() && QFileInfo::exists(renderScenePath)) {
        return renderScenePath;
    }
    return QString();
}

bool MainWindow::readSceneXmlBoundsXY(const QString& sceneXmlPath,
                                      double* minX, double* maxX,
                                      double* minY, double* maxY,
                                      QString* errorMessage) const {
    if (minX) *minX = 0.0;
    if (maxX) *maxX = 0.0;
    if (minY) *minY = 0.0;
    if (maxY) *maxY = 0.0;

    const QString xmlPath = sceneXmlPath.trimmed();
    if (xmlPath.isEmpty()) {
        if (errorMessage) {
            *errorMessage = tr("Automatic CKM extents need a loaded 3D scene or a valid scene XML path.");
        }
        return false;
    }
    if (!QFileInfo::exists(xmlPath)) {
        if (errorMessage) {
            *errorMessage = tr("Scene XML does not exist: %1").arg(xmlPath);
        }
        return false;
    }

    const QString pythonPath = simSettings_.pythonExecutable.trimmed().isEmpty() ? QStringLiteral("python3") : simSettings_.pythonExecutable.trimmed();
    const QString helperScriptPath = findBundledScript(QStringLiteral("scripts/scene_bounds_from_xml.py"));
    if (!QFileInfo::exists(helperScriptPath)) {
        if (errorMessage) {
            *errorMessage = tr("Scene-bounds helper script not found: %1").arg(helperScriptPath);
        }
        return false;
    }

    QProcess proc;
    proc.setProgram(pythonPath);
    proc.setArguments({helperScriptPath,
                       QStringLiteral("--scene-path"), xmlPath,
                       QStringLiteral("--mi-variant"), simSettings_.miVariant});
    proc.setProcessChannelMode(QProcess::SeparateChannels);
    proc.start();
    if (!proc.waitForStarted(3000)) {
        if (errorMessage) {
            *errorMessage = tr("Failed to start Python while reading scene XML bounds: %1").arg(pythonPath);
        }
        return false;
    }
    proc.waitForFinished(20000);
    const QString stdoutText = QString::fromLocal8Bit(proc.readAllStandardOutput()).trimmed();
    const QString stderrText = QString::fromLocal8Bit(proc.readAllStandardError()).trimmed();
    if (proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0) {
        if (errorMessage) {
            const QString detail = !stderrText.isEmpty() ? stderrText : stdoutText;
            *errorMessage = detail.isEmpty()
                                ? tr("Failed to read XY bounds from scene XML: %1").arg(xmlPath)
                                : tr("Failed to read XY bounds from scene XML: %1\n\n%2").arg(xmlPath, detail);
        }
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(stdoutText.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage) {
            *errorMessage = tr("Scene XML bounds helper returned invalid JSON for %1.").arg(xmlPath);
            if (!stderrText.isEmpty()) {
                *errorMessage += QStringLiteral("\n\n") + stderrText;
            } else if (!stdoutText.isEmpty()) {
                *errorMessage += QStringLiteral("\n\n") + stdoutText;
            }
        }
        return false;
    }

    const QJsonObject obj = doc.object();
    const double x0 = obj.value(QStringLiteral("min_x")).toDouble(std::numeric_limits<double>::quiet_NaN());
    const double x1 = obj.value(QStringLiteral("max_x")).toDouble(std::numeric_limits<double>::quiet_NaN());
    const double y0 = obj.value(QStringLiteral("min_y")).toDouble(std::numeric_limits<double>::quiet_NaN());
    const double y1 = obj.value(QStringLiteral("max_y")).toDouble(std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(x0) || !std::isfinite(x1) || !std::isfinite(y0) || !std::isfinite(y1) || x1 <= x0 || y1 <= y0) {
        if (errorMessage) {
            *errorMessage = tr("Scene XML bounds are invalid for %1.").arg(xmlPath);
        }
        return false;
    }

    if (minX) *minX = x0;
    if (maxX) *maxX = x1;
    if (minY) *minY = y0;
    if (maxY) *maxY = y1;
    return true;
}

bool MainWindow::resolveDenseCkmAutoBoundsXY(double* minX, double* maxX,
                                             double* minY, double* maxY,
                                             QString* sourceDescription,
                                             QString* errorMessage) const {
    double sceneMinX = 0.0;
    double sceneMaxX = 0.0;
    double sceneMinY = 0.0;
    double sceneMaxY = 0.0;
    if (sceneWidget_ && sceneWidget_->sceneBoundsXY(&sceneMinX, &sceneMaxX, &sceneMinY, &sceneMaxY)) {
        if (minX) *minX = sceneMinX;
        if (maxX) *maxX = sceneMaxX;
        if (minY) *minY = sceneMinY;
        if (maxY) *maxY = sceneMaxY;
        if (sourceDescription) {
            *sourceDescription = QStringLiteral("loaded_scene_bounds");
        }
        return true;
    }

    const QString xmlPath = effectiveDenseCkmBoundsScenePath();
    QString xmlError;
    if (readSceneXmlBoundsXY(xmlPath, minX, maxX, minY, maxY, &xmlError)) {
        if (sourceDescription) {
            *sourceDescription = QStringLiteral("scene_xml_bounds");
        }
        return true;
    }

    if (errorMessage) {
        *errorMessage = xmlError.isEmpty()
                            ? tr("Automatic CKM extents are unavailable. Load a 3D scene, or provide a valid scene XML, or disable the automatic-range option and enter manual X/Y ranges.")
                            : xmlError;
    }
    return false;
}

void MainWindow::enableDenseCkmAutoBoundsIfSceneAvailable() {
    double xMin = 0.0;
    double xMax = 0.0;
    double yMin = 0.0;
    double yMax = 0.0;
    if (resolveDenseCkmAutoBoundsXY(&xMin, &xMax, &yMin, &yMax, nullptr, nullptr)) {
        ckmSettings_.useSceneBounds = true;
    }
}

void MainWindow::refreshRosTopics() {
    const QString currentPose = poseTopicCombo_ ? poseTopicCombo_->currentText().trimmed() : QString();
    const QString currentTrajectory = trajectoryTopicCombo_ ? trajectoryTopicCombo_->currentText().trimmed() : QString();

    const auto topics = RosBridge::listTopics();
    {
        QSignalBlocker blockPose(poseTopicCombo_);
        QSignalBlocker blockTrajectory(trajectoryTopicCombo_);
        poseTopicCombo_->clear();
        trajectoryTopicCombo_->clear();
        for (const auto& topic : topics) {
            if (RosBridge::isSupportedPoseType(topic.second)) {
                poseTopicCombo_->addItem(topic.first, topic.second);
            }
            if (RosBridge::isSupportedTrajectoryType(topic.second)) {
                trajectoryTopicCombo_->addItem(topic.first, topic.second);
            }
        }
        poseTopicCombo_->setEditText(currentPose.isEmpty() ? QString("/airsim_node/PX4/odom_local_ned") : currentPose);
        trajectoryTopicCombo_->setEditText(currentTrajectory);
    }

    updateSelectedTopicTypes();
    statusBar()->showMessage(tr("ROS topic list refreshed."), 3000);
}

void MainWindow::updateSelectedTopicTypes() {
    const QString poseType = RosBridge::topicType(poseTopicCombo_->currentText().trimmed());
    const QString trajectoryTopic = trajectoryTopicCombo_->currentText().trimmed();
    const QString trajectoryType = trajectoryTopic.isEmpty() ? QStringLiteral("<internal from pose>") : RosBridge::topicType(trajectoryTopic);
    poseTypeValue_->setText(poseType.isEmpty() ? QString("<unknown>") : poseType);
    trajectoryTypeValue_->setText(trajectoryType.isEmpty() ? QString("<unknown>") : trajectoryType);
    rfTopicValue_->setText(internalRfTopic_);
    if (sysTopicValue_) {
        sysTopicValue_->setText(internalSysTopic_);
    }
}

void MainWindow::rebuildStationCards() {
    if (!liveStateCardsLayout_ || !liveStateCardsContainer_) {
        return;
    }

    for (int i = liveStateCardsLayout_->count() - 1; i >= 0; --i) {
        auto* item = liveStateCardsLayout_->itemAt(i);
        if (item && item->spacerItem()) {
            delete liveStateCardsLayout_->takeAt(i);
        }
    }

    QMap<QString, QFrame*> existing;
    const auto cards = liveStateCardsContainer_->findChildren<QFrame*>(QString(), Qt::FindDirectChildrenOnly);
    for (QFrame* card : cards) {
        const QString key = card->property("stationKey").toString();
        if (!key.isEmpty()) {
            existing.insert(key, card);
        }
    }

    for (int i = 0; i < static_cast<int>(stations_.size()); ++i) {
        const auto& station = stations_[static_cast<size_t>(i)];
        const auto stats = stationStatsFor(station, currentPaths_);
        const QString key = !station.id.isEmpty() ? station.id : station.name;

        QString body = QStringLiteral("Name: %1\n"
                                      "ID: %2\n"
                                      "Position: %3\n"
                                      "Active paths: %4")
                           .arg(station.name,
                                station.id,
                                vecToString(station.position),
                                QString::number(stats.count));
        if (!stats.strongestId.isEmpty()) {
            body += QStringLiteral("\nStrongest: %1 (%2 dB)")
                        .arg(stats.strongestId)
                        .arg(stats.strongestPowerDb, 0, 'f', 1);
        }

        QString title = station.name;
        if (i == selectedBaseStationIndex_) {
            title += QStringLiteral("  [selected]");
        }

        QFrame* card = existing.take(key);
        if (!card) {
            card = makeInfoCard(title, body, liveStateCardsContainer_);
            card->setProperty("stationKey", key);
        }
        if (auto* titleLabel = card->findChild<QLabel*>(QStringLiteral("cardTitleLabel"))) {
            titleLabel->setText(title);
        }
        if (auto* bodyLabel = card->findChild<QLabel*>(QStringLiteral("cardBodyLabel"))) {
            bodyLabel->setText(body);
        }
        const bool selected = (i == selectedBaseStationIndex_);
        card->setStyleSheet(selected
            ? QStringLiteral("QFrame { background: #7c2d12; border: 2px solid #fb923c; border-radius: 10px; } QLabel { color: #fff7ed; } QLabel#cardBodyLabel { color: #ffedd5; }")
            : QStringLiteral("QFrame { background: #1f2430; border: 1px solid #394150; border-radius: 10px; } QLabel { color: #e8ecf1; } QLabel#cardBodyLabel { color: #c9d2dd; }"));
        card->show();
        liveStateCardsLayout_->removeWidget(card);
        liveStateCardsLayout_->insertWidget(i, card);
    }

    for (auto it = existing.begin(); it != existing.end(); ++it) {
        if (it.value()) {
            liveStateCardsLayout_->removeWidget(it.value());
            it.value()->deleteLater();
        }
    }

    liveStateCardsLayout_->addStretch(1);
}

void MainWindow::resetLiveSessionState(bool clearSysSnapshot) {
    currentDrone_ = DroneState{};
    currentPaths_.clear();
    currentRfSnapshot_ = RfObservationSnapshot{};
    if (frameTransformManager_) {
        frameTransformManager_->clearCurrentDronePoseInScene();
    }

    sceneWidget_->setDroneState(currentDrone_);
    sceneWidget_->setBeamPaths(currentPaths_);
    if (airSimViewController_) {
        airSimViewController_->setDroneState(currentDrone_);
        airSimViewController_->setBeamPaths(currentPaths_);
    }
    syncStationCameraWindows();
    rebuildRfDataCards();

    if (clearSysSnapshot) {
        sysModeSummary_ = simSettings_.enableSysIntegration ? QStringLiteral("Enabled") : QStringLiteral("Disabled");
        sysServingSummary_ = QStringLiteral("-");
        sysLinkSummary_ = QStringLiteral("-");
        currentSysSnapshot_ = SysSnapshotView{};
        currentSysSnapshot_.enabled = simSettings_.enableSysIntegration;
        rebuildSysDataWindow();
    }

    refreshInfoPanel();
}

void MainWindow::refreshInfoPanel() {
    if (rosBridge_->isRunning()) {
        if (simulatorProcess_ && simulatorProcess_->state() == QProcess::Running) {
            modeValue_->setText("ROS1 + internal RF sim");
        } else if (rosbagPlaybackActive_) {
            QStringList modeParts;
            modeParts << QStringLiteral("ROS1 rosbag playback");
            if (rosbagPlaybackUsingRfTopic_) {
                modeParts << QStringLiteral("RF");
            }
            if (rosbagPlaybackUsingSysTopic_) {
                modeParts << QStringLiteral("SYS");
            }
            if (rosbagPlaybackUsingBeamTopic_) {
                modeParts << QStringLiteral("Beam");
            }
            modeValue_->setText(modeParts.join(QStringLiteral(" + ")));
        } else {
            modeValue_->setText("ROS1 bridge");
        }
    } else if (demoController_->isRunning()) {
        modeValue_->setText("Demo");
    } else {
        modeValue_->setText("Idle");
    }

    sceneSourceValue_->setText(currentSceneSource_);
    simulatorStatusValue_->setText(simulatorProcess_ && simulatorProcess_->state() == QProcess::Running ? "Running" : "Stopped");
    if (rfTopicValue_) {
        rfTopicValue_->setText(internalRfTopic_);
    }
    if (sysTopicValue_) {
        sysTopicValue_->setText(internalSysTopic_);
    }

    if (currentDrone_.valid) {
        positionValue_->setText(vecToString(currentDrone_.position));
        attitudeValue_->setText(QString("R %1°, P %2°, Y %3°")
                                    .arg(currentDrone_.rollDeg, 0, 'f', 1)
                                    .arg(currentDrone_.pitchDeg, 0, 'f', 1)
                                    .arg(currentDrone_.yawDeg, 0, 'f', 1));
    } else {
        positionValue_->setText("-");
        attitudeValue_->setText("-");
    }

    baseStationCountValue_->setText(QString::number(static_cast<int>(stations_.size())));
    if (baseStationFileValue_) {
        QString label = baseStationsConfigPath_.isEmpty() ? QStringLiteral("<unsaved>") : baseStationsConfigPath_;
        if (baseStationsDirty_) {
            label += QStringLiteral(" (modified)");
        }
        baseStationFileValue_->setText(label);
    }
    if (baseStationModeValue_) {
        switch (sceneWidget_->stationInteractionMode()) {
        case Scene3DWidget::StationInteractionMode::Add:
            baseStationModeValue_->setText(QStringLiteral("Add (click 3D scene)"));
            break;
        case Scene3DWidget::StationInteractionMode::Delete:
            baseStationModeValue_->setText(QStringLiteral("Delete (click station)"));
            break;
        default:
            baseStationModeValue_->setText(QStringLiteral("Select"));
            break;
        }
    }
    pathCountValue_->setText(QString::number(static_cast<int>(currentPaths_.size())));

    if (!currentPaths_.empty()) {
        const auto strongest = std::max_element(
            currentPaths_.begin(), currentPaths_.end(),
            [](const BeamPath& a, const BeamPath& b) { return a.powerDb < b.powerDb; });
        strongestPathValue_->setText(QString("%1 (%2 dB)").arg(strongest->id).arg(strongest->powerDb, 0, 'f', 1));
    } else {
        strongestPathValue_->setText("-");
    }

    if (sysModeValue_) {
        sysModeValue_->setText(sysModeSummary_);
    }
    if (sysServingValue_) {
        sysServingValue_->setText(sysServingSummary_);
    }
    if (sysLinkValue_) {
        sysLinkValue_->setText(sysLinkSummary_);
    }

    rebuildStationCards();
    rebuildRfDataCards();
}

void MainWindow::applyLayerState(const QString& key, bool enabled) {
    if (isDroneCameraImageLayerKey(key)) {
        Q_UNUSED(enabled);
        refreshCameraImagePreviews();
        return;
    }

    if (key == "axes") {
        sceneWidget_->setAxesVisible(enabled);
    } else if (key == "ground") {
        sceneWidget_->setGroundVisible(enabled);
    } else if (key == "buildings") {
        sceneWidget_->setBuildingsVisible(enabled);
    } else if (key == "scene_mesh") {
        sceneWidget_->setSceneMeshVisible(enabled);
    } else if (key == "base_stations") {
        sceneWidget_->setBaseStationsVisible(enabled);
    } else if (key == "drone") {
        sceneWidget_->setDroneVisible(enabled);
    } else if (key == "history") {
        sceneWidget_->setHistoryVisible(enabled);
    } else if (key == "paths") {
        sceneWidget_->setPathsVisible(enabled);
    } else if (key == "view_gizmo") {
        sceneWidget_->setViewGizmoVisible(enabled);
    }

    if (isAirSimLayerKey(key)) {
        applyAirSimLayerState(airSimViewController_, key, enabled);
        for (auto& window : stationCameraWindows_) {
            if (window.view) {
                applyAirSimLayerState(window.view->controller(), key, enabled);
            }
        }
    }
}

void MainWindow::applyAirSimLayerState(AirSimViewController* controller, const QString& key, bool enabled) {
    if (!controller) {
        return;
    }
    if (isDroneCameraImageLayerKey(key)) {
        return;
    }
    if (key == "base_stations") {
        controller->setBaseStationsVisible(enabled);
    } else if (key == "drone") {
        controller->setDroneVisible(enabled);
    } else if (key == "history") {
        controller->setTrajectoryVisible(enabled);
    } else if (key == "paths") {
        controller->setPathsVisible(enabled);
    } else if (key == "view_gizmo") {
        controller->setViewGizmoVisible(enabled);
    }
}

void MainWindow::applyCurrentLayerStatesToAirSimController(AirSimViewController* controller) {
    if (!controller || !layerTree_) {
        return;
    }
    for (QTreeWidgetItemIterator it(layerTree_); *it; ++it) {
        QTreeWidgetItem* item = *it;
        const QString key = item->data(0, Qt::UserRole).toString().trimmed();
        if (isAirSimLayerKey(key)) {
            applyAirSimLayerState(controller, key, item->checkState(0) == Qt::Checked);
        }
    }
}

void MainWindow::refreshLayerPanelForCurrentView() {
    if (!layerTree_) {
        return;
    }

    const QWidget* current = centerViewTabs_ ? centerViewTabs_->currentWidget() : nullptr;
    const bool isLive = centerViewTabs_ && ueViewWidget_ && current == ueViewWidget_;
    for (QTreeWidgetItemIterator it(layerTree_); *it; ++it) {
        QTreeWidgetItem* item = *it;
        const QString key = item->data(0, Qt::UserRole).toString().trimmed();
        item->setHidden(isLive && !isAirSimLayerKey(key));
    }
    if (layersDock_) {
        layersDock_->setWindowTitle(isLive ? tr("AirSim Layers") : tr("Layers"));
    }
}

bool MainWindow::isDroneCameraImageLayerKey(const QString& key) const {
    return isDroneCameraImageLayerKeyStatic(key);
}

void MainWindow::syncDroneCameraImageLayerItems() {
    if (!layerTree_) {
        return;
    }

    QMap<QString, Qt::CheckState> oldStates;
    for (QTreeWidgetItemIterator it(layerTree_); *it; ++it) {
        QTreeWidgetItem* item = *it;
        const QString key = item->data(0, Qt::UserRole).toString().trimmed();
        if (isDroneCameraImageLayerKey(key)) {
            oldStates.insert(key, item->checkState(0));
        }
    }

    for (int i = layerTree_->topLevelItemCount() - 1; i >= 0; --i) {
        QTreeWidgetItem* item = layerTree_->topLevelItem(i);
        const QString key = item->data(0, Qt::UserRole).toString().trimmed();
        if (isDroneCameraImageLayerKey(key)) {
            delete layerTree_->takeTopLevelItem(i);
        }
    }

    QVector<CameraImageLayerItem> parsed;
    const QString preferredVehicle = airsimLiveVehicleEdit_ ? airsimLiveVehicleEdit_->text().trimmed() : QString();
    for (const QString& candidate : effectiveAirSimSettingsPaths()) {
        if (!QFileInfo::exists(candidate)) {
            continue;
        }
        QString parseError;
        parsed = parseDroneFrontCameraImageLayerItems(candidate, preferredVehicle, &parseError);
        if (!parsed.isEmpty()) {
            break;
        }
    }
    droneCameraImageLayers_ = parsed;

    const QSignalBlocker blocker(layerTree_);
    for (const CameraImageLayerItem& itemData : droneCameraImageLayers_) {
        auto* item = new QTreeWidgetItem(layerTree_);
        item->setText(0, itemData.label);
        item->setData(0, Qt::UserRole, itemData.key);
        item->setData(0, Qt::ToolTipRole, itemData.topic);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, oldStates.value(itemData.key, Qt::Unchecked));
    }

    refreshLayerPanelForCurrentView();
    refreshCameraImagePreviews();
}

void MainWindow::refreshCameraImagePreviews() {
    QVector<CameraImageLayerItem> selected;
    if (layerTree_) {
        QSet<QString> checkedKeys;
        for (QTreeWidgetItemIterator it(layerTree_); *it; ++it) {
            QTreeWidgetItem* item = *it;
            const QString key = item->data(0, Qt::UserRole).toString().trimmed();
            if (isDroneCameraImageLayerKey(key) && item->checkState(0) == Qt::Checked) {
                checkedKeys.insert(key);
            }
        }
        for (const CameraImageLayerItem& item : droneCameraImageLayers_) {
            if (checkedKeys.contains(item.key)) {
                selected.push_back(item);
            }
        }
    }

    const QWidget* current = centerViewTabs_ ? centerViewTabs_->currentWidget() : nullptr;
    const bool editorActive = current == sceneWidget_;
    const bool liveActive = current == ueViewWidget_;
    if (sceneCameraImageStrip_) {
        sceneCameraImageStrip_->setStreams(selected, editorActive);
    }
    if (liveCameraImageStrip_) {
        liveCameraImageStrip_->setStreams(selected, liveActive);
    }
}

QString MainWindow::exportBaseStationsForSimulator() const {
    const QString dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QDir().mkpath(dir);
    const QString path = QDir(dir).filePath("runtime_base_stations.json");

    QJsonArray items;
    int index = 0;
    for (const auto& station : stations_) {
        QJsonObject obj;
        obj["anchor_id"] = index++;
        obj["id"] = station.id;
        obj["name"] = station.name;
        obj["position"] = QJsonArray{station.position.x, station.position.y, station.position.z};
        obj["preview_camera_name"] = station.previewCameraName;
        obj["preview_offset_z"] = station.previewOffsetZ;
        if (station.previewCameraTargetEnabled) {
            obj["preview_camera_target"] = QJsonArray{
                station.previewCameraTarget.x,
                station.previewCameraTarget.y,
                station.previewCameraTarget.z,
            };
        }
        items.push_back(obj);
    }

    QFile file(path);
    if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        file.write(QJsonDocument(items).toJson(QJsonDocument::Indented));
        file.close();
    }
    return path;
}

bool MainWindow::startInternalSimulator() {
    stopInternalSimulator();
    simulatorLog_.clear();
    if (simulatorLogView_) {
        simulatorLogView_->clear();
    }
    sysModeSummary_ = simSettings_.enableSysIntegration
        ? QStringLiteral("Enabled (waiting for first SYS result)")
        : QStringLiteral("Disabled");
    sysServingSummary_ = QStringLiteral("-");
    sysLinkSummary_ = QStringLiteral("-");
    currentSysSnapshot_ = SysSnapshotView{};
    currentSysSnapshot_.enabled = simSettings_.enableSysIntegration;
    refreshInfoPanel();
    const QString scriptPath = findBundledScript("scripts/sionna_internal_socket_bridge.py");
    const QString workerScriptPath = findBundledScript("scripts/sionna_resim_socket_worker.py");
    if (!QFileInfo::exists(scriptPath)) {
        onStatusMessage(QString("Internal simulator bridge script not found: %1").arg(scriptPath));
        return false;
    }
    if (!QFileInfo::exists(workerScriptPath)) {
        onStatusMessage(QString("Internal simulator worker script not found: %1").arg(workerScriptPath));
        return false;
    }
    if (simSettings_.scenePath.trimmed().isEmpty()) {
        onStatusMessage("Simulation scene_path is empty. Open Simulation Settings first.");
        return false;
    }
    if (!simSettings_.beamCodebookFilePath.trimmed().isEmpty() && !QFileInfo::exists(simSettings_.beamCodebookFilePath.trimmed())) {
        onStatusMessage(QStringLiteral("Custom beam codebook file does not exist: %1").arg(simSettings_.beamCodebookFilePath.trimmed()));
        return false;
    }

    lastExportedBsJsonPath_ = exportBaseStationsForSimulator();
    const QString poseTopic = poseTopicCombo_->currentText().trimmed();
    const QString poseTopicType = RosBridge::topicType(poseTopic);
    QString fcText = QString::number(simSettings_.fcHz, 'g', 16);
    if (!fcText.contains(QChar('.')) && !fcText.contains(QChar('e')) && !fcText.contains(QChar('E'))) {
        fcText += QStringLiteral(".0");
    }
    const QString fcArg = QStringLiteral("_fc:=%1").arg(fcText);

    QStringList args;
    args << scriptPath
         << "__name:=airsim_gui_UErealtime_internal_sim"
         << QString("_scene_path:=%1").arg(simSettings_.scenePath)
         << QString("_bs_list_json:=%1").arg(lastExportedBsJsonPath_)
         << QString("_pose_topic:=%1").arg(poseTopic)
         << QString("_pose_topic_type:=%1").arg(poseTopicType)
         << QString("_ros_to_map_matrix_json:=%1").arg(coordinateTransforms_.rosToSceneMatrixPath)
         << QString("_airsim_to_scene_matrix_json:=%1").arg(coordinateTransforms_.airsimToSceneMatrixPath)
         << QString("_rx_array_facing_direction:=%1").arg(coordinateTransforms_.rxArrayFacingDirection)
         << QString("_uav_to_rx_array_tx:=%1").arg(QString::number(coordinateTransforms_.uavToRxArrayTx, 'g', 12))
         << QString("_uav_to_rx_array_ty:=%1").arg(QString::number(coordinateTransforms_.uavToRxArrayTy, 'g', 12))
         << QString("_uav_to_rx_array_tz:=%1").arg(QString::number(coordinateTransforms_.uavToRxArrayTz, 'g', 12))
         << QString("_uav_to_rx_array_center_matrix_json:=%1").arg(coordinateTransforms_.uavToRxArrayCenterMatrixPath)
         << QString("_rx_array_elements_json:=%1").arg(coordinateTransforms_.rxArrayElementsJsonPath)
         << QString("_output_frame_key:=%1").arg(coordinateTransforms_.outputFrameKey)
         << QString("_sim_hz:=%1").arg(QString::number(simSettings_.simHz, 'f', 3))
         << QString("_max_odom_staleness_s:=%1").arg(QString::number(simSettings_.maxPoseStalenessS, 'f', 3))
         << QString("_rf_observation_topic:=%1").arg(internalRfTopic_)
         << QString("_sys_observation_topic:=%1").arg(internalSysTopic_)
         << QString("_beam_observation_topic:=%1").arg(internalBeamTopic_)
         << QString("_beam_codebook_topic:=%1").arg(internalBeamCodebookTopic_)
         << QString("_rf_frame_id:=%1").arg(simSettings_.rfFrameId)
         << fcArg
         << QString("_mi_variant:=%1").arg(simSettings_.miVariant)
         << QString("_tx_array_num_rows:=%1").arg(simSettings_.txArrayNumRows)
         << QString("_tx_array_num_cols:=%1").arg(simSettings_.txArrayNumCols)
         << QString("_tx_array_vertical_spacing:=%1").arg(QString::number(simSettings_.txArrayVerticalSpacing, 'g', 8))
         << QString("_tx_array_horizontal_spacing:=%1").arg(QString::number(simSettings_.txArrayHorizontalSpacing, 'g', 8))
         << QString("_tx_array_pattern:=%1").arg(simSettings_.txArrayPattern)
         << QString("_tx_array_polarization:=%1").arg(simSettings_.txArrayPolarization)
         << QString("_rx_array_num_rows:=%1").arg(simSettings_.rxArrayNumRows)
         << QString("_rx_array_num_cols:=%1").arg(simSettings_.rxArrayNumCols)
         << QString("_rx_array_vertical_spacing:=%1").arg(QString::number(simSettings_.rxArrayVerticalSpacing, 'g', 8))
         << QString("_rx_array_horizontal_spacing:=%1").arg(QString::number(simSettings_.rxArrayHorizontalSpacing, 'g', 8))
         << QString("_rx_array_pattern:=%1").arg(simSettings_.rxArrayPattern)
         << QString("_rx_array_polarization:=%1").arg(simSettings_.rxArrayPolarization)
         << QString("_max_depth:=%1").arg(simSettings_.maxDepth)
         << QString("_samples_per_src:=%1").arg(simSettings_.samplesPerSrc)
         << QString("_max_num_paths_per_src:=%1").arg(simSettings_.maxNumPathsPerSrc)
         << QString("_synthetic_array:=%1").arg(simSettings_.syntheticArray ? "true" : "false")
         << QString("_merge_shapes:=%1").arg(simSettings_.mergeShapes ? "true" : "false")
         << QString("_enable_sys_integration:=%1").arg(simSettings_.enableSysIntegration ? "true" : "false")
         << QString("_sys_num_subcarriers:=%1").arg(simSettings_.sysNumSubcarriers)
         << QString("_sys_subcarrier_spacing_hz:=%1").arg(QString::number(simSettings_.sysSubcarrierSpacingHz, 'g', 12))
         << QString("_sys_num_ofdm_symbols:=%1").arg(simSettings_.sysNumOfdmSymbols)
         << QString("_sys_temperature_k:=%1").arg(QString::number(simSettings_.sysTemperatureK, 'g', 8))
         << QString("_sys_bler_target:=%1").arg(QString::number(simSettings_.sysBlerTarget, 'g', 8))
         << QString("_sys_mcs_table_index:=%1").arg(simSettings_.sysMcsTableIndex)
         << QString("_sys_bs_tx_power_dbm:=%1").arg(QString::number(simSettings_.sysBsTxPowerDbm, 'g', 8))
         << QString("_enable_beamforming:=%1").arg(simSettings_.enableBeamforming ? "true" : "false")
         << QString("_beam_selection_mode:=%1").arg(simSettings_.beamSelectionMode)
         << QString("_beam_codebook_type:=%1").arg(simSettings_.beamCodebookType)
         << QString("_beam_codebook_num_beams:=%1").arg(simSettings_.beamCodebookNumBeams)
         << QString("_beam_oversampling_v:=%1").arg(simSettings_.beamOversamplingV)
         << QString("_beam_oversampling_h:=%1").arg(simSettings_.beamOversamplingH)
         << QString("_beam_manual_index:=%1").arg(simSettings_.beamManualIndex)
         << QString("_beam_normalize_power:=%1").arg(simSettings_.beamNormalizePower ? "true" : "false")
         << QString("_beam_codebook_file:=%1").arg(simSettings_.beamCodebookFilePath)
         << QString("_beam_model_checkpoint_path:=%1").arg(simSettings_.beamModelCheckpointPath)
         << QString("_beam_feature_mode:=%1").arg(simSettings_.beamFeatureMode)
         << QString("_beam_top_k:=%1").arg(simSettings_.beamTopK)
         << QString("_beam_export_training_dataset:=%1").arg(simSettings_.beamExportTrainingDataset ? "true" : "false")
         << QString("_beam_training_dataset_path:=%1").arg(simSettings_.beamTrainingDatasetPath)
         << QString("_beam_training_reset_dataset_on_start:=%1").arg(simSettings_.beamTrainingResetDatasetOnStart ? "true" : "false")
         << QString("_beam_training_output_checkpoint_path:=%1").arg(simSettings_.beamTrainingOutputCheckpointPath)
         << QString("_beam_training_epochs:=%1").arg(simSettings_.beamTrainingEpochs)
         << QString("_beam_training_batch_size:=%1").arg(simSettings_.beamTrainingBatchSize)
         << QString("_beam_training_learning_rate:=%1").arg(QString::number(simSettings_.beamTrainingLearningRate, 'g', 12))
         << QString("_beam_training_validation_split:=%1").arg(QString::number(simSettings_.beamTrainingValidationSplit, 'g', 8))
         << QString("_beam_training_hidden_dim:=%1").arg(simSettings_.beamTrainingHiddenDim)
         << QString("_los:=%1").arg(simSettings_.los ? "true" : "false")
         << QString("_specular_reflection:=%1").arg(simSettings_.specularReflection ? "true" : "false")
         << QString("_diffuse_reflection:=%1").arg(simSettings_.diffuseReflection ? "true" : "false")
         << QString("_refraction:=%1").arg(simSettings_.refraction ? "true" : "false")
         << QString("_diffraction:=%1").arg(simSettings_.diffraction ? "true" : "false")
         << QString("_edge_diffraction:=%1").arg(simSettings_.edgeDiffraction ? "true" : "false")
         << QString("_diffraction_lit_region:=%1").arg(simSettings_.diffractionLitRegion ? "true" : "false")
         << QString("_measurement_csv:=%1").arg(simSettings_.csvEnabled ? simSettings_.csvPath : QString())
         << QString("_reset_measurement_csv:=%1").arg(simSettings_.csvEnabled ? "false" : "false")
         << "_enable_preview_render:=false";

    const QString sionnaPythonProgram = simSettings_.pythonExecutable.trimmed().isEmpty() ? QStringLiteral("python3") : simSettings_.pythonExecutable.trimmed();
    const QString rosPythonProgram = QFileInfo(QStringLiteral("/usr/bin/python3")).exists()
                                        ? QStringLiteral("/usr/bin/python3")
                                        : QStringLiteral("python3");
    const QString pythonProgram = rosPythonProgram;
    args << QString("_sionna_python:=%1").arg(sionnaPythonProgram)
         << QString("_worker_script:=%1").arg(workerScriptPath);
    appendSimulatorLog(QStringLiteral("[gui] Launching internal simulator (ROS bridge + Sionna worker)"));
    appendSimulatorLog(QStringLiteral("[gui] ROS bridge Python: %1").arg(pythonProgram));
    appendSimulatorLog(QStringLiteral("[gui] Sionna worker Python: %1").arg(sionnaPythonProgram));
    appendSimulatorLog(QStringLiteral("[gui] Bridge script: %1").arg(scriptPath));
    appendSimulatorLog(QStringLiteral("[gui] Worker script: %1").arg(workerScriptPath));
    appendSimulatorLog(QStringLiteral("[gui] Scene: %1").arg(simSettings_.scenePath));
    appendSimulatorLog(QStringLiteral("[gui] Pose topic: %1 [%2]").arg(poseTopic, poseTopicType));
    appendSimulatorLog(QStringLiteral("[gui] ROS->3D matrix: %1").arg(coordinateTransforms_.rosToSceneMatrixPath));
    appendSimulatorLog(QStringLiteral("[gui] AirSim->3D matrix: %1").arg(coordinateTransforms_.airsimToSceneMatrixPath));
    appendSimulatorLog(QStringLiteral("[gui] RX array facing: %1").arg(rxArrayFacingDirectionDisplayName(coordinateTransforms_.rxArrayFacingDirection)));
    appendSimulatorLog(QStringLiteral("[gui] UAV COM->RX array offset (UAV frame): [%1, %2, %3] m")
                           .arg(QString::number(coordinateTransforms_.uavToRxArrayTx, 'g', 8),
                                QString::number(coordinateTransforms_.uavToRxArrayTy, 'g', 8),
                                QString::number(coordinateTransforms_.uavToRxArrayTz, 'g', 8)));
    appendSimulatorLog(QStringLiteral("[gui] UAV COM->RX array center matrix: %1")
                           .arg(coordinateTransforms_.uavToRxArrayCenterMatrixPath.isEmpty()
                                    ? QStringLiteral("<generated from facing + tx/ty/tz>")
                                    : coordinateTransforms_.uavToRxArrayCenterMatrixPath));
    appendSimulatorLog(QStringLiteral("[gui] RX array elements JSON: %1")
                           .arg(coordinateTransforms_.rxArrayElementsJsonPath.isEmpty()
                                    ? QStringLiteral("<none>")
                                    : coordinateTransforms_.rxArrayElementsJsonPath));
    appendSimulatorLog(QStringLiteral("[gui] Wireless output frame: %1").arg(coordinateOutputFrameDisplayName(coordinateTransforms_.outputFrameKey)));
    const QString trajectoryTopicText = trajectoryTopicCombo_ ? trajectoryTopicCombo_->currentText().trimmed() : QString();
    const QString trajectoryTypeText = trajectoryTopicText.isEmpty() ? QStringLiteral("<internal from pose>") : RosBridge::topicType(trajectoryTopicText);
    appendSimulatorLog(QStringLiteral("[gui] Trajectory topic: %1 [%2]").arg(trajectoryTopicText.isEmpty() ? QStringLiteral("<internal>") : trajectoryTopicText, trajectoryTypeText));
    appendSimulatorLog(QStringLiteral("[gui] RF topic: %1").arg(internalRfTopic_));
    appendSimulatorLog(QStringLiteral("[gui] SYS topic: %1").arg(internalSysTopic_));
    appendSimulatorLog(QStringLiteral("[gui] Beam topic: %1").arg(internalBeamTopic_));
    appendSimulatorLog(QStringLiteral("[gui] Beam codebook topic: %1").arg(internalBeamCodebookTopic_));
    appendSimulatorLog(QStringLiteral("[gui] Base station json: %1").arg(lastExportedBsJsonPath_));
    appendSimulatorLog(QStringLiteral("[gui] Simulator argv:"));
    appendSimulatorLog(QStringLiteral("[gui]   %1 %2").arg(pythonProgram, args.join(QStringLiteral(" "))));
    simulatorProcess_->start(pythonProgram, args);
    if (!simulatorProcess_->waitForStarted(3000)) {
        onStatusMessage("Failed to start internal Sionna simulator process.");
        return false;
    }
    onStatusMessage(QString("Internal simulator started. RF topic: %1 | SYS topic: %2 | Beam topic: %3").arg(internalRfTopic_, internalSysTopic_, internalBeamTopic_));
    return true;
}

void MainWindow::stopInternalSimulator() {
    if (!simulatorProcess_) {
        return;
    }
    if (simulatorProcess_->state() != QProcess::NotRunning) {
        simulatorProcess_->terminate();
        if (!simulatorProcess_->waitForFinished(2000)) {
            simulatorProcess_->kill();
            simulatorProcess_->waitForFinished(1000);
        }
    }
    if (!simSettings_.enableSysIntegration) {
        sysModeSummary_ = QStringLiteral("Disabled");
    }
    sysServingSummary_ = QStringLiteral("-");
    sysLinkSummary_ = QStringLiteral("-");
    currentSysSnapshot_ = SysSnapshotView{};
    currentSysSnapshot_.enabled = simSettings_.enableSysIntegration;
    rebuildSysDataWindow();
}

void MainWindow::refreshDenseCkmControls() {
    if (ckmMapWidget_) {
        ckmMapWidget_->setGenerationRunning(ckmProcess_ && ckmProcess_->state() != QProcess::NotRunning);
    }
}

QString MainWindow::createDenseCkmOutputRootDir(const QString& baseOutputDir) const {
    const QString baseDir = baseOutputDir.trimmed().isEmpty() ? defaultDenseCkmOutputDir() : baseOutputDir.trimmed();
    QDir dir(baseDir);
    if (!dir.exists() && !dir.mkpath(QStringLiteral("."))) {
        return QString();
    }
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
    const QString baseName = QStringLiteral("%1_%2").arg(timestamp).arg(QCoreApplication::applicationPid());
    QString candidate = dir.filePath(baseName);
    int suffix = 0;
    while (QFileInfo::exists(candidate)) {
        ++suffix;
        candidate = dir.filePath(QStringLiteral("%1_%2").arg(baseName).arg(suffix));
    }
    if (!QDir().mkpath(candidate)) {
        return QString();
    }
    return QFileInfo(candidate).absoluteFilePath();
}

bool MainWindow::removeDenseCkmOutputRoot(const QString& outputRootDir, QString* errorMessage) const {
    if (errorMessage) {
        errorMessage->clear();
    }
    const QString path = outputRootDir.trimmed();
    if (path.isEmpty()) {
        return true;
    }
    const QFileInfo info(path);
    if (!info.exists()) {
        return true;
    }
    QDir dir(path);
    if (dir.removeRecursively()) {
        return true;
    }
    if (errorMessage) {
        *errorMessage = tr("Failed to remove unfinished Dense CKM output folder: %1").arg(path);
    }
    return false;
}

void MainWindow::generateDenseCkm() {
    if (startDenseCkmGeneration()) {
        onStatusMessage(tr("Dense CKM generation started."));
    }
}

void MainWindow::stopDenseCkmFromUi() {
    if (!ckmProcess_ || ckmProcess_->state() == QProcess::NotRunning) {
        onStatusMessage(tr("No Dense CKM generation is currently running."));
        refreshDenseCkmControls();
        return;
    }
    stopDenseCkmGeneration();
}

void MainWindow::loadDenseCkmFromDirectory() {
    QString startDir;
    if (!lastCkmMetaPath_.trimmed().isEmpty()) {
        startDir = QFileInfo(lastCkmMetaPath_).absolutePath();
    }
    if (startDir.trimmed().isEmpty()) {
        startDir = ckmSettings_.outputDir.trimmed().isEmpty() ? defaultDenseCkmOutputDir() : ckmSettings_.outputDir.trimmed();
    }
    const QString selectedDir = QFileDialog::getExistingDirectory(this, tr("Select Dense CKM version folder"), startDir);
    if (selectedDir.trimmed().isEmpty()) {
        return;
    }
    QString metaFilePath = QDir(selectedDir).filePath(QStringLiteral("meta.json"));
    if (!QFileInfo::exists(metaFilePath)) {
        metaFilePath = findLatestDenseCkmMetaFile(selectedDir);
    }
    if (!QFileInfo::exists(metaFilePath)) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("The selected folder does not contain a CKM meta.json file: %1").arg(selectedDir));
        return;
    }
    loadDenseCkmResult(metaFilePath);
}

bool MainWindow::startDenseCkmGeneration() {
    if (!ckmProcess_) {
        onStatusMessage(tr("Dense CKM process is not available."));
        return false;
    }
    stopDenseCkmGeneration();

    if (simSettings_.scenePath.trimmed().isEmpty()) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("Simulation scene XML is empty. Open Simulation Settings first."));
        return false;
    }
    if (stations_.empty()) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("No base stations are loaded. Load or create at least one base station first."));
        return false;
    }

    const QStringList metrics = normalizeDenseCkmMetrics(ckmSettings_.selectedMetrics, ckmSettings_.metric);
    QStringList sysMetricsRequested;
    QStringList beamMetricsRequested;
    QStringList predictedBeamMetricsRequested;
    for (const QString& metric : metrics) {
        if (denseCkmMetricNeedsSys(metric)) {
            sysMetricsRequested << metric;
        }
        if (denseCkmMetricNeedsBeam(metric)) {
            beamMetricsRequested << metric;
        }
        if (denseCkmMetricNeedsPredictedBeamCheckpoint(metric)) {
            predictedBeamMetricsRequested << metric;
        }
    }
    if (!sysMetricsRequested.isEmpty() && !simSettings_.enableSysIntegration) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("The selected metrics require SYS over RT, but SYS is currently disabled: %1").arg(sysMetricsRequested.join(QStringLiteral(", "))));
        return false;
    }
    if (!beamMetricsRequested.isEmpty() && !simSettings_.enableBeamforming) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("The selected metrics require beamforming to be enabled in Sionna Settings: %1").arg(denseCkmMetricUiListText(beamMetricsRequested)));
        return false;
    }
    if (!predictedBeamMetricsRequested.isEmpty() && simSettings_.beamModelCheckpointPath.trimmed().isEmpty()) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("The selected metrics require a loaded beam-selection .pt checkpoint: %1").arg(denseCkmMetricUiListText(predictedBeamMetricsRequested)));
        return false;
    }
    if (!predictedBeamMetricsRequested.isEmpty() && !QFileInfo::exists(simSettings_.beamModelCheckpointPath.trimmed())) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("The selected predicted-beam CKM metrics require an existing .pt checkpoint file: %1").arg(simSettings_.beamModelCheckpointPath.trimmed()));
        return false;
    }
    if (ckmSettings_.resolutionM <= 0.0) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("Grid resolution must be greater than zero."));
        return false;
    }
    double xMin = ckmSettings_.xMin;
    double xMax = ckmSettings_.xMax;
    double yMin = ckmSettings_.yMin;
    double yMax = ckmSettings_.yMax;
    QString autoBoundsSource;
    if (ckmSettings_.useSceneBounds) {
        QString boundsError;
        if (!resolveDenseCkmAutoBoundsXY(&xMin, &xMax, &yMin, &yMax, &autoBoundsSource, &boundsError)) {
            QMessageBox::warning(this, tr("Dense CKM"),
                                 boundsError.isEmpty()
                                     ? tr("Automatic CKM extents are unavailable. Load a 3D scene, or provide a valid scene XML, or disable the automatic-range option and enter manual X/Y ranges.")
                                     : boundsError);
            return false;
        }
    }
    if (xMax <= xMin || yMax <= yMin) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("Dense CKM ranges are invalid. Make sure max is greater than min on both axes."));
        return false;
    }

    const QString renderScenePath = ckmSettings_.renderScenePath.trimmed().isEmpty() ? simSettings_.scenePath : ckmSettings_.renderScenePath.trimmed();
    if (!QFileInfo::exists(renderScenePath)) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("The high-detail CKM render XML does not exist: %1").arg(renderScenePath));
        return false;
    }
    if (!simSettings_.beamCodebookFilePath.trimmed().isEmpty() && !QFileInfo::exists(simSettings_.beamCodebookFilePath.trimmed())) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("The selected custom beam codebook file does not exist: %1").arg(simSettings_.beamCodebookFilePath.trimmed()));
        return false;
    }

    const QString scriptPath = findBundledScript(QStringLiteral("scripts/sionna_ckm_generator.py"));
    if (!QFileInfo::exists(scriptPath)) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("Dense CKM script not found: %1").arg(scriptPath));
        return false;
    }

    lastExportedBsJsonPath_ = exportBaseStationsForSimulator();
    QString outputDir = ckmSettings_.outputDir.trimmed();
    if (outputDir.isEmpty()) {
        outputDir = defaultDenseCkmOutputDir();
        ckmSettings_.outputDir = outputDir;
    }
    QDir().mkpath(outputDir);
    lastCkmMetaPath_.clear();
    activeCkmOutputRootDir_ = createDenseCkmOutputRootDir(outputDir);
    if (activeCkmOutputRootDir_.trimmed().isEmpty()) {
        QMessageBox::warning(this, tr("Dense CKM"), tr("Failed to create a new version folder under: %1").arg(outputDir));
        refreshDenseCkmControls();
        return false;
    }
    ckmStopRequested_ = false;

    QStringList args;
    args << scriptPath
         << QStringLiteral("--scene-path") << simSettings_.scenePath
         << QStringLiteral("--render-scene-path") << renderScenePath
         << QStringLiteral("--bs-list-json") << lastExportedBsJsonPath_;
    for (const QString& metric : metrics) {
        args << QStringLiteral("--metric") << metric;
    }
    args << QStringLiteral("--default-metric") << metrics.value(0, QStringLiteral("power_db"))
         << QStringLiteral("--x-min") << QString::number(xMin, 'g', 12)
         << QStringLiteral("--x-max") << QString::number(xMax, 'g', 12)
         << QStringLiteral("--y-min") << QString::number(yMin, 'g', 12)
         << QStringLiteral("--y-max") << QString::number(yMax, 'g', 12)
         << QStringLiteral("--z-fixed") << QString::number(ckmSettings_.zFixed, 'g', 12)
         << QStringLiteral("--resolution-m") << QString::number(ckmSettings_.resolutionM, 'g', 12)
         << QStringLiteral("--output-dir") << outputDir
         << QStringLiteral("--output-root-dir") << activeCkmOutputRootDir_
         << QStringLiteral("--range-source") << (ckmSettings_.useSceneBounds
                                                       ? (autoBoundsSource == QStringLiteral("scene_xml_bounds")
                                                              ? QStringLiteral("scene_xml_bounds")
                                                              : QStringLiteral("loaded_scene_bounds"))
                                                       : QStringLiteral("manual_range"))
         << QStringLiteral("--render-scene-overlay") << (ckmSettings_.renderSceneOverlay ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--fc-hz") << QString::number(simSettings_.fcHz, 'g', 16)
         << QStringLiteral("--mi-variant") << simSettings_.miVariant
         << QStringLiteral("--tx-array-num-rows") << QString::number(simSettings_.txArrayNumRows)
         << QStringLiteral("--tx-array-num-cols") << QString::number(simSettings_.txArrayNumCols)
         << QStringLiteral("--tx-array-vertical-spacing") << QString::number(simSettings_.txArrayVerticalSpacing, 'g', 12)
         << QStringLiteral("--tx-array-horizontal-spacing") << QString::number(simSettings_.txArrayHorizontalSpacing, 'g', 12)
         << QStringLiteral("--tx-array-pattern") << simSettings_.txArrayPattern
         << QStringLiteral("--tx-array-polarization") << simSettings_.txArrayPolarization
         << QStringLiteral("--rx-array-num-rows") << QString::number(simSettings_.rxArrayNumRows)
         << QStringLiteral("--rx-array-num-cols") << QString::number(simSettings_.rxArrayNumCols)
         << QStringLiteral("--rx-array-vertical-spacing") << QString::number(simSettings_.rxArrayVerticalSpacing, 'g', 12)
         << QStringLiteral("--rx-array-horizontal-spacing") << QString::number(simSettings_.rxArrayHorizontalSpacing, 'g', 12)
         << QStringLiteral("--rx-array-pattern") << simSettings_.rxArrayPattern
         << QStringLiteral("--rx-array-polarization") << simSettings_.rxArrayPolarization
         << QStringLiteral("--max-depth") << QString::number(simSettings_.maxDepth)
         << QStringLiteral("--samples-per-src") << QString::number(simSettings_.samplesPerSrc)
         << QStringLiteral("--max-num-paths-per-src") << QString::number(simSettings_.maxNumPathsPerSrc)
         << QStringLiteral("--synthetic-array") << (simSettings_.syntheticArray ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--merge-shapes") << (simSettings_.mergeShapes ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--enable-sys-integration") << (simSettings_.enableSysIntegration ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--enable-beamforming") << (simSettings_.enableBeamforming ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--beam-selection-mode") << (predictedBeamMetricsRequested.isEmpty() ? simSettings_.beamSelectionMode : QStringLiteral("deepsense_predictor"))
         << QStringLiteral("--beam-codebook-type") << simSettings_.beamCodebookType
         << QStringLiteral("--beam-codebook-num-beams") << QString::number(simSettings_.beamCodebookNumBeams)
         << QStringLiteral("--beam-oversampling-v") << QString::number(simSettings_.beamOversamplingV)
         << QStringLiteral("--beam-oversampling-h") << QString::number(simSettings_.beamOversamplingH)
         << QStringLiteral("--beam-manual-index") << QString::number(simSettings_.beamManualIndex)
         << QStringLiteral("--beam-normalize-power") << (simSettings_.beamNormalizePower ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--beam-codebook-file") << simSettings_.beamCodebookFilePath
         << QStringLiteral("--beam-model-checkpoint-path") << simSettings_.beamModelCheckpointPath
         << QStringLiteral("--beam-feature-mode") << simSettings_.beamFeatureMode
         << QStringLiteral("--beam-top-k") << QString::number(simSettings_.beamTopK)
         << QStringLiteral("--sys-num-subcarriers") << QString::number(simSettings_.sysNumSubcarriers)
         << QStringLiteral("--sys-subcarrier-spacing-hz") << QString::number(simSettings_.sysSubcarrierSpacingHz, 'g', 12)
         << QStringLiteral("--sys-num-ofdm-symbols") << QString::number(simSettings_.sysNumOfdmSymbols)
         << QStringLiteral("--sys-temperature-k") << QString::number(simSettings_.sysTemperatureK, 'g', 12)
         << QStringLiteral("--sys-bler-target") << QString::number(simSettings_.sysBlerTarget, 'g', 12)
         << QStringLiteral("--sys-mcs-table-index") << QString::number(simSettings_.sysMcsTableIndex)
         << QStringLiteral("--sys-bs-tx-power-dbm") << QString::number(simSettings_.sysBsTxPowerDbm, 'g', 12)
         << QStringLiteral("--los") << (simSettings_.los ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--specular-reflection") << (simSettings_.specularReflection ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--diffuse-reflection") << (simSettings_.diffuseReflection ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--refraction") << (simSettings_.refraction ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--diffraction") << (simSettings_.diffraction ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--edge-diffraction") << (simSettings_.edgeDiffraction ? QStringLiteral("true") : QStringLiteral("false"))
         << QStringLiteral("--diffraction-lit-region") << (simSettings_.diffractionLitRegion ? QStringLiteral("true") : QStringLiteral("false"));

    const QString pythonProgram = simSettings_.pythonExecutable.trimmed().isEmpty() ? QStringLiteral("python3") : simSettings_.pythonExecutable.trimmed();
    appendSimulatorLog(QStringLiteral("[ckm] Launching dense CKM generator"));
    appendSimulatorLog(QStringLiteral("[ckm] Python: %1").arg(pythonProgram));
    appendSimulatorLog(QStringLiteral("[ckm] Script: %1").arg(scriptPath));
    appendSimulatorLog(QStringLiteral("[ckm] Metrics: %1").arg(metrics.join(QStringLiteral(", "))));
    if (!beamMetricsRequested.isEmpty()) {
        appendSimulatorLog(QStringLiteral("[ckm] Beamforming metrics enabled for CKM"));
    }
    if (!predictedBeamMetricsRequested.isEmpty()) {
        appendSimulatorLog(QStringLiteral("[ckm] Predicted-beam CKM metrics requested: forcing offline beam selection mode to learned_selector for this CKM run"));
    }
    appendSimulatorLog(QStringLiteral("[ckm] Render scene XML: %1").arg(renderScenePath));
    appendSimulatorLog(QStringLiteral("[ckm] Extent: x [%1, %2], y [%3, %4]")
                           .arg(xMin, 0, 'f', 2)
                           .arg(xMax, 0, 'f', 2)
                           .arg(yMin, 0, 'f', 2)
                           .arg(yMax, 0, 'f', 2));
    appendSimulatorLog(QStringLiteral("[ckm] Output dir: %1").arg(outputDir));
    appendSimulatorLog(QStringLiteral("[ckm] Output root: %1").arg(activeCkmOutputRootDir_));
    appendSimulatorLog(QStringLiteral("[ckm] Argv: %1 %2").arg(pythonProgram, args.join(QStringLiteral(" "))));

    refreshDenseCkmControls();
    ckmProcess_->start(pythonProgram, args);
    if (!ckmProcess_->waitForStarted(3000)) {
        QString cleanupError;
        removeDenseCkmOutputRoot(activeCkmOutputRootDir_, &cleanupError);
        activeCkmOutputRootDir_.clear();
        ckmStopRequested_ = false;
        refreshDenseCkmControls();
        onStatusMessage(tr("Failed to start Dense CKM generator."));
        if (!cleanupError.trimmed().isEmpty()) {
            appendSimulatorLog(QStringLiteral("[ckm] %1").arg(cleanupError));
        }
        return false;
    }
    refreshDenseCkmControls();
    return true;
}

void MainWindow::stopDenseCkmGeneration() {
    if (!ckmProcess_) {
        refreshDenseCkmControls();
        return;
    }
    if (ckmProcess_->state() == QProcess::NotRunning) {
        refreshDenseCkmControls();
        return;
    }
    ckmStopRequested_ = true;
    appendSimulatorLog(QStringLiteral("[ckm] Stop requested by user."));
    onStatusMessage(tr("Stopping Dense CKM generation..."));
    stopProcessGracefully(ckmProcess_, 2000);
    if (ckmProcess_->state() == QProcess::NotRunning && !activeCkmOutputRootDir_.trimmed().isEmpty()) {
        QString cleanupError;
        if (removeDenseCkmOutputRoot(activeCkmOutputRootDir_, &cleanupError)) {
            appendSimulatorLog(QStringLiteral("[ckm] Deleted unfinished output folder: %1").arg(activeCkmOutputRootDir_));
        } else if (!cleanupError.trimmed().isEmpty()) {
            appendSimulatorLog(QStringLiteral("[ckm] %1").arg(cleanupError));
        }
        activeCkmOutputRootDir_.clear();
    }
    refreshDenseCkmControls();
}

void MainWindow::loadDenseCkmResult(const QString& metaFilePath) {
    if (!ckmMapWidget_) {
        return;
    }
    QString errorMessage;
    if (!ckmMapWidget_->loadFromMetaFile(metaFilePath, &errorMessage)) {
        QMessageBox::warning(this, tr("Dense CKM"), errorMessage);
        return;
    }
    lastCkmMetaPath_ = QFileInfo(metaFilePath).absoluteFilePath();
    switchToCkmMap();
    onStatusMessage(tr("Loaded Dense CKM result: %1").arg(QFileInfo(metaFilePath).absolutePath()));
}

void MainWindow::refreshRightPanelMode() {
    const QWidget* current = centerViewTabs_ ? centerViewTabs_->currentWidget() : nullptr;
    const bool isCkm = (current == ckmMapWidget_);
    if (liveStateBox_) {
        liveStateBox_->setVisible(!isCkm);
    }
    if (ckmDataBox_) {
        ckmDataBox_->setVisible(isCkm);
    }
}

void MainWindow::onCkmPointSelectionChanged(const QString& title, const QString& summary, const QString& details) {
    if (ckmDataTitleValue_) {
        ckmDataTitleValue_->setText(title.trimmed().isEmpty() ? tr("Dense CKM point") : title);
    }
    if (ckmDataSummaryValue_) {
        ckmDataSummaryValue_->setText(summary);
    }
    if (ckmDataDetailsView_) {
        ckmDataDetailsView_->setPlainText(details);
    }
}

void MainWindow::onCkmPointSelectionCleared(const QString& hint) {
    if (ckmDataTitleValue_) {
        ckmDataTitleValue_->setText(tr("No CKM point selected"));
    }
    if (ckmDataSummaryValue_) {
        ckmDataSummaryValue_->setText(hint.trimmed().isEmpty()
            ? tr("Load a Dense CKM result, switch to the CKM Map page, then click the scene overlay to inspect the nearest offline sample.")
            : hint);
    }
    if (ckmDataDetailsView_) {
        ckmDataDetailsView_->clear();
    }
}

void MainWindow::onCkmProcessOutput() {
    if (!ckmProcess_) {
        return;
    }
    const QString text = QString::fromLocal8Bit(ckmProcess_->readAllStandardOutput());
    const QStringList lines = text.split(QRegularExpression(QStringLiteral("[\r\n]+")), QString::SkipEmptyParts);
    for (const QString& rawLine : lines) {
        const QString line = rawLine.trimmed();
        if (line.isEmpty()) {
            continue;
        }
        appendSimulatorLog(QStringLiteral("[ckm] %1").arg(line));
        if (line.startsWith(QStringLiteral("META_JSON="))) {
            lastCkmMetaPath_ = line.mid(QStringLiteral("META_JSON=").size()).trimmed();
        } else if (line.startsWith(QStringLiteral("OUTPUT_ROOT="))) {
            activeCkmOutputRootDir_ = line.mid(QStringLiteral("OUTPUT_ROOT=").size()).trimmed();
        }
    }
}

void MainWindow::onCkmProcessError(QProcess::ProcessError error) {
    Q_UNUSED(error);
    if (!ckmProcess_) {
        return;
    }
    appendSimulatorLog(QStringLiteral("[ckm] Dense CKM process error: %1").arg(ckmProcess_->errorString()));
}

void MainWindow::onCkmProcessFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    appendSimulatorLog(QStringLiteral("[ckm] Dense CKM finished (exitCode=%1, status=%2)")
                           .arg(exitCode)
                           .arg(exitStatus == QProcess::NormalExit ? QStringLiteral("normal") : QStringLiteral("crash")));
    const bool stoppedByUser = ckmStopRequested_;
    const QString outputRootDir = activeCkmOutputRootDir_.trimmed();
    ckmStopRequested_ = false;
    activeCkmOutputRootDir_.clear();
    refreshDenseCkmControls();

    if (stoppedByUser) {
        QString cleanupError;
        if (removeDenseCkmOutputRoot(outputRootDir, &cleanupError)) {
            if (!outputRootDir.isEmpty()) {
                appendSimulatorLog(QStringLiteral("[ckm] Deleted unfinished output folder: %1").arg(outputRootDir));
            }
        } else if (!cleanupError.trimmed().isEmpty()) {
            appendSimulatorLog(QStringLiteral("[ckm] %1").arg(cleanupError));
        }
        onStatusMessage(tr("Dense CKM generation stopped and unfinished files were removed."));
        return;
    }

    if (exitStatus == QProcess::NormalExit && exitCode == 0) {
        if (lastCkmMetaPath_.trimmed().isEmpty()) {
            lastCkmMetaPath_ = findLatestDenseCkmMetaFile(ckmSettings_.outputDir.trimmed().isEmpty() ? defaultDenseCkmOutputDir() : ckmSettings_.outputDir);
        }
        if (ckmSettings_.autoLoadResult && QFileInfo::exists(lastCkmMetaPath_)) {
            loadDenseCkmResult(lastCkmMetaPath_);
        }
        onStatusMessage(tr("Dense CKM generation finished."));
        return;
    }
    QMessageBox::warning(this, tr("Dense CKM"), tr("Dense CKM generation failed. Check the simulator log for details."));
}

void MainWindow::appendSimulatorLog(const QString& text) {
    const QString line = sanitizeBeamTerminologyForUi(text.trimmed());
    if (line.isEmpty()) {
        return;
    }
    appendRuntimeLogLine(QStringLiteral("sim"), line);
    simulatorLog_ += line + QStringLiteral("\n");
    if (simulatorLogView_) {
        simulatorLogView_->appendPlainText(line);
    }
}

QString MainWindow::ensureRuntimeLogFile() {
    if (!runtimeLogFilePath_.trimmed().isEmpty()) {
        return runtimeLogFilePath_;
    }
    QString root = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
    if (root.trimmed().isEmpty()) {
        root = QDir::home().filePath(QStringLiteral(".local/share/airsim_gui_UErealtime"));
    }
    const QString logDir = QDir(root).filePath(QStringLiteral("logs"));
    QDir().mkpath(logDir);
    runtimeLogFilePath_ = QDir(logDir).filePath(
        QStringLiteral("gui_runtime_%1.log").arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"))));
    return runtimeLogFilePath_;
}

void MainWindow::appendRuntimeLogLine(const QString& channel, const QString& text) {
    const QString line = text.trimmed();
    if (line.isEmpty()) {
        return;
    }
    const QString path = ensureRuntimeLogFile();
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        return;
    }
    QTextStream stream(&file);
    stream << QDateTime::currentDateTime().toString(Qt::ISODateWithMs)
           << " [" << channel.trimmed() << "] "
           << line << '\n';
}

void MainWindow::showShortDiagnosticWarning(const QString& title, const QString& summary, const QString& detail) {
    const QString trimmedDetail = detail.trimmed();
    if (!trimmedDetail.isEmpty()) {
        appendRuntimeLogLine(QStringLiteral("diagnostic"), trimmedDetail);
    }
    const QString logPath = ensureRuntimeLogFile();
    QMessageBox::warning(this,
                         title,
                         tr("%1\n\nDetails were written to the runtime log:\n%2").arg(summary, logPath));
}

bool MainWindow::waitForInternalRfTopic(QString* errorMessage, int timeoutMs) {
    QElapsedTimer timer;
    timer.start();
    QString lastType;
    while (timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents();
        if (simulatorProcess_ && simulatorProcess_->state() == QProcess::NotRunning) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("Internal simulator exited before advertising the RF topic.");
            }
            return false;
        }
        const QString type = RosBridge::topicType(internalRfTopic_);
        if (!type.isEmpty()) {
            if (type == QLatin1String("rf_msgs/RfObservationArray") || type == QLatin1String("std_msgs/String")) {
                appendSimulatorLog(QStringLiteral("[gui] RF topic discovered: %1 [%2]").arg(internalRfTopic_, type));
                return true;
            }
            if (errorMessage) {
                *errorMessage = QStringLiteral("Internal RF topic %1 appeared with unsupported type %2.").arg(internalRfTopic_, type);
            }
            return false;
        }
        if (lastType.isEmpty() && (timer.elapsed() > 1500)) {
            appendSimulatorLog(QStringLiteral("[gui] Waiting for internal RF topic %1 to appear...").arg(internalRfTopic_));
            lastType = QStringLiteral("waiting");
        }
        QThread::msleep(100);
    }
    if (errorMessage) {
        *errorMessage = QStringLiteral("Timed out waiting for internal RF topic %1.").arg(internalRfTopic_);
    }
    return false;
}

bool MainWindow::waitForRosBridgePose(QString* errorMessage, int timeoutMs) {
    QElapsedTimer timer;
    timer.start();
    bool loggedWaiting = false;
    while (timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents();
        if (!rosBridge_ || !rosBridge_->isRunning()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("ROS1 bridge stopped before the first pose message arrived.");
            }
            return false;
        }
        if (currentDrone_.valid) {
            return true;
        }
        if (!loggedWaiting && timer.elapsed() > 1200) {
            appendSimulatorLog(QStringLiteral("[gui] Waiting for the first pose message to reach the GUI bridge..."));
            loggedWaiting = true;
        }
        QThread::msleep(50);
    }
    if (errorMessage) {
        *errorMessage = QStringLiteral("Timed out waiting for the first pose message on %1.").arg(
            poseTopicCombo_ ? poseTopicCombo_->currentText().trimmed() : QStringLiteral("<empty>"));
    }
    return false;
}

bool MainWindow::restoreRosbagPlaybackPoseBridge(QString* errorMessage) {
    const bool rosbagPlaybackRunning = rosbagPlayProcess_ && rosbagPlayProcess_->state() != QProcess::NotRunning;
    if (!rosbagPlaybackRunning) {
        if (errorMessage) {
            *errorMessage = tr("Rosbag playback is not running.");
        }
        return false;
    }
    const QString poseTopic = !rosbagPlaybackPoseTopic_.trimmed().isEmpty()
        ? rosbagPlaybackPoseTopic_.trimmed()
        : (poseTopicCombo_ ? poseTopicCombo_->currentText().trimmed() : QString());
    const QString trajectoryTopic = rosbagPlaybackTrajectoryTopic_.trimmed();
    if (poseTopic.isEmpty()) {
        if (errorMessage) {
            *errorMessage = tr("Playback pose topic is empty.");
        }
        return false;
    }
    return connectRosBridgeOnly(poseTopic, trajectoryTopic, QString(), QString(), QString(), errorMessage, 5000);
}

QString MainWindow::diagnosticSummary() const {
    QStringList lines;
    lines << QStringLiteral("Last status: %1").arg(lastStatusMessage_.isEmpty() ? QStringLiteral("<none>") : lastStatusMessage_);
    lines << QStringLiteral("Simulator state: %1").arg(processStateText(simulatorProcess_ ? simulatorProcess_->state() : QProcess::NotRunning));
    if (simulatorProcess_) {
        lines << QStringLiteral("Simulator error: %1").arg(simulatorProcess_->errorString());
    }
    lines << QStringLiteral("Pose topic: %1 [%2]").arg(poseTopicCombo_->currentText().trimmed(), RosBridge::topicType(poseTopicCombo_->currentText().trimmed()));
    const QString trajectoryTopic = trajectoryTopicCombo_->currentText().trimmed();
    const QString trajectoryType = trajectoryTopic.isEmpty() ? QStringLiteral("<internal from pose>") : RosBridge::topicType(trajectoryTopic);
    lines << QStringLiteral("Trajectory topic: %1 [%2]").arg(trajectoryTopic.isEmpty() ? QStringLiteral("<internal>") : trajectoryTopic, trajectoryType);
    lines << QStringLiteral("Internal RF topic: %1 [%2]").arg(internalRfTopic_, RosBridge::topicType(internalRfTopic_));
    lines << QStringLiteral("Internal SYS topic: %1 [%2]").arg(internalSysTopic_, RosBridge::topicType(internalSysTopic_));
    lines << QStringLiteral("Internal beam topic: %1 [%2]").arg(internalBeamTopic_, RosBridge::topicType(internalBeamTopic_));
    lines << QStringLiteral("Internal beam codebook topic: %1 [%2]").arg(internalBeamCodebookTopic_, RosBridge::topicType(internalBeamCodebookTopic_));
    lines << QStringLiteral("Python: %1").arg(simSettings_.pythonExecutable.trimmed().isEmpty() ? QStringLiteral("python3") : simSettings_.pythonExecutable.trimmed());
    lines << QStringLiteral("Scene: %1").arg(simSettings_.scenePath);
    lines << QStringLiteral("ROS->3D matrix: %1").arg(coordinateTransforms_.rosToSceneMatrixPath);
    lines << QStringLiteral("AirSim->3D matrix: %1").arg(coordinateTransforms_.airsimToSceneMatrixPath);
    lines << QStringLiteral("RX array facing: %1").arg(rxArrayFacingDirectionDisplayName(coordinateTransforms_.rxArrayFacingDirection));
    lines << QStringLiteral("UAV COM->RX offset (UAV frame): [%1, %2, %3] m")
                 .arg(QString::number(coordinateTransforms_.uavToRxArrayTx, 'g', 8),
                      QString::number(coordinateTransforms_.uavToRxArrayTy, 'g', 8),
                      QString::number(coordinateTransforms_.uavToRxArrayTz, 'g', 8));
    lines << QStringLiteral("UAV COM->RX array center matrix: %1").arg(
        coordinateTransforms_.uavToRxArrayCenterMatrixPath.isEmpty()
            ? QStringLiteral("<generated from facing + tx/ty/tz>")
            : coordinateTransforms_.uavToRxArrayCenterMatrixPath);
    lines << QStringLiteral("RX array elements JSON: %1").arg(
        coordinateTransforms_.rxArrayElementsJsonPath.isEmpty()
            ? QStringLiteral("<none>")
            : coordinateTransforms_.rxArrayElementsJsonPath);
    lines << QStringLiteral("Wireless output frame: %1").arg(coordinateOutputFrameDisplayName(coordinateTransforms_.outputFrameKey));
    if (frameTransformManager_) {
        lines << QStringLiteral("Detected ROS frame: %1").arg(frameTransformManager_->resolvedRosFrameId());
    }
    if (!lastExportedBsJsonPath_.isEmpty()) {
        lines << QStringLiteral("Base station json: %1").arg(lastExportedBsJsonPath_);
    }
    const QString trimmedLog = simulatorLog_.trimmed();
    if (!trimmedLog.isEmpty()) {
        lines << QStringLiteral("") << QStringLiteral("Recent simulator log:") << trimmedLog.right(4000);
    }
    return lines.join(QStringLiteral("\n"));
}

void MainWindow::connectRos() {
    const bool rosbagPlaybackRunning = rosbagPlayProcess_ && rosbagPlayProcess_->state() != QProcess::NotRunning;
    if (rosbagPlaybackRunning && rosbagPlaybackBagHasWirelessTopics_) {
        QMessageBox::information(this, tr("Rosbag Playback"),
                                 tr("This rosbag contains recorded RF/SYS/Beam topics. During playback the GUI only replays selected recorded topics and data, so Start Simulation is disabled until playback stops."));
        return;
    }
    if (rosbagPlaybackRunning) {
        appendSimulatorLog(QStringLiteral("[gui] Rosbag playback has no recorded RF/SYS/Beam topics; starting internal Sionna simulation from the playback pose stream."));
    }

    stopDemo();
    updateSelectedTopicTypes();

    const QString poseTopic = poseTopicCombo_->currentText().trimmed();
    const QString trajectoryTopic = trajectoryTopicCombo_->currentText().trimmed();
    const QString poseType = RosBridge::topicType(poseTopic);
    const QString trajectoryType = RosBridge::topicType(trajectoryTopic);

    if (!RosBridge::isSupportedPoseType(poseType)) {
        const QString detail = diagnosticSummary();
        showShortDiagnosticWarning(tr("ROS1 / Pose Topic"),
                                   tr("Pose topic is not available or has unsupported type."),
                                   detail);
        return;
    }
    if (!trajectoryTopic.isEmpty() && !RosBridge::isSupportedTrajectoryType(trajectoryType)) {
        const QString detail = diagnosticSummary();
        showShortDiagnosticWarning(tr("ROS1 / Trajectory Topic"),
                                   tr("Trajectory topic is not available or has unsupported type."),
                                   detail);
        return;
    }

    resetLiveSessionState(true);

    if (!startInternalSimulator()) {
        showShortDiagnosticWarning(tr("Simulator"),
                                   tr("Failed to start the internal Sionna simulator."),
                                   diagnosticSummary());
        refreshInfoPanel();
        return;
    }

    QString rfWaitError;
    if (!waitForInternalRfTopic(&rfWaitError, 12000)) {
        stopInternalSimulator();
        showShortDiagnosticWarning(tr("Internal RF Simulator"),
                                   rfWaitError,
                                   diagnosticSummary());
        refreshInfoPanel();
        return;
    }

    const QString sysTopicForBridge = simSettings_.enableSysIntegration ? internalSysTopic_ : QString();
    const QString beamTopicForBridge = simSettings_.enableBeamforming ? internalBeamTopic_ : QString();
    auto startGuiRosBridge = [&]() {
        return rosBridge_->start(poseTopic, trajectoryTopic, internalRfTopic_, sysTopicForBridge, beamTopicForBridge);
    };

    if (!startGuiRosBridge()) {
        const QString message = tr("Failed to connect ROS1 bridge after the internal simulator started.");
        stopInternalSimulator();
        if (rosbagPlaybackRunning && !rosbagPlaybackBagHasWirelessTopics_) {
            QString restoreError;
            if (!restoreRosbagPlaybackPoseBridge(&restoreError)) {
                appendSimulatorLog(QStringLiteral("[gui] Failed to restore pose-only rosbag playback bridge after ROS1 connect failure: %1").arg(restoreError));
            }
        }
        showShortDiagnosticWarning(tr("ROS1"), message, diagnosticSummary());
        refreshInfoPanel();
        return;
    }

    QString poseWaitError;
    if (!waitForRosBridgePose(&poseWaitError, 5000)) {
        appendSimulatorLog(QStringLiteral("[gui] %1").arg(poseWaitError));
        appendSimulatorLog(QStringLiteral("[gui] First pose did not reach the GUI after reconnect; retrying the ROS1 bridge once."));
        rosBridge_->stop();
        QCoreApplication::processEvents();
        QThread::msleep(150);
        if (!startGuiRosBridge() || !waitForRosBridgePose(&poseWaitError, 5000)) {
            const QString message = tr("%1\n\nThe internal simulator was started, but the GUI did not receive pose updates after reconnecting to ROS1.")
                                        .arg(poseWaitError);
            rosBridge_->stop();
            stopInternalSimulator();
            if (rosbagPlaybackRunning && !rosbagPlaybackBagHasWirelessTopics_) {
                QString restoreError;
                if (!restoreRosbagPlaybackPoseBridge(&restoreError)) {
                    appendSimulatorLog(QStringLiteral("[gui] Failed to restore pose-only rosbag playback bridge after pose wait failure: %1").arg(restoreError));
                }
            }
            showShortDiagnosticWarning(tr("ROS1 / Pose Topic"), message, diagnosticSummary());
            refreshInfoPanel();
            return;
        }
    }
    refreshInfoPanel();
}

void MainWindow::disconnectRos() {
    const bool rosbagPlaybackRunning = rosbagPlayProcess_ && rosbagPlayProcess_->state() != QProcess::NotRunning;
    if (rosbagPlaybackRunning && rosbagPlaybackActive_ && !rosbagPlaybackBagHasWirelessTopics_) {
        appendSimulatorLog(QStringLiteral("[gui] Disconnect requested during pose-only rosbag playback; stopping internal Sionna simulation and keeping the playback pose bridge active."));
        stopInternalSimulator();
        rosbagPlaybackUsingRfTopic_ = false;
        rosbagPlaybackUsingSysTopic_ = false;
        rosbagPlaybackUsingBeamTopic_ = false;
        QString restoreError;
        if (!restoreRosbagPlaybackPoseBridge(&restoreError)) {
            appendSimulatorLog(QStringLiteral("[gui] Failed to restore pose-only rosbag playback bridge after disconnect: %1").arg(restoreError));
            if (rosbagPlaybackStatusValue_) {
                rosbagPlaybackStatusValue_->setText(tr("Stopped Sionna simulation, but failed to restore playback pose bridge. Check the log."));
            }
        } else if (rosbagPlaybackStatusValue_) {
            rosbagPlaybackStatusValue_->setText(tr("Sionna simulation stopped. Rosbag pose playback remains connected."));
        }
        rosbagPlaybackActive_ = true;
        rosbagPlaybackBagHasWirelessTopics_ = false;
        refreshInfoPanel();
        return;
    }
    rosbagPlaybackActive_ = false;
    rosbagPlaybackBagHasWirelessTopics_ = false;
    rosbagPlaybackUsingRfTopic_ = false;
    rosbagPlaybackUsingSysTopic_ = false;
    rosbagPlaybackUsingBeamTopic_ = false;
    rosbagPlaybackPoseTopic_.clear();
    rosbagPlaybackTrajectoryTopic_.clear();
    rosBridge_->stop();
    stopInternalSimulator();
    refreshInfoPanel();
}

void MainWindow::startDemo() {
    disconnectRos();
    demoController_->setBaseStations(stations_);
    demoController_->start();
    refreshInfoPanel();
}

void MainWindow::stopDemo() {
    demoController_->stop();
    refreshInfoPanel();
}

void MainWindow::applySceneBundle(const SceneGeometryBundle& bundle, const QString& successMessage) {
    QString error;
    if (!sceneWidget_->setSceneGeometry(bundle, &error)) {
        QMessageBox::warning(this, tr("Scene Import"), error);
        return;
    }
    const QString reloadPath = !bundle.generatedMeshPath.trimmed().isEmpty()
        ? bundle.generatedMeshPath.trimmed()
        : bundle.sourcePath.trimmed();
    currentSceneMeshPath_ = isSupportedSceneMeshFilePath(reloadPath)
        ? QFileInfo(reloadPath).absoluteFilePath()
        : QString();
    currentSceneSource_ = successMessage;
    enableDenseCkmAutoBoundsIfSceneAvailable();
    statusBar()->showMessage(successMessage, 5000);
    markGuiConfigDirty();
    refreshInfoPanel();
}

void MainWindow::loadSceneMesh() {
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Load OBJ, FBX, STL, or GLTF Mesh"), QString(), tr("3D Mesh (*.obj *.fbx *.stl *.gltf *.glb)"));
    if (path.isEmpty()) {
        return;
    }

    const auto bundle = sceneManager_->loadFromFile(path);
    applySceneBundle(bundle, QString("File mesh: %1").arg(path));
}

void MainWindow::importOsmScene() {
    OsmImportDialog dialog(this);
    useCommittedSpinBoxEdits(&dialog);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    const OsmImportSelection sel = dialog.selection();
    if (!sel.isValid()) {
        QMessageBox::information(this, tr("OSM Import"), tr("The selected OSM bounding box is invalid."));
        return;
    }

    QProgressDialog progressDialog(tr("Preparing OSM import..."), QString(), 0, 100, this);
    progressDialog.setWindowTitle(tr("OSM Import"));
    progressDialog.setWindowModality(Qt::ApplicationModal);
    progressDialog.setCancelButton(nullptr);
    progressDialog.setMinimumDuration(0);
    progressDialog.setValue(0);
    progressDialog.show();
    QApplication::processEvents();

    auto progressCallback = [&](const QString& message, int value, int maximum) {
        if (maximum <= 0) {
            maximum = 100;
        }
        progressDialog.setMaximum(maximum);
        progressDialog.setLabelText(message);
        progressDialog.setValue(std::max(0, std::min(value, maximum)));
        statusBar()->showMessage(message);
        QApplication::processEvents();
    };

    const auto bundle = sceneManager_->loadFromOsmBoundingBox(
        sel.southLat, sel.westLon, sel.northLat, sel.eastLon,
        sel.displayName.isEmpty() ? tr("OSM imported area") : sel.displayName,
        sel.includeTrees, sel.includeStreetFurniture, sel.includeGreenAreas, sel.includeWater,
        QStringLiteral("https://overpass-api.de/api/interpreter"),
        sel.useOsm2World,
        sel.osm2worldExecutable,
        sel.osm2worldOutputFormat,
        sel.osm2worldOutputDirectory,
        sel.osm2worldExtraArguments,
        sel.osm2worldJavaExecutable,
        sel.keepDownloadedOsmCopy,
        progressCallback);
    progressDialog.setValue(progressDialog.maximum());
    const QString selectionText = QStringLiteral("S:%1 W:%2 N:%3 E:%4")
                                      .arg(sel.southLat, 0, 'f', 5)
                                      .arg(sel.westLon, 0, 'f', 5)
                                      .arg(sel.northLat, 0, 'f', 5)
                                      .arg(sel.eastLon, 0, 'f', 5);
    if (osmSelectionValue_) {
        osmSelectionValue_->setText(selectionText);
    }
    if (osmSourceValue_) {
        if (sel.useOsm2World && !bundle.generatedMeshPath.isEmpty()) {
            osmSourceValue_->setText(bundle.generatedMeshPath);
        } else {
            osmSourceValue_->setText(sel.displayName.isEmpty() ? tr("OSM bounding box search/import") : sel.displayName);
        }
    }
    QString successMessage = QStringLiteral("OSM area imported: %1 (%2)").arg(sel.displayName.isEmpty() ? tr("selected area") : sel.displayName, selectionText);
    if (sel.useOsm2World && bundle.valid) {
        successMessage = QStringLiteral("OSM2World mesh imported: %1 (%2)").arg(sel.displayName.isEmpty() ? tr("selected area") : sel.displayName, selectionText);
        if (!bundle.generatedMeshPath.isEmpty()) {
            successMessage += QStringLiteral(" -> %1").arg(bundle.generatedMeshPath);
        }
    }
    applySceneBundle(bundle, successMessage);
}

void MainWindow::clearSceneGeometry() {
    sceneWidget_->clearSceneGeometry();
    currentSceneMeshPath_.clear();
    currentSceneSource_ = "None";
    statusBar()->showMessage("Cleared imported scene geometry.", 3000);
    markGuiConfigDirty();
    refreshInfoPanel();
}

void MainWindow::reloadConfigs() {
    if (!promptSaveBaseStationsJsonIfDirty(
            tr("The current base-station JSON has unsaved changes. Save it before reloading JSON configs?"))) {
        return;
    }
    loadInitialData();
    setGuiConfigDirty(false);
    setBaseStationsDirty(false);
    clearBaseStationUndoHistory();
    refreshRosTopics();
    statusBar()->showMessage("Reloaded JSON configs.", 3000);
}

void MainWindow::openCoordinateTransformSettings() {
    QDialog dialog(this);
    dialog.setWindowTitle(tr("Coordinate Frame Manager"));
    dialog.resize(940, 720);

    auto* layout = new QVBoxLayout(&dialog);
    auto* intro = new QLabel(tr("The GUI keeps one tf2-style internal frame graph with global frames `ROS → 3D → AirSim`, plus runtime receiver frames `uav_com → rx_array_center → rx_ant_*`. Incoming pose / odom messages still define the UAV centroid pose, but Sionna now uses the runtime RX array-center frame instead of assuming the centroid and antenna center coincide. Wireless output topics can still be emitted in ROS / 3D / AirSim coordinates."), &dialog);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    auto* form = new QFormLayout();
    auto* rosPathEdit = new QLineEdit(coordinateTransforms_.rosToSceneMatrixPath, &dialog);
    auto* browseRosButton = new QPushButton(tr("Browse..."), &dialog);
    auto* rosRowWidget = new QWidget(&dialog);
    auto* rosRow = new QHBoxLayout(rosRowWidget);
    rosRow->setContentsMargins(0, 0, 0, 0);
    rosRow->addWidget(rosPathEdit, 1);
    rosRow->addWidget(browseRosButton);

    auto* airsimPathEdit = new QLineEdit(coordinateTransforms_.airsimToSceneMatrixPath, &dialog);
    auto* browseAirsimButton = new QPushButton(tr("Browse..."), &dialog);
    auto* airsimRowWidget = new QWidget(&dialog);
    auto* airsimRow = new QHBoxLayout(airsimRowWidget);
    airsimRow->setContentsMargins(0, 0, 0, 0);
    airsimRow->addWidget(airsimPathEdit, 1);
    airsimRow->addWidget(browseAirsimButton);

    auto* rxFacingCombo = new QComboBox(&dialog);
    rxFacingCombo->addItem(tr("Front (+X, Y-Z plane)"), QStringLiteral("front"));
    rxFacingCombo->addItem(tr("Back (-X, Y-Z plane)"), QStringLiteral("back"));
    rxFacingCombo->addItem(tr("Left (+Y, X-Z plane)"), QStringLiteral("left"));
    rxFacingCombo->addItem(tr("Right (-Y, X-Z plane)"), QStringLiteral("right"));
    rxFacingCombo->addItem(tr("Up (+Z, X-Y plane)"), QStringLiteral("up"));
    rxFacingCombo->addItem(tr("Down (-Z, X-Y plane)"), QStringLiteral("down"));
    const QString normalizedFacingDirection = normalizeRxArrayFacingDirectionKey(coordinateTransforms_.rxArrayFacingDirection);
    const int facingIndex = rxFacingCombo->findData(normalizedFacingDirection);
    rxFacingCombo->setCurrentIndex(facingIndex >= 0 ? facingIndex : 0);

    auto* uavTxSpin = new QDoubleSpinBox(&dialog);
    auto* uavTySpin = new QDoubleSpinBox(&dialog);
    auto* uavTzSpin = new QDoubleSpinBox(&dialog);
    const auto configureOffsetSpin = [](QDoubleSpinBox* spin, double value) {
        spin->setRange(-10000.0, 10000.0);
        spin->setDecimals(6);
        spin->setSingleStep(0.01);
        spin->setValue(std::isfinite(value) ? value : 0.0);
        spin->setSuffix(QStringLiteral(" m"));
    };
    configureOffsetSpin(uavTxSpin, coordinateTransforms_.uavToRxArrayTx);
    configureOffsetSpin(uavTySpin, coordinateTransforms_.uavToRxArrayTy);
    configureOffsetSpin(uavTzSpin, coordinateTransforms_.uavToRxArrayTz);
    auto* uavOffsetRowWidget = new QWidget(&dialog);
    auto* uavOffsetRow = new QHBoxLayout(uavOffsetRowWidget);
    uavOffsetRow->setContentsMargins(0, 0, 0, 0);
    uavOffsetRow->addWidget(new QLabel(tr("tx"), uavOffsetRowWidget));
    uavOffsetRow->addWidget(uavTxSpin);
    uavOffsetRow->addSpacing(8);
    uavOffsetRow->addWidget(new QLabel(tr("ty"), uavOffsetRowWidget));
    uavOffsetRow->addWidget(uavTySpin);
    uavOffsetRow->addSpacing(8);
    uavOffsetRow->addWidget(new QLabel(tr("tz"), uavOffsetRowWidget));
    uavOffsetRow->addWidget(uavTzSpin);
    uavOffsetRow->addStretch(1);

    auto* uavRxPathEdit = new QLineEdit(coordinateTransforms_.uavToRxArrayCenterMatrixPath, &dialog);
    auto* browseUavRxButton = new QPushButton(tr("Browse..."), &dialog);
    auto* uavRxRowWidget = new QWidget(&dialog);
    auto* uavRxRow = new QHBoxLayout(uavRxRowWidget);
    uavRxRow->setContentsMargins(0, 0, 0, 0);
    uavRxRow->addWidget(uavRxPathEdit, 1);
    uavRxRow->addWidget(browseUavRxButton);

    auto* antennaPathEdit = new QLineEdit(coordinateTransforms_.rxArrayElementsJsonPath, &dialog);
    auto* browseAntennaButton = new QPushButton(tr("Browse..."), &dialog);
    auto* antennaRowWidget = new QWidget(&dialog);
    auto* antennaRow = new QHBoxLayout(antennaRowWidget);
    antennaRow->setContentsMargins(0, 0, 0, 0);
    antennaRow->addWidget(antennaPathEdit, 1);
    antennaRow->addWidget(browseAntennaButton);

    auto* rosPreview = new QLabel(&dialog);
    auto* airsimPreview = new QLabel(&dialog);
    auto* uavRxPreview = new QLabel(&dialog);
    auto* antennaPreview = new QLabel(&dialog);
    const auto configurePreviewLabel = [](QLabel* label) {
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        label->setWordWrap(true);
        label->setStyleSheet(QStringLiteral("QLabel { background: #1f2430; color: #e8ecf1; border: 1px solid #394150; border-radius: 6px; padding: 8px; font-family: monospace; }"));
    };
    configurePreviewLabel(rosPreview);
    configurePreviewLabel(airsimPreview);
    configurePreviewLabel(uavRxPreview);
    configurePreviewLabel(antennaPreview);

    auto* outputFrameCombo = new QComboBox(&dialog);
    outputFrameCombo->addItem(tr("ROS"), coordinateOutputFrameToKey(CoordinateOutputFrame::Ros));
    outputFrameCombo->addItem(tr("3D"), coordinateOutputFrameToKey(CoordinateOutputFrame::Scene3D));
    outputFrameCombo->addItem(tr("AirSim"), coordinateOutputFrameToKey(CoordinateOutputFrame::AirSim));
    const QString normalizedOutputFrameKey = coordinateOutputFrameToKey(coordinateOutputFrameFromKey(coordinateTransforms_.outputFrameKey));
    const int outputFrameIndex = outputFrameCombo->findData(normalizedOutputFrameKey);
    outputFrameCombo->setCurrentIndex(outputFrameIndex >= 0 ? outputFrameIndex : 1);

    auto* detectedRosFrameValue = new QLabel(&dialog);
    detectedRosFrameValue->setTextInteractionFlags(Qt::TextSelectableByMouse);
    const QString detectedRosFrame = (!currentDrone_.sourceFrameId.trimmed().isEmpty())
                                         ? currentDrone_.sourceFrameId.trimmed()
                                         : (frameTransformManager_ ? frameTransformManager_->resolvedRosFrameId() : QStringLiteral("ROS"));
    detectedRosFrameValue->setText(tr("%1 (auto-detected from the current pose / odom header.frame_id; if the incoming pose frame_id is empty, runtime fallback frame id is used when publishing ROS-frame wireless topics)")
                                       .arg(detectedRosFrame));

    const auto updateMatrixPreview = [](QLabel* label,
                                        const QString& path,
                                        const QString& title,
                                        bool optional,
                                        const QString& emptySummary) -> bool {
        const QString trimmedPath = path.trimmed();
        if (optional && trimmedPath.isEmpty()) {
            label->setText(QStringLiteral("%1\n%2").arg(title, emptySummary));
            return true;
        }
        QString error;
        const TransformMatrix4x4 matrix = loadTransformMatrixFromJson(trimmedPath, &error);
        if (!error.trimmed().isEmpty()) {
            label->setText(QStringLiteral("%1\n%2").arg(title, error));
            return false;
        }
        label->setText(QStringLiteral("%1\nPath: %2\n%3").arg(title, trimmedPath, transformMatrixToDisplayString(matrix)));
        return true;
    };

    const auto updateUavRxPreview = [=](QLabel* label,
                                        const QString& path,
                                        const QString& facingDirection,
                                        double tx,
                                        double ty,
                                        double tz,
                                        const QString& title) -> bool {
        const QString trimmedPath = path.trimmed();
        if (!trimmedPath.isEmpty()) {
            QString error;
            const TransformMatrix4x4 matrix = loadTransformMatrixFromJson(trimmedPath, &error);
            if (!error.trimmed().isEmpty()) {
                label->setText(QStringLiteral("%1\n%2").arg(title, error));
                return false;
            }
            label->setText(QStringLiteral("%1\nMode: JSON override\nPath: %2\nFacing / tx / ty / tz controls are ignored while this file is set.\n%3")
                               .arg(title, trimmedPath, transformMatrixToDisplayString(matrix)));
            return true;
        }

        const QString normalizedDirection = normalizeRxArrayFacingDirectionKey(facingDirection);
        const TransformMatrix4x4 generated = buildUavToRxArrayCenterTransform(normalizedDirection, tx, ty, tz);
        label->setText(QStringLiteral("%1\nMode: Generated from facing + tx/ty/tz\nFacing: %2\nOffset in UAV frame: [tx=%3, ty=%4, tz=%5] m\n%6")
                           .arg(title,
                                rxArrayFacingDirectionDisplayName(normalizedDirection),
                                QString::number(tx, 'g', 8),
                                QString::number(ty, 'g', 8),
                                QString::number(tz, 'g', 8),
                                transformMatrixToDisplayString(generated)));
        return true;
    };

    const auto updateAntennaPreview = [this](QLabel* label, const QString& path, const QString& title) -> bool {
        const QString trimmedPath = path.trimmed();
        if (trimmedPath.isEmpty()) {
            constexpr double kSpeedOfLight = 299792458.0;
            const int rows = std::max(simSettings_.rxArrayNumRows, 1);
            const int cols = std::max(simSettings_.rxArrayNumCols, 1);
            const double wavelength = (std::isfinite(simSettings_.fcHz) && simSettings_.fcHz > 0.0)
                                          ? (kSpeedOfLight / simSettings_.fcHz)
                                          : 1.0;
            const double verticalSpacingM = std::max(0.0, simSettings_.rxArrayVerticalSpacing) * wavelength;
            const double horizontalSpacingM = std::max(0.0, simSettings_.rxArrayHorizontalSpacing) * wavelength;
            label->setText(QStringLiteral("%1\n<auto-generated from current Sionna RX array settings>\nRows x Cols: %2 x %3\nAntenna frames: %4\nVertical spacing: %5 lambda = %6 m\nHorizontal spacing: %7 lambda = %8 m")
                               .arg(title)
                               .arg(rows)
                               .arg(cols)
                               .arg(rows * cols)
                               .arg(simSettings_.rxArrayVerticalSpacing, 0, 'g', 6)
                               .arg(verticalSpacingM, 0, 'g', 6)
                               .arg(simSettings_.rxArrayHorizontalSpacing, 0, 'g', 6)
                               .arg(horizontalSpacingM, 0, 'g', 6));
            return true;
        }
        QFile file(trimmedPath);
        if (!file.exists()) {
            label->setText(QStringLiteral("%1\nTransform list file does not exist: %2").arg(title, trimmedPath));
            return false;
        }
        if (!file.open(QIODevice::ReadOnly)) {
            label->setText(QStringLiteral("%1\nCould not open transform list file: %2").arg(title, trimmedPath));
            return false;
        }
        QJsonParseError parseError;
        const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
        if (parseError.error != QJsonParseError::NoError || (!doc.isArray() && !doc.isObject())) {
            label->setText(QStringLiteral("%1\nFailed to parse JSON: %2").arg(title, parseError.errorString()));
            return false;
        }

        const auto looksLikeMatrix = [](const QJsonArray& array) -> bool {
            if (array.size() != 4) {
                return false;
            }
            for (int rowIndex = 0; rowIndex < 4; ++rowIndex) {
                if (!array.at(rowIndex).isArray() || array.at(rowIndex).toArray().size() != 4) {
                    return false;
                }
            }
            return true;
        };
        const auto isValidEntry = [&](const QJsonValue& value) -> bool {
            if (value.isArray()) {
                return looksLikeMatrix(value.toArray());
            }
            if (!value.isObject()) {
                return false;
            }
            const QJsonObject obj = value.toObject();
            const QStringList matrixKeys = {
                QStringLiteral("matrix"),
                QStringLiteral("transform"),
                QStringLiteral("T"),
            };
            for (const QString& key : matrixKeys) {
                if (obj.contains(key) && obj.value(key).isArray() && looksLikeMatrix(obj.value(key).toArray())) {
                    return true;
                }
            }
            return false;
        };

        int entryCount = 0;
        QStringList frameNames;
        auto collectArrayEntries = [&](const QJsonArray& array) {
            for (int idx = 0; idx < array.size(); ++idx) {
                const QJsonValue value = array.at(idx);
                if (!isValidEntry(value)) {
                    label->setText(QStringLiteral("%1\nEntry %2 is not a valid 4x4 antenna transform.").arg(title).arg(idx));
                    entryCount = -1;
                    frameNames.clear();
                    return;
                }
                ++entryCount;
                if (value.isObject()) {
                    const QJsonObject obj = value.toObject();
                    const QString frameId = obj.value(QStringLiteral("frame_id")).toString(
                        obj.value(QStringLiteral("name")).toString(
                            obj.value(QStringLiteral("id")).toString(QStringLiteral("rx_ant_%1").arg(idx))));
                    frameNames << frameId;
                } else {
                    frameNames << QStringLiteral("rx_ant_%1").arg(idx);
                }
            }
        };

        if (doc.isArray()) {
            const QJsonArray array = doc.array();
            if (looksLikeMatrix(array)) {
                entryCount = 1;
                frameNames << QStringLiteral("rx_ant_0");
            } else {
                collectArrayEntries(array);
                if (entryCount < 0) {
                    return false;
                }
            }
        } else {
            const QJsonObject obj = doc.object();
            if (obj.contains(QStringLiteral("elements")) && obj.value(QStringLiteral("elements")).isArray()) {
                collectArrayEntries(obj.value(QStringLiteral("elements")).toArray());
                if (entryCount < 0) {
                    return false;
                }
            } else if (obj.contains(QStringLiteral("antennas")) && obj.value(QStringLiteral("antennas")).isArray()) {
                collectArrayEntries(obj.value(QStringLiteral("antennas")).toArray());
                if (entryCount < 0) {
                    return false;
                }
            } else {
                for (auto it = obj.constBegin(); it != obj.constEnd(); ++it) {
                    if (it.key() == QStringLiteral("parent_frame")
                        || it.key() == QStringLiteral("source_frame")
                        || it.key() == QStringLiteral("frame_prefix")) {
                        continue;
                    }
                    if (!isValidEntry(it.value())) {
                        label->setText(QStringLiteral("%1\nEntry '%2' is not a valid 4x4 antenna transform.").arg(title, it.key()));
                        return false;
                    }
                    ++entryCount;
                    frameNames << it.key();
                }
            }
        }

        label->setText(QStringLiteral("%1\nPath: %2\nLoaded entries: %3\nFrames: %4")
                           .arg(title,
                                trimmedPath,
                                QString::number(entryCount),
                                frameNames.isEmpty() ? QStringLiteral("<none>") : frameNames.join(QStringLiteral(", "))));
        return true;
    };

    updateMatrixPreview(rosPreview, rosPathEdit->text(), tr("ROS → 3D matrix"), false, QString());
    updateMatrixPreview(airsimPreview, airsimPathEdit->text(), tr("AirSim → 3D matrix"), false, QString());
    updateUavRxPreview(
        uavRxPreview,
        uavRxPathEdit->text(),
        rxFacingCombo->currentData().toString(),
        uavTxSpin->value(),
        uavTySpin->value(),
        uavTzSpin->value(),
        tr("Effective UAV COM → RX array center transform"));
    updateAntennaPreview(antennaPreview, antennaPathEdit->text(), tr("RX array center → antenna transforms"));

    QObject::connect(browseRosButton, &QPushButton::clicked, &dialog, [this, rosPathEdit]() {
        const QString path = QFileDialog::getOpenFileName(this, tr("Select ROS → 3D matrix JSON"), QFileInfo(rosPathEdit->text()).absolutePath(), tr("JSON (*.json)"));
        if (!path.isEmpty()) {
            rosPathEdit->setText(path);
        }
    });
    QObject::connect(browseAirsimButton, &QPushButton::clicked, &dialog, [this, airsimPathEdit]() {
        const QString path = QFileDialog::getOpenFileName(this, tr("Select AirSim → 3D matrix JSON"), QFileInfo(airsimPathEdit->text()).absolutePath(), tr("JSON (*.json)"));
        if (!path.isEmpty()) {
            airsimPathEdit->setText(path);
        }
    });
    QObject::connect(browseUavRxButton, &QPushButton::clicked, &dialog, [this, uavRxPathEdit]() {
        const QString path = QFileDialog::getOpenFileName(this, tr("Select UAV COM → RX array center matrix JSON"), QFileInfo(uavRxPathEdit->text()).absolutePath(), tr("JSON (*.json)"));
        if (!path.isEmpty()) {
            uavRxPathEdit->setText(path);
        }
    });
    QObject::connect(browseAntennaButton, &QPushButton::clicked, &dialog, [this, antennaPathEdit]() {
        const QString path = QFileDialog::getOpenFileName(this, tr("Select RX array elements JSON"), QFileInfo(antennaPathEdit->text()).absolutePath(), tr("JSON (*.json)"));
        if (!path.isEmpty()) {
            antennaPathEdit->setText(path);
        }
    });
    QObject::connect(rosPathEdit, &QLineEdit::textChanged, &dialog, [updateMatrixPreview, rosPreview](const QString& text) {
        updateMatrixPreview(rosPreview, text, QObject::tr("ROS → 3D matrix"), false, QString());
    });
    QObject::connect(airsimPathEdit, &QLineEdit::textChanged, &dialog, [updateMatrixPreview, airsimPreview](const QString& text) {
        updateMatrixPreview(airsimPreview, text, QObject::tr("AirSim → 3D matrix"), false, QString());
    });
    QObject::connect(uavRxPathEdit, &QLineEdit::textChanged, &dialog, [=](const QString& text) {
        updateUavRxPreview(
            uavRxPreview,
            text,
            rxFacingCombo->currentData().toString(),
            uavTxSpin->value(),
            uavTySpin->value(),
            uavTzSpin->value(),
            QObject::tr("Effective UAV COM → RX array center transform"));
    });
    QObject::connect(rxFacingCombo, &QComboBox::currentTextChanged, &dialog, [=](const QString&) {
        updateUavRxPreview(
            uavRxPreview,
            uavRxPathEdit->text(),
            rxFacingCombo->currentData().toString(),
            uavTxSpin->value(),
            uavTySpin->value(),
            uavTzSpin->value(),
            QObject::tr("Effective UAV COM → RX array center transform"));
    });
    QObject::connect(uavTxSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), &dialog, [=](double) {
        updateUavRxPreview(
            uavRxPreview,
            uavRxPathEdit->text(),
            rxFacingCombo->currentData().toString(),
            uavTxSpin->value(),
            uavTySpin->value(),
            uavTzSpin->value(),
            QObject::tr("Effective UAV COM → RX array center transform"));
    });
    QObject::connect(uavTySpin, qOverload<double>(&QDoubleSpinBox::valueChanged), &dialog, [=](double) {
        updateUavRxPreview(
            uavRxPreview,
            uavRxPathEdit->text(),
            rxFacingCombo->currentData().toString(),
            uavTxSpin->value(),
            uavTySpin->value(),
            uavTzSpin->value(),
            QObject::tr("Effective UAV COM → RX array center transform"));
    });
    QObject::connect(uavTzSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), &dialog, [=](double) {
        updateUavRxPreview(
            uavRxPreview,
            uavRxPathEdit->text(),
            rxFacingCombo->currentData().toString(),
            uavTxSpin->value(),
            uavTySpin->value(),
            uavTzSpin->value(),
            QObject::tr("Effective UAV COM → RX array center transform"));
    });
    QObject::connect(antennaPathEdit, &QLineEdit::textChanged, &dialog, [updateAntennaPreview, antennaPreview](const QString& text) {
        updateAntennaPreview(antennaPreview, text, QObject::tr("RX array center → antenna transforms"));
    });

    form->addRow(tr("ROS → 3D JSON"), rosRowWidget);
    form->addRow(QString(), rosPreview);
    form->addRow(tr("AirSim → 3D JSON"), airsimRowWidget);
    form->addRow(QString(), airsimPreview);
    form->addRow(tr("RX array facing"), rxFacingCombo);
    form->addRow(tr("UAV COM → RX center offset"), uavOffsetRowWidget);
    form->addRow(tr("UAV COM → RX center JSON (advanced override)"), uavRxRowWidget);
    form->addRow(QString(), uavRxPreview);
    form->addRow(tr("RX array center → antennas JSON"), antennaRowWidget);
    form->addRow(QString(), antennaPreview);
    form->addRow(tr("Wireless output frame"), outputFrameCombo);
    form->addRow(tr("Detected ROS frame"), detectedRosFrameValue);
    layout->addLayout(form);

    auto* note = new QLabel(tr("The GUI now keeps one tf2-style internal graph. For the local RX mount, the normal workflow is: choose which UAV face the RX array plane points toward, then enter `tx / ty / tz` as the RX array-center offset in the UAV body frame. The optional `UAV COM → RX center JSON` is an advanced override; when it is set, it takes priority over the facing + offset controls. `RX array center → antennas` is also optional: if left empty, the GUI auto-generates `rx_ant_*` frames from the current RX rows/cols plus V/H spacing in Sionna Settings, using the current carrier frequency to convert wavelength-based spacing into meters. AirSim drawing still queries the inverse of AirSim→3D when it needs 3D→AirSim."), &dialog);
    note->setWordWrap(true);
    layout->addWidget(note);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    layout->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&]() {
        const bool rosOk = updateMatrixPreview(rosPreview, rosPathEdit->text(), tr("ROS → 3D matrix"), false, QString());
        const bool airsimOk = updateMatrixPreview(airsimPreview, airsimPathEdit->text(), tr("AirSim → 3D matrix"), false, QString());
        const bool uavRxOk = updateUavRxPreview(
            uavRxPreview,
            uavRxPathEdit->text(),
            rxFacingCombo->currentData().toString(),
            uavTxSpin->value(),
            uavTySpin->value(),
            uavTzSpin->value(),
            tr("Effective UAV COM → RX array center transform"));
        const bool antennaOk = updateAntennaPreview(antennaPreview, antennaPathEdit->text(), tr("RX array center → antenna transforms"));
        if (!rosOk || !airsimOk || !uavRxOk || !antennaOk) {
            QMessageBox::warning(&dialog, tr("Coordinate Frames"), tr("Please provide valid global transform JSON files. The two local RX files are optional, but if provided they must also parse successfully."));
            return;
        }
        dialog.accept();
    });

    useCommittedSpinBoxEdits(&dialog);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    coordinateTransforms_.rosToSceneMatrixPath = rosPathEdit->text().trimmed();
    coordinateTransforms_.airsimToSceneMatrixPath = airsimPathEdit->text().trimmed();
    coordinateTransforms_.rxArrayFacingDirection = normalizeRxArrayFacingDirectionKey(rxFacingCombo->currentData().toString());
    coordinateTransforms_.uavToRxArrayTx = uavTxSpin->value();
    coordinateTransforms_.uavToRxArrayTy = uavTySpin->value();
    coordinateTransforms_.uavToRxArrayTz = uavTzSpin->value();
    coordinateTransforms_.uavToRxArrayCenterMatrixPath = uavRxPathEdit->text().trimmed();
    coordinateTransforms_.rxArrayElementsJsonPath = antennaPathEdit->text().trimmed();
    coordinateTransforms_.outputFrameKey = outputFrameCombo->currentData().toString().trimmed();
    coordinateTransforms_.outputFrameKey = coordinateOutputFrameToKey(coordinateOutputFrameFromKey(coordinateTransforms_.outputFrameKey));

    QSettings settings(QStringLiteral("OpenAI"), QStringLiteral("airsim_gui_UErealtime"));
    settings.setValue(QStringLiteral("coordinateFrames/rosToSceneMatrixPath"), coordinateTransforms_.rosToSceneMatrixPath);
    settings.setValue(QStringLiteral("coordinateFrames/airsimToSceneMatrixPath"), coordinateTransforms_.airsimToSceneMatrixPath);
    settings.setValue(QStringLiteral("coordinateFrames/rxArrayFacingDirection"), coordinateTransforms_.rxArrayFacingDirection);
    settings.setValue(QStringLiteral("coordinateFrames/uavToRxArrayTx"), coordinateTransforms_.uavToRxArrayTx);
    settings.setValue(QStringLiteral("coordinateFrames/uavToRxArrayTy"), coordinateTransforms_.uavToRxArrayTy);
    settings.setValue(QStringLiteral("coordinateFrames/uavToRxArrayTz"), coordinateTransforms_.uavToRxArrayTz);
    settings.setValue(QStringLiteral("coordinateFrames/uavToRxArrayCenterMatrixPath"), coordinateTransforms_.uavToRxArrayCenterMatrixPath);
    settings.setValue(QStringLiteral("coordinateFrames/rxArrayElementsJsonPath"), coordinateTransforms_.rxArrayElementsJsonPath);
    settings.setValue(QStringLiteral("coordinateFrames/outputFrameKey"), coordinateTransforms_.outputFrameKey);

    applyCoordinateTransformSettings(true);
    markGuiConfigDirty();
    onStatusMessage(tr("Coordinate frame settings updated. ROS→3D: %1 | AirSim→3D: %2 | RX facing: %3 | UAV→RX offset: [%4, %5, %6] m | UAV→RX override: %7 | Antennas: %8 | Output frame: %9")
        .arg(coordinateTransforms_.rosToSceneMatrixPath,
             coordinateTransforms_.airsimToSceneMatrixPath,
             rxArrayFacingDirectionDisplayName(coordinateTransforms_.rxArrayFacingDirection),
             QString::number(coordinateTransforms_.uavToRxArrayTx, 'g', 8),
             QString::number(coordinateTransforms_.uavToRxArrayTy, 'g', 8),
             QString::number(coordinateTransforms_.uavToRxArrayTz, 'g', 8),
             coordinateTransforms_.uavToRxArrayCenterMatrixPath.isEmpty() ? QStringLiteral("<generated from facing + tx/ty/tz>") : coordinateTransforms_.uavToRxArrayCenterMatrixPath,
             coordinateTransforms_.rxArrayElementsJsonPath.isEmpty() ? QStringLiteral("<none>") : coordinateTransforms_.rxArrayElementsJsonPath,
             coordinateOutputFrameDisplayName(coordinateTransforms_.outputFrameKey)));
}

void MainWindow::openSimulationSettings() {
    QDialog dialog(this);
    dialog.setWindowTitle(tr("Sionna Simulation Settings"));
    dialog.resize(1280, 820);
    dialog.setMinimumSize(1280, 820);

    auto* layout = new QVBoxLayout(&dialog);
    auto* scrollArea = new QScrollArea(&dialog);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);

    auto* content = new QWidget(scrollArea);
    auto* contentLayout = new QVBoxLayout(content);
    contentLayout->setContentsMargins(0, 0, 0, 0);
    contentLayout->setSpacing(12);

    auto makeEditableCombo = [&dialog](const QStringList& options, const QString& value) {
        auto* combo = new QComboBox(&dialog);
        combo->setEditable(true);
        combo->addItems(options);
        const int index = combo->findText(value);
        if (index >= 0) {
            combo->setCurrentIndex(index);
        } else {
            combo->setEditText(value);
        }
        return combo;
    };

    struct ArrayEditors {
        QComboBox* pattern{nullptr};
        QComboBox* polarization{nullptr};
        QSpinBox* rows{nullptr};
        QSpinBox* cols{nullptr};
        QDoubleSpinBox* verticalSpacing{nullptr};
        QDoubleSpinBox* horizontalSpacing{nullptr};
    };

    auto* envGroup = new QGroupBox(tr("Environment"), content);
    auto* envForm = new QFormLayout(envGroup);

    auto* pythonPathEdit = new QLineEdit(simSettings_.pythonExecutable, envGroup);
    pythonPathEdit->setPlaceholderText(tr("Select the Python interpreter that already has sionna installed"));
    auto* selectPythonButton = new QPushButton(tr("Select Environment..."), envGroup);
    auto* browsePythonButton = new QPushButton(tr("Browse..."), envGroup);
    auto* testPythonButton = new QPushButton(tr("Test Environment"), envGroup);
    auto* pythonRowWidget = new QWidget(envGroup);
    auto* pythonRow = new QHBoxLayout(pythonRowWidget);
    pythonRow->setContentsMargins(0, 0, 0, 0);
    pythonRow->addWidget(pythonPathEdit, 1);
    pythonRow->addWidget(selectPythonButton);
    pythonRow->addWidget(browsePythonButton);
    pythonRow->addWidget(testPythonButton);

    auto* scenePathEdit = new QLineEdit(simSettings_.scenePath, envGroup);
    auto* browseSceneButton = new QPushButton(tr("Browse..."), envGroup);
    auto* sceneRowWidget = new QWidget(envGroup);
    auto* sceneRow = new QHBoxLayout(sceneRowWidget);
    sceneRow->setContentsMargins(0, 0, 0, 0);
    sceneRow->addWidget(scenePathEdit);
    sceneRow->addWidget(browseSceneButton);

    envForm->addRow(tr("Python environment"), pythonRowWidget);
    envForm->addRow(tr("Simulation scene XML"), sceneRowWidget);

    auto* solverGroup = new QGroupBox(tr("Solver"), content);
    auto* solverForm = new QFormLayout(solverGroup);

    auto* fcSpin = new QDoubleSpinBox(solverGroup);
    fcSpin->setRange(1e6, 1e12);
    fcSpin->setDecimals(0);
    fcSpin->setValue(simSettings_.fcHz);

    auto* miVariantEdit = new QLineEdit(simSettings_.miVariant, solverGroup);

    auto* maxDepthSpin = new QSpinBox(solverGroup);
    maxDepthSpin->setRange(0, 16);
    maxDepthSpin->setValue(simSettings_.maxDepth);

    auto* samplesSpin = new QSpinBox(solverGroup);
    samplesSpin->setRange(1, 1000000);
    samplesSpin->setValue(simSettings_.samplesPerSrc);

    auto* maxPathsSpin = new QSpinBox(solverGroup);
    maxPathsSpin->setRange(1, 4096);
    maxPathsSpin->setValue(simSettings_.maxNumPathsPerSrc);

    auto* syntheticCheck = new QCheckBox(tr("Enable synthetic array"), solverGroup);
    syntheticCheck->setChecked(simSettings_.syntheticArray);
    auto* mergeShapesCheck = new QCheckBox(tr("Merge scene shapes when loading"), solverGroup);
    mergeShapesCheck->setChecked(simSettings_.mergeShapes);
    mergeShapesCheck->setToolTip(tr("Maps to sionna.rt.load_scene(..., merge_shapes=...)."));

    solverForm->addRow(tr("Carrier frequency (Hz)"), fcSpin);
    solverForm->addRow(tr("Mitsuba variant"), miVariantEdit);
    solverForm->addRow(tr("Max depth"), maxDepthSpin);
    solverForm->addRow(tr("Samples per source"), samplesSpin);
    solverForm->addRow(tr("Max paths per source"), maxPathsSpin);
    solverForm->addRow(QString(), syntheticCheck);
    solverForm->addRow(QString(), mergeShapesCheck);

    auto* arraysGroup = new QGroupBox(tr("Antenna arrays"), content);
    auto* arraysLayout = new QHBoxLayout(arraysGroup);
    arraysLayout->setSpacing(12);

    auto buildArrayEditors = [&](const QString& title,
                                 int rowsValue,
                                 int colsValue,
                                 double vSpacingValue,
                                 double hSpacingValue,
                                 const QString& patternValue,
                                 const QString& polarizationValue) {
        ArrayEditors editors;
        auto* group = new QGroupBox(title, arraysGroup);
        auto* form = new QFormLayout(group);

        editors.pattern = makeEditableCombo(QStringList{QStringLiteral("iso"), QStringLiteral("dipole"), QStringLiteral("tr38901")}, patternValue);
        editors.polarization = makeEditableCombo(QStringList{QStringLiteral("V"), QStringLiteral("H"), QStringLiteral("VH"), QStringLiteral("cross")}, polarizationValue);

        editors.rows = new QSpinBox(group);
        editors.rows->setRange(1, 64);
        editors.rows->setValue(rowsValue);

        editors.cols = new QSpinBox(group);
        editors.cols->setRange(1, 64);
        editors.cols->setValue(colsValue);

        auto* dimsWidget = new QWidget(group);
        auto* dimsLayout = new QHBoxLayout(dimsWidget);
        dimsLayout->setContentsMargins(0, 0, 0, 0);
        dimsLayout->addWidget(editors.rows);
        dimsLayout->addWidget(editors.cols);

        editors.verticalSpacing = new QDoubleSpinBox(group);
        editors.verticalSpacing->setRange(0.0, 100.0);
        editors.verticalSpacing->setDecimals(3);
        editors.verticalSpacing->setSingleStep(0.05);
        editors.verticalSpacing->setValue(vSpacingValue);

        editors.horizontalSpacing = new QDoubleSpinBox(group);
        editors.horizontalSpacing->setRange(0.0, 100.0);
        editors.horizontalSpacing->setDecimals(3);
        editors.horizontalSpacing->setSingleStep(0.05);
        editors.horizontalSpacing->setValue(hSpacingValue);

        auto* spacingWidget = new QWidget(group);
        auto* spacingLayout = new QHBoxLayout(spacingWidget);
        spacingLayout->setContentsMargins(0, 0, 0, 0);
        spacingLayout->addWidget(editors.verticalSpacing);
        spacingLayout->addWidget(editors.horizontalSpacing);

        form->addRow(tr("Pattern"), editors.pattern);
        form->addRow(tr("Polarization"), editors.polarization);
        form->addRow(tr("Rows / cols"), dimsWidget);
        form->addRow(tr("V/H spacing"), spacingWidget);

        arraysLayout->addWidget(group, 1);
        return editors;
    };

    const ArrayEditors txEditors = buildArrayEditors(
        tr("TX array"),
        simSettings_.txArrayNumRows,
        simSettings_.txArrayNumCols,
        simSettings_.txArrayVerticalSpacing,
        simSettings_.txArrayHorizontalSpacing,
        simSettings_.txArrayPattern,
        simSettings_.txArrayPolarization);
    const ArrayEditors rxEditors = buildArrayEditors(
        tr("RX array"),
        simSettings_.rxArrayNumRows,
        simSettings_.rxArrayNumCols,
        simSettings_.rxArrayVerticalSpacing,
        simSettings_.rxArrayHorizontalSpacing,
        simSettings_.rxArrayPattern,
        simSettings_.rxArrayPolarization);

    auto* propagationGroup = new QGroupBox(tr("Propagation"), content);
    auto* propagationLayout = new QGridLayout(propagationGroup);

    auto* losCheck = new QCheckBox(tr("Line of sight"), propagationGroup);
    losCheck->setChecked(simSettings_.los);
    auto* specularCheck = new QCheckBox(tr("Specular reflection"), propagationGroup);
    specularCheck->setChecked(simSettings_.specularReflection);
    auto* diffuseCheck = new QCheckBox(tr("Diffuse reflection"), propagationGroup);
    diffuseCheck->setChecked(simSettings_.diffuseReflection);
    auto* refractionCheck = new QCheckBox(tr("Refraction"), propagationGroup);
    refractionCheck->setChecked(simSettings_.refraction);
    auto* diffractionCheck = new QCheckBox(tr("Diffraction"), propagationGroup);
    diffractionCheck->setChecked(simSettings_.diffraction);
    auto* edgeDiffractionCheck = new QCheckBox(tr("Edge diffraction"), propagationGroup);
    edgeDiffractionCheck->setChecked(simSettings_.edgeDiffraction);
    auto* litRegionCheck = new QCheckBox(tr("Diffraction lit region"), propagationGroup);
    litRegionCheck->setChecked(simSettings_.diffractionLitRegion);

    propagationLayout->addWidget(losCheck, 0, 0);
    propagationLayout->addWidget(specularCheck, 0, 1);
    propagationLayout->addWidget(diffuseCheck, 1, 0);
    propagationLayout->addWidget(refractionCheck, 1, 1);
    propagationLayout->addWidget(diffractionCheck, 2, 0);
    propagationLayout->addWidget(edgeDiffractionCheck, 2, 1);
    propagationLayout->addWidget(litRegionCheck, 3, 1);

    auto updateDiffractionDependentWidgets = [=]() {
        const bool enabled = diffractionCheck->isChecked();
        if (!enabled) {
            edgeDiffractionCheck->setChecked(false);
            litRegionCheck->setChecked(false);
        }
        edgeDiffractionCheck->setEnabled(enabled);
        litRegionCheck->setEnabled(enabled);
    };
    updateDiffractionDependentWidgets();
    connect(diffractionCheck, &QCheckBox::toggled, &dialog, updateDiffractionDependentWidgets);

    auto* runtimeGroup = new QGroupBox(tr("Runtime and output"), content);
    auto* runtimeForm = new QFormLayout(runtimeGroup);

    auto* simHzSpin = new QDoubleSpinBox(runtimeGroup);
    simHzSpin->setRange(0.1, 100.0);
    simHzSpin->setDecimals(2);
    simHzSpin->setValue(simSettings_.simHz);

    auto* staleSpin = new QDoubleSpinBox(runtimeGroup);
    staleSpin->setRange(0.01, 30.0);
    staleSpin->setDecimals(2);
    staleSpin->setValue(simSettings_.maxPoseStalenessS);

    auto* frameIdEdit = new QLineEdit(simSettings_.rfFrameId, runtimeGroup);
    frameIdEdit->setToolTip(tr("Used only as a fallback when wireless outputs are configured to use the ROS frame but the incoming pose / odom message does not provide a valid header.frame_id."));
    auto* csvCheck = new QCheckBox(tr("Enable CSV output"), runtimeGroup);
    csvCheck->setChecked(simSettings_.csvEnabled);
    auto* csvPathEdit = new QLineEdit(simSettings_.csvPath, runtimeGroup);
    auto* browseCsvButton = new QPushButton(tr("Browse..."), runtimeGroup);
    auto* csvRowWidget = new QWidget(runtimeGroup);
    auto* csvRow = new QHBoxLayout(csvRowWidget);
    csvRow->setContentsMargins(0, 0, 0, 0);
    csvRow->addWidget(csvPathEdit);
    csvRow->addWidget(browseCsvButton);

    runtimeForm->addRow(tr("Simulation rate (Hz)"), simHzSpin);
    runtimeForm->addRow(tr("Max pose staleness (s)"), staleSpin);
    runtimeForm->addRow(tr("Fallback RF frame id"), frameIdEdit);
    runtimeForm->addRow(QString(), csvCheck);
    runtimeForm->addRow(tr("CSV path"), csvRowWidget);

    auto* sysGroup = new QGroupBox(tr("SYS over RT"), content);
    auto* sysForm = new QFormLayout(sysGroup);

    auto* enableSysCheck = new QCheckBox(tr("Enable Sionna SYS link adaptation on top of RT"), sysGroup);
    enableSysCheck->setChecked(simSettings_.enableSysIntegration);

    auto* sysNumSubcarriersSpin = new QSpinBox(sysGroup);
    sysNumSubcarriersSpin->setRange(8, 4096);
    sysNumSubcarriersSpin->setSingleStep(8);
    sysNumSubcarriersSpin->setValue(simSettings_.sysNumSubcarriers);

    auto* sysSubcarrierSpacingSpin = new QDoubleSpinBox(sysGroup);
    sysSubcarrierSpacingSpin->setRange(1000.0, 1.0e6);
    sysSubcarrierSpacingSpin->setDecimals(1);
    sysSubcarrierSpacingSpin->setSingleStep(15000.0);
    sysSubcarrierSpacingSpin->setValue(simSettings_.sysSubcarrierSpacingHz);
    sysSubcarrierSpacingSpin->setSuffix(tr(" Hz"));

    auto* sysNumSymbolsSpin = new QSpinBox(sysGroup);
    sysNumSymbolsSpin->setRange(1, 64);
    sysNumSymbolsSpin->setValue(simSettings_.sysNumOfdmSymbols);

    auto* sysTemperatureSpin = new QDoubleSpinBox(sysGroup);
    sysTemperatureSpin->setRange(50.0, 500.0);
    sysTemperatureSpin->setDecimals(1);
    sysTemperatureSpin->setValue(simSettings_.sysTemperatureK);
    sysTemperatureSpin->setSuffix(tr(" K"));

    auto* sysBlerTargetSpin = new QDoubleSpinBox(sysGroup);
    sysBlerTargetSpin->setRange(0.001, 0.999);
    sysBlerTargetSpin->setDecimals(3);
    sysBlerTargetSpin->setSingleStep(0.01);
    sysBlerTargetSpin->setValue(simSettings_.sysBlerTarget);

    auto* sysMcsTableSpin = new QSpinBox(sysGroup);
    sysMcsTableSpin->setRange(0, 4);
    sysMcsTableSpin->setValue(simSettings_.sysMcsTableIndex);

    auto* sysTxPowerSpin = new QDoubleSpinBox(sysGroup);
    sysTxPowerSpin->setRange(-50.0, 80.0);
    sysTxPowerSpin->setDecimals(2);
    sysTxPowerSpin->setValue(simSettings_.sysBsTxPowerDbm);
    sysTxPowerSpin->setSuffix(tr(" dBm"));

    sysForm->addRow(QString(), enableSysCheck);
    sysForm->addRow(tr("Subcarriers"), sysNumSubcarriersSpin);
    sysForm->addRow(tr("Subcarrier spacing"), sysSubcarrierSpacingSpin);
    sysForm->addRow(tr("OFDM symbols per slot"), sysNumSymbolsSpin);
    sysForm->addRow(tr("Noise temperature"), sysTemperatureSpin);
    sysForm->addRow(tr("Target BLER"), sysBlerTargetSpin);
    sysForm->addRow(tr("MCS table index"), sysMcsTableSpin);
    sysForm->addRow(tr("BS TX power"), sysTxPowerSpin);

    auto updateSysWidgets = [=]() {
        const bool enabled = enableSysCheck->isChecked();
        sysNumSubcarriersSpin->setEnabled(enabled);
        sysSubcarrierSpacingSpin->setEnabled(enabled);
        sysNumSymbolsSpin->setEnabled(enabled);
        sysTemperatureSpin->setEnabled(enabled);
        sysBlerTargetSpin->setEnabled(enabled);
        sysMcsTableSpin->setEnabled(enabled);
        sysTxPowerSpin->setEnabled(enabled);
    };
    updateSysWidgets();
    connect(enableSysCheck, &QCheckBox::toggled, &dialog, updateSysWidgets);

    auto* beamGroup = new QGroupBox(tr("Beamforming & Beam Selection"), content);
    auto* beamForm = new QFormLayout(beamGroup);

    auto* enableBeamformingCheck = new QCheckBox(tr("Enable TX beam codebook generation and selection"), beamGroup);
    enableBeamformingCheck->setChecked(simSettings_.enableBeamforming);

    auto* beamModeCombo = makeEditableCombo(
        QStringList{QStringLiteral("off"), QStringLiteral("exhaustive_sweep"), QStringLiteral("manual_index"), QStringLiteral("learned_selector")},
        beamSelectionModeToUi(simSettings_.beamSelectionMode));
    auto* beamCodebookCombo = makeEditableCombo(
        QStringList{QStringLiteral("auto"), QStringLiteral("dft_ula"), QStringLiteral("dft_ura")},
        simSettings_.beamCodebookType);
    auto* beamCodebookNumBeamsCombo = new QComboBox(beamGroup);
    beamCodebookNumBeamsCombo->addItem(QStringLiteral("8"), 8);
    beamCodebookNumBeamsCombo->addItem(QStringLiteral("64"), 64);
    beamCodebookNumBeamsCombo->addItem(QStringLiteral("128"), 128);
    int beamCodebookNumBeamsIndex = beamCodebookNumBeamsCombo->findData(simSettings_.beamCodebookNumBeams);
    if (beamCodebookNumBeamsIndex < 0) {
        beamCodebookNumBeamsIndex = beamCodebookNumBeamsCombo->findData(8);
    }
    beamCodebookNumBeamsCombo->setCurrentIndex(std::max(0, beamCodebookNumBeamsIndex));

    auto* beamOversamplingVSpin = new QSpinBox(beamGroup);
    beamOversamplingVSpin->setRange(1, 16);
    beamOversamplingVSpin->setValue(simSettings_.beamOversamplingV);

    auto* beamOversamplingHSpin = new QSpinBox(beamGroup);
    beamOversamplingHSpin->setRange(1, 16);
    beamOversamplingHSpin->setValue(simSettings_.beamOversamplingH);

    auto* beamOversamplingWidget = new QWidget(beamGroup);
    auto* beamOversamplingLayout = new QHBoxLayout(beamOversamplingWidget);
    beamOversamplingLayout->setContentsMargins(0, 0, 0, 0);
    beamOversamplingLayout->addWidget(beamOversamplingVSpin);
    beamOversamplingLayout->addWidget(beamOversamplingHSpin);

    auto* beamManualIndexSpin = new QSpinBox(beamGroup);
    beamManualIndexSpin->setRange(0, 65535);
    beamManualIndexSpin->setValue(std::max(0, simSettings_.beamManualIndex));

    auto* beamNormalizePowerCheck = new QCheckBox(tr("Normalize each codebook beam to unit transmit power"), beamGroup);
    beamNormalizePowerCheck->setChecked(simSettings_.beamNormalizePower);

    auto* beamCodebookFileEdit = new QLineEdit(simSettings_.beamCodebookFilePath, beamGroup);
    beamCodebookFileEdit->setPlaceholderText(tr("Leave empty to use the Sionna-generated codebook (.json)."));
    auto* beamCodebookBrowseButton = new QPushButton(tr("Browse..."), beamGroup);
    auto* beamCodebookClearButton = new QPushButton(tr("Clear"), beamGroup);
    auto* beamCodebookWidget = new QWidget(beamGroup);
    auto* beamCodebookLayout = new QHBoxLayout(beamCodebookWidget);
    beamCodebookLayout->setContentsMargins(0, 0, 0, 0);
    beamCodebookLayout->addWidget(beamCodebookFileEdit, 1);
    beamCodebookLayout->addWidget(beamCodebookBrowseButton);
    beamCodebookLayout->addWidget(beamCodebookClearButton);

    auto* beamExportCodebookButton = new QPushButton(tr("Export current generated codebook..."), beamGroup);
    auto* beamCodebookHintLabel = new QLabel(tr("Leave the custom codebook file empty to let Sionna generate the codebook from the settings above. If a local JSON codebook file is selected here, it overrides the generated codebook during simulation. Clearing this field switches back to the Sionna-generated codebook."), beamGroup);
    beamCodebookHintLabel->setWordWrap(true);

    QDialog* beamDialogPtr = &dialog;
    connect(beamCodebookBrowseButton, &QPushButton::clicked, &dialog,
            [beamCodebookFileEdit, beamDialogPtr, this]() {
        const QString filePath = QFileDialog::getOpenFileName(
            beamDialogPtr,
            tr("Select custom beam codebook JSON"),
            beamCodebookFileEdit->text().trimmed().isEmpty() ? QDir::homePath() : QFileInfo(beamCodebookFileEdit->text()).absolutePath(),
            tr("Codebook JSON (*.json);;All files (*)"));
        if (!filePath.isEmpty()) {
            beamCodebookFileEdit->setText(filePath);
        }
    });
    connect(beamCodebookClearButton, &QPushButton::clicked, &dialog, [beamCodebookFileEdit]() {
        beamCodebookFileEdit->clear();
    });
    connect(beamExportCodebookButton, &QPushButton::clicked, &dialog,
            [this, beamDialogPtr, pythonPathEdit, beamCodebookCombo, beamCodebookNumBeamsCombo, beamOversamplingVSpin, beamOversamplingHSpin,
             beamNormalizePowerCheck, txEditors]() {
        const QString pythonProgram = pythonPathEdit->text().trimmed().isEmpty() ? QStringLiteral("python3") : pythonPathEdit->text().trimmed();
        const QString helperScriptPath = findBundledScript(QStringLiteral("scripts/export_sionna_codebook.py"));
        if (helperScriptPath.isEmpty() || !QFileInfo::exists(helperScriptPath)) {
            QMessageBox::warning(this, tr("Export Codebook"), tr("Codebook export helper script not found:\n%1").arg(helperScriptPath));
            return;
        }
        QString codebookType = beamCodebookCombo->currentText().trimmed();
        if (codebookType.isEmpty()) {
            codebookType = QStringLiteral("auto");
        }
        QString safeType = codebookType.toLower();
        safeType.replace(QRegularExpression(QStringLiteral("[^a-z0-9_]+")), QStringLiteral("_"));
        const QString defaultName = QStringLiteral("sionna_codebook_%1_nb%2_r%3_c%4_ov%5_oh%6.json")
                                        .arg(safeType)
                                        .arg(beamCodebookNumBeamsCombo->currentData().toInt())
                                        .arg(txEditors.rows->value())
                                        .arg(txEditors.cols->value())
                                        .arg(beamOversamplingVSpin->value())
                                        .arg(beamOversamplingHSpin->value());
        const QString defaultPath = QDir::home().filePath(defaultName);
        const QString filePath = QFileDialog::getSaveFileName(
            beamDialogPtr,
            tr("Export current generated codebook"),
            defaultPath,
            tr("Codebook JSON (*.json);;All files (*)"));
        if (filePath.isEmpty()) {
            return;
        }
        QString outputPath = filePath;
        if (!outputPath.toLower().endsWith(QStringLiteral(".json"))) {
            outputPath += QStringLiteral(".json");
        }
        QStringList arguments;
        arguments << helperScriptPath
                  << QStringLiteral("--output") << outputPath
                  << QStringLiteral("--tx-array-num-rows") << QString::number(txEditors.rows->value())
                  << QStringLiteral("--tx-array-num-cols") << QString::number(txEditors.cols->value())
                  << QStringLiteral("--beam-codebook-type") << codebookType
                  << QStringLiteral("--beam-codebook-num-beams") << QString::number(beamCodebookNumBeamsCombo->currentData().toInt())
                  << QStringLiteral("--beam-oversampling-v") << QString::number(beamOversamplingVSpin->value())
                  << QStringLiteral("--beam-oversampling-h") << QString::number(beamOversamplingHSpin->value())
                  << QStringLiteral("--beam-normalize-power") << (beamNormalizePowerCheck->isChecked() ? QStringLiteral("true") : QStringLiteral("false"));

        QProcess process;
        process.setProgram(pythonProgram);
        process.setArguments(arguments);
        process.setProcessChannelMode(QProcess::MergedChannels);

        QProgressDialog progress(tr("Exporting Sionna-generated codebook..."), QString(), 0, 0, beamDialogPtr);
        progress.setWindowTitle(tr("Export Codebook"));
        progress.setWindowModality(Qt::WindowModal);
        progress.setCancelButton(nullptr);
        progress.setMinimumDuration(0);
        progress.show();
        QApplication::setOverrideCursor(Qt::WaitCursor);
        process.start();
        if (!process.waitForStarted(5000)) {
            QApplication::restoreOverrideCursor();
            progress.close();
            QMessageBox::warning(this, tr("Export Codebook"), tr("Failed to start Python helper:\n%1").arg(pythonProgram));
            return;
        }
        while (process.state() != QProcess::NotRunning) {
            process.waitForFinished(200);
            QApplication::processEvents();
        }
        QApplication::restoreOverrideCursor();
        progress.close();
        const QString outputText = QString::fromLocal8Bit(process.readAllStandardOutput()).trimmed();
        if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
            QMessageBox::warning(this, tr("Export Codebook"),
                                 tr("Failed to export the generated codebook. Exit code: %1\n\n%2")
                                     .arg(process.exitCode())
                                     .arg(outputText.isEmpty() ? tr("No output captured.") : outputText));
            return;
        }
        QMessageBox::information(this, tr("Export Codebook"),
                                 tr("The current Sionna-generated codebook has been exported to:\n%1").arg(outputPath));
    });

    auto* beamCheckpointEdit = new QLineEdit(simSettings_.beamModelCheckpointPath, beamGroup);
    beamCheckpointEdit->setPlaceholderText(tr("Path to a beam-selection PyTorch checkpoint (.pt/.pth/.ckpt)"));
    auto* beamCheckpointBrowseButton = new QPushButton(tr("Browse..."), beamGroup);
    auto* beamCheckpointWidget = new QWidget(beamGroup);
    auto* beamCheckpointLayout = new QHBoxLayout(beamCheckpointWidget);
    beamCheckpointLayout->setContentsMargins(0, 0, 0, 0);
    beamCheckpointLayout->addWidget(beamCheckpointEdit, 1);
    beamCheckpointLayout->addWidget(beamCheckpointBrowseButton);
    connect(beamCheckpointBrowseButton, &QPushButton::clicked, &dialog,
            [beamCheckpointEdit, beamDialogPtr, this]() {
        const QString filePath = QFileDialog::getOpenFileName(
            beamDialogPtr,
            tr("Select beam selection checkpoint"),
            beamCheckpointEdit->text(),
            tr("PyTorch checkpoints (*.pt *.pth *.ckpt);;All files (*)"));
        if (!filePath.isEmpty()) {
            beamCheckpointEdit->setText(filePath);
        }
    });

    auto* beamFeatureModeCombo = makeEditableCombo(
        QStringList{
            QStringLiteral("geom_vel_path13"),
            QStringLiteral("pos_bbox4"),
            QStringLiteral("geom4"),
            QStringLiteral("rel_xyzd"),
            QStringLiteral("rel_xyazel"),
        },
        beamFeatureModeToUi(simSettings_.beamFeatureMode));

    auto* beamTopKSpin = new QSpinBox(beamGroup);
    beamTopKSpin->setRange(1, 16);
    beamTopKSpin->setValue(std::max(1, simSettings_.beamTopK));

    auto* beamDatasetExportCheck = new QCheckBox(tr("Export oracle beam samples for training"), beamGroup);
    beamDatasetExportCheck->setChecked(simSettings_.beamExportTrainingDataset);

    auto* beamDatasetEdit = new QLineEdit(simSettings_.beamTrainingDatasetPath, beamGroup);
    beamDatasetEdit->setPlaceholderText(tr("CSV file to append beam training samples"));
    auto* beamDatasetBrowseButton = new QPushButton(tr("Browse..."), beamGroup);
    auto* beamDatasetWidget = new QWidget(beamGroup);
    auto* beamDatasetLayout = new QHBoxLayout(beamDatasetWidget);
    beamDatasetLayout->setContentsMargins(0, 0, 0, 0);
    beamDatasetLayout->addWidget(beamDatasetEdit, 1);
    beamDatasetLayout->addWidget(beamDatasetBrowseButton);
    connect(beamDatasetBrowseButton, &QPushButton::clicked, &dialog,
            [beamDatasetEdit, beamDialogPtr, this]() {
        const QString filePath = QFileDialog::getSaveFileName(
            beamDialogPtr,
            tr("Select beam training dataset CSV"),
            beamDatasetEdit->text().trimmed().isEmpty() ? QDir::homePath() : beamDatasetEdit->text(),
            tr("CSV files (*.csv);;All files (*)"));
        if (!filePath.isEmpty()) {
            beamDatasetEdit->setText(filePath);
        }
    });

    auto* beamTrainDatasetsList = new QListWidget(beamGroup);
    beamTrainDatasetsList->setSelectionMode(QAbstractItemView::SingleSelection);
    beamTrainDatasetsList->setMinimumHeight(84);
    beamTrainDatasetsList->setToolTip(tr("This list stores the training CSVs explicitly selected for the beam trainer. If the list is empty, training falls back to the Dataset export CSV above."));
    setBeamTrainingDatasetPathsOnListWidget(beamTrainDatasetsList, splitBeamTrainingDatasetPaths(simSettings_.beamTrainingInputPaths));
    auto* beamTrainDatasetsAddButton = new QPushButton(tr("Add CSV..."), beamGroup);
    auto* beamTrainDatasetsDeleteButton = new QPushButton(tr("Delete CSV"), beamGroup);
    auto* beamTrainDatasetsAddExportButton = new QPushButton(tr("Add export CSV"), beamGroup);
    auto* beamTrainDatasetsButtons = new QWidget(beamGroup);
    auto* beamTrainDatasetsButtonsLayout = new QVBoxLayout(beamTrainDatasetsButtons);
    beamTrainDatasetsButtonsLayout->setContentsMargins(0, 0, 0, 0);
    beamTrainDatasetsButtonsLayout->setSpacing(6);
    beamTrainDatasetsButtonsLayout->addWidget(beamTrainDatasetsAddButton);
    beamTrainDatasetsButtonsLayout->addWidget(beamTrainDatasetsDeleteButton);
    beamTrainDatasetsButtonsLayout->addWidget(beamTrainDatasetsAddExportButton);
    beamTrainDatasetsButtonsLayout->addStretch(1);
    auto* beamTrainDatasetsWidget = new QWidget(beamGroup);
    auto* beamTrainDatasetsLayout = new QHBoxLayout(beamTrainDatasetsWidget);
    beamTrainDatasetsLayout->setContentsMargins(0, 0, 0, 0);
    beamTrainDatasetsLayout->setSpacing(6);
    beamTrainDatasetsLayout->addWidget(beamTrainDatasetsList, 1);
    beamTrainDatasetsLayout->addWidget(beamTrainDatasetsButtons);
    auto updateBeamTrainingDatasetButtons = [beamTrainDatasetsList, beamTrainDatasetsDeleteButton]() {
        beamTrainDatasetsDeleteButton->setEnabled(beamTrainDatasetsList->currentRow() >= 0);
    };
    updateBeamTrainingDatasetButtons();
    connect(beamTrainDatasetsList, &QListWidget::currentRowChanged, &dialog, updateBeamTrainingDatasetButtons);
    connect(beamTrainDatasetsAddButton, &QPushButton::clicked, &dialog,
            [beamTrainDatasetsList, beamDatasetEdit, beamDialogPtr, this]() {
        const QString initialPath = beamTrainDatasetsList->currentItem()
                                        ? beamTrainDatasetsList->currentItem()->text().trimmed()
                                        : beamDatasetEdit->text().trimmed();
        const QString filePath = QFileDialog::getOpenFileName(
            beamDialogPtr,
            tr("Select one beam training CSV file"),
            initialPath.isEmpty() ? QDir::homePath() : initialPath,
            tr("CSV files (*.csv);;All files (*)"));
        if (!filePath.isEmpty()) {
            addBeamTrainingDatasetPathToListWidget(beamTrainDatasetsList, filePath);
        }
    });
    connect(beamTrainDatasetsDeleteButton, &QPushButton::clicked, &dialog,
            [beamTrainDatasetsList]() {
        const int currentRow = beamTrainDatasetsList->currentRow();
        if (currentRow < 0) {
            return;
        }
        delete beamTrainDatasetsList->takeItem(currentRow);
    });
    connect(beamTrainDatasetsAddExportButton, &QPushButton::clicked, &dialog,
            [beamTrainDatasetsList, beamDatasetEdit]() {
        const QString fallbackPath = beamDatasetEdit->text().trimmed();
        if (!fallbackPath.isEmpty()) {
            addBeamTrainingDatasetPathToListWidget(beamTrainDatasetsList, fallbackPath);
        }
    });

    auto* beamResetDatasetCheck = new QCheckBox(tr("Reset beam dataset file when the simulator starts"), beamGroup);
    beamResetDatasetCheck->setChecked(simSettings_.beamTrainingResetDatasetOnStart);

    auto* beamTrainOutputEdit = new QLineEdit(simSettings_.beamTrainingOutputCheckpointPath, beamGroup);
    beamTrainOutputEdit->setPlaceholderText(tr("Where to save the trained beam-selection checkpoint"));
    auto* beamTrainOutputBrowseButton = new QPushButton(tr("Browse..."), beamGroup);
    auto* beamTrainOutputWidget = new QWidget(beamGroup);
    auto* beamTrainOutputLayout = new QHBoxLayout(beamTrainOutputWidget);
    beamTrainOutputLayout->setContentsMargins(0, 0, 0, 0);
    beamTrainOutputLayout->addWidget(beamTrainOutputEdit, 1);
    beamTrainOutputLayout->addWidget(beamTrainOutputBrowseButton);
    connect(beamTrainOutputBrowseButton, &QPushButton::clicked, &dialog,
            [beamTrainOutputEdit, beamDialogPtr, this]() {
        const QString filePath = QFileDialog::getSaveFileName(
            beamDialogPtr,
            tr("Select beam-selection checkpoint output path"),
            beamTrainOutputEdit->text().trimmed().isEmpty() ? QDir::homePath() : beamTrainOutputEdit->text(),
            tr("PyTorch checkpoints (*.pt *.pth *.ckpt);;All files (*)"));
        if (!filePath.isEmpty()) {
            beamTrainOutputEdit->setText(filePath);
        }
    });

    auto* beamTrainEpochsSpin = new QSpinBox(beamGroup);
    beamTrainEpochsSpin->setRange(1, 10000);
    beamTrainEpochsSpin->setValue(std::max(1, simSettings_.beamTrainingEpochs));

    auto* beamTrainBatchSpin = new QSpinBox(beamGroup);
    beamTrainBatchSpin->setRange(1, 65536);
    beamTrainBatchSpin->setValue(std::max(1, simSettings_.beamTrainingBatchSize));

    auto* beamTrainHiddenDimSpin = new QSpinBox(beamGroup);
    beamTrainHiddenDimSpin->setRange(8, 8192);
    beamTrainHiddenDimSpin->setValue(std::max(8, simSettings_.beamTrainingHiddenDim));

    auto* beamTrainLrSpin = new QDoubleSpinBox(beamGroup);
    beamTrainLrSpin->setDecimals(6);
    beamTrainLrSpin->setRange(1.0e-6, 1.0);
    beamTrainLrSpin->setSingleStep(1.0e-4);
    beamTrainLrSpin->setValue(std::max(1.0e-6, simSettings_.beamTrainingLearningRate));

    auto* beamTrainValSplitSpin = new QDoubleSpinBox(beamGroup);
    beamTrainValSplitSpin->setDecimals(3);
    beamTrainValSplitSpin->setRange(0.0, 0.9);
    beamTrainValSplitSpin->setSingleStep(0.05);
    const double beamTrainValSplit = simSettings_.beamTrainingValidationSplit < 0.0
                                        ? 0.0
                                        : (simSettings_.beamTrainingValidationSplit > 0.9
                                               ? 0.9
                                               : simSettings_.beamTrainingValidationSplit);
    beamTrainValSplitSpin->setValue(beamTrainValSplit);

    auto* beamTrainNowButton = new QPushButton(tr("Train beam-selection model from dataset now..."), beamGroup);
    connect(beamTrainNowButton, &QPushButton::clicked, &dialog,
            [this, beamDialogPtr, pythonPathEdit, beamDatasetEdit, beamTrainDatasetsList, beamTrainOutputEdit, beamFeatureModeCombo,
             beamTopKSpin, beamTrainEpochsSpin, beamTrainBatchSpin, beamTrainHiddenDimSpin,
             beamTrainLrSpin, beamTrainValSplitSpin, beamCheckpointEdit, beamModeCombo]() {
        const QString datasetPath = beamDatasetEdit->text().trimmed();
        const QStringList explicitTrainDatasetPaths = beamTrainingDatasetPathsFromListWidget(beamTrainDatasetsList);
        const QStringList trainDatasetPaths = explicitTrainDatasetPaths.isEmpty()
                                                 ? splitBeamTrainingDatasetPaths(datasetPath)
                                                 : explicitTrainDatasetPaths;
        const QString outputPath = beamTrainOutputEdit->text().trimmed();
        const QString pythonProgram = pythonPathEdit->text().trimmed().isEmpty() ? QStringLiteral("python3") : pythonPathEdit->text().trimmed();
        const QString trainScriptPath = findBundledScript(QStringLiteral("scripts/train_deepsense_beam_model.py"));
        if (trainDatasetPaths.isEmpty()) {
            QMessageBox::warning(this, tr("Beam Training"), tr("Please choose at least one beam training dataset CSV first."));
            return;
        }
        QStringList missingDatasetPaths;
        for (const QString& csvPath : trainDatasetPaths) {
            if (!QFileInfo::exists(csvPath)) {
                missingDatasetPaths << csvPath;
            }
        }
        if (!missingDatasetPaths.isEmpty()) {
            QMessageBox::warning(this, tr("Beam Training"), tr("The following dataset file(s) do not exist yet:\n%1\n\nRun the simulator with dataset export enabled to collect oracle beam samples, or remove the missing paths before training.").arg(missingDatasetPaths.join(QStringLiteral("\n"))));
            return;
        }
        if (outputPath.isEmpty()) {
            QMessageBox::warning(this, tr("Beam Training"), tr("Please choose where to save the trained checkpoint."));
            return;
        }
        if (trainScriptPath.isEmpty() || !QFileInfo::exists(trainScriptPath)) {
            QMessageBox::warning(this, tr("Beam Training"), tr("Training script not found:\n%1").arg(sanitizeBeamTerminologyForUi(trainScriptPath)));
            return;
        }

        QStringList arguments;
        arguments << trainScriptPath;
        for (const QString& csvPath : trainDatasetPaths) {
            arguments << QStringLiteral("--dataset") << csvPath;
        }
        arguments << QStringLiteral("--output-checkpoint") << outputPath
                  << QStringLiteral("--feature-mode") << (beamFeatureModeCombo->currentText().trimmed().isEmpty() ? QStringLiteral("geom_vel_path13") : beamFeatureModeFromUi(beamFeatureModeCombo->currentText()))
                  << QStringLiteral("--epochs") << QString::number(beamTrainEpochsSpin->value())
                  << QStringLiteral("--batch-size") << QString::number(beamTrainBatchSpin->value())
                  << QStringLiteral("--hidden-dim") << QString::number(beamTrainHiddenDimSpin->value())
                  << QStringLiteral("--learning-rate") << QString::number(beamTrainLrSpin->value(), 'g', 12)
                  << QStringLiteral("--val-split") << QString::number(beamTrainValSplitSpin->value(), 'g', 8)
                  << QStringLiteral("--top-k") << QString::number(beamTopKSpin->value());

        appendSimulatorLog(QStringLiteral("[beam-train] Python: %1").arg(pythonProgram));
        appendSimulatorLog(QStringLiteral("[beam-train] Script: %1").arg(trainScriptPath));
        appendSimulatorLog(QStringLiteral("[beam-train] Datasets (%1): %2").arg(trainDatasetPaths.size()).arg(trainDatasetPaths.join(QStringLiteral("; "))));
        appendSimulatorLog(QStringLiteral("[beam-train] Output checkpoint: %1").arg(outputPath));

        QProcess process;
        process.setProgram(pythonProgram);
        process.setArguments(arguments);
        process.setProcessChannelMode(QProcess::MergedChannels);

        QProgressDialog progress(tr("Training beam-selection model..."), QString(), 0, 0, beamDialogPtr);
        progress.setWindowTitle(tr("Beam Training"));
        progress.setWindowModality(Qt::WindowModal);
        progress.setCancelButton(nullptr);
        progress.setMinimumDuration(0);
        progress.show();
        QApplication::setOverrideCursor(Qt::WaitCursor);
        process.start();
        if (!process.waitForStarted(5000)) {
            QApplication::restoreOverrideCursor();
            progress.close();
            QMessageBox::warning(this, tr("Beam Training"), tr("Failed to start the training process."));
            return;
        }
        while (process.state() != QProcess::NotRunning) {
            process.waitForFinished(200);
            QApplication::processEvents();
        }
        QApplication::restoreOverrideCursor();
        const QString outputText = QString::fromLocal8Bit(process.readAllStandardOutput()).trimmed();
        if (!outputText.isEmpty()) {
            const QStringList lines = outputText.split(QRegularExpression(QStringLiteral("\\r?\\n")), QString::SkipEmptyParts);
            for (const QString& line : lines) {
                appendSimulatorLog(QStringLiteral("[beam-train] %1").arg(line));
            }
        }
        if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
            progress.close();
            QMessageBox::warning(this, tr("Beam Training"),
                                 tr("Training failed. Exit code: %1\n\n%2")
                                     .arg(process.exitCode())
                                     .arg(outputText.isEmpty() ? tr("No output captured.") : sanitizeBeamTerminologyForUi(outputText)));
            return;
        }
        beamCheckpointEdit->setText(outputPath);
        beamModeCombo->setCurrentText(QStringLiteral("learned_selector"));
        progress.close();
        QMessageBox::information(this, tr("Beam Training"),
                                 tr("Training finished successfully.\n\nCheckpoint saved to:\n%1\n\nThe checkpoint field has been updated and the selection mode has been switched to learned_selector.")
                                     .arg(outputPath));
    });

    auto* beamHintLabel = new QLabel(tr("Beam selection supports Sionna codebook generation, oracle sweep/manual modes, optional CSV export of oracle-labeled beam samples from your own scene, and an offline trainer for a learned beam selector. After training, point the predictor to the new checkpoint and run learned_selector mode to apply it live. The safest workflow is: exhaustive_sweep + export dataset -> train beam-selection model -> learned_selector."), beamGroup);
    beamHintLabel->setWordWrap(true);

    beamForm->addRow(QString(), enableBeamformingCheck);
    beamForm->addRow(tr("Selection mode"), beamModeCombo);
    beamForm->addRow(tr("Codebook type"), beamCodebookCombo);
    beamForm->addRow(tr("Generated beam count"), beamCodebookNumBeamsCombo);
    beamForm->addRow(tr("Oversampling (V / H)"), beamOversamplingWidget);
    beamForm->addRow(tr("Manual beam index"), beamManualIndexSpin);
    beamForm->addRow(QString(), beamNormalizePowerCheck);
    beamForm->addRow(tr("Custom codebook file"), beamCodebookWidget);
    beamForm->addRow(QString(), beamExportCodebookButton);
    beamForm->addRow(QString(), beamCodebookHintLabel);
    beamForm->addRow(tr("Beam selection checkpoint"), beamCheckpointWidget);
    beamForm->addRow(tr("Beam selection feature mode"), beamFeatureModeCombo);
    beamForm->addRow(tr("Beam selection Top-K"), beamTopKSpin);
    beamForm->addRow(QString(), beamDatasetExportCheck);
    beamForm->addRow(tr("Dataset export CSV"), beamDatasetWidget);
    beamForm->addRow(tr("Training CSV(s)"), beamTrainDatasetsWidget);
    beamForm->addRow(QString(), beamResetDatasetCheck);
    beamForm->addRow(tr("Checkpoint output"), beamTrainOutputWidget);
    beamForm->addRow(tr("Train epochs"), beamTrainEpochsSpin);
    beamForm->addRow(tr("Train batch size"), beamTrainBatchSpin);
    beamForm->addRow(tr("Train hidden dim"), beamTrainHiddenDimSpin);
    beamForm->addRow(tr("Train learning rate"), beamTrainLrSpin);
    beamForm->addRow(tr("Validation split"), beamTrainValSplitSpin);
    beamForm->addRow(QString(), beamTrainNowButton);
    beamForm->addRow(QString(), beamHintLabel);

    auto updateBeamWidgets = [=]() {
        const bool enabled = enableBeamformingCheck->isChecked();
        const QString mode = beamModeCombo->currentText().trimmed().toLower();
        const bool isManual = (mode == QStringLiteral("manual_index"));
        const bool isLearnedSelector = (beamSelectionModeFromUi(mode) == QStringLiteral("deepsense_predictor"));
        const bool exportDataset = beamDatasetExportCheck->isChecked();
        beamModeCombo->setEnabled(enabled);
        beamCodebookCombo->setEnabled(enabled);
        beamCodebookNumBeamsCombo->setEnabled(enabled);
        beamOversamplingWidget->setEnabled(enabled);
        beamNormalizePowerCheck->setEnabled(enabled);
        beamCodebookWidget->setEnabled(enabled);
        beamExportCodebookButton->setEnabled(enabled);
        beamManualIndexSpin->setEnabled(enabled && isManual);
        beamCheckpointWidget->setEnabled(enabled && isLearnedSelector);
        beamFeatureModeCombo->setEnabled(enabled);
        beamTopKSpin->setEnabled(enabled);
        beamDatasetExportCheck->setEnabled(enabled);
        beamDatasetWidget->setEnabled(enabled && exportDataset);
        beamResetDatasetCheck->setEnabled(enabled && exportDataset);
        beamTrainDatasetsWidget->setEnabled(enabled);
        beamTrainOutputWidget->setEnabled(enabled);
        beamTrainEpochsSpin->setEnabled(enabled);
        beamTrainBatchSpin->setEnabled(enabled);
        beamTrainHiddenDimSpin->setEnabled(enabled);
        beamTrainLrSpin->setEnabled(enabled);
        beamTrainValSplitSpin->setEnabled(enabled);
        beamTrainNowButton->setEnabled(enabled);
    };
    updateBeamWidgets();
    connect(enableBeamformingCheck, &QCheckBox::toggled, &dialog, updateBeamWidgets);
    connect(beamModeCombo, &QComboBox::currentTextChanged, &dialog, updateBeamWidgets);
    connect(beamModeCombo->lineEdit(), &QLineEdit::textChanged, &dialog, updateBeamWidgets);
    connect(beamDatasetExportCheck, &QCheckBox::toggled, &dialog, updateBeamWidgets);

    auto* ckmGroup = new QGroupBox(tr("Dense CKM (offline grid)"), content);
    auto* ckmForm = new QFormLayout(ckmGroup);

    const QStringList selectedCkmMetrics = normalizeDenseCkmMetrics(ckmSettings_.selectedMetrics, ckmSettings_.metric);
    QMap<QString, QCheckBox*> ckmMetricChecks;
    auto* ckmMetricWidget = new QWidget(ckmGroup);
    auto* ckmMetricLayout = new QGridLayout(ckmMetricWidget);
    ckmMetricLayout->setContentsMargins(0, 0, 0, 0);
    ckmMetricLayout->setHorizontalSpacing(12);
    ckmMetricLayout->setVerticalSpacing(6);
    const QStringList supportedCkmMetrics = denseCkmMetricList();
    for (int i = 0; i < supportedCkmMetrics.size(); ++i) {
        const QString metric = supportedCkmMetrics.at(i);
        auto* check = new QCheckBox(QStringLiteral("%1 (%2)").arg(denseCkmMetricDisplayName(metric), denseCkmMetricUiKey(metric)), ckmGroup);
        check->setChecked(selectedCkmMetrics.contains(metric));
        ckmMetricChecks.insert(metric, check);
        ckmMetricLayout->addWidget(check, i / 2, i % 2);
    }

    double loadedSceneMinX = 0.0;
    double loadedSceneMaxX = 0.0;
    double loadedSceneMinY = 0.0;
    double loadedSceneMaxY = 0.0;
    const bool hasLoadedSceneBounds = sceneWidget_ && sceneWidget_->sceneBoundsXY(&loadedSceneMinX, &loadedSceneMaxX, &loadedSceneMinY, &loadedSceneMaxY);

    const QString effectiveXmlScenePath = !scenePathEdit->text().trimmed().isEmpty()
                                              ? scenePathEdit->text().trimmed()
                                              : effectiveDenseCkmBoundsScenePath();
    double xmlSceneMinX = 0.0;
    double xmlSceneMaxX = 0.0;
    double xmlSceneMinY = 0.0;
    double xmlSceneMaxY = 0.0;
    QString xmlSceneBoundsError;
    const bool hasXmlSceneBounds = readSceneXmlBoundsXY(effectiveXmlScenePath,
                                                        &xmlSceneMinX, &xmlSceneMaxX,
                                                        &xmlSceneMinY, &xmlSceneMaxY,
                                                        &xmlSceneBoundsError);
    const bool hasAnyAutoBounds = hasLoadedSceneBounds || hasXmlSceneBounds;

    auto* ckmUseSceneBoundsCheck = new QCheckBox(tr("Use loaded scene bounds automatically (prefer 3D scene, otherwise Scene XML)"), ckmGroup);
    ckmUseSceneBoundsCheck->setChecked(ckmSettings_.useSceneBounds && hasAnyAutoBounds);
    ckmUseSceneBoundsCheck->setEnabled(hasAnyAutoBounds);

    auto* ckmSceneBoundsLabel = new QLabel(ckmGroup);
    ckmSceneBoundsLabel->setWordWrap(true);
    if (hasLoadedSceneBounds && hasXmlSceneBounds) {
        ckmSceneBoundsLabel->setText(tr("Detected auto range source: loaded 3D scene first, Scene XML as fallback. 3D extent: x [%1, %2], y [%3, %4]. XML extent: x [%5, %6], y [%7, %8].")
                                         .arg(loadedSceneMinX, 0, 'f', 2)
                                         .arg(loadedSceneMaxX, 0, 'f', 2)
                                         .arg(loadedSceneMinY, 0, 'f', 2)
                                         .arg(loadedSceneMaxY, 0, 'f', 2)
                                         .arg(xmlSceneMinX, 0, 'f', 2)
                                         .arg(xmlSceneMaxX, 0, 'f', 2)
                                         .arg(xmlSceneMinY, 0, 'f', 2)
                                         .arg(xmlSceneMaxY, 0, 'f', 2));
    } else if (hasLoadedSceneBounds) {
        ckmSceneBoundsLabel->setText(tr("Detected auto range source: loaded 3D scene. Extent: x [%1, %2], y [%3, %4].")
                                         .arg(loadedSceneMinX, 0, 'f', 2)
                                         .arg(loadedSceneMaxX, 0, 'f', 2)
                                         .arg(loadedSceneMinY, 0, 'f', 2)
                                         .arg(loadedSceneMaxY, 0, 'f', 2));
    } else if (hasXmlSceneBounds) {
        ckmSceneBoundsLabel->setText(tr("Detected auto range source: Scene XML. Extent: x [%1, %2], y [%3, %4].")
                                         .arg(xmlSceneMinX, 0, 'f', 2)
                                         .arg(xmlSceneMaxX, 0, 'f', 2)
                                         .arg(xmlSceneMinY, 0, 'f', 2)
                                         .arg(xmlSceneMaxY, 0, 'f', 2));
    } else {
        ckmSceneBoundsLabel->setText(xmlSceneBoundsError.isEmpty()
                                         ? tr("No imported 3D scene or valid Scene XML bounds are available right now. Automatic CKM extents will use the loaded 3D scene first; if no 3D scene is loaded, they will fall back to the Scene XML.")
                                         : tr("No imported 3D scene bounds are available right now, and Scene XML bounds could not be read yet.\n\n%1").arg(xmlSceneBoundsError));
    }

    auto* ckmXMinSpin = new QDoubleSpinBox(ckmGroup);
    ckmXMinSpin->setRange(-100000.0, 100000.0);
    ckmXMinSpin->setDecimals(2);
    ckmXMinSpin->setValue(ckmSettings_.xMin);
    auto* ckmXMaxSpin = new QDoubleSpinBox(ckmGroup);
    ckmXMaxSpin->setRange(-100000.0, 100000.0);
    ckmXMaxSpin->setDecimals(2);
    ckmXMaxSpin->setValue(ckmSettings_.xMax);
    auto* ckmYMinSpin = new QDoubleSpinBox(ckmGroup);
    ckmYMinSpin->setRange(-100000.0, 100000.0);
    ckmYMinSpin->setDecimals(2);
    ckmYMinSpin->setValue(ckmSettings_.yMin);
    auto* ckmYMaxSpin = new QDoubleSpinBox(ckmGroup);
    ckmYMaxSpin->setRange(-100000.0, 100000.0);
    ckmYMaxSpin->setDecimals(2);
    ckmYMaxSpin->setValue(ckmSettings_.yMax);

    auto* ckmXRangeWidget = new QWidget(ckmGroup);
    auto* ckmXRangeLayout = new QHBoxLayout(ckmXRangeWidget);
    ckmXRangeLayout->setContentsMargins(0, 0, 0, 0);
    ckmXRangeLayout->addWidget(ckmXMinSpin);
    ckmXRangeLayout->addWidget(ckmXMaxSpin);

    auto* ckmYRangeWidget = new QWidget(ckmGroup);
    auto* ckmYRangeLayout = new QHBoxLayout(ckmYRangeWidget);
    ckmYRangeLayout->setContentsMargins(0, 0, 0, 0);
    ckmYRangeLayout->addWidget(ckmYMinSpin);
    ckmYRangeLayout->addWidget(ckmYMaxSpin);

    auto* ckmZSpin = new QDoubleSpinBox(ckmGroup);
    ckmZSpin->setRange(-10000.0, 10000.0);
    ckmZSpin->setDecimals(2);
    ckmZSpin->setValue(ckmSettings_.zFixed);
    ckmZSpin->setSuffix(tr(" m"));

    auto* ckmResolutionSpin = new QDoubleSpinBox(ckmGroup);
    ckmResolutionSpin->setRange(0.1, 1000.0);
    ckmResolutionSpin->setDecimals(2);
    ckmResolutionSpin->setValue(ckmSettings_.resolutionM);
    ckmResolutionSpin->setSuffix(tr(" m"));

    auto* ckmOutputEdit = new QLineEdit(ckmSettings_.outputDir.trimmed().isEmpty() ? defaultDenseCkmOutputDir() : ckmSettings_.outputDir, ckmGroup);
    auto* browseCkmOutputButton = new QPushButton(tr("Browse..."), ckmGroup);
    auto* ckmOutputRowWidget = new QWidget(ckmGroup);
    auto* ckmOutputRow = new QHBoxLayout(ckmOutputRowWidget);
    ckmOutputRow->setContentsMargins(0, 0, 0, 0);
    ckmOutputRow->addWidget(ckmOutputEdit);
    ckmOutputRow->addWidget(browseCkmOutputButton);

    auto* ckmAutoLoadCheck = new QCheckBox(tr("Auto-load latest result into CKM Map tab after generation"), ckmGroup);
    ckmAutoLoadCheck->setChecked(ckmSettings_.autoLoadResult);

    auto* ckmRenderSceneOverlayCheck = new QCheckBox(tr("Also render official-style scene overlay images"), ckmGroup);
    ckmRenderSceneOverlayCheck->setChecked(ckmSettings_.renderSceneOverlay);

    auto* ckmRenderScenePathEdit = new QLineEdit(ckmSettings_.renderScenePath.trimmed(), ckmGroup);
    ckmRenderScenePathEdit->setPlaceholderText(tr("Optional: higher-quality XML scene used only for CKM rendering"));
    auto* browseCkmRenderSceneButton = new QPushButton(tr("Browse..."), ckmGroup);
    auto* clearCkmRenderSceneButton = new QPushButton(tr("Use simulation XML"), ckmGroup);
    auto* ckmRenderSceneRowWidget = new QWidget(ckmGroup);
    auto* ckmRenderSceneRow = new QHBoxLayout(ckmRenderSceneRowWidget);
    ckmRenderSceneRow->setContentsMargins(0, 0, 0, 0);
    ckmRenderSceneRow->addWidget(ckmRenderScenePathEdit);
    ckmRenderSceneRow->addWidget(browseCkmRenderSceneButton);
    ckmRenderSceneRow->addWidget(clearCkmRenderSceneButton);

    auto* ckmHintLabel = new QLabel(tr("Select one or more RT/SYS/beam metrics to generate. Oracle-beam CKM requires beamforming to be enabled. Predicted-beam CKM is only generated when a .pt checkpoint is loaded; for those metrics, Dense CKM will automatically run the offline beam selector in learned_selector mode for this CKM job. The simulation still uses the Scene XML above for RT/SYS calculations. If you provide a second, higher-detail XML here, Dense CKM will keep computing on the simulation scene but render the final official-style CKM overlay on top of this higher-quality XML so the visualization looks better."), ckmGroup);
    ckmHintLabel->setWordWrap(true);

    auto updateCkmRangeWidgets = [=]() {
        const bool manualRanges = !ckmUseSceneBoundsCheck->isChecked();
        ckmXRangeWidget->setEnabled(manualRanges);
        ckmYRangeWidget->setEnabled(manualRanges);
    };
    updateCkmRangeWidgets();
    connect(ckmUseSceneBoundsCheck, &QCheckBox::toggled, &dialog, updateCkmRangeWidgets);

    ckmForm->addRow(tr("Metrics to generate"), ckmMetricWidget);
    ckmForm->addRow(QString(), ckmUseSceneBoundsCheck);
    ckmForm->addRow(tr("Detected scene extent"), ckmSceneBoundsLabel);
    ckmForm->addRow(tr("X range (min / max)"), ckmXRangeWidget);
    ckmForm->addRow(tr("Y range (min / max)"), ckmYRangeWidget);
    ckmForm->addRow(tr("Fixed RX height"), ckmZSpin);
    ckmForm->addRow(tr("Grid resolution"), ckmResolutionSpin);
    ckmForm->addRow(tr("Output directory"), ckmOutputRowWidget);
    ckmForm->addRow(QString(), ckmAutoLoadCheck);
    ckmForm->addRow(QString(), ckmRenderSceneOverlayCheck);
    ckmForm->addRow(tr("High-detail CKM render XML"), ckmRenderSceneRowWidget);
    ckmForm->addRow(QString(), ckmHintLabel);

    contentLayout->addWidget(envGroup);
    contentLayout->addWidget(solverGroup);
    contentLayout->addWidget(arraysGroup);
    contentLayout->addWidget(propagationGroup);
    contentLayout->addWidget(sysGroup);
    contentLayout->addWidget(beamGroup);
    contentLayout->addWidget(ckmGroup);
    contentLayout->addWidget(runtimeGroup);

    auto* note = new QLabel(tr("The GUI starts the internal Sionna simulator by directly launching the selected Python interpreter, just like VS Code selects an interpreter instead of running conda activate. Base station JSON is exported automatically from the stations currently loaded in the GUI. Antenna array pattern/polarization boxes are editable so you can type any Sionna-supported value even if it is not listed here. When SYS over RT is enabled, the current single-UAV workflow uses RT-generated CFRs plus a lightweight SYS pipeline (best-BS selection, OLLA, MCS selection, and PHY abstraction). Beamforming keeps the legacy RF/SYS pipeline untouched, adds Sionna DFT codebook generation plus exhaustive/manual selection, and can run a learned beam selector in the Python worker, with predictor-vs-oracle results surfaced in the live SYS monitor. Dense CKM generation reuses the same RT and SYS parameters on an offline 2D grid without removing any existing real-time or rosbag features. You can provide a second, higher-detail XML scene specifically for CKM visualization: the low-detail Scene XML above is still used for RT/SYS computation, while the optional CKM render XML is only used when producing the final official-style in-scene CKM image."), content);
    note->setWordWrap(true);
    contentLayout->addWidget(note);
    contentLayout->addStretch(1);

    scrollArea->setWidget(content);
    layout->addWidget(scrollArea, 1);

    connect(browseSceneButton, &QPushButton::clicked, &dialog, [this, scenePathEdit]() {
        const QString path = QFileDialog::getOpenFileName(this, tr("Select scene XML"), scenePathEdit->text(), tr("XML files (*.xml)"));
        if (!path.isEmpty()) {
            scenePathEdit->setText(path);
        }
    });
    connect(browseCsvButton, &QPushButton::clicked, &dialog, [this, csvPathEdit]() {
        const QString path = QFileDialog::getSaveFileName(this, tr("Select CSV path"), csvPathEdit->text(), tr("CSV files (*.csv)"));
        if (!path.isEmpty()) {
            csvPathEdit->setText(path);
        }
    });
    connect(browseCkmOutputButton, &QPushButton::clicked, &dialog, [this, ckmOutputEdit]() {
        const QString path = QFileDialog::getExistingDirectory(this, tr("Select Dense CKM output directory"), ckmOutputEdit->text().trimmed().isEmpty() ? defaultDenseCkmOutputDir() : ckmOutputEdit->text().trimmed());
        if (!path.isEmpty()) {
            ckmOutputEdit->setText(path);
        }
    });
    connect(browseCkmRenderSceneButton, &QPushButton::clicked, &dialog, [this, ckmRenderScenePathEdit]() {
        const QString path = QFileDialog::getOpenFileName(this, tr("Select high-detail CKM render scene XML"), ckmRenderScenePathEdit->text().trimmed(), tr("XML files (*.xml)"));
        if (!path.isEmpty()) {
            ckmRenderScenePathEdit->setText(path);
        }
    });
    connect(clearCkmRenderSceneButton, &QPushButton::clicked, &dialog, [ckmRenderScenePathEdit]() {
        ckmRenderScenePathEdit->clear();
    });
    connect(selectPythonButton, &QPushButton::clicked, &dialog, [this, pythonPathEdit]() {
        const QString selected = selectPythonEnvironment(this, pythonPathEdit->text().trimmed());
        if (!selected.isEmpty()) {
            pythonPathEdit->setText(selected);
        }
    });
    connect(browsePythonButton, &QPushButton::clicked, &dialog, [this, pythonPathEdit]() {
        const QString path = QFileDialog::getOpenFileName(this, tr("Select Python executable"), pythonPathEdit->text().trimmed().isEmpty() ? QDir::homePath() : QFileInfo(pythonPathEdit->text()).absolutePath());
        if (!path.isEmpty()) {
            pythonPathEdit->setText(path);
        }
    });
    connect(testPythonButton, &QPushButton::clicked, &dialog, [this, pythonPathEdit]() {
        const QString path = pythonPathEdit->text().trimmed().isEmpty() ? QStringLiteral("python3") : pythonPathEdit->text().trimmed();
        QApplication::setOverrideCursor(Qt::WaitCursor);
        const QString result = testPythonEnvironment(path);
        QApplication::restoreOverrideCursor();
        if (result.contains(QStringLiteral("OK"))) {
            QMessageBox::information(this, tr("Python Environment"), tr("Environment check passed.\n\n%1").arg(result));
        } else {
            QMessageBox::warning(this, tr("Python Environment"), tr("Environment check failed.\n\n%1").arg(result));
        }
    });

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);

    useCommittedSpinBoxEdits(&dialog);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    simSettings_.pythonExecutable = pythonPathEdit->text().trimmed().isEmpty() ? QStringLiteral("python3") : pythonPathEdit->text().trimmed();
    simSettings_.scenePath = scenePathEdit->text().trimmed();
    simSettings_.fcHz = fcSpin->value();
    simSettings_.miVariant = miVariantEdit->text().trimmed();

    simSettings_.txArrayNumRows = txEditors.rows->value();
    simSettings_.txArrayNumCols = txEditors.cols->value();
    simSettings_.txArrayVerticalSpacing = txEditors.verticalSpacing->value();
    simSettings_.txArrayHorizontalSpacing = txEditors.horizontalSpacing->value();
    simSettings_.txArrayPattern = txEditors.pattern->currentText().trimmed().isEmpty() ? QStringLiteral("iso") : txEditors.pattern->currentText().trimmed();
    simSettings_.txArrayPolarization = txEditors.polarization->currentText().trimmed().isEmpty() ? QStringLiteral("V") : txEditors.polarization->currentText().trimmed();

    simSettings_.rxArrayNumRows = rxEditors.rows->value();
    simSettings_.rxArrayNumCols = rxEditors.cols->value();
    simSettings_.rxArrayVerticalSpacing = rxEditors.verticalSpacing->value();
    simSettings_.rxArrayHorizontalSpacing = rxEditors.horizontalSpacing->value();
    simSettings_.rxArrayPattern = rxEditors.pattern->currentText().trimmed().isEmpty() ? QStringLiteral("iso") : rxEditors.pattern->currentText().trimmed();
    simSettings_.rxArrayPolarization = rxEditors.polarization->currentText().trimmed().isEmpty() ? QStringLiteral("V") : rxEditors.polarization->currentText().trimmed();

    simSettings_.maxDepth = maxDepthSpin->value();
    simSettings_.samplesPerSrc = samplesSpin->value();
    simSettings_.maxNumPathsPerSrc = maxPathsSpin->value();
    simSettings_.syntheticArray = syntheticCheck->isChecked();
    simSettings_.mergeShapes = mergeShapesCheck->isChecked();

    simSettings_.enableSysIntegration = enableSysCheck->isChecked();
    simSettings_.sysNumSubcarriers = sysNumSubcarriersSpin->value();
    simSettings_.sysSubcarrierSpacingHz = sysSubcarrierSpacingSpin->value();
    simSettings_.sysNumOfdmSymbols = sysNumSymbolsSpin->value();
    simSettings_.sysTemperatureK = sysTemperatureSpin->value();
    simSettings_.sysBlerTarget = sysBlerTargetSpin->value();
    simSettings_.sysMcsTableIndex = sysMcsTableSpin->value();
    simSettings_.sysBsTxPowerDbm = sysTxPowerSpin->value();

    simSettings_.enableBeamforming = enableBeamformingCheck->isChecked();
    simSettings_.beamSelectionMode = beamModeCombo->currentText().trimmed().isEmpty() ? QStringLiteral("exhaustive_sweep") : beamSelectionModeFromUi(beamModeCombo->currentText());
    simSettings_.beamCodebookType = beamCodebookCombo->currentText().trimmed().isEmpty() ? QStringLiteral("auto") : beamCodebookCombo->currentText().trimmed();
    simSettings_.beamCodebookNumBeams = beamCodebookNumBeamsCombo->currentData().toInt();
    simSettings_.beamOversamplingV = beamOversamplingVSpin->value();
    simSettings_.beamOversamplingH = beamOversamplingHSpin->value();
    simSettings_.beamManualIndex = beamManualIndexSpin->value();
    simSettings_.beamNormalizePower = beamNormalizePowerCheck->isChecked();
    simSettings_.beamCodebookFilePath = beamCodebookFileEdit->text().trimmed();
    simSettings_.beamModelCheckpointPath = beamCheckpointEdit->text().trimmed();
    simSettings_.beamFeatureMode = beamFeatureModeCombo->currentText().trimmed().isEmpty() ? QStringLiteral("geom_vel_path13") : beamFeatureModeFromUi(beamFeatureModeCombo->currentText());
    simSettings_.beamTopK = beamTopKSpin->value();
    simSettings_.beamExportTrainingDataset = beamDatasetExportCheck->isChecked();
    simSettings_.beamTrainingDatasetPath = beamDatasetEdit->text().trimmed();
    simSettings_.beamTrainingInputPaths = joinBeamTrainingDatasetPaths(beamTrainingDatasetPathsFromListWidget(beamTrainDatasetsList));
    simSettings_.beamTrainingResetDatasetOnStart = beamResetDatasetCheck->isChecked();
    simSettings_.beamTrainingOutputCheckpointPath = beamTrainOutputEdit->text().trimmed();
    simSettings_.beamTrainingEpochs = beamTrainEpochsSpin->value();
    simSettings_.beamTrainingBatchSize = beamTrainBatchSpin->value();
    simSettings_.beamTrainingLearningRate = beamTrainLrSpin->value();
    simSettings_.beamTrainingValidationSplit = beamTrainValSplitSpin->value();
    simSettings_.beamTrainingHiddenDim = beamTrainHiddenDimSpin->value();

    simSettings_.los = losCheck->isChecked();
    simSettings_.specularReflection = specularCheck->isChecked();
    simSettings_.diffuseReflection = diffuseCheck->isChecked();
    simSettings_.refraction = refractionCheck->isChecked();
    simSettings_.diffraction = diffractionCheck->isChecked();
    simSettings_.edgeDiffraction = edgeDiffractionCheck->isChecked();
    simSettings_.diffractionLitRegion = litRegionCheck->isChecked();

    simSettings_.simHz = simHzSpin->value();
    simSettings_.maxPoseStalenessS = staleSpin->value();
    simSettings_.rfFrameId = frameIdEdit->text().trimmed().isEmpty() ? QStringLiteral("map") : frameIdEdit->text().trimmed();
    simSettings_.csvEnabled = csvCheck->isChecked();
    simSettings_.csvPath = csvPathEdit->text().trimmed();

    QStringList selectedMetricsForGeneration;
    for (const QString& metricKey : supportedCkmMetrics) {
        QCheckBox* check = ckmMetricChecks.value(metricKey, nullptr);
        if (check && check->isChecked()) {
            selectedMetricsForGeneration << metricKey;
        }
    }
    ckmSettings_.selectedMetrics = normalizeDenseCkmMetrics(selectedMetricsForGeneration, ckmSettings_.metric);
    ckmSettings_.metric = ckmSettings_.selectedMetrics.value(0, QStringLiteral("power_db"));
    ckmSettings_.useSceneBounds = ckmUseSceneBoundsCheck->isChecked() && ckmUseSceneBoundsCheck->isEnabled();
    ckmSettings_.xMin = ckmXMinSpin->value();
    ckmSettings_.xMax = ckmXMaxSpin->value();
    ckmSettings_.yMin = ckmYMinSpin->value();
    ckmSettings_.yMax = ckmYMaxSpin->value();
    ckmSettings_.zFixed = ckmZSpin->value();
    ckmSettings_.resolutionM = ckmResolutionSpin->value();
    ckmSettings_.outputDir = ckmOutputEdit->text().trimmed().isEmpty() ? defaultDenseCkmOutputDir() : ckmOutputEdit->text().trimmed();
    ckmSettings_.autoLoadResult = ckmAutoLoadCheck->isChecked();
    ckmSettings_.renderSceneOverlay = ckmRenderSceneOverlayCheck->isChecked();
    ckmSettings_.renderScenePath = ckmRenderScenePathEdit->text().trimmed();
    if (!scenePathEdit->text().trimmed().isEmpty() || !ckmRenderScenePathEdit->text().trimmed().isEmpty()) {
        ckmSettings_.useSceneBounds = true;
    }

    syncAirSimLiveViewSettings();
    syncSionnaPreviewSettings();
    applyCoordinateTransformSettings(false);
    markGuiConfigDirty();
    onStatusMessage(QStringLiteral("Simulation settings updated. Python: %1").arg(simSettings_.pythonExecutable));
}

void MainWindow::onDroneUpdated(const airsim_gui::DroneState& state) {
    currentDrone_ = state;
    if (frameTransformManager_) {
        if (!currentDrone_.sourceFrameId.trimmed().isEmpty()) {
            frameTransformManager_->setRosFrameId(currentDrone_.sourceFrameId);
        }
        if (currentDrone_.valid) {
            frameTransformManager_->setCurrentDronePoseInScene(
                currentDrone_.position,
                currentDrone_.rollDeg,
                currentDrone_.pitchDeg,
                currentDrone_.yawDeg);
        } else {
            frameTransformManager_->clearCurrentDronePoseInScene();
        }
    }
    sceneWidget_->setDroneState(currentDrone_);
    if (airSimViewController_) {
        airSimViewController_->setDroneState(currentDrone_);
    }
    syncStationCameraWindows();
    refreshInfoPanel();
}

void MainWindow::onPathsUpdated(const airsim_gui::BeamPathList& paths) {
    currentPaths_ = paths;
    sceneWidget_->setBeamPaths(currentPaths_);
    if (airSimViewController_) {
        airSimViewController_->setBeamPaths(currentPaths_);
    }
    syncStationCameraWindows();
    refreshInfoPanel();
}

void MainWindow::onRfDataUpdated(const airsim_gui::RfObservationSnapshot& snapshot) {
    currentRfSnapshot_ = snapshot;
    rebuildRfDataCards();
    scheduleSysDataWindowRefresh();
}

void MainWindow::onSysObservationUpdated(const QString& jsonText) {
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(jsonText.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        return;
    }

    const QJsonObject obj = doc.object();
    const std::vector<SysCandidateView> previousCandidates = currentSysSnapshot_.candidates;
    currentSysSnapshot_.valid = obj.value(QStringLiteral("valid")).toBool(true);
    currentSysSnapshot_.enabled = obj.value(QStringLiteral("enabled")).toBool(false);
    currentSysSnapshot_.schedulerPolicy = obj.value(QStringLiteral("scheduler_policy")).toString();
    currentSysSnapshot_.simIdx = obj.value(QStringLiteral("sim_idx")).toInt(-1);
    currentSysSnapshot_.odomStampS = obj.value(QStringLiteral("odom_stamp_s")).toDouble(std::numeric_limits<double>::quiet_NaN());
    currentSysSnapshot_.servingIdx = obj.value(QStringLiteral("serving_idx")).toInt(-1);
    currentSysSnapshot_.servingBsName = obj.value(QStringLiteral("serving_bs_name")).toString();
    currentSysSnapshot_.servingAnchorId = obj.value(QStringLiteral("serving_anchor_id")).toInt(-1);
    currentSysSnapshot_.servingMcsIndex = obj.value(QStringLiteral("serving_mcs_index")).toInt(-1);
    currentSysSnapshot_.servingSinrEffDb = obj.value(QStringLiteral("serving_sinr_eff_db")).toDouble(std::numeric_limits<double>::quiet_NaN());
    currentSysSnapshot_.servingSpectralEfficiencyBpsHz = obj.value(QStringLiteral("serving_spectral_efficiency_bpshz")).toDouble(std::numeric_limits<double>::quiet_NaN());
    currentSysSnapshot_.servingTbOk = obj.contains(QStringLiteral("serving_tb_ok")) ? (obj.value(QStringLiteral("serving_tb_ok")).toBool(false) ? 1 : 0) : -1;

    currentSysSnapshot_.candidates.clear();
    const QJsonArray candidates = obj.value(QStringLiteral("candidates")).toArray();
    currentSysSnapshot_.candidates.reserve(static_cast<size_t>(candidates.size()));
    for (const auto& value : candidates) {
        if (!value.isObject()) {
            continue;
        }
        const QJsonObject c = value.toObject();
        SysCandidateView candidate;
        candidate.simIdx = c.value(QStringLiteral("sim_idx")).toInt(-1);
        candidate.odomStampS = c.value(QStringLiteral("odom_stamp_s")).toDouble(std::numeric_limits<double>::quiet_NaN());
        candidate.bsName = c.value(QStringLiteral("bs_name")).toString();
        candidate.anchorId = c.value(QStringLiteral("anchor_id")).toInt(-1);
        candidate.numPaths = c.value(QStringLiteral("num_paths")).toInt(-1);
        candidate.strongestPathPowerDb = c.value(QStringLiteral("power_db")).toDouble(std::numeric_limits<double>::quiet_NaN());
        candidate.strongestPathType = c.value(QStringLiteral("path_type")).toString();
        candidate.strongestPathOrder = c.value(QStringLiteral("path_order")).toInt(-1);
        candidate.sysEnabled = c.value(QStringLiteral("sys_enabled")).toBool(currentSysSnapshot_.enabled);
        candidate.candidateRateBpsHz = c.value(QStringLiteral("sys_candidate_rate_bpshz")).toDouble(std::numeric_limits<double>::quiet_NaN());
        candidate.candidateSinrEffDb = c.value(QStringLiteral("sys_candidate_sinr_eff_db")).toDouble(std::numeric_limits<double>::quiet_NaN());
        candidate.mcsIndex = c.value(QStringLiteral("sys_mcs_index")).toInt(-1);
        candidate.numDecodedBits = c.value(QStringLiteral("sys_num_decoded_bits")).toInt(-1);
        candidate.tbOk = c.contains(QStringLiteral("sys_tb_ok")) ? (c.value(QStringLiteral("sys_tb_ok")).toBool(false) ? 1 : 0) : -1;
        candidate.spectralEfficiencyBpsHz = c.value(QStringLiteral("sys_spectral_efficiency_bpshz")).toDouble(std::numeric_limits<double>::quiet_NaN());
        candidate.isServingBs = c.value(QStringLiteral("sys_is_serving_bs")).toBool(false);
        candidate.servingBsName = c.value(QStringLiteral("sys_serving_bs_name")).toString();
        candidate.servingAnchorId = c.value(QStringLiteral("sys_serving_anchor_id")).toInt(-1);
        candidate.blerTarget = c.value(QStringLiteral("sys_bler_target")).toDouble(std::numeric_limits<double>::quiet_NaN());
        for (const auto& previous : previousCandidates) {
            if (sysCandidateMatchesIdentity(previous, candidate.anchorId, candidate.bsName)) {
                copySysCandidateBeamFields(&candidate, previous);
                break;
            }
        }
        currentSysSnapshot_.candidates.push_back(candidate);
    }

    normalizeCurrentSysCandidates();
    for (const auto& candidate : currentSysSnapshot_.candidates) {
        if (candidate.isServingBs) {
            syncCurrentBeamSummaryFromCandidate(candidate);
            break;
        }
    }

    sysModeSummary_ = currentSysSnapshot_.enabled ? QStringLiteral("Enabled") : QStringLiteral("Disabled");
    if (currentSysSnapshot_.servingBsName.trimmed().isEmpty()) {
        sysServingSummary_ = QStringLiteral("-");
    } else if (currentSysSnapshot_.servingAnchorId >= 0) {
        sysServingSummary_ = QStringLiteral("%1 (#%2)").arg(currentSysSnapshot_.servingBsName).arg(currentSysSnapshot_.servingAnchorId);
    } else {
        sysServingSummary_ = currentSysSnapshot_.servingBsName;
    }
    QStringList sysParts;
    if (currentSysSnapshot_.servingMcsIndex >= 0) {
        sysParts << QStringLiteral("MCS %1").arg(currentSysSnapshot_.servingMcsIndex);
    }
    if (std::isfinite(currentSysSnapshot_.servingSinrEffDb)) {
        sysParts << QStringLiteral("SINR %1 dB").arg(QString::number(currentSysSnapshot_.servingSinrEffDb, 'f', 1));
    }
    if (std::isfinite(currentSysSnapshot_.servingSpectralEfficiencyBpsHz)) {
        sysParts << QStringLiteral("SE %1").arg(QString::number(currentSysSnapshot_.servingSpectralEfficiencyBpsHz, 'f', 2));
    }
    sysParts << ackStateLabel(currentSysSnapshot_.servingTbOk);
    sysLinkSummary_ = sysParts.join(QStringLiteral(" | "));

    scheduleSysDataWindowRefresh();
    refreshInfoPanel();
}

void MainWindow::onBeamObservationUpdated(const QString& jsonText) {
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(jsonText.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        return;
    }

    const QJsonObject obj = doc.object();
    currentSysSnapshot_.valid = obj.value(QStringLiteral("valid")).toBool(true) || currentSysSnapshot_.valid;
    if (obj.contains(QStringLiteral("sim_idx"))) {
        currentSysSnapshot_.simIdx = obj.value(QStringLiteral("sim_idx")).toInt(currentSysSnapshot_.simIdx);
    }
    const double odomStamp = obj.value(QStringLiteral("odom_stamp_s")).toDouble(std::numeric_limits<double>::quiet_NaN());
    if (std::isfinite(odomStamp)) {
        currentSysSnapshot_.odomStampS = odomStamp;
    }

    const QJsonArray observations = obj.value(QStringLiteral("observations")).toArray();
    for (const auto& value : observations) {
        if (!value.isObject()) {
            continue;
        }
        const QJsonObject c = value.toObject();
        const int anchorId = c.value(QStringLiteral("anchor_id")).toInt(-1);
        const QString bsName = c.value(QStringLiteral("bs_name")).toString();

        SysCandidateView* target = nullptr;
        for (auto& existing : currentSysSnapshot_.candidates) {
            if (sysCandidateMatchesIdentity(existing, anchorId, bsName)) {
                target = &existing;
                break;
            }
        }
        if (!target) {
            SysCandidateView candidate;
            candidate.anchorId = anchorId;
            candidate.bsName = bsName;
            currentSysSnapshot_.candidates.push_back(candidate);
            target = &currentSysSnapshot_.candidates.back();
        }

        target->simIdx = c.value(QStringLiteral("sim_idx")).toInt(target->simIdx);
        target->odomStampS = c.value(QStringLiteral("odom_stamp_s")).toDouble(target->odomStampS);
        target->beamEnabled = c.value(QStringLiteral("beam_enabled")).toBool(target->beamEnabled || simSettings_.enableBeamforming);
        target->beamSelectionMode = c.value(QStringLiteral("beam_selection_mode")).toString(target->beamSelectionMode);
        target->beamCodebookType = c.value(QStringLiteral("beam_codebook_type")).toString(target->beamCodebookType);
        target->beamNumBeams = c.value(QStringLiteral("beam_num_beams")).toInt(target->beamNumBeams);
        target->beamSelectedIndex = c.value(QStringLiteral("beam_selected_index")).toInt(target->beamSelectedIndex);
        target->beamSelectedGainDb = c.value(QStringLiteral("beam_selected_gain_db")).toDouble(target->beamSelectedGainDb);
        target->beamOracleIndex = c.value(QStringLiteral("beam_oracle_index")).toInt(target->beamOracleIndex);
        target->beamOracleGainDb = c.value(QStringLiteral("beam_oracle_gain_db")).toDouble(target->beamOracleGainDb);
        target->beamPredictorStatus = c.value(QStringLiteral("beam_predictor_status")).toString(target->beamPredictorStatus);
        target->beamFeatureMode = c.value(QStringLiteral("beam_feature_mode")).toString(target->beamFeatureMode);
        target->beamTopK = c.value(QStringLiteral("beam_top_k")).toInt(target->beamTopK);
        target->beamPredictedIndex = c.value(QStringLiteral("beam_predicted_index")).toInt(target->beamPredictedIndex);
        target->beamPredictedConfidence = c.value(QStringLiteral("beam_predicted_confidence")).toDouble(target->beamPredictedConfidence);
        target->beamOracleInTopK = c.contains(QStringLiteral("beam_oracle_in_topk")) ? (c.value(QStringLiteral("beam_oracle_in_topk")).toBool(false) ? 1 : 0) : target->beamOracleInTopK;
        target->beamHit = c.contains(QStringLiteral("beam_hit")) ? (c.value(QStringLiteral("beam_hit")).toBool(false) ? 1 : 0) : target->beamHit;
        target->beamSelectionSource = c.value(QStringLiteral("beam_selection_source")).toString(target->beamSelectionSource);
        if (c.value(QStringLiteral("beam_topk_indices")).isArray()) {
            QStringList parts;
            for (const auto& v : c.value(QStringLiteral("beam_topk_indices")).toArray()) {
                parts << QString::number(v.toInt());
            }
            target->beamTopkIndices = parts.join(QStringLiteral(","));
        } else {
            target->beamTopkIndices = c.value(QStringLiteral("beam_topk_indices")).toString(target->beamTopkIndices);
        }
        if (target->servingBsName.trimmed().isEmpty() && !currentSysSnapshot_.servingBsName.trimmed().isEmpty()) {
            target->servingBsName = currentSysSnapshot_.servingBsName;
        }
        if (target->servingAnchorId < 0 && currentSysSnapshot_.servingAnchorId >= 0) {
            target->servingAnchorId = currentSysSnapshot_.servingAnchorId;
        }
        target->isServingBs = c.value(QStringLiteral("sys_is_serving_bs")).toBool(target->isServingBs)
            || sysCandidateMatchesServing(*target, currentSysSnapshot_);
        normalizeSysCandidateBeamFields(target);
        target->beamSelectedAnchor = target->isServingBs && sysCandidateHasBeamData(*target);

        if (target->isServingBs) {
            if (currentSysSnapshot_.servingBsName.trimmed().isEmpty() && !target->bsName.trimmed().isEmpty()) {
                currentSysSnapshot_.servingBsName = target->bsName;
            }
            if (currentSysSnapshot_.servingAnchorId < 0 && target->anchorId >= 0) {
                currentSysSnapshot_.servingAnchorId = target->anchorId;
            }
            syncCurrentBeamSummaryFromCandidate(*target, &c);
        }
    }

    normalizeCurrentSysCandidates();

    scheduleSysDataWindowRefresh();
    refreshInfoPanel();
}

void MainWindow::onStatusMessage(const QString& message) {
    lastStatusMessage_ = message;
    statusBar()->showMessage(message, 5000);
    appendSimulatorLog(QStringLiteral("[status] %1").arg(message));
    refreshInfoPanel();
}

void MainWindow::onLayerItemChanged(QTreeWidgetItem* item, int column) {
    if (column != 0 || !item) {
        return;
    }
    const QString key = item->data(0, Qt::UserRole).toString();
    applyLayerState(key, item->checkState(0) == Qt::Checked);
}

void MainWindow::onSimulatorOutput() {
    const QString text = QString::fromLocal8Bit(simulatorProcess_->readAllStandardOutput());
    if (!text.trimmed().isEmpty()) {
        const QStringList lines = text.split(QRegularExpression("[\r\n]+"), QString::SkipEmptyParts);
        const bool preferRosTopicState = rosBridge_ && rosBridge_->isRunning();
        for (const QString& line : lines) {
            appendSimulatorLog(line);
            if (line.startsWith(QStringLiteral("[sys-log] "))) {
                if (preferRosTopicState) {
                    continue;
                }
                const QByteArray payload = line.mid(QStringLiteral("[sys-log] ").size()).toUtf8();
                QJsonParseError parseError;
                const QJsonDocument doc = QJsonDocument::fromJson(payload, &parseError);
                if (parseError.error == QJsonParseError::NoError && doc.isObject()) {
                    const QJsonObject obj = doc.object();
                    const bool enabled = obj.value(QStringLiteral("enabled")).toBool(simSettings_.enableSysIntegration);
                    sysModeSummary_ = enabled ? QStringLiteral("Enabled") : QStringLiteral("Disabled");

                    currentSysSnapshot_.valid = true;
                    currentSysSnapshot_.enabled = enabled;
                    currentSysSnapshot_.schedulerPolicy = obj.value(QStringLiteral("scheduler_policy")).toString();
                    currentSysSnapshot_.servingIdx = obj.value(QStringLiteral("serving_idx")).toInt(-1);

                    const QString servingName = obj.value(QStringLiteral("serving_bs_name")).toString().trimmed();
                    currentSysSnapshot_.servingBsName = servingName;
                    currentSysSnapshot_.servingAnchorId = obj.value(QStringLiteral("serving_anchor_id")).toInt(-1);
                    currentSysSnapshot_.servingMcsIndex = obj.value(QStringLiteral("serving_mcs_index")).toInt(-1);
                    currentSysSnapshot_.servingSinrEffDb = obj.value(QStringLiteral("serving_sinr_eff_db")).toDouble(std::numeric_limits<double>::quiet_NaN());
                    currentSysSnapshot_.servingSpectralEfficiencyBpsHz = obj.value(QStringLiteral("serving_spectral_efficiency_bpshz")).toDouble(std::numeric_limits<double>::quiet_NaN());
                    currentSysSnapshot_.servingTbOk = obj.value(QStringLiteral("serving_tb_ok")).isBool()
                        ? (obj.value(QStringLiteral("serving_tb_ok")).toBool() ? 1 : 0)
                        : -1;

                    if (servingName.isEmpty()) {
                        sysServingSummary_ = QStringLiteral("-");
                    } else {
                        const int anchorId = currentSysSnapshot_.servingAnchorId;
                        sysServingSummary_ = anchorId >= 0
                            ? QStringLiteral("%1 (#%2)").arg(servingName).arg(anchorId)
                            : servingName;
                    }

                    const int mcsIndex = currentSysSnapshot_.servingMcsIndex;
                    const double sinrEffDb = currentSysSnapshot_.servingSinrEffDb;
                    const double seBpsHz = currentSysSnapshot_.servingSpectralEfficiencyBpsHz;
                    const QString tbText = ackStateLabel(currentSysSnapshot_.servingTbOk);
                    QStringList sysParts;
                    if (mcsIndex >= 0) {
                        sysParts << QStringLiteral("MCS %1").arg(mcsIndex);
                    }
                    if (std::isfinite(sinrEffDb)) {
                        sysParts << QStringLiteral("SINR %1 dB").arg(QString::number(sinrEffDb, 'f', 1));
                    }
                    if (std::isfinite(seBpsHz)) {
                        sysParts << QStringLiteral("SE %1").arg(QString::number(seBpsHz, 'f', 2));
                    }
                    sysParts << tbText;
                    sysLinkSummary_ = sysParts.join(QStringLiteral(" | "));
                    normalizeCurrentSysCandidates();
                    for (const auto& candidate : currentSysSnapshot_.candidates) {
                        if (candidate.isServingBs) {
                            syncCurrentBeamSummaryFromCandidate(candidate);
                            break;
                        }
                    }
                    refreshInfoPanel();
                }
            } else if (line.startsWith(QStringLiteral("[beam-log] "))) {
                if (preferRosTopicState) {
                    continue;
                }
                const QByteArray payload = line.mid(QStringLiteral("[beam-log] ").size()).toUtf8();
                QJsonParseError parseError;
                const QJsonDocument doc = QJsonDocument::fromJson(payload, &parseError);
                if (parseError.error == QJsonParseError::NoError && doc.isObject()) {
                    const QJsonObject obj = doc.object();
                    currentSysSnapshot_.valid = true;
                    currentSysSnapshot_.beamEnabled = obj.value(QStringLiteral("enabled")).toBool(simSettings_.enableBeamforming);
                    currentSysSnapshot_.beamSelectionMode = obj.value(QStringLiteral("selection_mode")).toString();
                    currentSysSnapshot_.beamCodebookType = obj.value(QStringLiteral("codebook_type")).toString();
                    currentSysSnapshot_.beamNumBeams = obj.value(QStringLiteral("num_beams")).toInt(-1);
                    currentSysSnapshot_.beamReferenceBsName = obj.value(QStringLiteral("reference_bs_name")).toString();
                    currentSysSnapshot_.beamReferenceAnchorId = obj.value(QStringLiteral("reference_anchor_id")).toInt(-1);
                    currentSysSnapshot_.beamReferenceTxIdx = obj.value(QStringLiteral("reference_tx_idx")).toInt(-1);
                    currentSysSnapshot_.beamSelectedIndex = obj.value(QStringLiteral("selected_beam_index")).toInt(-1);
                    currentSysSnapshot_.beamSelectedGainDb = obj.value(QStringLiteral("selected_beam_gain_db")).toDouble(std::numeric_limits<double>::quiet_NaN());
                    currentSysSnapshot_.beamOracleIndex = obj.value(QStringLiteral("oracle_beam_index")).toInt(-1);
                    currentSysSnapshot_.beamOracleGainDb = obj.value(QStringLiteral("oracle_beam_gain_db")).toDouble(std::numeric_limits<double>::quiet_NaN());
                    currentSysSnapshot_.beamManualIndex = obj.value(QStringLiteral("manual_beam_index")).toInt(-1);
                    currentSysSnapshot_.beamServingSource = obj.value(QStringLiteral("serving_source")).toString();
                    currentSysSnapshot_.beamPredictorAvailable = obj.value(QStringLiteral("predictor_available")).toBool(false);
                    currentSysSnapshot_.beamPredictorStatus = obj.value(QStringLiteral("predictor_status")).toString();
                    currentSysSnapshot_.beamPredictorError = obj.value(QStringLiteral("predictor_error")).toString();
                    currentSysSnapshot_.beamFeatureMode = obj.value(QStringLiteral("feature_mode")).toString();
                    currentSysSnapshot_.beamTopK = obj.value(QStringLiteral("top_k")).toInt(-1);
                    currentSysSnapshot_.beamPredictedIndex = obj.value(QStringLiteral("predicted_beam_index")).toInt(-1);
                    currentSysSnapshot_.beamPredictedConfidence = obj.value(QStringLiteral("predicted_beam_confidence")).toDouble(std::numeric_limits<double>::quiet_NaN());
                    currentSysSnapshot_.beamOracleInTopK = obj.contains(QStringLiteral("oracle_in_topk")) ? (obj.value(QStringLiteral("oracle_in_topk")).toBool(false) ? 1 : 0) : -1;
                    currentSysSnapshot_.beamHit = obj.contains(QStringLiteral("beam_hit")) ? (obj.value(QStringLiteral("beam_hit")).toBool(false) ? 1 : 0) : -1;
                    currentSysSnapshot_.beamSelectionSource = obj.value(QStringLiteral("selection_source")).toString();
                    {
                        const QJsonValue topkValue = obj.value(QStringLiteral("topk_indices"));
                        if (topkValue.isArray()) {
                            QStringList parts;
                            for (const auto& v : topkValue.toArray()) parts << QString::number(v.toInt());
                            currentSysSnapshot_.beamTopkIndices = parts.join(QStringLiteral(","));
                        } else {
                            currentSysSnapshot_.beamTopkIndices = topkValue.toString();
                        }
                    }
                    normalizeSysSnapshotBeamFields(&currentSysSnapshot_);
                    normalizeCurrentSysCandidates();
                    scheduleSysDataWindowRefresh();
                    refreshInfoPanel();
                }
            } else if (line.startsWith(QStringLiteral("[sim-log] "))) {
                if (preferRosTopicState) {
                    continue;
                }
                const QByteArray payload = line.mid(QStringLiteral("[sim-log] ").size()).toUtf8();
                QJsonParseError parseError;
                const QJsonDocument doc = QJsonDocument::fromJson(payload, &parseError);
                if (parseError.error == QJsonParseError::NoError && doc.isObject()) {
                    const QJsonObject obj = doc.object();
                    if (obj.contains(QStringLiteral("sys_enabled"))
                        || obj.contains(QStringLiteral("sys_candidate_rate_bpshz"))
                        || obj.contains(QStringLiteral("sys_is_serving_bs"))) {
                        const int simIdx = obj.value(QStringLiteral("sim_idx")).toInt(-1);
                        if (simIdx >= 0 && currentSysSnapshot_.simIdx != simIdx) {
                            currentSysSnapshot_.candidates.clear();
                            currentSysSnapshot_.simIdx = simIdx;
                        }
                        currentSysSnapshot_.valid = true;
                        currentSysSnapshot_.simIdx = simIdx >= 0 ? simIdx : currentSysSnapshot_.simIdx;
                        const double odomStamp = obj.value(QStringLiteral("odom_stamp_s")).toDouble(std::numeric_limits<double>::quiet_NaN());
                        if (std::isfinite(odomStamp)) {
                            currentSysSnapshot_.odomStampS = odomStamp;
                        }
                        if (obj.contains(QStringLiteral("sys_enabled"))) {
                            currentSysSnapshot_.enabled = obj.value(QStringLiteral("sys_enabled")).toBool(currentSysSnapshot_.enabled);
                        }

                        SysCandidateView candidate;
                        SysCandidateView* existingCandidate = nullptr;
                        for (auto& existing : currentSysSnapshot_.candidates) {
                            if (sysCandidateMatchesIdentity(existing, obj.value(QStringLiteral("anchor_id")).toInt(-1), obj.value(QStringLiteral("bs_name")).toString())) {
                                existingCandidate = &existing;
                                candidate = existing;
                                break;
                            }
                        }
                        candidate.simIdx = simIdx;
                        candidate.odomStampS = odomStamp;
                        candidate.bsName = obj.value(QStringLiteral("bs_name")).toString();
                        candidate.anchorId = obj.value(QStringLiteral("anchor_id")).toInt(-1);
                        candidate.numPaths = obj.value(QStringLiteral("num_paths")).toInt(-1);
                        candidate.strongestPathPowerDb = obj.value(QStringLiteral("power_db")).toDouble(std::numeric_limits<double>::quiet_NaN());
                        candidate.strongestPathType = obj.value(QStringLiteral("path_type")).toString();
                        candidate.strongestPathOrder = obj.value(QStringLiteral("path_order")).toInt(-1);
                        candidate.sysEnabled = obj.value(QStringLiteral("sys_enabled")).toBool(currentSysSnapshot_.enabled);
                        candidate.candidateRateBpsHz = obj.value(QStringLiteral("sys_candidate_rate_bpshz")).toDouble(std::numeric_limits<double>::quiet_NaN());
                        candidate.candidateSinrEffDb = obj.value(QStringLiteral("sys_candidate_sinr_eff_db")).toDouble(std::numeric_limits<double>::quiet_NaN());
                        candidate.mcsIndex = obj.value(QStringLiteral("sys_mcs_index")).toInt(-1);
                        candidate.numDecodedBits = obj.value(QStringLiteral("sys_num_decoded_bits")).toInt(-1);
                        candidate.tbOk = obj.value(QStringLiteral("sys_tb_ok")).toInt(-1);
                        candidate.spectralEfficiencyBpsHz = obj.value(QStringLiteral("sys_spectral_efficiency_bpshz")).toDouble(std::numeric_limits<double>::quiet_NaN());
                        candidate.isServingBs = obj.value(QStringLiteral("sys_is_serving_bs")).toBool(false);
                        candidate.servingBsName = obj.value(QStringLiteral("sys_serving_bs_name")).toString();
                        candidate.servingAnchorId = obj.value(QStringLiteral("sys_serving_anchor_id")).toInt(-1);
                        candidate.blerTarget = obj.value(QStringLiteral("sys_bler_target")).toDouble(std::numeric_limits<double>::quiet_NaN());
                        if (obj.contains(QStringLiteral("beam_enabled"))) candidate.beamEnabled = obj.value(QStringLiteral("beam_enabled")).toBool(candidate.beamEnabled);
                        if (obj.contains(QStringLiteral("beam_selection_mode"))) candidate.beamSelectionMode = obj.value(QStringLiteral("beam_selection_mode")).toString(candidate.beamSelectionMode);
                        if (obj.contains(QStringLiteral("beam_codebook_type"))) candidate.beamCodebookType = obj.value(QStringLiteral("beam_codebook_type")).toString(candidate.beamCodebookType);
                        if (obj.contains(QStringLiteral("beam_num_beams"))) candidate.beamNumBeams = obj.value(QStringLiteral("beam_num_beams")).toInt(candidate.beamNumBeams);
                        if (obj.contains(QStringLiteral("beam_selected_index"))) candidate.beamSelectedIndex = obj.value(QStringLiteral("beam_selected_index")).toInt(candidate.beamSelectedIndex);
                        if (obj.contains(QStringLiteral("beam_selected_gain_db"))) candidate.beamSelectedGainDb = obj.value(QStringLiteral("beam_selected_gain_db")).toDouble(candidate.beamSelectedGainDb);
                        if (obj.contains(QStringLiteral("beam_oracle_index"))) candidate.beamOracleIndex = obj.value(QStringLiteral("beam_oracle_index")).toInt(candidate.beamOracleIndex);
                        if (obj.contains(QStringLiteral("beam_oracle_gain_db"))) candidate.beamOracleGainDb = obj.value(QStringLiteral("beam_oracle_gain_db")).toDouble(candidate.beamOracleGainDb);
                        if (obj.contains(QStringLiteral("beam_selected_anchor"))) candidate.beamSelectedAnchor = obj.value(QStringLiteral("beam_selected_anchor")).toBool(candidate.beamSelectedAnchor);
                        if (obj.contains(QStringLiteral("beam_predictor_status"))) candidate.beamPredictorStatus = obj.value(QStringLiteral("beam_predictor_status")).toString(candidate.beamPredictorStatus);
                        if (obj.contains(QStringLiteral("beam_feature_mode"))) candidate.beamFeatureMode = obj.value(QStringLiteral("beam_feature_mode")).toString(candidate.beamFeatureMode);
                        if (obj.contains(QStringLiteral("beam_top_k"))) candidate.beamTopK = obj.value(QStringLiteral("beam_top_k")).toInt(candidate.beamTopK);
                        if (obj.contains(QStringLiteral("beam_predicted_index"))) candidate.beamPredictedIndex = obj.value(QStringLiteral("beam_predicted_index")).toInt(candidate.beamPredictedIndex);
                        if (obj.contains(QStringLiteral("beam_predicted_confidence"))) candidate.beamPredictedConfidence = obj.value(QStringLiteral("beam_predicted_confidence")).toDouble(candidate.beamPredictedConfidence);
                        if (obj.contains(QStringLiteral("beam_oracle_in_topk"))) candidate.beamOracleInTopK = obj.value(QStringLiteral("beam_oracle_in_topk")).toInt(candidate.beamOracleInTopK);
                        if (obj.contains(QStringLiteral("beam_hit"))) candidate.beamHit = obj.value(QStringLiteral("beam_hit")).toInt(candidate.beamHit);
                        if (obj.contains(QStringLiteral("beam_selection_source"))) candidate.beamSelectionSource = obj.value(QStringLiteral("beam_selection_source")).toString(candidate.beamSelectionSource);
                        if (obj.contains(QStringLiteral("beam_topk_indices"))) candidate.beamTopkIndices = obj.value(QStringLiteral("beam_topk_indices")).toString(candidate.beamTopkIndices);
                        if (candidate.servingBsName.trimmed().isEmpty() && !currentSysSnapshot_.servingBsName.trimmed().isEmpty()) {
                            candidate.servingBsName = currentSysSnapshot_.servingBsName;
                        }
                        if (candidate.servingAnchorId < 0 && currentSysSnapshot_.servingAnchorId >= 0) {
                            candidate.servingAnchorId = currentSysSnapshot_.servingAnchorId;
                        }
                        candidate.isServingBs = candidate.isServingBs || sysCandidateMatchesServing(candidate, currentSysSnapshot_);
                        normalizeSysCandidateBeamFields(&candidate);

                        bool updated = false;
                        for (auto& existing : currentSysSnapshot_.candidates) {
                            if (sysCandidateMatchesIdentity(existing, candidate.anchorId, candidate.bsName)) {
                                existing = candidate;
                                updated = true;
                                break;
                            }
                        }
                        if (!updated) {
                            currentSysSnapshot_.candidates.push_back(candidate);
                        }

                        if (candidate.isServingBs) {
                            currentSysSnapshot_.servingBsName = candidate.servingBsName.isEmpty() ? candidate.bsName : candidate.servingBsName;
                            currentSysSnapshot_.servingAnchorId = candidate.servingAnchorId >= 0 ? candidate.servingAnchorId : candidate.anchorId;
                            if (candidate.mcsIndex >= 0) currentSysSnapshot_.servingMcsIndex = candidate.mcsIndex;
                            if (std::isfinite(candidate.candidateSinrEffDb)) currentSysSnapshot_.servingSinrEffDb = candidate.candidateSinrEffDb;
                            if (std::isfinite(candidate.spectralEfficiencyBpsHz)) currentSysSnapshot_.servingSpectralEfficiencyBpsHz = candidate.spectralEfficiencyBpsHz;
                            if (candidate.tbOk >= 0) currentSysSnapshot_.servingTbOk = candidate.tbOk;
                            syncCurrentBeamSummaryFromCandidate(candidate, &obj);

                            sysModeSummary_ = currentSysSnapshot_.enabled ? QStringLiteral("Enabled") : QStringLiteral("Disabled");
                            sysServingSummary_ = currentSysSnapshot_.servingAnchorId >= 0
                                ? QStringLiteral("%1 (#%2)").arg(currentSysSnapshot_.servingBsName).arg(currentSysSnapshot_.servingAnchorId)
                                : currentSysSnapshot_.servingBsName;
                            QStringList sysParts;
                            if (currentSysSnapshot_.servingMcsIndex >= 0) {
                                sysParts << QStringLiteral("MCS %1").arg(currentSysSnapshot_.servingMcsIndex);
                            }
                            if (std::isfinite(currentSysSnapshot_.servingSinrEffDb)) {
                                sysParts << QStringLiteral("SINR %1 dB").arg(QString::number(currentSysSnapshot_.servingSinrEffDb, 'f', 1));
                            }
                            if (std::isfinite(currentSysSnapshot_.servingSpectralEfficiencyBpsHz)) {
                                sysParts << QStringLiteral("SE %1").arg(QString::number(currentSysSnapshot_.servingSpectralEfficiencyBpsHz, 'f', 2));
                            }
                            sysParts << ackStateLabel(currentSysSnapshot_.servingTbOk);
                            sysLinkSummary_ = sysParts.join(QStringLiteral(" | "));
                            if (sysModeValue_) sysModeValue_->setText(sysModeSummary_);
                            if (sysServingValue_) sysServingValue_->setText(sysServingSummary_);
                            if (sysLinkValue_) sysLinkValue_->setText(sysLinkSummary_);
                        }

                        Q_UNUSED(existingCandidate);
                        normalizeCurrentSysCandidates();
                        scheduleSysDataWindowRefresh();
                    }
                }
            }
        }
        statusBar()->showMessage(text.trimmed().left(300), 3000);
    }
}

void MainWindow::onSimulatorError(QProcess::ProcessError error) {
    Q_UNUSED(error)
    appendSimulatorLog(QStringLiteral("[gui] Internal simulator process error: %1").arg(simulatorProcess_->errorString()));
    onStatusMessage(QString("Internal simulator process error: %1").arg(simulatorProcess_->errorString()));
}

void MainWindow::onSimulatorFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    appendSimulatorLog(QStringLiteral("[gui] Internal simulator stopped (exitCode=%1, status=%2)").arg(exitCode).arg(exitStatus == QProcess::NormalExit ? QStringLiteral("normal") : QStringLiteral("crash")));
    onStatusMessage(QString("Internal simulator stopped (exitCode=%1, status=%2)").arg(exitCode).arg(exitStatus == QProcess::NormalExit ? "normal" : "crash"));
    refreshInfoPanel();
}

}  // namespace airsim_gui
