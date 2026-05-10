#include "airsim_gui_UErealtime/TransformUtils.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonValue>

#include <ros/package.h>

#include <algorithm>
#include <cmath>

namespace airsim_gui {

TransformMatrix4x4 identityTransformMatrix4x4() {
    TransformMatrix4x4 out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out.m[r][c] = (r == c) ? 1.0 : 0.0;
        }
    }
    out.valid = true;
    return out;
}

bool invertTransformMatrix4x4(const TransformMatrix4x4& in, TransformMatrix4x4* out) {
    if (!out) {
        return false;
    }
    double a[4][8];
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            a[r][c] = in.m[r][c];
            a[r][c + 4] = (r == c) ? 1.0 : 0.0;
        }
    }
    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        double best = std::abs(a[col][col]);
        for (int r = col + 1; r < 4; ++r) {
            const double cand = std::abs(a[r][col]);
            if (cand > best) {
                best = cand;
                pivot = r;
            }
        }
        if (best < 1e-10) {
            return false;
        }
        if (pivot != col) {
            for (int c = 0; c < 8; ++c) {
                std::swap(a[pivot][c], a[col][c]);
            }
        }
        const double div = a[col][col];
        for (int c = 0; c < 8; ++c) {
            a[col][c] /= div;
        }
        for (int r = 0; r < 4; ++r) {
            if (r == col) {
                continue;
            }
            const double factor = a[r][col];
            for (int c = 0; c < 8; ++c) {
                a[r][c] -= factor * a[col][c];
            }
        }
    }
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out->m[r][c] = a[r][c + 4];
        }
    }
    out->valid = true;
    return true;
}

Vec3 transformPointByMatrix(const TransformMatrix4x4& matrix, const Vec3& point) {
    const double x = matrix.m[0][0] * point.x + matrix.m[0][1] * point.y + matrix.m[0][2] * point.z + matrix.m[0][3];
    const double y = matrix.m[1][0] * point.x + matrix.m[1][1] * point.y + matrix.m[1][2] * point.z + matrix.m[1][3];
    const double z = matrix.m[2][0] * point.x + matrix.m[2][1] * point.y + matrix.m[2][2] * point.z + matrix.m[2][3];
    const double w = matrix.m[3][0] * point.x + matrix.m[3][1] * point.y + matrix.m[3][2] * point.z + matrix.m[3][3];
    if (std::abs(w) > 1e-10 && std::abs(w - 1.0) > 1e-10) {
        return {x / w, y / w, z / w};
    }
    return {x, y, z};
}

Vec3 transformVectorByMatrix(const TransformMatrix4x4& matrix, const Vec3& vec) {
    return {
        matrix.m[0][0] * vec.x + matrix.m[0][1] * vec.y + matrix.m[0][2] * vec.z,
        matrix.m[1][0] * vec.x + matrix.m[1][1] * vec.y + matrix.m[1][2] * vec.z,
        matrix.m[2][0] * vec.x + matrix.m[2][1] * vec.y + matrix.m[2][2] * vec.z,
    };
}

TransformMatrix4x4 loadTransformMatrixFromJson(const QString& filePath, QString* errorMessage) {
    if (errorMessage) {
        errorMessage->clear();
    }

    if (filePath.trimmed().isEmpty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Transform matrix path is empty.");
        }
        return identityTransformMatrix4x4();
    }

    QFile file(filePath);
    if (!file.exists()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Transform matrix file does not exist: %1").arg(filePath);
        }
        return identityTransformMatrix4x4();
    }
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Could not open transform matrix file: %1").arg(filePath);
        }
        return identityTransformMatrix4x4();
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isArray()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to parse transform matrix JSON %1: %2")
                .arg(filePath, parseError.errorString());
        }
        return identityTransformMatrix4x4();
    }

    const QJsonArray rows = doc.array();
    if (rows.size() != 4) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Transform matrix in %1 must have 4 rows.").arg(filePath);
        }
        return identityTransformMatrix4x4();
    }

    TransformMatrix4x4 matrix = identityTransformMatrix4x4();
    for (int r = 0; r < 4; ++r) {
        const QJsonValue rowValue = rows.at(r);
        if (!rowValue.isArray()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("Row %1 in %2 is not an array.").arg(r).arg(filePath);
            }
            return identityTransformMatrix4x4();
        }
        const QJsonArray row = rowValue.toArray();
        if (row.size() != 4) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("Row %1 in %2 must have 4 columns.").arg(r).arg(filePath);
            }
            return identityTransformMatrix4x4();
        }
        for (int c = 0; c < 4; ++c) {
            matrix.m[r][c] = row.at(c).toDouble((r == c) ? 1.0 : 0.0);
        }
    }
    matrix.valid = true;
    return matrix;
}

bool saveTransformMatrixToJson(const QString& filePath, const TransformMatrix4x4& matrix, QString* errorMessage) {
    if (errorMessage) {
        errorMessage->clear();
    }

    const QString trimmedPath = filePath.trimmed();
    if (trimmedPath.isEmpty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Transform matrix path is empty.");
        }
        return false;
    }

    const QFileInfo info(trimmedPath);
    const QDir dir = info.dir();
    if (!dir.exists() && !QDir().mkpath(dir.absolutePath())) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Could not create directory for transform matrix file: %1").arg(dir.absolutePath());
        }
        return false;
    }

    QJsonArray rows;
    for (int r = 0; r < 4; ++r) {
        QJsonArray row;
        for (int c = 0; c < 4; ++c) {
            row.append(matrix.m[r][c]);
        }
        rows.append(row);
    }

    QFile file(trimmedPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Could not write transform matrix file: %1").arg(trimmedPath);
        }
        return false;
    }

    const QJsonDocument doc(rows);
    if (file.write(doc.toJson(QJsonDocument::Indented)) < 0) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to write transform matrix JSON: %1").arg(trimmedPath);
        }
        return false;
    }
    file.close();
    return true;
}

QString transformMatrixToDisplayString(const TransformMatrix4x4& matrix) {
    QStringList lines;
    for (int r = 0; r < 4; ++r) {
        QStringList cols;
        for (int c = 0; c < 4; ++c) {
            cols << QString::number(matrix.m[r][c], 'f', 6);
        }
        lines << QStringLiteral("[ %1 ]").arg(cols.join(QStringLiteral(", ")));
    }
    return lines.join(QStringLiteral("\n"));
}

QString findPackagedConfigFile(const QString& relativePath) {
    const std::string pkgPathStd = ros::package::getPath("airsim_gui_UErealtime");
    if (!pkgPathStd.empty()) {
        const QString pkgPath = QString::fromStdString(pkgPathStd);
        const QString candidate = QDir(pkgPath).filePath(relativePath);
        if (QFileInfo::exists(candidate) || QFileInfo(candidate).absolutePath().contains(QStringLiteral("airsim_gui_UErealtime"))) {
            return QFileInfo(candidate).absoluteFilePath();
        }
    }

    const QString base = QCoreApplication::applicationDirPath();
    const QString configName = QFileInfo(relativePath).fileName();
    const QStringList candidates = {
        QDir(base).filePath(relativePath),
        QDir(base).filePath(QStringLiteral("../") + relativePath),
        QDir(base).filePath(QStringLiteral("../../") + relativePath),
        QDir(base).filePath(QStringLiteral("../../../src/airsim_gui_UErealtime/") + relativePath),
        QDir(base).filePath(QStringLiteral("../../../../src/airsim_gui_UErealtime/") + relativePath),
        QDir(base).filePath(QStringLiteral("../share/airsim_gui_UErealtime/") + relativePath),
        QDir(base).filePath(QStringLiteral("../../share/airsim_gui_UErealtime/") + relativePath),
        QDir(base).filePath(QStringLiteral("../../../share/airsim_gui_UErealtime/") + relativePath),
        QDir(base).filePath(QStringLiteral("../share/airsim_gui_UErealtime/config/") + configName),
        QDir(base).filePath(QStringLiteral("../../share/airsim_gui_UErealtime/config/") + configName),
        QDir(base).filePath(QStringLiteral("../../../share/airsim_gui_UErealtime/config/") + configName)
    };

    for (const QString& candidate : candidates) {
        if (QFileInfo::exists(candidate)) {
            return QFileInfo(candidate).absoluteFilePath();
        }
    }
    return QFileInfo(QDir(base).filePath(relativePath)).absoluteFilePath();
}

}  // namespace airsim_gui
