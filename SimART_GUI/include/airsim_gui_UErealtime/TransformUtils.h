#pragma once

#include "airsim_gui_UErealtime/AppTypes.h"

#include <QString>
#include <array>

namespace airsim_gui {

struct TransformMatrix4x4 {
    std::array<std::array<double, 4>, 4> m{};
    bool valid{false};
};

TransformMatrix4x4 identityTransformMatrix4x4();
bool invertTransformMatrix4x4(const TransformMatrix4x4& in, TransformMatrix4x4* out);
Vec3 transformPointByMatrix(const TransformMatrix4x4& matrix, const Vec3& point);
Vec3 transformVectorByMatrix(const TransformMatrix4x4& matrix, const Vec3& vec);
TransformMatrix4x4 loadTransformMatrixFromJson(const QString& filePath, QString* errorMessage = nullptr);
bool saveTransformMatrixToJson(const QString& filePath, const TransformMatrix4x4& matrix, QString* errorMessage = nullptr);
QString transformMatrixToDisplayString(const TransformMatrix4x4& matrix);
QString findPackagedConfigFile(const QString& relativePath);

}  // namespace airsim_gui
