#include "airsim_gui_UErealtime/Scene3DWidget.h"

#include "airsim_gui_UErealtime/FileSceneProvider.h"

#include <QEvent>
#include <QCursor>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QPainter>
#include <QVBoxLayout>
#include <QToolButton>
#include <QLabel>
#include <QFrame>
#include <QWheelEvent>

#include <QVTKWidget.h>

#include <vtkActor2D.h>
#include <vtkActor.h>
#include <vtkAssembly.h>
#include <vtkAxesActor.h>
#include <vtkBoundingBox.h>
#include <vtkCamera.h>
#include <vtkCaptionActor2D.h>
#include <vtkCellArray.h>
#include <vtkCellPicker.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkCoordinate.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkFloatArray.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyLine.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkRegularPolygonSource.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

#include <algorithm>
#include <cmath>
#include <vtkTexture.h>

namespace airsim_gui {

namespace {

constexpr int kHoverPickIntervalMs = 90;

vtkSmartPointer<vtkActor> makeCubeActor(const Vec3& center, const Vec3& size, const Vec3& color) {
    auto cube = vtkSmartPointer<vtkCubeSource>::New();
    cube->SetCenter(center.x, center.y, center.z);
    cube->SetXLength(size.x);
    cube->SetYLength(size.y);
    cube->SetZLength(size.z);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cube->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color.x, color.y, color.z);
    actor->GetProperty()->SetOpacity(0.95);
    return actor;
}

void configureAxesCaption(vtkCaptionActor2D* caption) {
    if (!caption) {
        return;
    }
    caption->SetLeader(0);
    caption->SetBorder(0);
    caption->SetPadding(0);
    caption->SetWidth(0.05);
    caption->SetHeight(0.05);
    if (caption->GetTextActor()) {
        caption->GetTextActor()->SetTextScaleModeToNone();
    }
    if (caption->GetCaptionTextProperty()) {
        caption->GetCaptionTextProperty()->SetFontSize(14);
        caption->GetCaptionTextProperty()->SetBold(1);
        caption->GetCaptionTextProperty()->SetItalic(0);
        caption->GetCaptionTextProperty()->SetShadow(0);
    }
}

void configureAxesActor(vtkAxesActor* axesActor) {
    if (!axesActor) {
        return;
    }
    axesActor->SetTotalLength(15.0, 15.0, 15.0);
    axesActor->SetNormalizedLabelPosition(1.04, 1.04, 1.04);
    configureAxesCaption(axesActor->GetXAxisCaptionActor2D());
    configureAxesCaption(axesActor->GetYAxisCaptionActor2D());
    configureAxesCaption(axesActor->GetZAxisCaptionActor2D());
}

vtkSmartPointer<vtkActor> makeTetrahedronActor(const Vec3& center, double height, const Vec3& color) {
    auto points = vtkSmartPointer<vtkPoints>::New();
    constexpr double kPi = 3.14159265358979323846;
    const double baseRadius = height * 0.24;
    const double baseZ = center.z - height * 0.25;
    const double apexZ = center.z + height * 0.75;
    for (int i = 0; i < 3; ++i) {
        const double angle = kPi * 0.5 + static_cast<double>(i) * 2.0 * kPi / 3.0;
        points->InsertNextPoint(center.x + std::cos(angle) * baseRadius,
                                center.y + std::sin(angle) * baseRadius,
                                baseZ);
    }
    points->InsertNextPoint(center.x, center.y, apexZ);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    auto polys = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType faces[4][3] = {
        {0, 2, 1},
        {0, 1, 3},
        {1, 2, 3},
        {2, 0, 3}
    };
    for (int i = 0; i < 4; ++i) {
        polys->InsertNextCell(3, faces[i]);
    }
    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(polys);
    mapper->SetInputData(polyData);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color.x, color.y, color.z);
    actor->GetProperty()->SetOpacity(0.96);
    actor->GetProperty()->SetInterpolationToFlat();
    actor->GetProperty()->SetSpecular(0.25);
    actor->GetProperty()->SetSpecularPower(24.0);
    return actor;
}

vtkSmartPointer<vtkActor> makePolylineActor(const std::vector<Vec3>& points,
                                            const Vec3& color,
                                            double lineWidth,
                                            double opacity) {
    auto vtkPointsObj = vtkSmartPointer<vtkPoints>::New();
    auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
    polyLine->GetPointIds()->SetNumberOfIds(static_cast<vtkIdType>(points.size()));

    for (vtkIdType i = 0; i < static_cast<vtkIdType>(points.size()); ++i) {
        vtkPointsObj->InsertNextPoint(points[static_cast<size_t>(i)].x,
                                      points[static_cast<size_t>(i)].y,
                                      points[static_cast<size_t>(i)].z);
        polyLine->GetPointIds()->SetId(i, i);
    }

    auto cells = vtkSmartPointer<vtkCellArray>::New();
    cells->InsertNextCell(polyLine);

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(vtkPointsObj);
    polyData->SetLines(cells);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color.x, color.y, color.z);
    actor->GetProperty()->SetLineWidth(lineWidth);
    actor->GetProperty()->SetOpacity(opacity);
    return actor;
}

vtkSmartPointer<vtkTexture> makeTextureFromFile(const QString& texturePath) {
    if (texturePath.isEmpty() || !QFileInfo::exists(texturePath)) {
        return nullptr;
    }

    vtkSmartPointer<vtkImageReader2Factory> factory = vtkSmartPointer<vtkImageReader2Factory>::New();
    vtkImageReader2* rawReader = factory->CreateImageReader2(texturePath.toStdString().c_str());
    if (!rawReader) {
        return nullptr;
    }

    vtkSmartPointer<vtkImageReader2> reader = rawReader;
    reader->SetFileName(texturePath.toStdString().c_str());
    reader->Update();

    auto texture = vtkSmartPointer<vtkTexture>::New();
    texture->SetInputConnection(reader->GetOutputPort());
    texture->InterpolateOn();
    texture->RepeatOff();
    texture->EdgeClampOn();
    return texture;
}

vtkSmartPointer<vtkActor> makeTriangleMeshActor(const MeshGeometry& mesh) {
    auto points = vtkSmartPointer<vtkPoints>::New();
    for (size_t i = 0; i + 2 < mesh.vertices.size(); i += 3) {
        points->InsertNextPoint(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]);
    }

    auto polys = vtkSmartPointer<vtkCellArray>::New();
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        vtkIdType tri[3] = {
            static_cast<vtkIdType>(mesh.indices[i]),
            static_cast<vtkIdType>(mesh.indices[i + 1]),
            static_cast<vtkIdType>(mesh.indices[i + 2])
        };
        polys->InsertNextCell(3, tri);
    }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(polys);

    if (mesh.normals.size() >= points->GetNumberOfPoints() * 3) {
        auto normals = vtkSmartPointer<vtkFloatArray>::New();
        normals->SetName("Normals");
        normals->SetNumberOfComponents(3);
        normals->SetNumberOfTuples(points->GetNumberOfPoints());
        for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
            const float n[3] = {
                mesh.normals[static_cast<size_t>(i) * 3],
                mesh.normals[static_cast<size_t>(i) * 3 + 1],
                mesh.normals[static_cast<size_t>(i) * 3 + 2]
            };
            normals->SetTypedTuple(i, n);
        }
        polyData->GetPointData()->SetNormals(normals);
    }

    if (mesh.texCoords.size() >= points->GetNumberOfPoints() * 2) {
        auto tcoords = vtkSmartPointer<vtkFloatArray>::New();
        tcoords->SetName("TCoords");
        tcoords->SetNumberOfComponents(2);
        tcoords->SetNumberOfTuples(points->GetNumberOfPoints());
        for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
            const float uv[2] = {
                mesh.texCoords[static_cast<size_t>(i) * 2],
                mesh.texCoords[static_cast<size_t>(i) * 2 + 1]
            };
            tcoords->SetTypedTuple(i, uv);
        }
        polyData->GetPointData()->SetTCoords(tcoords);
    }

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    auto* prop = actor->GetProperty();
    prop->SetColor(mesh.material.diffuseColor.x(), mesh.material.diffuseColor.y(), mesh.material.diffuseColor.z());
    prop->SetOpacity(mesh.material.opacity);
    prop->SetAmbient(0.22);
    prop->SetDiffuse(0.78);
    prop->SetSpecular(0.06);
    prop->SetSpecularPower(18.0);

    if (!mesh.material.texturePath.isEmpty()) {
        if (auto texture = makeTextureFromFile(mesh.material.texturePath)) {
            actor->SetTexture(texture);
            prop->SetColor(1.0, 1.0, 1.0);
        }
    }

    actor->SetPosition(mesh.translation.x(), mesh.translation.y(), mesh.translation.z());
    actor->SetScale(mesh.scale.x(), mesh.scale.y(), mesh.scale.z());
    actor->SetOrientation(mesh.rotationDeg.x(), mesh.rotationDeg.y(), mesh.rotationDeg.z());
    return actor;
}

Vec3 brighten(const Vec3& color, double amount = 0.30) {
    return {
        std::min(1.0, color.x + amount),
        std::min(1.0, color.y + amount),
        std::min(1.0, color.z + amount)
    };
}


QString stationInfoBody(const airsim_gui::BaseStation& station, const airsim_gui::BeamPathList& paths) {
    int count = 0;
    QString strongestId;
    double strongestPower = -std::numeric_limits<double>::infinity();
    for (const auto& path : paths) {
        if (path.txId == station.name || path.txId == station.id) {
            ++count;
            if (strongestId.isEmpty() || path.powerDb > strongestPower) {
                strongestId = path.id;
                strongestPower = path.powerDb;
            }
        }
    }
    QString text = QStringLiteral("Name: %1\nPosition: %2\nPaths: %3")
        .arg(station.name, airsim_gui::vecToString(station.position))
        .arg(count);
    if (!strongestId.isEmpty()) {
        text += QStringLiteral("\nStrongest: %1 (%2 dB)").arg(strongestId).arg(strongestPower, 0, 'f', 1);
    }
    return text;
}

QFrame* createOverlayCard(QWidget* parent, QLabel** outLabel, QToolButton** outPin, bool pinnable, bool closable) {
    auto* frame = new QFrame(parent);
    frame->setFrameShape(QFrame::StyledPanel);
    frame->setStyleSheet(QStringLiteral("QFrame { background: rgba(24,28,36,235); border: 1px solid #6d7890; border-radius: 10px; } QLabel { color: #eef3f8; } QToolButton { color: #eef3f8; border: none; font-weight: 700; }"));
    frame->resize(250, 120);
    auto* root = new QVBoxLayout(frame);
    root->setContentsMargins(10, 8, 10, 8);
    root->setSpacing(6);
    auto* top = new QHBoxLayout();
    auto* title = new QLabel(QStringLiteral("Base Station"), frame);
    title->setStyleSheet(QStringLiteral("font-weight:700; font-size:13px;"));
    top->addWidget(title);
    top->addStretch(1);
    QToolButton* pinButton = nullptr;
    if (pinnable) {
        pinButton = new QToolButton(frame);
        pinButton->setText(QStringLiteral("Pin"));
        pinButton->setToolTip(QStringLiteral("Pin this card"));
        top->addWidget(pinButton);
    }
    if (closable) {
        auto* closeButton = new QToolButton(frame);
        closeButton->setText(QStringLiteral("×"));
        closeButton->setToolTip(QStringLiteral("Close"));
        QObject::connect(closeButton, &QToolButton::clicked, frame, &QWidget::deleteLater);
        top->addWidget(closeButton);
    }
    root->addLayout(top);
    auto* body = new QLabel(frame);
    body->setWordWrap(true);
    body->setTextInteractionFlags(Qt::TextSelectableByMouse);
    root->addWidget(body);
    if (outLabel) *outLabel = body;
    if (outPin) *outPin = pinButton;
    return frame;
}

}  // namespace

class ViewDragBall : public QWidget {
public:
    explicit ViewDragBall(QWidget* parent = nullptr)
        : QWidget(parent) {
        setObjectName(QStringLiteral("scene3DViewDragBall"));
        setAttribute(Qt::WA_TranslucentBackground, true);
        setAttribute(Qt::WA_NoSystemBackground, true);
        setAttribute(Qt::WA_AlwaysStackOnTop, true);
        setAutoFillBackground(false);
        setMouseTracking(true);
        setFocusPolicy(Qt::NoFocus);
        setCursor(Qt::OpenHandCursor);
        setToolTip(QStringLiteral("3D view camera gizmo: drag to orbit, right-drag to roll, wheel to zoom."));
    }

    ~ViewDragBall() override {
        removeCameraObserver();
    }

    void setRenderer(vtkRenderer* renderer, vtkRenderWindow* renderWindow) {
        renderer_ = renderer;
        renderWindow_ = renderWindow;
        syncCameraObserver();
        update();
    }

    void refreshFromCamera() {
        syncCameraObserver();
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        painter.setPen(QPen(QColor(103, 116, 139, 128), 1.0));
        painter.setBrush(QColor(15, 23, 42, 214));
        painter.drawRoundedRect(rect().adjusted(0, 0, -1, -1), 8.0, 8.0);

        const Basis basis = currentBasis();
        if (!basis.valid) {
            drawUnavailable(painter);
            drawRollButtons(painter);
            return;
        }

        std::vector<AxisItem> axes = axisItems(basis);
        painter.setFont(axisFont());
        for (const AxisItem& item : axes) {
            drawAxis(painter, item);
        }

        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(241, 245, 249, 205));
        painter.drawEllipse(centerPoint(), 3.2, 3.2);

        drawRollButtons(painter);
    }

    void mousePressEvent(QMouseEvent* event) override {
        if (!event || !(event->button() == Qt::LeftButton || event->button() == Qt::RightButton)) {
            QWidget::mousePressEvent(event);
            return;
        }

        if (event->button() == Qt::LeftButton) {
            const int buttonDirection = rollButtonAt(event->pos());
            if (buttonDirection != 0) {
                pressedRollDirection_ = buttonDirection;
                update();
                event->accept();
                return;
            }
            pressedAxis_ = axisAt(event->pos());
        }

        if (!orbitArea().contains(event->pos())) {
            event->ignore();
            return;
        }

        dragging_ = true;
        dragMoved_ = false;
        dragButton_ = event->button();
        dragStartPos_ = event->pos();
        dragLastPos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (!event) {
            return;
        }

        const int hovered = rollButtonAt(event->pos());
        if (hovered != hoveredRollDirection_) {
            hoveredRollDirection_ = hovered;
            update();
        }

        if (pressedRollDirection_ != 0) {
            event->accept();
            return;
        }

        if (!dragging_) {
            setCursor(hovered != 0 ? Qt::PointingHandCursor : Qt::OpenHandCursor);
            QWidget::mouseMoveEvent(event);
            return;
        }

        const QPoint delta = event->pos() - dragLastPos_;
        dragLastPos_ = event->pos();
        if ((event->pos() - dragStartPos_).manhattanLength() > 2) {
            dragMoved_ = true;
            pressedAxis_ = Vec3{};
        }
        if (!delta.isNull()) {
            if (dragButton_ == Qt::RightButton) {
                rollCameraPixels(delta.x());
            } else {
                orbitCamera(delta.x(), delta.y());
            }
        }
        event->accept();
    }

    void mouseReleaseEvent(QMouseEvent* event) override {
        if (!event) {
            return;
        }

        if (pressedRollDirection_ != 0) {
            const int direction = pressedRollDirection_;
            pressedRollDirection_ = 0;
            if (event->button() == Qt::LeftButton && rollButtonAt(event->pos()) == direction) {
                rollCameraQuarter(direction);
            }
            update();
            event->accept();
            return;
        }

        if (dragging_ && event->button() == dragButton_) {
            const Vec3 axis = pressedAxis_;
            dragging_ = false;
            dragButton_ = Qt::NoButton;
            pressedAxis_ = Vec3{};
            setCursor(Qt::OpenHandCursor);
            if (!dragMoved_ && norm(axis) > 0.5 && axisEqual(axis, axisAt(event->pos()))) {
                setCameraToAxis(axis);
            }
            event->accept();
            return;
        }

        QWidget::mouseReleaseEvent(event);
    }

    void leaveEvent(QEvent* event) override {
        if (!dragging_) {
            setCursor(Qt::OpenHandCursor);
        }
        hoveredRollDirection_ = 0;
        update();
        QWidget::leaveEvent(event);
    }

    void wheelEvent(QWheelEvent* event) override {
        if (!event || !rect().contains(event->pos())) {
            QWidget::wheelEvent(event);
            return;
        }
        const int delta = event->angleDelta().y();
        if (delta != 0) {
            zoomCamera(delta);
        }
        event->accept();
    }

private:
    struct Basis {
        Vec3 right;
        Vec3 up;
        Vec3 forward;
        bool valid{false};
    };

    struct AxisItem {
        QString label;
        Vec3 axis;
        QColor color;
        QPointF point;
        double depth{0.0};
        qreal diameter{18.0};
    };

    static double dot(const Vec3& a, const Vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    static double norm(const Vec3& v) {
        return std::sqrt(dot(v, v));
    }

    static Vec3 normalized(const Vec3& v) {
        const double length = norm(v);
        if (length < 1e-9 || !std::isfinite(length)) {
            return {0.0, 0.0, 0.0};
        }
        return {v.x / length, v.y / length, v.z / length};
    }

    static bool axisEqual(const Vec3& a, const Vec3& b) {
        return std::fabs(a.x - b.x) < 1e-6
            && std::fabs(a.y - b.y) < 1e-6
            && std::fabs(a.z - b.z) < 1e-6;
    }

    QPointF centerPoint() const {
        return QPointF(width() * 0.5, 56.0);
    }

    QRect orbitArea() const {
        return QRect(0, 0, width(), 108);
    }

    QRect rollButtonRect(int direction) const {
        const int x = direction < 0 ? 18 : 70;
        return QRect(x, 112, 40, 20);
    }

    int rollButtonAt(const QPoint& pos) const {
        if (rollButtonRect(-1).contains(pos)) {
            return -1;
        }
        if (rollButtonRect(1).contains(pos)) {
            return 1;
        }
        return 0;
    }

    QFont axisFont() const {
        QFont f = font();
        f.setPixelSize(10);
        f.setBold(true);
        return f;
    }

    QFont buttonFont() const {
        QFont f = font();
        f.setPixelSize(10);
        f.setBold(true);
        return f;
    }

    void drawUnavailable(QPainter& painter) const {
        painter.setPen(QColor(203, 213, 225, 170));
        painter.setFont(axisFont());
        painter.drawText(orbitArea(), Qt::AlignCenter, QStringLiteral("3D"));
    }

    void drawAxis(QPainter& painter, const AxisItem& item) const {
        const bool front = item.depth >= 0.0;
        QColor lineColor = item.color;
        lineColor.setAlpha(front ? 230 : 112);
        painter.setPen(QPen(lineColor, front ? 3.0 : 2.0, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(centerPoint(), item.point);

        QColor nodeColor = item.color;
        nodeColor.setAlpha(front ? 242 : 132);
        const QRectF node(item.point.x() - item.diameter * 0.5,
                          item.point.y() - item.diameter * 0.5,
                          item.diameter,
                          item.diameter);
        painter.setPen(QPen(QColor(248, 250, 252, front ? 220 : 110), front ? 1.2 : 0.8));
        painter.setBrush(nodeColor);
        painter.drawEllipse(node);

        painter.setPen(QColor(255, 255, 255, front ? 245 : 165));
        painter.drawText(node, Qt::AlignCenter, item.label);
    }

    void drawRollButtons(QPainter& painter) const {
        painter.setFont(buttonFont());
        drawRollButton(painter, -1, QStringLiteral("CCW"));
        drawRollButton(painter, 1, QStringLiteral("CW"));
    }

    void drawRollButton(QPainter& painter, int direction, const QString& text) const {
        const QRect rect = rollButtonRect(direction);
        const bool hovered = hoveredRollDirection_ == direction;
        const bool pressed = pressedRollDirection_ == direction;
        QColor fill = pressed ? QColor(51, 65, 85) : (hovered ? QColor(47, 59, 75) : QColor(31, 41, 55));
        painter.setPen(QPen(hovered || pressed ? QColor(229, 231, 235) : QColor(100, 116, 139), 1.0));
        painter.setBrush(fill);
        painter.drawRoundedRect(rect, 7.0, 7.0);
        painter.setPen(QColor(255, 255, 255));
        painter.drawText(rect, Qt::AlignCenter, text);
    }

    vtkCamera* activeCamera() const {
        return renderer_ ? renderer_->GetActiveCamera() : nullptr;
    }

    Basis currentBasis() const {
        Basis basis;
        vtkCamera* camera = activeCamera();
        if (!camera) {
            return basis;
        }

        double position[3] = {0.0, 0.0, 0.0};
        double focal[3] = {0.0, 0.0, 0.0};
        double viewUp[3] = {0.0, 0.0, 1.0};
        camera->GetPosition(position);
        camera->GetFocalPoint(focal);
        camera->GetViewUp(viewUp);

        basis.forward = normalized({
            focal[0] - position[0],
            focal[1] - position[1],
            focal[2] - position[2]
        });
        if (norm(basis.forward) < 1e-9) {
            return basis;
        }

        Vec3 rawUp = normalized({viewUp[0], viewUp[1], viewUp[2]});
        if (norm(rawUp) < 1e-9) {
            rawUp = {0.0, 0.0, 1.0};
        }
        basis.right = normalized(cross(basis.forward, rawUp));
        if (norm(basis.right) < 1e-9) {
            const Vec3 fallback = std::fabs(basis.forward.z) < 0.95 ? Vec3{0.0, 0.0, 1.0} : Vec3{0.0, 1.0, 0.0};
            basis.right = normalized(cross(basis.forward, fallback));
        }
        basis.up = normalized(cross(basis.right, basis.forward));
        basis.valid = norm(basis.right) >= 1e-9 && norm(basis.up) >= 1e-9;
        return basis;
    }

    std::vector<AxisItem> axisItems(const Basis& basis) const {
        std::vector<AxisItem> axes = {
            {QStringLiteral("x"), {-1.0, 0.0, 0.0}, QColor(185, 28, 28), QPointF(), 0.0, 18.0},
            {QStringLiteral("X"), {1.0, 0.0, 0.0}, QColor(239, 68, 68), QPointF(), 0.0, 24.0},
            {QStringLiteral("y"), {0.0, -1.0, 0.0}, QColor(101, 163, 13), QPointF(), 0.0, 18.0},
            {QStringLiteral("Y"), {0.0, 1.0, 0.0}, QColor(132, 204, 22), QPointF(), 0.0, 24.0},
            {QStringLiteral("z"), {0.0, 0.0, -1.0}, QColor(29, 78, 216), QPointF(), 0.0, 18.0},
            {QStringLiteral("Z"), {0.0, 0.0, 1.0}, QColor(59, 130, 246), QPointF(), 0.0, 24.0}
        };

        const QPointF center = centerPoint();
        constexpr double axisScale = 37.0;
        for (AxisItem& item : axes) {
            const Vec3 axis = normalized(item.axis);
            item.point = QPointF(center.x() + dot(axis, basis.right) * axisScale,
                                 center.y() - dot(axis, basis.up) * axisScale);
            item.depth = dot(axis, basis.forward);
            if (item.depth < 0.0) {
                item.diameter *= 0.82;
            }
        }
        std::sort(axes.begin(), axes.end(), [](const AxisItem& a, const AxisItem& b) {
            return a.depth < b.depth;
        });
        return axes;
    }

    Vec3 axisAt(const QPoint& pos) const {
        const Basis basis = currentBasis();
        if (!basis.valid) {
            return {};
        }
        const std::vector<AxisItem> axes = axisItems(basis);
        for (auto it = axes.rbegin(); it != axes.rend(); ++it) {
            const QPointF d = QPointF(pos) - it->point;
            const qreal radius = it->diameter * 0.5 + 3.0;
            if (d.x() * d.x() + d.y() * d.y() <= radius * radius) {
                return it->axis;
            }
        }
        return {};
    }

    void orbitCamera(int dx, int dy) {
        vtkCamera* camera = activeCamera();
        if (!camera || !renderer_) {
            return;
        }
        constexpr double kOrbitSensitivity = 0.38;
        camera->Azimuth(-dx * kOrbitSensitivity);
        camera->Elevation(dy * kOrbitSensitivity);
        camera->OrthogonalizeViewUp();
        renderer_->ResetCameraClippingRange();
        renderAndUpdate();
    }

    void rollCameraPixels(int dx) {
        vtkCamera* camera = activeCamera();
        if (!camera || !renderer_) {
            return;
        }
        constexpr double kRollSensitivity = 0.45;
        camera->Roll(-dx * kRollSensitivity);
        camera->OrthogonalizeViewUp();
        renderer_->ResetCameraClippingRange();
        renderAndUpdate();
    }

    void rollCameraQuarter(int direction) {
        vtkCamera* camera = activeCamera();
        if (!camera || !renderer_) {
            return;
        }
        camera->Roll(direction < 0 ? -90.0 : 90.0);
        camera->OrthogonalizeViewUp();
        renderer_->ResetCameraClippingRange();
        renderAndUpdate();
    }

    void setCameraToAxis(const Vec3& axis) {
        vtkCamera* camera = activeCamera();
        if (!camera || !renderer_) {
            return;
        }
        const Vec3 forward = normalized(axis);
        if (norm(forward) < 1e-9) {
            return;
        }
        double focal[3] = {0.0, 0.0, 0.0};
        camera->GetFocalPoint(focal);
        const double distance = std::max(1e-3, camera->GetDistance());
        camera->SetPosition(focal[0] - forward.x * distance,
                            focal[1] - forward.y * distance,
                            focal[2] - forward.z * distance);

        Vec3 viewUp = std::fabs(forward.z) < 0.92 ? Vec3{0.0, 0.0, 1.0} : Vec3{0.0, 1.0, 0.0};
        viewUp = normalized({viewUp.x - forward.x * dot(viewUp, forward),
                             viewUp.y - forward.y * dot(viewUp, forward),
                             viewUp.z - forward.z * dot(viewUp, forward)});
        camera->SetViewUp(viewUp.x, viewUp.y, viewUp.z);
        renderer_->ResetCameraClippingRange();
        renderAndUpdate();
    }

    void zoomCamera(int wheelDelta) {
        vtkCamera* camera = activeCamera();
        if (!camera || !renderer_) {
            return;
        }
        const double factor = std::pow(1.0015, static_cast<double>(wheelDelta));
        camera->Dolly(factor);
        renderer_->ResetCameraClippingRange();
        renderAndUpdate();
    }

    void renderAndUpdate() {
        if (renderWindow_) {
            renderWindow_->Render();
        }
        update();
    }

    void syncCameraObserver() {
        vtkCamera* camera = activeCamera();
        if (camera == observedCamera_) {
            return;
        }
        removeCameraObserver();
        observedCamera_ = camera;
        if (!observedCamera_) {
            return;
        }
        if (!cameraModifiedCallback_) {
            cameraModifiedCallback_ = vtkSmartPointer<vtkCallbackCommand>::New();
            cameraModifiedCallback_->SetClientData(this);
            cameraModifiedCallback_->SetCallback([](vtkObject*, unsigned long, void* clientData, void*) {
                auto* self = static_cast<ViewDragBall*>(clientData);
                if (self) {
                    self->update();
                }
            });
        }
        cameraObserverTag_ = observedCamera_->AddObserver(vtkCommand::ModifiedEvent, cameraModifiedCallback_);
    }

    void removeCameraObserver() {
        if (observedCamera_ && cameraObserverTag_ != 0) {
            observedCamera_->RemoveObserver(cameraObserverTag_);
        }
        observedCamera_ = nullptr;
        cameraObserverTag_ = 0;
    }

    vtkRenderer* renderer_{nullptr};
    vtkRenderWindow* renderWindow_{nullptr};
    vtkCamera* observedCamera_{nullptr};
    unsigned long cameraObserverTag_{0};
    vtkSmartPointer<vtkCallbackCommand> cameraModifiedCallback_;
    bool dragging_{false};
    bool dragMoved_{false};
    Qt::MouseButton dragButton_{Qt::NoButton};
    QPoint dragStartPos_;
    QPoint dragLastPos_;
    Vec3 pressedAxis_;
    int hoveredRollDirection_{0};
    int pressedRollDirection_{0};
};

class VtkViewDragBall {
public:
    VtkViewDragBall(vtkRenderer* sceneRenderer, vtkRenderWindow* renderWindow)
        : sceneRenderer_(sceneRenderer), renderWindow_(renderWindow) {
        overlayRenderer_ = vtkSmartPointer<vtkRenderer>::New();
        overlayRenderer_->SetLayer(1);
        overlayRenderer_->InteractiveOff();
        overlayRenderer_->EraseOff();

        if (renderWindow_) {
            renderWindow_->SetNumberOfLayers(std::max(2, renderWindow_->GetNumberOfLayers()));
            renderWindow_->AddRenderer(overlayRenderer_);
        }

        panel_ = makeShapeActor();
        ccwButton_ = makeShapeActor();
        cwButton_ = makeShapeActor();
        centerDot_ = makeCircleActor();

        ccwText_ = makeTextActor(QStringLiteral("CCW"), 10);
        cwText_ = makeTextActor(QStringLiteral("CW"), 10);

        axes_ = {
            makeAxis(QStringLiteral("x"), {-1.0, 0.0, 0.0}, QColor(185, 28, 28), 18.0),
            makeAxis(QStringLiteral("X"), {1.0, 0.0, 0.0}, QColor(239, 68, 68), 24.0),
            makeAxis(QStringLiteral("y"), {0.0, -1.0, 0.0}, QColor(101, 163, 13), 18.0),
            makeAxis(QStringLiteral("Y"), {0.0, 1.0, 0.0}, QColor(132, 204, 22), 24.0),
            makeAxis(QStringLiteral("z"), {0.0, 0.0, -1.0}, QColor(29, 78, 216), 18.0),
            makeAxis(QStringLiteral("Z"), {0.0, 0.0, 1.0}, QColor(59, 130, 246), 24.0)
        };

        syncCameraObserver();
        updateActors(false);
    }

    ~VtkViewDragBall() {
        removeCameraObserver();
        if (renderWindow_ && overlayRenderer_) {
            renderWindow_->RemoveRenderer(overlayRenderer_);
        }
    }

    void setViewportSize(int width, int height) {
        viewportWidth_ = std::max(1, width);
        viewportHeight_ = std::max(1, height);
        panelX_ = std::max(0, viewportWidth_ - kPanelWidth - kMargin);
        panelY_ = kMargin;
        updateActors(true);
    }

    void refreshFromCamera() {
        if (!enabled_) {
            return;
        }
        syncCameraObserver();
        updateActors(false);
    }

    void setEnabled(bool enabled) {
        if (enabled_ == enabled) {
            return;
        }
        enabled_ = enabled;
        dragging_ = false;
        pressedAxis_ = Vec3{};
        pressedRollDirection_ = 0;
        hoveredRollDirection_ = 0;
        if (!enabled_) {
            removeCameraObserver();
            if (overlayRenderer_) {
                overlayRenderer_->RemoveAllViewProps();
            }
            if (renderWindow_) {
                renderWindow_->Render();
            }
            return;
        }
        syncCameraObserver();
        updateActors(true);
    }

    bool handleMousePress(QMouseEvent* event) {
        if (!enabled_) {
            return false;
        }
        if (!event || !(event->button() == Qt::LeftButton || event->button() == Qt::RightButton)) {
            return false;
        }
        const QPoint pos = event->pos();
        if (!panelRect().contains(pos)) {
            return false;
        }

        if (event->button() == Qt::LeftButton) {
            const int rollDirection = rollButtonAt(pos);
            if (rollDirection != 0) {
                pressedRollDirection_ = rollDirection;
                updateActors(true);
                return true;
            }
            pressedAxis_ = axisAt(pos);
        }

        if (!orbitRect().contains(pos)) {
            return true;
        }

        dragging_ = true;
        dragMoved_ = false;
        dragButton_ = event->button();
        dragStartPos_ = pos;
        dragLastPos_ = pos;
        return true;
    }

    bool handleMouseMove(QMouseEvent* event) {
        if (!enabled_) {
            return false;
        }
        if (!event) {
            return false;
        }

        const QPoint pos = event->pos();
        const int hoverDirection = rollButtonAt(pos);
        if (hoverDirection != hoveredRollDirection_) {
            hoveredRollDirection_ = hoverDirection;
            updateActors(true);
        }

        if (pressedRollDirection_ != 0) {
            return true;
        }

        if (!dragging_) {
            return panelRect().contains(pos);
        }

        const QPoint delta = pos - dragLastPos_;
        dragLastPos_ = pos;
        if ((pos - dragStartPos_).manhattanLength() > 2) {
            dragMoved_ = true;
            pressedAxis_ = Vec3{};
        }
        if (!delta.isNull()) {
            if (dragButton_ == Qt::RightButton) {
                rollCameraPixels(delta.x());
            } else {
                orbitCamera(delta.x(), delta.y());
            }
        }
        return true;
    }

    bool handleMouseRelease(QMouseEvent* event) {
        if (!enabled_) {
            return false;
        }
        if (!event) {
            return false;
        }

        const QPoint pos = event->pos();
        if (pressedRollDirection_ != 0) {
            const int direction = pressedRollDirection_;
            pressedRollDirection_ = 0;
            if (event->button() == Qt::LeftButton && rollButtonAt(pos) == direction) {
                rollCameraQuarter(direction);
            } else {
                updateActors(true);
            }
            return true;
        }

        if (!dragging_ || event->button() != dragButton_) {
            return panelRect().contains(pos);
        }

        const Vec3 axis = pressedAxis_;
        dragging_ = false;
        dragButton_ = Qt::NoButton;
        pressedAxis_ = Vec3{};
        if (!dragMoved_ && norm(axis) > 0.5 && axisEqual(axis, axisAt(pos))) {
            setCameraToAxis(axis);
        } else {
            updateActors(true);
        }
        return true;
    }

    bool handleWheel(QWheelEvent* event) {
        if (!enabled_) {
            return false;
        }
        if (!event || !panelRect().contains(event->pos())) {
            return false;
        }
        const int delta = event->angleDelta().y();
        if (delta != 0) {
            zoomCamera(delta);
        }
        return true;
    }

private:
    struct Basis {
        Vec3 right;
        Vec3 up;
        Vec3 forward;
        bool valid{false};
    };

    struct ShapeActor {
        vtkSmartPointer<vtkActor2D> actor;
        vtkSmartPointer<vtkPolyData> polyData;
    };

    struct CircleActor {
        vtkSmartPointer<vtkActor2D> actor;
        vtkSmartPointer<vtkRegularPolygonSource> source;
    };

    struct AxisActor {
        QString label;
        Vec3 axis;
        QColor color;
        qreal baseDiameter{18.0};
        QPointF point;
        double depth{0.0};
        ShapeActor line;
        CircleActor node;
        vtkSmartPointer<vtkTextActor> text;
    };

    static constexpr int kPanelWidth = 128;
    static constexpr int kPanelHeight = 140;
    static constexpr int kMargin = 14;

    static double dot(const Vec3& a, const Vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    static double norm(const Vec3& v) {
        return std::sqrt(dot(v, v));
    }

    static Vec3 normalized(const Vec3& v) {
        const double length = norm(v);
        if (length < 1e-9 || !std::isfinite(length)) {
            return {0.0, 0.0, 0.0};
        }
        return {v.x / length, v.y / length, v.z / length};
    }

    static bool axisEqual(const Vec3& a, const Vec3& b) {
        return std::fabs(a.x - b.x) < 1e-6
            && std::fabs(a.y - b.y) < 1e-6
            && std::fabs(a.z - b.z) < 1e-6;
    }

    ShapeActor makeShapeActor() const {
        ShapeActor out;
        out.polyData = vtkSmartPointer<vtkPolyData>::New();
        auto mapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
        auto coordinate = vtkSmartPointer<vtkCoordinate>::New();
        coordinate->SetCoordinateSystemToDisplay();
        mapper->SetTransformCoordinate(coordinate);
        mapper->SetInputData(out.polyData);
        out.actor = vtkSmartPointer<vtkActor2D>::New();
        out.actor->SetMapper(mapper);
        return out;
    }

    CircleActor makeCircleActor() const {
        CircleActor out;
        out.source = vtkSmartPointer<vtkRegularPolygonSource>::New();
        out.source->SetNumberOfSides(40);
        out.source->GeneratePolygonOn();
        out.source->GeneratePolylineOff();
        auto mapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
        auto coordinate = vtkSmartPointer<vtkCoordinate>::New();
        coordinate->SetCoordinateSystemToDisplay();
        mapper->SetTransformCoordinate(coordinate);
        mapper->SetInputConnection(out.source->GetOutputPort());
        out.actor = vtkSmartPointer<vtkActor2D>::New();
        out.actor->SetMapper(mapper);
        return out;
    }

    vtkSmartPointer<vtkTextActor> makeTextActor(const QString& text, int fontSize) const {
        auto actor = vtkSmartPointer<vtkTextActor>::New();
        actor->SetInput(text.toUtf8().constData());
        actor->GetPositionCoordinate()->SetCoordinateSystemToDisplay();
        auto* property = actor->GetTextProperty();
        property->SetFontSize(fontSize);
        property->SetBold(1);
        property->SetColor(1.0, 1.0, 1.0);
        property->SetOpacity(1.0);
        property->SetJustificationToCentered();
        property->SetVerticalJustificationToCentered();
        return actor;
    }

    AxisActor makeAxis(const QString& label, const Vec3& axis, const QColor& color, qreal diameter) const {
        AxisActor out;
        out.label = label;
        out.axis = axis;
        out.color = color;
        out.baseDiameter = diameter;
        out.line = makeShapeActor();
        out.node = makeCircleActor();
        out.text = makeTextActor(label, diameter >= 22.0 ? 11 : 9);
        return out;
    }

    QRect panelRect() const {
        return QRect(panelX_, panelY_, kPanelWidth, kPanelHeight);
    }

    QRect orbitRect() const {
        return QRect(panelX_, panelY_, kPanelWidth, 108);
    }

    QRect rollButtonRect(int direction) const {
        const int x = panelX_ + (direction < 0 ? 18 : 70);
        return QRect(x, panelY_ + 112, 40, 20);
    }

    int rollButtonAt(const QPoint& pos) const {
        if (rollButtonRect(-1).contains(pos)) {
            return -1;
        }
        if (rollButtonRect(1).contains(pos)) {
            return 1;
        }
        return 0;
    }

    QPointF centerPoint() const {
        return QPointF(panelX_ + kPanelWidth * 0.5, panelY_ + 56.0);
    }

    QPointF toDisplay(const QPointF& point) const {
        return QPointF(point.x(), viewportHeight_ - point.y());
    }

    void setActorColor(vtkActor2D* actor, const QColor& color) const {
        if (!actor) {
            return;
        }
        actor->GetProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
        actor->GetProperty()->SetOpacity(color.alphaF());
    }

    void setShapeRect(ShapeActor& shape, const QRect& rect, const QColor& color) {
        auto points = vtkSmartPointer<vtkPoints>::New();
        const QPointF tl = toDisplay(QPointF(rect.left(), rect.top()));
        const QPointF tr = toDisplay(QPointF(rect.left() + rect.width(), rect.top()));
        const QPointF br = toDisplay(QPointF(rect.left() + rect.width(), rect.top() + rect.height()));
        const QPointF bl = toDisplay(QPointF(rect.left(), rect.top() + rect.height()));
        points->InsertNextPoint(bl.x(), bl.y(), 0.0);
        points->InsertNextPoint(br.x(), br.y(), 0.0);
        points->InsertNextPoint(tr.x(), tr.y(), 0.0);
        points->InsertNextPoint(tl.x(), tl.y(), 0.0);
        vtkIdType ids[4] = {0, 1, 2, 3};
        auto polys = vtkSmartPointer<vtkCellArray>::New();
        polys->InsertNextCell(4, ids);
        shape.polyData->SetPoints(points);
        shape.polyData->SetPolys(polys);
        shape.polyData->Modified();
        setActorColor(shape.actor, color);
    }

    void setShapeLine(ShapeActor& shape, const QPointF& a, const QPointF& b, const QColor& color, double width) {
        auto points = vtkSmartPointer<vtkPoints>::New();
        const QPointF da = toDisplay(a);
        const QPointF db = toDisplay(b);
        points->InsertNextPoint(da.x(), da.y(), 0.0);
        points->InsertNextPoint(db.x(), db.y(), 0.0);
        vtkIdType ids[2] = {0, 1};
        auto lines = vtkSmartPointer<vtkCellArray>::New();
        lines->InsertNextCell(2, ids);
        shape.polyData->SetPoints(points);
        shape.polyData->SetLines(lines);
        shape.polyData->Modified();
        setActorColor(shape.actor, color);
        shape.actor->GetProperty()->SetLineWidth(width);
    }

    void setCircle(CircleActor& circle, const QPointF& center, qreal radius, const QColor& color) {
        const QPointF dc = toDisplay(center);
        circle.source->SetCenter(dc.x(), dc.y(), 0.0);
        circle.source->SetRadius(radius);
        circle.source->Modified();
        setActorColor(circle.actor, color);
    }

    void setTextPosition(vtkTextActor* actor, const QPointF& center, const QColor& color) const {
        if (!actor) {
            return;
        }
        const QPointF dc = toDisplay(center);
        actor->SetPosition(dc.x(), dc.y());
        actor->GetTextProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
        actor->GetTextProperty()->SetOpacity(color.alphaF());
    }

    vtkCamera* activeCamera() const {
        return sceneRenderer_ ? sceneRenderer_->GetActiveCamera() : nullptr;
    }

    Basis currentBasis() const {
        Basis basis;
        vtkCamera* camera = activeCamera();
        if (!camera) {
            return basis;
        }

        double position[3] = {0.0, 0.0, 0.0};
        double focal[3] = {0.0, 0.0, 0.0};
        double viewUp[3] = {0.0, 0.0, 1.0};
        camera->GetPosition(position);
        camera->GetFocalPoint(focal);
        camera->GetViewUp(viewUp);

        basis.forward = normalized({
            focal[0] - position[0],
            focal[1] - position[1],
            focal[2] - position[2]
        });
        if (norm(basis.forward) < 1e-9) {
            return basis;
        }

        Vec3 rawUp = normalized({viewUp[0], viewUp[1], viewUp[2]});
        if (norm(rawUp) < 1e-9) {
            rawUp = {0.0, 0.0, 1.0};
        }
        basis.right = normalized(cross(basis.forward, rawUp));
        if (norm(basis.right) < 1e-9) {
            const Vec3 fallback = std::fabs(basis.forward.z) < 0.95 ? Vec3{0.0, 0.0, 1.0} : Vec3{0.0, 1.0, 0.0};
            basis.right = normalized(cross(basis.forward, fallback));
        }
        basis.up = normalized(cross(basis.right, basis.forward));
        basis.valid = norm(basis.right) >= 1e-9 && norm(basis.up) >= 1e-9;
        return basis;
    }

    void updateAxisPositions(const Basis& basis) {
        const QPointF center = centerPoint();
        constexpr double axisScale = 37.0;
        for (AxisActor& axis : axes_) {
            const Vec3 v = normalized(axis.axis);
            axis.point = QPointF(center.x() + dot(v, basis.right) * axisScale,
                                 center.y() - dot(v, basis.up) * axisScale);
            axis.depth = dot(v, basis.forward);
        }
    }

    Vec3 axisAt(const QPoint& pos) {
        const Basis basis = currentBasis();
        if (!basis.valid) {
            return {};
        }
        updateAxisPositions(basis);
        std::vector<AxisActor*> ordered;
        for (AxisActor& axis : axes_) {
            ordered.push_back(&axis);
        }
        std::sort(ordered.begin(), ordered.end(), [](const AxisActor* a, const AxisActor* b) {
            return a->depth < b->depth;
        });
        for (auto it = ordered.rbegin(); it != ordered.rend(); ++it) {
            const AxisActor* axis = *it;
            const qreal diameter = axis->depth >= 0.0 ? axis->baseDiameter : axis->baseDiameter * 0.82;
            const QPointF d = QPointF(pos) - axis->point;
            const qreal radius = diameter * 0.5 + 3.0;
            if (d.x() * d.x() + d.y() * d.y() <= radius * radius) {
                return axis->axis;
            }
        }
        return {};
    }

    void updateActors(bool render) {
        if (!enabled_) {
            return;
        }
        if (!overlayRenderer_) {
            return;
        }

        setShapeRect(panel_, panelRect(), QColor(15, 23, 42, 224));
        updateRollButton(-1, ccwButton_, ccwText_, QStringLiteral("CCW"));
        updateRollButton(1, cwButton_, cwText_, QStringLiteral("CW"));

        const Basis basis = currentBasis();
        if (basis.valid) {
            updateAxisPositions(basis);
        }

        overlayRenderer_->RemoveAllViewProps();
        overlayRenderer_->AddActor2D(panel_.actor);

        if (basis.valid) {
            std::vector<AxisActor*> ordered;
            for (AxisActor& axis : axes_) {
                ordered.push_back(&axis);
            }
            std::sort(ordered.begin(), ordered.end(), [](const AxisActor* a, const AxisActor* b) {
                return a->depth < b->depth;
            });
            for (AxisActor* axis : ordered) {
                updateAxisActor(*axis);
                overlayRenderer_->AddActor2D(axis->line.actor);
                overlayRenderer_->AddActor2D(axis->node.actor);
                overlayRenderer_->AddActor2D(axis->text);
            }
        }

        setCircle(centerDot_, centerPoint(), 3.2, QColor(241, 245, 249, 210));
        overlayRenderer_->AddActor2D(centerDot_.actor);
        overlayRenderer_->AddActor2D(ccwButton_.actor);
        overlayRenderer_->AddActor2D(cwButton_.actor);
        overlayRenderer_->AddActor2D(ccwText_);
        overlayRenderer_->AddActor2D(cwText_);

        if (render && renderWindow_) {
            renderWindow_->Render();
        }
    }

    void updateAxisActor(AxisActor& axis) {
        const bool front = axis.depth >= 0.0;
        QColor lineColor = axis.color;
        lineColor.setAlpha(front ? 230 : 112);
        setShapeLine(axis.line, centerPoint(), axis.point, lineColor, front ? 3.0 : 2.0);

        QColor nodeColor = axis.color;
        nodeColor.setAlpha(front ? 242 : 132);
        const qreal diameter = front ? axis.baseDiameter : axis.baseDiameter * 0.82;
        setCircle(axis.node, axis.point, diameter * 0.5, nodeColor);
        setTextPosition(axis.text, axis.point, QColor(255, 255, 255, front ? 245 : 165));
    }

    void updateRollButton(int direction, ShapeActor& button, vtkTextActor* text, const QString&) {
        const bool hovered = hoveredRollDirection_ == direction;
        const bool pressed = pressedRollDirection_ == direction;
        const QColor fill = pressed ? QColor(51, 65, 85, 255)
                                    : (hovered ? QColor(47, 59, 75, 255) : QColor(31, 41, 55, 255));
        setShapeRect(button, rollButtonRect(direction), fill);
        setTextPosition(text, QPointF(rollButtonRect(direction).center()), QColor(255, 255, 255, 255));
    }

    void orbitCamera(int dx, int dy) {
        vtkCamera* camera = activeCamera();
        if (!camera || !sceneRenderer_) {
            return;
        }
        constexpr double kOrbitSensitivity = 0.38;
        camera->Azimuth(-dx * kOrbitSensitivity);
        camera->Elevation(dy * kOrbitSensitivity);
        camera->OrthogonalizeViewUp();
        sceneRenderer_->ResetCameraClippingRange();
        renderAndUpdate();
    }

    void rollCameraPixels(int dx) {
        vtkCamera* camera = activeCamera();
        if (!camera || !sceneRenderer_) {
            return;
        }
        constexpr double kRollSensitivity = 0.45;
        camera->Roll(-dx * kRollSensitivity);
        camera->OrthogonalizeViewUp();
        sceneRenderer_->ResetCameraClippingRange();
        renderAndUpdate();
    }

    void rollCameraQuarter(int direction) {
        vtkCamera* camera = activeCamera();
        if (!camera || !sceneRenderer_) {
            return;
        }
        camera->Roll(direction < 0 ? -90.0 : 90.0);
        camera->OrthogonalizeViewUp();
        sceneRenderer_->ResetCameraClippingRange();
        renderAndUpdate();
    }

    void setCameraToAxis(const Vec3& axis) {
        vtkCamera* camera = activeCamera();
        if (!camera || !sceneRenderer_) {
            return;
        }
        const Vec3 forward = normalized(axis);
        if (norm(forward) < 1e-9) {
            return;
        }
        double focal[3] = {0.0, 0.0, 0.0};
        camera->GetFocalPoint(focal);
        const double distance = std::max(1e-3, camera->GetDistance());
        camera->SetPosition(focal[0] - forward.x * distance,
                            focal[1] - forward.y * distance,
                            focal[2] - forward.z * distance);

        Vec3 viewUp = std::fabs(forward.z) < 0.92 ? Vec3{0.0, 0.0, 1.0} : Vec3{0.0, 1.0, 0.0};
        viewUp = normalized({viewUp.x - forward.x * dot(viewUp, forward),
                             viewUp.y - forward.y * dot(viewUp, forward),
                             viewUp.z - forward.z * dot(viewUp, forward)});
        camera->SetViewUp(viewUp.x, viewUp.y, viewUp.z);
        sceneRenderer_->ResetCameraClippingRange();
        renderAndUpdate();
    }

    void zoomCamera(int wheelDelta) {
        vtkCamera* camera = activeCamera();
        if (!camera || !sceneRenderer_) {
            return;
        }
        const double factor = std::pow(1.0015, static_cast<double>(wheelDelta));
        camera->Dolly(factor);
        sceneRenderer_->ResetCameraClippingRange();
        renderAndUpdate();
    }

    void renderAndUpdate() {
        updateActors(false);
        if (renderWindow_) {
            renderWindow_->Render();
        }
    }

    void syncCameraObserver() {
        vtkCamera* camera = activeCamera();
        if (camera == observedCamera_) {
            return;
        }
        removeCameraObserver();
        observedCamera_ = camera;
        if (!observedCamera_) {
            return;
        }
        if (!cameraModifiedCallback_) {
            cameraModifiedCallback_ = vtkSmartPointer<vtkCallbackCommand>::New();
            cameraModifiedCallback_->SetClientData(this);
            cameraModifiedCallback_->SetCallback([](vtkObject*, unsigned long, void* clientData, void*) {
                auto* self = static_cast<VtkViewDragBall*>(clientData);
                if (self) {
                    self->updateActors(false);
                }
            });
        }
        cameraObserverTag_ = observedCamera_->AddObserver(vtkCommand::ModifiedEvent, cameraModifiedCallback_);
    }

    void removeCameraObserver() {
        if (observedCamera_ && cameraObserverTag_ != 0) {
            observedCamera_->RemoveObserver(cameraObserverTag_);
        }
        observedCamera_ = nullptr;
        cameraObserverTag_ = 0;
    }

    vtkRenderer* sceneRenderer_{nullptr};
    vtkRenderWindow* renderWindow_{nullptr};
    vtkSmartPointer<vtkRenderer> overlayRenderer_;
    vtkCamera* observedCamera_{nullptr};
    unsigned long cameraObserverTag_{0};
    vtkSmartPointer<vtkCallbackCommand> cameraModifiedCallback_;

    int viewportWidth_{1};
    int viewportHeight_{1};
    int panelX_{0};
    int panelY_{0};

    ShapeActor panel_;
    ShapeActor ccwButton_;
    ShapeActor cwButton_;
    CircleActor centerDot_;
    vtkSmartPointer<vtkTextActor> ccwText_;
    vtkSmartPointer<vtkTextActor> cwText_;
    std::vector<AxisActor> axes_;

    bool dragging_{false};
    bool dragMoved_{false};
    bool enabled_{true};
    Qt::MouseButton dragButton_{Qt::NoButton};
    QPoint dragStartPos_;
    QPoint dragLastPos_;
    Vec3 pressedAxis_;
    int hoveredRollDirection_{0};
    int pressedRollDirection_{0};
};

Scene3DWidget::Scene3DWidget(QWidget* parent)
    : QWidget(parent) {
    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    vtkWidget_ = new QVTKWidget(this);
    layout->addWidget(vtkWidget_);

    renderWindow_ = vtkSmartPointer<vtkRenderWindow>::New();
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderWindow_->AddRenderer(renderer_);
    vtkWidget_->SetRenderWindow(renderWindow_);
    vtkWidget_->installEventFilter(this);
    this->installEventFilter(this);

    initializeScene();

    viewDragBall_ = new VtkViewDragBall(renderer_, renderWindow_);
    updateViewDragBallGeometry();
}

Scene3DWidget::~Scene3DWidget() {
    delete viewDragBall_;
    viewDragBall_ = nullptr;
}

void Scene3DWidget::initializeScene() {
    renderer_->SetBackground(0.07, 0.08, 0.10);

    auto interactorStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    vtkWidget_->GetRenderWindow()->GetInteractor()->SetInteractorStyle(interactorStyle);

    axesActor_ = vtkSmartPointer<vtkAxesActor>::New();
    configureAxesActor(axesActor_);
    renderer_->AddActor(axesActor_);

    rebuildGround();
    rebuildDroneActor();
    rebuildHistoryActor();
    renderer_->ResetCamera();
    auto* camera = renderer_->GetActiveCamera();
    camera->SetPosition(130.0, -160.0, 110.0);
    camera->SetFocalPoint(0.0, 0.0, 15.0);
    camera->SetViewUp(0.0, 0.0, 1.0);
    requestRender();
}

void Scene3DWidget::setWorldConfig(const WorldConfig& world) {
    world_ = world;
    rebuildGround();
    rebuildBlocks();
    requestRender();
}

void Scene3DWidget::setBaseStations(const std::vector<BaseStation>& stations) {
    stations_ = stations;
    if (selectedStationIndex_ >= static_cast<int>(stations_.size())) {
        selectedStationIndex_ = stations_.empty() ? -1 : static_cast<int>(stations_.size()) - 1;
    }
    rebuildBaseStations();
    emitCurrentSelection();
    refreshPinnedCards();
    requestRender();
}

void Scene3DWidget::setDroneState(const DroneState& state) {
    drone_ = state;
    rebuildDroneActor();
    rebuildHistoryActor();
    requestRender();
}

void Scene3DWidget::setBeamPaths(const BeamPathList& paths) {
    paths_ = paths;
    rebuildPathActors();
    refreshPinnedCards();
    requestRender();
}

bool Scene3DWidget::setSceneGeometry(const SceneGeometryBundle& bundle, QString* errorMessage) {
    if (!bundle.valid) {
        if (errorMessage) {
            *errorMessage = bundle.errorMessage;
        }
        return false;
    }

    clearSceneGeometry();
    sceneBundle_ = bundle;
    for (const auto& mesh : bundle.meshes) {
        auto actor = makeTriangleMeshActor(mesh);
        sceneMeshActors_.push_back(actor);
        renderer_->AddActor(actor);
    }
    updateSceneVisibilityState();
    focusSceneGeometry();
    requestRender();
    return true;
}

bool Scene3DWidget::hasImportedSceneGeometry() const {
    return !sceneMeshActors_.empty();
}

void Scene3DWidget::updateSceneVisibilityState() {
    if (axesActor_) {
        axesActor_->SetVisibility(axesLayerVisible_ ? 1 : 0);
    }
    const bool showPlaceholder = !hasImportedSceneGeometry();
    if (groundActor_) {
        groundActor_->SetVisibility((showPlaceholder && groundLayerVisible_) ? 1 : 0);
    }
    for (auto& actor : blockActors_) {
        if (actor) {
            actor->SetVisibility((showPlaceholder && buildingsLayerVisible_) ? 1 : 0);
        }
    }
    for (auto& actor : sceneMeshActors_) {
        if (actor) {
            actor->SetVisibility(sceneMeshLayerVisible_ ? 1 : 0);
        }
    }
}

void Scene3DWidget::focusSceneGeometry() {
    if (sceneMeshActors_.empty()) {
        return;
    }
    vtkBoundingBox box;
    for (const auto& actor : sceneMeshActors_) {
        double bounds[6];
        actor->GetBounds(bounds);
        if (vtkBoundingBox::IsValid(bounds)) {
            box.AddBounds(bounds);
        }
    }
    if (!box.IsValid()) {
        return;
    }
    double bounds[6];
    box.GetBounds(bounds);
    renderer_->ResetCamera(bounds);
    requestRender();
}

bool Scene3DWidget::sceneBoundsXY(double* minX, double* maxX, double* minY, double* maxY) const {
    if (!minX || !maxX || !minY || !maxY) {
        return false;
    }
    vtkBoundingBox box;
    for (const auto& actor : sceneMeshActors_) {
        if (!actor) {
            continue;
        }
        double bounds[6];
        actor->GetBounds(bounds);
        if (vtkBoundingBox::IsValid(bounds)) {
            box.AddBounds(bounds);
        }
    }
    if (!box.IsValid()) {
        return false;
    }
    double bounds[6];
    box.GetBounds(bounds);
    *minX = bounds[0];
    *maxX = bounds[1];
    *minY = bounds[2];
    *maxY = bounds[3];
    return std::isfinite(*minX) && std::isfinite(*maxX) && std::isfinite(*minY) && std::isfinite(*maxY) && (*maxX > *minX) && (*maxY > *minY);
}

QWidget* Scene3DWidget::overlayHostWidget() const {
    return vtkWidget_;
}

void Scene3DWidget::updateViewDragBallGeometry() {
    if (!viewDragBall_ || !vtkWidget_) {
        return;
    }
    viewDragBall_->setViewportSize(vtkWidget_->width(), vtkWidget_->height());
}

bool Scene3DWidget::loadMapMesh(const QString& filePath, QString* errorMessage) {
    FileSceneProvider provider;
    SceneLoadRequest request;
    request.type = SceneSourceType::FileMesh;
    request.filePath = filePath;
    const SceneGeometryBundle bundle = provider.load(request);
    return setSceneGeometry(bundle, errorMessage);
}

void Scene3DWidget::clearSceneGeometry() {
    for (auto& actor : sceneMeshActors_) {
        renderer_->RemoveActor(actor);
    }
    sceneMeshActors_.clear();
    sceneBundle_ = SceneGeometryBundle{};
    updateSceneVisibilityState();
    requestRender();
}

void Scene3DWidget::clearMapMesh() {
    clearSceneGeometry();
}

void Scene3DWidget::resetCamera() {
    renderer_->ResetCamera();
    auto* camera = renderer_->GetActiveCamera();
    camera->Azimuth(20.0);
    camera->Elevation(20.0);
    camera->OrthogonalizeViewUp();
    requestRender();
}

void Scene3DWidget::setGroundVisible(bool visible) {
    groundLayerVisible_ = visible;
    updateSceneVisibilityState();
    requestRender();
}

void Scene3DWidget::setBuildingsVisible(bool visible) {
    buildingsLayerVisible_ = visible;
    updateSceneVisibilityState();
    requestRender();
}

void Scene3DWidget::setBaseStationsVisible(bool visible) {
    for (auto& actor : stationActors_) {
        actor->SetVisibility(visible ? 1 : 0);
    }
    requestRender();
}

void Scene3DWidget::setDroneVisible(bool visible) {
    if (droneAssembly_) droneAssembly_->SetVisibility(visible ? 1 : 0);
    requestRender();
}

void Scene3DWidget::setHistoryVisible(bool visible) {
    if (historyActor_) historyActor_->SetVisibility(visible ? 1 : 0);
    requestRender();
}

void Scene3DWidget::setPathsVisible(bool visible) {
    for (auto& actor : pathActors_) {
        actor->SetVisibility(visible ? 1 : 0);
    }
    requestRender();
}

void Scene3DWidget::setSceneMeshVisible(bool visible) {
    sceneMeshLayerVisible_ = visible;
    updateSceneVisibilityState();
    requestRender();
}

void Scene3DWidget::setAxesVisible(bool visible) {
    axesLayerVisible_ = visible;
    updateSceneVisibilityState();
    requestRender();
}

void Scene3DWidget::setViewGizmoVisible(bool visible) {
    if (viewDragBall_) {
        viewDragBall_->setEnabled(visible);
    }
}

void Scene3DWidget::setStationInteractionMode(Scene3DWidget::StationInteractionMode mode) {
    stationInteractionMode_ = mode;
    QString text = QStringLiteral("Base-station edit mode: ");
    switch (mode) {
    case StationInteractionMode::None:
        text += QStringLiteral("Select");
        break;
    case StationInteractionMode::Add:
        text += QStringLiteral("Add (next click on empty scene)");
        break;
    case StationInteractionMode::Delete:
        text += QStringLiteral("Delete (click a station)");
        break;
    }
    emit statusMessage(text);
}

void Scene3DWidget::clearBaseStationSelection() {
    draggingStation_ = false;
    dragMovedStation_ = false;
    dragStationIndex_ = -1;
    selectedStationIndex_ = -1;
    rebuildBaseStations();
    emit statusMessage(QStringLiteral("Base-station selection cleared."));
    emit stationSelectionChanged(-1, BaseStation{});
    requestRender();
}

bool Scene3DWidget::setSelectedBaseStationIndex(int index) {
    if (index < 0 || index >= static_cast<int>(stations_.size())) {
        clearBaseStationSelection();
        return false;
    }
    draggingStation_ = false;
    dragMovedStation_ = false;
    dragStationIndex_ = -1;
    selectedStationIndex_ = index;
    rebuildBaseStations();
    emitCurrentSelection();
    requestRender();
    return true;
}

bool Scene3DWidget::deleteSelectedBaseStation() {
    if (selectedStationIndex_ < 0 || selectedStationIndex_ >= static_cast<int>(stations_.size())) {
        emit statusMessage(QStringLiteral("No base station selected."));
        return false;
    }
    const QString name = stations_[static_cast<size_t>(selectedStationIndex_)].name;
    stations_.erase(stations_.begin() + selectedStationIndex_);
    if (selectedStationIndex_ >= static_cast<int>(stations_.size())) {
        selectedStationIndex_ = static_cast<int>(stations_.size()) - 1;
    }
    rebuildBaseStations();
    emit baseStationsEdited(stations_);
    emitCurrentSelection();
    emit statusMessage(QStringLiteral("Deleted base station: %1").arg(name));
    requestRender();
    return true;
}

void Scene3DWidget::addBaseStation() {
    setStationInteractionMode(StationInteractionMode::Add);
}

bool Scene3DWidget::updateSelectedBaseStationPosition(const Vec3& position) {
    if (selectedStationIndex_ < 0 || selectedStationIndex_ >= static_cast<int>(stations_.size())) {
        emit statusMessage(QStringLiteral("No base station selected."));
        return false;
    }
    stations_[static_cast<size_t>(selectedStationIndex_)].position = position;
    rebuildBaseStations();
    emit baseStationsEdited(stations_);
    emitCurrentSelection();
    requestRender();
    return true;
}

bool Scene3DWidget::eventFilter(QObject* watched, QEvent* event) {
    if (!event) {
        return QWidget::eventFilter(watched, event);
    }

    if ((watched == vtkWidget_ || watched == this)
        && (event->type() == QEvent::Resize || event->type() == QEvent::Move || event->type() == QEvent::Show)) {
        updateViewDragBallGeometry();
    }

    if (watched == vtkWidget_ && viewDragBall_) {
        if (event->type() == QEvent::MouseButtonPress) {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            if (viewDragBall_->handleMousePress(mouseEvent)) {
                return true;
            }
        } else if (event->type() == QEvent::MouseMove) {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            if (viewDragBall_->handleMouseMove(mouseEvent)) {
                return true;
            }
        } else if (event->type() == QEvent::MouseButtonRelease) {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            if (viewDragBall_->handleMouseRelease(mouseEvent)) {
                return true;
            }
        } else if (event->type() == QEvent::Wheel) {
            auto* wheelEvent = static_cast<QWheelEvent*>(event);
            if (viewDragBall_->handleWheel(wheelEvent)) {
                return true;
            }
        }
    }

    const bool hoverCardWatched = hoverCard_ && (watched == hoverCard_ || hoverCard_->isAncestorOf(qobject_cast<QWidget*>(watched)));

    if ((watched == vtkWidget_ || watched == this)) {
        if (event->type() == QEvent::MouseButtonPress) {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton && watched == vtkWidget_) {
                if (handleMousePress(mouseEvent->x(), mouseEvent->y())) {
                    return true;
                }
            }
        } else if (event->type() == QEvent::MouseMove && watched == vtkWidget_) {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            hoverCardAnchor_ = mouseEvent->pos();
            if (draggingStation_ && dragStationIndex_ >= 0 && dragStationIndex_ < static_cast<int>(stations_.size())) {
                Vec3 pickedPoint;
                if (pickWorldPoint(mouseEvent->x(), mouseEvent->y(), &pickedPoint, nullptr)) {
                    Vec3 newPos = stations_[static_cast<size_t>(dragStationIndex_)].position;
                    newPos.x = pickedPoint.x;
                    newPos.y = pickedPoint.y;
                    if (std::fabs(newPos.x - stations_[static_cast<size_t>(dragStationIndex_)].position.x) > 1e-6 ||
                        std::fabs(newPos.y - stations_[static_cast<size_t>(dragStationIndex_)].position.y) > 1e-6) {
                        selectedStationIndex_ = dragStationIndex_;
                        updateSelectedBaseStationPosition(newPos);
                        dragMovedStation_ = true;
                    }
                }
                return true;
            }
            if (mouseEvent->buttons() != Qt::NoButton) {
                if (!shouldKeepHoverCardVisible(QCursor::pos())) {
                    hideHoverCard();
                }
                return QWidget::eventFilter(watched, event);
            }
            if (!shouldRunHoverPick(mouseEvent->pos())) {
                return QWidget::eventFilter(watched, event);
            }
            Vec3 pickedPoint;
            vtkActor* pickedActor = nullptr;
            if (pickWorldPoint(mouseEvent->x(), mouseEvent->y(), &pickedPoint, &pickedActor)) {
                const int pickedStation = findStationActorIndex(pickedActor);
                if (pickedStation >= 0) {
                    showHoverCardForStation(pickedStation, mouseEvent->pos());
                } else if (!shouldKeepHoverCardVisible(QCursor::pos())) {
                    hideHoverCard();
                }
            } else if (!shouldKeepHoverCardVisible(QCursor::pos())) {
                hideHoverCard();
            }
        } else if (event->type() == QEvent::MouseButtonRelease && watched == vtkWidget_) {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton && draggingStation_) {
                draggingStation_ = false;
                dragStationIndex_ = -1;
                emit baseStationEditFinished();
                if (dragMovedStation_) {
                    dragMovedStation_ = false;
                    emit statusMessage(QStringLiteral("Moved base station by dragging."));
                    return true;
                }
            }
        } else if ((event->type() == QEvent::Leave) && watched == vtkWidget_) {
            if (!shouldKeepHoverCardVisible(QCursor::pos())) {
                hideHoverCard();
            }
        }
    } else if (hoverCardWatched) {
        if (event->type() == QEvent::Leave) {
            if (!shouldKeepHoverCardVisible(QCursor::pos())) {
                hideHoverCard();
            }
        } else if (event->type() == QEvent::MouseMove || event->type() == QEvent::Enter) {
            if (hoverCard_ && !hoverCard_->isVisible() && shouldKeepHoverCardVisible(QCursor::pos())) {
                hoverCard_->show();
                hoverCard_->raise();
            }
        }
    }

    return QWidget::eventFilter(watched, event);
}

void Scene3DWidget::rebuildGround() {
    if (groundActor_) {
        renderer_->RemoveActor(groundActor_);
        groundActor_ = nullptr;
    }

    auto plane = vtkSmartPointer<vtkPlaneSource>::New();
    plane->SetOrigin(-world_.groundSizeX * 0.5, -world_.groundSizeY * 0.5, 0.0);
    plane->SetPoint1(world_.groundSizeX * 0.5, -world_.groundSizeY * 0.5, 0.0);
    plane->SetPoint2(-world_.groundSizeX * 0.5, world_.groundSizeY * 0.5, 0.0);
    plane->SetXResolution(20);
    plane->SetYResolution(20);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(plane->GetOutputPort());

    groundActor_ = vtkSmartPointer<vtkActor>::New();
    groundActor_->SetMapper(mapper);
    groundActor_->GetProperty()->SetColor(0.13, 0.14, 0.17);
    groundActor_->GetProperty()->SetRepresentationToSurface();
    renderer_->AddActor(groundActor_);
    updateSceneVisibilityState();
}

void Scene3DWidget::rebuildBlocks() {
    for (auto& actor : blockActors_) {
        renderer_->RemoveActor(actor);
    }
    blockActors_.clear();

    for (const SceneBlock& block : world_.blocks) {
        auto actor = makeCubeActor(block.center, block.size, block.color);
        blockActors_.push_back(actor);
        renderer_->AddActor(actor);
    }
    updateSceneVisibilityState();
}

void Scene3DWidget::rebuildBaseStations() {
    for (auto& actor : stationActors_) {
        renderer_->RemoveActor(actor);
    }
    stationActors_.clear();

    for (size_t i = 0; i < stations_.size(); ++i) {
        const BaseStation& station = stations_[i];
        const bool selected = static_cast<int>(i) == selectedStationIndex_;
        const double height = 7.5;
        auto actor = makeTetrahedronActor(station.position, height, selected ? brighten(station.color, 0.25) : station.color);
        if (selected) {
            actor->GetProperty()->SetEdgeVisibility(1);
            actor->GetProperty()->SetEdgeColor(1.0, 1.0, 0.2);
            actor->GetProperty()->SetLineWidth(2.0);
            actor->GetProperty()->SetSpecular(0.4);
        }
        stationActors_.push_back(actor);
        renderer_->AddActor(actor);
    }
}

void Scene3DWidget::rebuildDroneActor() {
    if (!droneAssembly_) {
        auto sphere = vtkSmartPointer<vtkSphereSource>::New();
        sphere->SetRadius(1.8);
        sphere->SetThetaResolution(24);
        sphere->SetPhiResolution(24);

        auto sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        sphereMapper->SetInputConnection(sphere->GetOutputPort());
        auto bodyActor = vtkSmartPointer<vtkActor>::New();
        bodyActor->SetMapper(sphereMapper);
        bodyActor->GetProperty()->SetColor(0.95, 0.95, 0.10);

        auto cone = vtkSmartPointer<vtkConeSource>::New();
        cone->SetRadius(1.0);
        cone->SetHeight(4.0);
        cone->SetResolution(20);
        cone->SetDirection(1.0, 0.0, 0.0);
        cone->SetCenter(2.4, 0.0, 0.0);

        auto coneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        coneMapper->SetInputConnection(cone->GetOutputPort());
        auto headingActor = vtkSmartPointer<vtkActor>::New();
        headingActor->SetMapper(coneMapper);
        headingActor->GetProperty()->SetColor(0.95, 0.25, 0.20);

        droneAssembly_ = vtkSmartPointer<vtkAssembly>::New();
        droneAssembly_->AddPart(bodyActor);
        droneAssembly_->AddPart(headingActor);
        renderer_->AddActor(droneAssembly_);
    }

    if (drone_.valid) {
        droneAssembly_->SetVisibility(1);
        droneAssembly_->SetPosition(drone_.position.x, drone_.position.y, drone_.position.z);
        droneAssembly_->SetOrientation(drone_.rollDeg, drone_.pitchDeg, drone_.yawDeg);
    } else {
        droneAssembly_->SetVisibility(0);
    }
}

void Scene3DWidget::rebuildHistoryActor() {
    if (historyActor_) {
        renderer_->RemoveActor(historyActor_);
        historyActor_ = nullptr;
    }

    if (drone_.history.size() < 2) {
        return;
    }

    historyActor_ = makePolylineActor(drone_.history, {1.0, 0.95, 0.35}, 2.5, 0.85);
    renderer_->AddActor(historyActor_);
}

void Scene3DWidget::rebuildPathActors() {
    for (auto& actor : pathActors_) {
        renderer_->RemoveActor(actor);
    }
    pathActors_.clear();

    for (const BeamPath& path : paths_) {
        if (path.points.size() < 2) {
            continue;
        }
        const Vec3 color = colorFromPowerDb(path.powerDb);
        const double opacity = path.kind == "los" ? 0.95 : 0.55;
        const double width = path.kind == "los" ? 4.0 : 2.5;
        auto actor = makePolylineActor(path.points, color, width, opacity);
        pathActors_.push_back(actor);
        renderer_->AddActor(actor);
    }
}

bool Scene3DWidget::shouldRunHoverPick(const QPoint& localPos) {
    if (hoverPickTimer_.isValid() && hoverPickTimer_.elapsed() < kHoverPickIntervalMs) {
        return false;
    }
    lastHoverPickPos_ = localPos;
    hoverPickTimer_.restart();
    return true;
}

bool Scene3DWidget::pickWorldPoint(int x, int y, Vec3* worldPoint, vtkActor** pickedActor) const {
    if (!renderer_ || !vtkWidget_ || !renderWindow_) {
        return false;
    }
    if (!picker_) {
        picker_ = vtkSmartPointer<vtkCellPicker>::New();
        picker_->SetTolerance(0.0005);
    }
    const int flippedY = vtkWidget_->height() - y - 1;
    if (!picker_->Pick(x, flippedY, 0.0, renderer_)) {
        return false;
    }
    double picked[3] = {0.0, 0.0, 0.0};
    picker_->GetPickPosition(picked);
    if (worldPoint) {
        worldPoint->x = picked[0];
        worldPoint->y = picked[1];
        worldPoint->z = picked[2];
    }
    if (pickedActor) {
        *pickedActor = picker_->GetActor();
    }
    return true;
}

int Scene3DWidget::findStationActorIndex(vtkActor* actor) const {
    if (!actor) {
        return -1;
    }
    for (size_t i = 0; i < stationActors_.size(); ++i) {
        if (stationActors_[i] == actor) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void Scene3DWidget::emitCurrentSelection() {
    if (selectedStationIndex_ >= 0 && selectedStationIndex_ < static_cast<int>(stations_.size())) {
        emit stationSelectionChanged(selectedStationIndex_, stations_[static_cast<size_t>(selectedStationIndex_)]);
    } else {
        emit stationSelectionChanged(-1, BaseStation{});
    }
}

QString Scene3DWidget::nextStationId() const {
    int maxIndex = 0;
    for (const BaseStation& station : stations_) {
        const QString digits = station.id;
        Q_UNUSED(digits);
        QString tail;
        for (int i = station.id.size() - 1; i >= 0; --i) {
            if (!station.id.at(i).isDigit()) break;
            tail.prepend(station.id.at(i));
        }
        if (!tail.isEmpty()) {
            maxIndex = std::max(maxIndex, tail.toInt());
        }
    }
    return QStringLiteral("bs_%1").arg(maxIndex + 1);
}

double Scene3DWidget::defaultStationHeight() const {
    if (!stations_.empty()) {
        return stations_.back().position.z;
    }
    return 27.0;
}

bool Scene3DWidget::handleMousePress(int x, int y) {
    Vec3 pickedPoint;
    vtkActor* pickedActor = nullptr;
    if (!pickWorldPoint(x, y, &pickedPoint, &pickedActor)) {
        return false;
    }

    const int pickedStation = findStationActorIndex(pickedActor);

    if (stationInteractionMode_ == StationInteractionMode::Delete) {
        if (pickedStation >= 0) {
            selectedStationIndex_ = pickedStation;
            return deleteSelectedBaseStation();
        }
        emit statusMessage(QStringLiteral("Delete mode: click a base station to remove it."));
        return true;
    }

    if (stationInteractionMode_ == StationInteractionMode::Add) {
        if (pickedStation >= 0) {
            selectedStationIndex_ = pickedStation;
            rebuildBaseStations();
            emitCurrentSelection();
            emit statusMessage(QStringLiteral("Selected base station %1").arg(stations_[static_cast<size_t>(selectedStationIndex_)].name));
            requestRender();
            stationInteractionMode_ = StationInteractionMode::None;
            return true;
        }

        BaseStation station;
        station.id = nextStationId();
        station.name = station.id;
        station.position = {pickedPoint.x, pickedPoint.y, defaultStationHeight()};
        station.previewCameraName = defaultPreviewCameraNameForStation(station);
        station.previewRosTopic = defaultPreviewRosTopicForStation(station);
        stations_.push_back(station);
        selectedStationIndex_ = static_cast<int>(stations_.size()) - 1;
        rebuildBaseStations();
        emit baseStationsEdited(stations_);
        emitCurrentSelection();
        emit statusMessage(QStringLiteral("Added base station %1 at %2").arg(station.name, vecToString(station.position)));
        requestRender();
        stationInteractionMode_ = StationInteractionMode::None;
        return true;
    }

    if (pickedStation >= 0) {
        selectedStationIndex_ = pickedStation;
        dragStationIndex_ = pickedStation;
        draggingStation_ = true;
        dragMovedStation_ = false;
        emit baseStationEditStarted();
        rebuildBaseStations();
        emitCurrentSelection();
        emit statusMessage(QStringLiteral("Selected base station %1. Drag to move.").arg(stations_[static_cast<size_t>(selectedStationIndex_)].name));
        requestRender();
        return true;
    }

    if (selectedStationIndex_ >= 0) {
        clearBaseStationSelection();
    }
    hideHoverCard();
    return false;
}

int Scene3DWidget::pathCountForStation(const QString& stationName) const {
    int count = 0;
    for (const auto& path : paths_) {
        if (path.txId == stationName) {
            ++count;
        }
    }
    return count;
}

QString Scene3DWidget::stationCardText(int index) const {
    if (index < 0 || index >= static_cast<int>(stations_.size())) {
        return QString();
    }
    return stationInfoBody(stations_[static_cast<size_t>(index)], paths_);
}

void Scene3DWidget::ensureHoverCard() {
    if (hoverCard_) return;
    QWidget* host = window() ? window() : this;
    hoverCard_ = createOverlayCard(host, &hoverCardLabel_, &hoverPinButton_, true, false);
    hoverCard_->setAttribute(Qt::WA_ShowWithoutActivating, true);
    hoverCard_->setMouseTracking(true);
    hoverCard_->installEventFilter(this);
    const auto hoverChildren = hoverCard_->findChildren<QWidget*>();
    for (QWidget* child : hoverChildren) {
        child->setMouseTracking(true);
        child->installEventFilter(this);
    }
    hoverCard_->hide();
    if (hoverPinButton_) {
        QObject::connect(hoverPinButton_, &QToolButton::clicked, this, [this]() { pinHoverCard(); });
    }
}

void Scene3DWidget::showHoverCardForStation(int index, const QPoint& localPos) {
    if (index < 0 || index >= static_cast<int>(stations_.size())) {
        hideHoverCard();
        return;
    }
    ensureHoverCard();
    hoverCardStationIndex_ = index;
    hoverCardAnchor_ = localPos;
    if (hoverCardLabel_) {
        hoverCardLabel_->setText(stationCardText(index));
    }
    QWidget* host = hoverCard_->parentWidget() ? hoverCard_->parentWidget() : this;
    QPoint pos = host->mapFromGlobal(vtkWidget_->mapToGlobal(localPos)) + QPoint(18, 18);
    const QSize sz = hoverCard_->sizeHint().expandedTo(QSize(250, 120));
    hoverCard_->resize(sz);
    const QRect bounds = host->rect();
    if (pos.x() + sz.width() > bounds.width() - 8) pos.setX(bounds.width() - sz.width() - 8);
    if (pos.y() + sz.height() > bounds.height() - 8) pos.setY(bounds.height() - sz.height() - 8);
    pos.setX(std::max(8, pos.x()));
    pos.setY(std::max(8, pos.y()));
    hoverCard_->move(pos);
    hoverCard_->show();
    hoverCard_->raise();
}

void Scene3DWidget::hideHoverCard() {
    if (hoverCard_) hoverCard_->hide();
    hoverCardStationIndex_ = -1;
}

QPoint Scene3DWidget::hoverAnchorGlobal() const {
    if (!vtkWidget_) {
        return QPoint();
    }
    return vtkWidget_->mapToGlobal(hoverCardAnchor_);
}

bool Scene3DWidget::shouldKeepHoverCardVisible(const QPoint& globalPos) const {
    if (!hoverCard_ || !hoverCard_->isVisible() || hoverCardStationIndex_ < 0) {
        return false;
    }
    const QRect cardRect = QRect(hoverCard_->mapToGlobal(QPoint(0, 0)), hoverCard_->size());
    if (cardRect.contains(globalPos)) {
        return true;
    }
    const QPoint anchor = hoverAnchorGlobal();
    if (anchor.isNull()) {
        return false;
    }
    const int dx = globalPos.x() - anchor.x();
    const int dy = globalPos.y() - anchor.y();
    return (dx * dx + dy * dy) <= (hoverPersistenceRadiusPx_ * hoverPersistenceRadiusPx_);
}

void Scene3DWidget::pinHoverCard() {
    if (hoverCardStationIndex_ < 0 || hoverCardStationIndex_ >= static_cast<int>(stations_.size())) return;
    QLabel* label = nullptr;
    QToolButton* unused = nullptr;
    QWidget* host = window() ? window() : this;
    auto* card = createOverlayCard(host, &label, &unused, false, true);
    card->setProperty("stationIndex", hoverCardStationIndex_);
    if (label) label->setText(stationCardText(hoverCardStationIndex_));
    QPoint pos = hoverCard_ ? hoverCard_->pos() + QPoint(16 * static_cast<int>(pinnedCards_.size() % 4), 16 * static_cast<int>(pinnedCards_.size() % 4)) : QPoint(24, 24);
    card->move(pos);
    card->show();
    card->raise();
    pinnedCards_.push_back(card);
}

void Scene3DWidget::refreshPinnedCards() {
    for (auto it = pinnedCards_.begin(); it != pinnedCards_.end();) {
        QFrame* card = *it;
        if (!card) { it = pinnedCards_.erase(it); continue; }
        if (!card->parent()) { it = pinnedCards_.erase(it); continue; }
        const int index = card->property("stationIndex").toInt();
        if (index < 0 || index >= static_cast<int>(stations_.size())) {
            card->deleteLater();
            it = pinnedCards_.erase(it);
            continue;
        }
        const auto labels = card->findChildren<QLabel*>();
        if (labels.size() >= 2) {
            labels[1]->setText(stationCardText(index));
        }
        ++it;
    }
}

void Scene3DWidget::requestRender() {
    if (viewDragBall_) {
        viewDragBall_->refreshFromCamera();
    }
    if (renderWindow_) {
        renderWindow_->Render();
    }
}

}  // namespace airsim_gui
