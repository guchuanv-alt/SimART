#pragma once

#include "airsim_gui_UErealtime/AppTypes.h"
#include "airsim_gui_UErealtime/SceneTypes.h"

#include <QWidget>
#include <QString>
#include <vector>

#include <vtkSmartPointer.h>

class QVTKWidget;
class vtkRenderWindow;
class vtkRenderer;
class vtkAxesActor;
class vtkActor;
class vtkAssembly;
class QFrame;
class QToolButton;
class QLabel;

namespace airsim_gui {

class VtkViewDragBall;

class Scene3DWidget : public QWidget {
    Q_OBJECT
public:
    enum class StationInteractionMode {
        None = 0,
        Add,
        Delete
    };

    explicit Scene3DWidget(QWidget* parent = nullptr);
    ~Scene3DWidget() override;

    void setWorldConfig(const WorldConfig& world);
    void setBaseStations(const std::vector<BaseStation>& stations);
    void setDroneState(const DroneState& state);
    void setBeamPaths(const BeamPathList& paths);

    bool setSceneGeometry(const SceneGeometryBundle& bundle, QString* errorMessage = nullptr);
    bool loadMapMesh(const QString& filePath, QString* errorMessage = nullptr);
    void clearSceneGeometry();
    void clearMapMesh();
    bool sceneBoundsXY(double* minX, double* maxX, double* minY, double* maxY) const;
    QWidget* overlayHostWidget() const;

    int selectedBaseStationIndex() const { return selectedStationIndex_; }
    StationInteractionMode stationInteractionMode() const { return stationInteractionMode_; }

signals:
    void baseStationEditStarted();
    void baseStationEditFinished();
    void baseStationsEdited(const std::vector<BaseStation>& stations);
    void stationSelectionChanged(int index, const airsim_gui::BaseStation& station);
    void statusMessage(const QString& message);

public slots:
    void resetCamera();
    void setGroundVisible(bool visible);
    void setBuildingsVisible(bool visible);
    void setBaseStationsVisible(bool visible);
    void setDroneVisible(bool visible);
    void setHistoryVisible(bool visible);
    void setPathsVisible(bool visible);
    void setSceneMeshVisible(bool visible);
    void setAxesVisible(bool visible);
    void setViewGizmoVisible(bool visible);
    void setStationInteractionMode(airsim_gui::Scene3DWidget::StationInteractionMode mode);
    void clearBaseStationSelection();
    bool setSelectedBaseStationIndex(int index);
    bool deleteSelectedBaseStation();
    void addBaseStation();
    bool updateSelectedBaseStationPosition(const Vec3& position);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;


private:
    void initializeScene();
    void rebuildGround();
    void rebuildBlocks();
    void rebuildBaseStations();
    void rebuildDroneActor();
    void rebuildHistoryActor();
    void rebuildPathActors();
    void requestRender();
    void focusSceneGeometry();
    void updateSceneVisibilityState();
    bool hasImportedSceneGeometry() const;
    bool handleMousePress(int x, int y);
    bool pickWorldPoint(int x, int y, Vec3* worldPoint, vtkActor** pickedActor = nullptr) const;
    int findStationActorIndex(vtkActor* actor) const;
    void emitCurrentSelection();
    QString nextStationId() const;
    double defaultStationHeight() const;
    int pathCountForStation(const QString& stationName) const;
    QString stationCardText(int index) const;
    void ensureHoverCard();
    void showHoverCardForStation(int index, const QPoint& localPos);
    void hideHoverCard();
    void pinHoverCard();
    void refreshPinnedCards();
    bool shouldKeepHoverCardVisible(const QPoint& globalPos) const;
    QPoint hoverAnchorGlobal() const;
    void updateViewDragBallGeometry();

    QVTKWidget* vtkWidget_{nullptr};
    vtkSmartPointer<vtkRenderWindow> renderWindow_;
    vtkSmartPointer<vtkRenderer> renderer_;
    VtkViewDragBall* viewDragBall_{nullptr};

    WorldConfig world_;
    std::vector<BaseStation> stations_;
    DroneState drone_;
    BeamPathList paths_;
    SceneGeometryBundle sceneBundle_;

    vtkSmartPointer<vtkActor> groundActor_;
    vtkSmartPointer<vtkActor> historyActor_;
    vtkSmartPointer<vtkAssembly> droneAssembly_;
    vtkSmartPointer<vtkAxesActor> axesActor_;
    std::vector<vtkSmartPointer<vtkActor>> blockActors_;
    std::vector<vtkSmartPointer<vtkActor>> stationActors_;
    std::vector<vtkSmartPointer<vtkActor>> pathActors_;
    std::vector<vtkSmartPointer<vtkActor>> sceneMeshActors_;
    int selectedStationIndex_{-1};

    bool groundLayerVisible_{true};
    bool buildingsLayerVisible_{true};
    bool sceneMeshLayerVisible_{true};
    bool axesLayerVisible_{true};

    bool draggingStation_{false};
    bool dragMovedStation_{false};
    int dragStationIndex_{-1};
    StationInteractionMode stationInteractionMode_{StationInteractionMode::None};

    QFrame* hoverCard_{nullptr};
    QLabel* hoverCardLabel_{nullptr};
    QToolButton* hoverPinButton_{nullptr};
    int hoverCardStationIndex_{-1};
    QPoint hoverCardAnchor_;
    int hoverPersistenceRadiusPx_{48};
    std::vector<QFrame*> pinnedCards_;
};

}  // namespace airsim_gui
