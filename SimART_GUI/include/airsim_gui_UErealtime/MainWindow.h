#pragma once

#include "airsim_gui_UErealtime/AppTypes.h"
#include "airsim_gui_UErealtime/FrameTransformManager.h"
#include "airsim_gui_UErealtime/SceneTypes.h"
#include "airsim_gui_UErealtime/TransformUtils.h"

#include <QMainWindow>
#include <QMap>
#include <QString>
#include <QProcess>
#include <QStringList>
#include <QVector>
#include <memory>
#include <vector>

class QTreeWidget;
class QLabel;
class QLineEdit;
class QPushButton;
class QAction;
class QTreeWidgetItem;
class QCheckBox;
class QComboBox;
class QPlainTextEdit;
class QDoubleSpinBox;
class QScrollArea;
class QVBoxLayout;
class QGridLayout;
class QSplitter;
class QTabWidget;
class QTimer;
class QActionGroup;
class QSpinBox;
class QDockWidget;
class QToolButton;
class QGroupBox;
class QFrame;
class QProgressBar;
class QSlider;
class QTableWidget;
class QCloseEvent;
class QEvent;

namespace airsim_gui {

class Scene3DWidget;
class UeViewWidget;
class AirSimViewController;
class CameraImageStripWidget;
class SionnaPreviewWidget;
class CkmMapWidget;
class RosBridge;
class DemoController;
class SceneManager;

struct CoordinateTransformSettings {
    QString rosToSceneMatrixPath;
    QString airsimToSceneMatrixPath;
    QString rxArrayFacingDirection{QStringLiteral("front")};
    double uavToRxArrayTx{0.0};
    double uavToRxArrayTy{0.0};
    double uavToRxArrayTz{0.0};
    QString uavToRxArrayCenterMatrixPath;
    QString rxArrayElementsJsonPath;
    QString outputFrameKey{QStringLiteral("3d")};
};

struct SimulationSettings {
    QString scenePath;
    double fcHz{3.5e9};
    QString miVariant{"cuda_ad_mono_polarized"};

    int txArrayNumRows{1};
    int txArrayNumCols{1};
    double txArrayVerticalSpacing{0.5};
    double txArrayHorizontalSpacing{0.5};
    QString txArrayPattern{"iso"};
    QString txArrayPolarization{"V"};

    int rxArrayNumRows{1};
    int rxArrayNumCols{1};
    double rxArrayVerticalSpacing{0.5};
    double rxArrayHorizontalSpacing{0.5};
    QString rxArrayPattern{"iso"};
    QString rxArrayPolarization{"V"};

    int maxDepth{1};
    int samplesPerSrc{256};
    int maxNumPathsPerSrc{64};
    bool syntheticArray{true};
    bool mergeShapes{false};

    bool enableSysIntegration{false};
    int sysNumSubcarriers{128};
    double sysSubcarrierSpacingHz{30000.0};
    int sysNumOfdmSymbols{12};
    double sysTemperatureK{294.0};
    double sysBlerTarget{0.1};
    int sysMcsTableIndex{1};
    double sysBsTxPowerDbm{10.0};

    bool enableBeamforming{false};
    QString beamSelectionMode{QStringLiteral("exhaustive_sweep")};
    QString beamCodebookType{QStringLiteral("auto")};
    int beamCodebookNumBeams{8};
    int beamOversamplingV{1};
    int beamOversamplingH{1};
    int beamManualIndex{0};
    bool beamNormalizePower{true};
    QString beamCodebookFilePath;
    QString beamModelCheckpointPath;
    QString beamFeatureMode{QStringLiteral("geom_vel_path13")};
    int beamTopK{3};
    bool beamExportTrainingDataset{false};
    QString beamTrainingDatasetPath{QStringLiteral("beam_training_samples.csv")};
    QString beamTrainingInputPaths;
    bool beamTrainingResetDatasetOnStart{false};
    QString beamTrainingOutputCheckpointPath{QStringLiteral("beam_trained_model.pt")};
    int beamTrainingEpochs{30};
    int beamTrainingBatchSize{256};
    double beamTrainingLearningRate{1e-3};
    double beamTrainingValidationSplit{0.2};
    int beamTrainingHiddenDim{512};

    bool los{true};
    bool specularReflection{true};
    bool diffuseReflection{false};
    bool refraction{false};
    bool diffraction{false};
    bool edgeDiffraction{false};
    bool diffractionLitRegion{false};

    double simHz{10.0};
    double maxPoseStalenessS{1.0};
    QString rfFrameId{"map"};
    bool csvEnabled{false};
    QString csvPath;
    QString pythonExecutable{"python3"};
};

struct CkmSettings {
    QString metric{"power_db"};
    QStringList selectedMetrics{QStringList{QStringLiteral("power_db")}};
    bool useSceneBounds{false};
    double xMin{-100.0};
    double xMax{100.0};
    double yMin{-100.0};
    double yMax{100.0};
    double zFixed{1.5};
    double resolutionM{5.0};
    QString outputDir{""};
    bool autoLoadResult{true};
    bool renderSceneOverlay{true};
    QString renderScenePath;
};

struct CameraImageLayerItem {
    QString key;
    QString label;
    QString topic;
    int imageType{0};
};

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void connectRos();
    void disconnectRos();
    void startDemo();
    void stopDemo();
    void loadSceneMesh();
    void importOsmScene();
    void clearSceneGeometry();
    void reloadConfigs();
    void refreshRosTopics();
    void updateSelectedTopicTypes();
    void openCoordinateTransformSettings();
    void openSimulationSettings();
    void loadBaseStationsJson();
    void saveBaseStationsJson();
    void saveBaseStationsJsonAs();
    void openGuiConfig();
    void saveGuiConfig();
    void saveGuiConfigAs();
    void enterAddBaseStationMode();
    void deleteSelectedBaseStation();
    void undoBaseStationEdit();
    void syncAirSimSettingsExternalCameras();
    void browseAirSimSettingsPath();
    void useAutoAirSimSettingsPath();
    void onAirSimSettingsPathEditingFinished();
    void onBaseStationEditStarted();
    void onBaseStationEditFinished();
    void onStationCoordinateEdited();
    void onStationDetailsEdited();
    void onBaseStationsEdited(const std::vector<airsim_gui::BaseStation>& stations);
    void onStationSelectionChanged(int index, const airsim_gui::BaseStation& station);
    void onDroneUpdated(const airsim_gui::DroneState& state);
    void onPathsUpdated(const airsim_gui::BeamPathList& paths);
    void onStatusMessage(const QString& message);
    void onLayerItemChanged(QTreeWidgetItem* item, int column);
    void onSimulatorOutput();
    void onSimulatorError(QProcess::ProcessError error);
    void onSimulatorFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void startAirSimLiveView();
    void stopAirSimLiveView();
    void restartAirSimLiveView();
    void switchToEditorView();
    void switchToLiveView();
    void switchToSionnaPreview();
    void switchToCkmMap();
    void onCenterViewTabChanged(int index);
    void restartSionnaPreview();
    void syncSionnaPreviewSettings();
    void onAirSimLiveViewStatus(const QString& text);
    void onAirSimBrightnessSliderChanged(int value);
    void onStationCameraFpsTopicChanged(int index);
    void onStationCameraFpsValueChanged(int value);
    void startPublishingAllStationCameraTopics();
    void stopPublishingAllStationCameraTopics();
    void refreshDeveloperTopicList();
    void applyDeveloperTopicPublishing();
    void restoreDefaultTopicPublishing();
    void syncAirSimLiveViewSettings();
    void onRfDataUpdated(const airsim_gui::RfObservationSnapshot& snapshot);
    void onSysObservationUpdated(const QString& jsonText);
    void onBeamObservationUpdated(const QString& jsonText);
    void showRfDataWindow();
    void showSysDataWindow();
    void showDeveloperToolsPanel();
    void refreshAirSimCameraList();
    void browseRosbagRecordOutput();
    void toggleRosbagRecording();
    void showRosbagToolsWindow();
    void browseRosbagPlaybackInput();
    void toggleRosbagPlayback();
    void browseRosbagResimInput();
    void browseRosbagResimOutput();
    void toggleRosbagResim();
    void onRosbagRecordOutput();
    void onRosbagRecordFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onRosbagPlaybackOutput();
    void onRosbagPlaybackFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onRosbagResimOutput();
    void onRosbagResimFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void generateDenseCkm();
    void stopDenseCkmFromUi();
    void loadDenseCkmFromDirectory();
    void onCkmProcessOutput();
    void onCkmProcessError(QProcess::ProcessError error);
    void onCkmProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onCkmPointSelectionChanged(const QString& title, const QString& summary, const QString& details);
    void onCkmPointSelectionCleared(const QString& hint);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void closeEvent(QCloseEvent* event) override;

private:
    struct StationCameraWindow {
        QString stationKey;
        QDockWidget* dock{nullptr};
        UeViewWidget* view{nullptr};
        QToolButton* pinButton{nullptr};
        QToolButton* saveViewButton{nullptr};
        QLabel* topicLabel{nullptr};
        QLabel* statusLabel{nullptr};
        bool pinned{false};
        bool previewTargetAppliedInitialized{false};
        bool appliedPreviewCameraTargetEnabled{false};
        Vec3 appliedPreviewCameraTarget{};
        Vec3 appliedAnchorPoint{};
    };

    struct RosPublisherRegistration {
        QString topicName;
        QString topicType;
        QString nodeName;
        QString nodeUri;
    };

    struct RfCardView {
        QFrame* frame{nullptr};
        QLabel* titleLabel{nullptr};
        QLabel* bodyLabel{nullptr};
        int row{-1};
        int column{-1};
        int rowSpan{1};
        int columnSpan{1};
    };

    struct SysCandidateView {
        int simIdx{-1};
        double odomStampS{std::numeric_limits<double>::quiet_NaN()};
        QString bsName;
        int anchorId{-1};
        int numPaths{-1};
        double strongestPathPowerDb{std::numeric_limits<double>::quiet_NaN()};
        QString strongestPathType;
        int strongestPathOrder{-1};
        bool sysEnabled{false};
        double candidateRateBpsHz{std::numeric_limits<double>::quiet_NaN()};
        double candidateSinrEffDb{std::numeric_limits<double>::quiet_NaN()};
        int mcsIndex{-1};
        int numDecodedBits{-1};
        int tbOk{-1};
        double spectralEfficiencyBpsHz{std::numeric_limits<double>::quiet_NaN()};
        bool isServingBs{false};
        QString servingBsName;
        int servingAnchorId{-1};
        double blerTarget{std::numeric_limits<double>::quiet_NaN()};
        bool beamEnabled{false};
        QString beamSelectionMode;
        QString beamCodebookType;
        int beamNumBeams{-1};
        int beamSelectedIndex{-1};
        double beamSelectedGainDb{std::numeric_limits<double>::quiet_NaN()};
        int beamOracleIndex{-1};
        double beamOracleGainDb{std::numeric_limits<double>::quiet_NaN()};
        bool beamSelectedAnchor{false};
        QString beamPredictorStatus;
        QString beamFeatureMode;
        int beamTopK{-1};
        int beamPredictedIndex{-1};
        double beamPredictedConfidence{std::numeric_limits<double>::quiet_NaN()};
        int beamOracleInTopK{-1};
        int beamHit{-1};
        QString beamSelectionSource;
        QString beamTopkIndices;
    };

    struct SysSnapshotView {
        bool valid{false};
        bool enabled{false};
        QString schedulerPolicy;
        int simIdx{-1};
        double odomStampS{std::numeric_limits<double>::quiet_NaN()};
        QString servingBsName;
        int servingAnchorId{-1};
        int servingIdx{-1};
        int servingMcsIndex{-1};
        double servingSinrEffDb{std::numeric_limits<double>::quiet_NaN()};
        double servingSpectralEfficiencyBpsHz{std::numeric_limits<double>::quiet_NaN()};
        int servingTbOk{-1};
        bool beamEnabled{false};
        QString beamSelectionMode;
        QString beamCodebookType;
        int beamNumBeams{-1};
        QString beamReferenceBsName;
        int beamReferenceAnchorId{-1};
        int beamReferenceTxIdx{-1};
        int beamSelectedIndex{-1};
        double beamSelectedGainDb{std::numeric_limits<double>::quiet_NaN()};
        int beamOracleIndex{-1};
        double beamOracleGainDb{std::numeric_limits<double>::quiet_NaN()};
        int beamManualIndex{-1};
        QString beamServingSource;
        bool beamPredictorAvailable{false};
        QString beamPredictorStatus;
        QString beamPredictorError;
        QString beamFeatureMode;
        int beamTopK{-1};
        int beamPredictedIndex{-1};
        double beamPredictedConfidence{std::numeric_limits<double>::quiet_NaN()};
        int beamOracleInTopK{-1};
        int beamHit{-1};
        QString beamSelectionSource;
        QString beamTopkIndices;
        std::vector<SysCandidateView> candidates;
    };

    struct AirSimCameraCaptureConfig {
        int width{1280};
        int height{720};
        int fovDeg{90};
    };

    void buildUi();
    void buildMenuBar();
    void buildToolBar();
    void buildLeftDock();
    void buildRightDock();
    void buildDeveloperToolsDock();
    void loadInitialData();
    void promptStartupConfigChoice();
    QString findConfigFile(const QString& relativePath) const;
    QString normalizedGuiConfigPath(const QString& path) const;
    CoordinateTransformSettings defaultCoordinateTransformSettings() const;
    void loadCoordinateTransformSettings();
    void applyCoordinateTransformSettings(bool restartRosPipeline);
    Vec3 scenePointToAirSim(const Vec3& scenePoint) const;
    void refreshInfoPanel();
    void resetLiveSessionState(bool clearSysSnapshot = true);
    void rebuildStationCards();
    void rebuildRfDataCards();
    void rebuildSysDataWindow();
    void scheduleSysDataWindowRefresh(bool immediate = false);
    void refreshSysDataWindowIfNeeded();
    void ensureRfDataWindow();
    void ensureSysDataWindow();
    void applyLayerState(const QString& key, bool enabled);
    void applyAirSimLayerState(AirSimViewController* controller, const QString& key, bool enabled);
    void applyCurrentLayerStatesToAirSimController(AirSimViewController* controller);
    void refreshLayerPanelForCurrentView();
    void refreshAirSimSettingsSyncStatus();
    QString selectedAirSimSettingsPath() const;
    void setSelectedAirSimSettingsPath(const QString& path);
    QStringList effectiveAirSimSettingsPaths() const;
    QString currentAirSimSettingsPath(QString* errorMessage = nullptr) const;
    QString currentAirSimPreviewCameraName() const;
    bool isStationExternalCameraName(const QString& cameraName) const;
    bool hasCustomAirSimPreviewExternalCamera(const QJsonObject& externalCameras) const;
    bool missingAirSimGuiPreviewExternalCamera(const QString& settingsPath,
                                               QString* errorMessage = nullptr) const;
    QVector<int> missingAirSimExternalCameraStationIndices(const QString& settingsPath,
                                                           QString* errorMessage = nullptr) const;
    QJsonObject buildAirSimGuiPreviewExternalCameraEntry(const AirSimCameraCaptureConfig& captureConfig) const;
    QJsonObject buildAirSimExternalCameraEntries(const QVector<int>& stationIndices,
                                                 const AirSimCameraCaptureConfig& stationCaptureConfig,
                                                 bool includeGuiPreview,
                                                 const AirSimCameraCaptureConfig& guiPreviewCaptureConfig) const;
    bool buildAirSimSettingsPreview(const QString& settingsPath,
                                    const QVector<int>& stationIndices,
                                    const AirSimCameraCaptureConfig& stationCaptureConfig,
                                    bool includeGuiPreview,
                                    const AirSimCameraCaptureConfig& guiPreviewCaptureConfig,
                                    QString* previewText,
                                    QVector<QPair<int, int>>* addedLineRanges,
                                    QString* errorMessage = nullptr) const;
    bool writeMissingAirSimExternalCameras(const QString& settingsPath,
                                           const QVector<int>& stationIndices,
                                           const AirSimCameraCaptureConfig& stationCaptureConfig,
                                           bool includeGuiPreview,
                                           const AirSimCameraCaptureConfig& guiPreviewCaptureConfig,
                                           QString* backupPath = nullptr,
                                           QString* errorMessage = nullptr);
    bool isDroneCameraImageLayerKey(const QString& key) const;
    void syncDroneCameraImageLayerItems();
    void refreshCameraImagePreviews();
    void applySceneBundle(const SceneGeometryBundle& bundle, const QString& successMessage);
    bool startInternalSimulator();
    void stopInternalSimulator();
    bool startDenseCkmGeneration();
    void stopDenseCkmGeneration();
    void loadDenseCkmResult(const QString& metaFilePath);
    void refreshDenseCkmControls();
    QString createDenseCkmOutputRootDir(const QString& baseOutputDir) const;
    bool removeDenseCkmOutputRoot(const QString& outputRootDir, QString* errorMessage = nullptr) const;
    void refreshRightPanelMode();
    QString exportBaseStationsForSimulator() const;
    QString findBundledScript(const QString& relativePath) const;
    QString effectiveDenseCkmBoundsScenePath() const;
    bool readSceneXmlBoundsXY(const QString& sceneXmlPath,
                              double* minX, double* maxX,
                              double* minY, double* maxY,
                              QString* errorMessage = nullptr) const;
    bool resolveDenseCkmAutoBoundsXY(double* minX, double* maxX,
                                     double* minY, double* maxY,
                                     QString* sourceDescription = nullptr,
                                     QString* errorMessage = nullptr) const;
    void enableDenseCkmAutoBoundsIfSceneAvailable();
    bool waitForInternalRfTopic(QString* errorMessage = nullptr, int timeoutMs = 10000);
    QString diagnosticSummary() const;
    void appendSimulatorLog(const QString& text);
    void applyBaseStations(const std::vector<BaseStation>& stations);
    bool persistBaseStationsToFile(const QString& filePath);
    bool saveBaseStationsJsonForPrompt();
    bool promptSaveBaseStationsJsonIfDirty(const QString& message);
    void pushBaseStationUndoState(const std::vector<BaseStation>& stations);
    void clearBaseStationUndoHistory();
    void updateUndoActionState();
    void markGuiConfigDirty();
    void markBaseStationsDirty();
    void setGuiConfigDirty(bool dirty);
    void setBaseStationsDirty(bool dirty);
    void updateMainWindowTitle();
    void updateWindowModifiedState();
    bool hasUnsavedConfigChanges() const;
    bool saveGuiConfigForClose();
    bool promptSaveConfigOnClose();
    QString stationCameraKey(const BaseStation& station) const;
    int stationIndexForKey(const QString& key) const;
    StationCameraWindow* findStationCameraWindow(const QString& key);
    const StationCameraWindow* findStationCameraWindow(const QString& key) const;
    Vec3 airSimPointToScene(const Vec3& airsimPoint) const;
    void saveStationCameraDefaultView(const QString& key);
    void ensureStationCameraWindow(int index);
    void syncStationCameraWindows();
    void syncStationCameraWindow(int index);
    void syncStationCameraFpsControls();
    int cameraFpsForControl(const QString& controlKey, const QString& rosTopic) const;
    QString imageTopicFpsParamName(const QString& rosTopic) const;
    void setAirsimImageTopicFpsLimit(const QString& rosTopic, int fps);
    void applyCameraFpsForControl(const QString& controlKey, const QString& rosTopic, int fps, bool showStatus = true);
    void applyKnownCameraFpsLimits();
    void syncStationCameraSelection();
    void refreshStationCameraStreamingState();
    bool stationCameraHasRosSubscribers(const BaseStation& station) const;
    QString stationCameraPublishControlService(const BaseStation& station) const;
    bool setStationCameraAirsimNodePublishing(const BaseStation& station, bool enabled,
                                              QString* errorMessage = nullptr);
    void disableAllStationCameraAirsimNodePublishing();
    void syncDeveloperTopicPublishingControls();
    bool suppressRosTopicPublisherRegistrations(const QString& topicName, QString* errorMessage = nullptr);
    bool restoreRosTopicPublisherRegistrations(const QString& topicName, QString* errorMessage = nullptr);
    void restoreAllRosTopicPublisherRegistrations(bool showWarnings = true);
    void removeMissingStationCameraWindows();
    void updateStationCameraWindowTitle(int index);
    void stopAllStationCameraWindows();
    void ensureRosbagToolsWindow();
    QString defaultRosbagOutputPath(const QString& baseNameHint) const;
    bool connectRosBridgeOnly(const QString& poseTopic, const QString& trajectoryTopic, const QString& rfTopic,
                              const QString& sysTopic = QString(), const QString& beamTopic = QString(),
                              QString* errorMessage = nullptr, int timeoutMs = 10000);
    void updateRosbagUiState();
    void updateRosbagPlaybackTopicPresence();
    void updateRosbagResimTopicPresence();
    void appendRosbagLog(const QString& text);
    bool saveGuiConfigToFile(const QString& filePath, QString* errorMessage = nullptr);
    bool loadGuiConfigFromFile(const QString& filePath, QString* errorMessage = nullptr);
    void stopProcessGracefully(QProcess* process, int waitMs = 5000);
    bool ensureRoscoreRunning(QString* errorMessage = nullptr, int timeoutMs = 10000);
    bool waitForRosBridgePose(QString* errorMessage = nullptr, int timeoutMs = 5000);
    bool restoreRosbagPlaybackPoseBridge(QString* errorMessage = nullptr);
    QString ensureRuntimeLogFile();
    void appendRuntimeLogLine(const QString& channel, const QString& text);
    void showShortDiagnosticWarning(const QString& title, const QString& summary, const QString& detail);
    double currentAirSimBrightnessFactor() const;
    static QVector<int> parseBeamTopkIndices(const QString& text);
    static bool sysCandidateMatchesIdentity(const SysCandidateView& candidate, int anchorId, const QString& bsName);
    static bool sysCandidateMatchesServing(const SysCandidateView& candidate, const SysSnapshotView& snapshot);
    static bool sysCandidateHasUsablePath(const SysCandidateView& candidate);
    static bool sysCandidateHasBeamData(const SysCandidateView& candidate);
    static void normalizeSysCandidateBeamFields(SysCandidateView* candidate);
    static void normalizeSysSnapshotBeamFields(SysSnapshotView* snapshot);
    static void copySysCandidateBeamFields(SysCandidateView* target, const SysCandidateView& source);
    void normalizeCurrentSysCandidates();
    void syncCurrentBeamSummaryFromCandidate(const SysCandidateView& candidate, const QJsonObject* payload = nullptr);

    Scene3DWidget* sceneWidget_{nullptr};
    UeViewWidget* ueViewWidget_{nullptr};
    CameraImageStripWidget* sceneCameraImageStrip_{nullptr};
    CameraImageStripWidget* liveCameraImageStrip_{nullptr};
    SionnaPreviewWidget* sionnaPreviewWidget_{nullptr};
    CkmMapWidget* ckmMapWidget_{nullptr};
    AirSimViewController* airSimViewController_{nullptr};
    RosBridge* rosBridge_{nullptr};
    DemoController* demoController_{nullptr};
    SceneManager* sceneManager_{nullptr};
    std::shared_ptr<FrameTransformManager> frameTransformManager_;

    std::vector<BaseStation> stations_;
    std::vector<std::vector<BaseStation>> baseStationUndoStack_;
    std::vector<BaseStation> baseStationEditTransactionBaseline_;
    WorldConfig world_;
    DroneState currentDrone_;
    BeamPathList currentPaths_;
    RfObservationSnapshot currentRfSnapshot_;
    QString currentSceneSource_{"None"};
    QString currentSceneMeshPath_;
    QString baseStationsConfigPath_;
    QString internalRfTopic_{"/airsim_gui_UErealtime/rf_observations"};
    QString internalSysTopic_{"/airsim_gui_UErealtime/sys_observations"};
    QString internalBeamTopic_{"/airsim_gui_UErealtime/beam_observations"};
    QString internalBeamCodebookTopic_{"/airsim_gui_UErealtime/beam_codebook"};
    QString lastExportedBsJsonPath_;
    QString currentGuiConfigPath_;
    SimulationSettings simSettings_;
    CkmSettings ckmSettings_;
    CoordinateTransformSettings coordinateTransforms_;
    QProcess* simulatorProcess_{nullptr};
    QProcess* ckmProcess_{nullptr};
    QProcess* rosbagRecordProcess_{nullptr};
    QProcess* rosbagPlayProcess_{nullptr};
    QProcess* rosbagResimProcess_{nullptr};
    QProcess* managedRoscoreProcess_{nullptr};
    QString simulatorLog_;
    QString runtimeLogFilePath_;
    QString rosbagToolsLog_;
    QString lastCkmMetaPath_;
    QString activeCkmOutputRootDir_;
    QString lastStatusMessage_;
    QString sysModeSummary_{QStringLiteral("Disabled")};
    QString sysServingSummary_{QStringLiteral("-")};
    QString sysLinkSummary_{QStringLiteral("-")};
    SysSnapshotView currentSysSnapshot_;
    int selectedBaseStationIndex_{-1};
    bool suppressStationCoordinateUpdates_{false};
    bool suppressDirtyTracking_{false};
    bool guiConfigDirty_{false};
    bool baseStationsDirty_{false};
    bool guiConfigSessionActive_{false};
    bool undoingBaseStationEdit_{false};
    bool baseStationEditTransactionActive_{false};
    bool baseStationUndoCapturedForTransaction_{false};
    bool reconnectAirSimLiveViewOnReturn_{false};
    bool liveViewTabActive_{false};
    bool ckmStopRequested_{false};
    bool rosbagPlaybackActive_{false};
    bool rosbagPlaybackBagHasWirelessTopics_{false};
    bool rosbagPlaybackUsingRfTopic_{false};
    bool rosbagPlaybackUsingSysTopic_{false};
    bool rosbagPlaybackUsingBeamTopic_{false};
    QString rosbagPlaybackPoseTopic_;
    QString rosbagPlaybackTrajectoryTopic_;
    bool forceAllStationCameraPublishing_{false};
    bool manualRosTopicPublishingEnabled_{false};
    QMap<QString, bool> manualRosTopicPublishing_;
    QMap<QString, std::vector<RosPublisherRegistration>> suppressedRosPublisherRegistrations_;
    QMap<QString, bool> stationCameraAirsimNodePublishingState_;
    QMap<QString, qint64> stationCameraAirsimNodePublishingLastRequestMs_;
    QMap<QString, int> cameraFpsOverrides_;
    QMap<QString, int> airsimImageTopicFpsLastApplied_;
    QVector<CameraImageLayerItem> droneCameraImageLayers_;
    qint64 stationCameraLastAirSimConnectedMs_{0};

    QDockWidget* layersDock_{nullptr};
    QDockWidget* lastLeftDock_{nullptr};
    std::vector<StationCameraWindow> stationCameraWindows_;
    QTimer* stationCameraDemandTimer_{nullptr};

    QTreeWidget* layerTree_{nullptr};
    QFrame* airsimSettingsSyncPanel_{nullptr};
    QLabel* airsimSettingsSyncHintLabel_{nullptr};
    QPushButton* syncAirSimSettingsButton_{nullptr};
    QLabel* modeValue_{nullptr};
    QLabel* positionValue_{nullptr};
    QLabel* attitudeValue_{nullptr};
    QLabel* baseStationCountValue_{nullptr};
    QLabel* pathCountValue_{nullptr};
    QLabel* strongestPathValue_{nullptr};
    QLabel* sysModeValue_{nullptr};
    QLabel* sysServingValue_{nullptr};
    QLabel* sysLinkValue_{nullptr};
    QLabel* sceneSourceValue_{nullptr};
    QLabel* poseTypeValue_{nullptr};
    QLabel* trajectoryTypeValue_{nullptr};
    QLabel* rfTopicValue_{nullptr};
    QLabel* sysTopicValue_{nullptr};
    QLabel* simulatorStatusValue_{nullptr};
    QLabel* baseStationFileValue_{nullptr};
    QLabel* baseStationSelectionValue_{nullptr};
    QLabel* baseStationModeValue_{nullptr};
    QPlainTextEdit* simulatorLogView_{nullptr};
    QGroupBox* liveStateBox_{nullptr};
    QGroupBox* ckmDataBox_{nullptr};
    QLabel* ckmDataTitleValue_{nullptr};
    QLabel* ckmDataSummaryValue_{nullptr};
    QPlainTextEdit* ckmDataDetailsView_{nullptr};
    QScrollArea* liveStateScroll_{nullptr};
    QWidget* liveStateCardsContainer_{nullptr};
    QVBoxLayout* liveStateCardsLayout_{nullptr};
    QSplitter* rightPanelSplitter_{nullptr};
    QTabWidget* centerViewTabs_{nullptr};
    QComboBox* poseTopicCombo_{nullptr};
    QComboBox* trajectoryTopicCombo_{nullptr};
    QPushButton* refreshTopicsButton_{nullptr};
    QPushButton* connectButton_{nullptr};
    QPushButton* disconnectButton_{nullptr};
    QPushButton* demoButton_{nullptr};
    QPushButton* stopDemoButton_{nullptr};
    QPushButton* simSettingsButton_{nullptr};
    QPushButton* coordinateTransformsButton_{nullptr};
    QPushButton* loadStationsButton_{nullptr};
    QPushButton* saveStationsButton_{nullptr};
    QPushButton* addStationButton_{nullptr};
    QPushButton* deleteStationButton_{nullptr};
    QLineEdit* stationIdEdit_{nullptr};
    QLineEdit* stationNameEdit_{nullptr};
    QLineEdit* stationPreviewCameraEdit_{nullptr};
    QLineEdit* stationPreviewRosTopicEdit_{nullptr};
    QDoubleSpinBox* stationXSpin_{nullptr};
    QDoubleSpinBox* stationYSpin_{nullptr};
    QDoubleSpinBox* stationZSpin_{nullptr};
    QDoubleSpinBox* stationPreviewOffsetZSpin_{nullptr};
    QDoubleSpinBox* stationColorRSpin_{nullptr};
    QDoubleSpinBox* stationColorGSpin_{nullptr};
    QDoubleSpinBox* stationColorBSpin_{nullptr};
    QLineEdit* airsimHostEdit_{nullptr};
    QLabel* osmSelectionValue_{nullptr};
    QLabel* osmSourceValue_{nullptr};
    QPushButton* importOsmButton_{nullptr};
    QComboBox* airsimLiveCameraCombo_{nullptr};
    QSlider* airsimBrightnessSlider_{nullptr};
    QLineEdit* airsimLiveVehicleEdit_{nullptr};
    QLineEdit* airsimPortEdit_{nullptr};
    QCheckBox* airsimCacheCheck_{nullptr};
    QCheckBox* airsimLiveFollowCheck_{nullptr};
    QLineEdit* airsimSettingsPathEdit_{nullptr};
    QLineEdit* airsimCachePathEdit_{nullptr};
    QDoubleSpinBox* airsimLiveFpsSpin_{nullptr};
    QComboBox* stationCameraFpsTopicCombo_{nullptr};
    QSpinBox* stationCameraFpsSpin_{nullptr};
    QPushButton* publishAllStationCameraTopicsButton_{nullptr};
    QPushButton* stopStationCameraPublishingButton_{nullptr};
    QMainWindow* developerToolsWindow_{nullptr};
    QMainWindow* sionnaPreviewWindow_{nullptr};
    QTreeWidget* rosTopicPublishTree_{nullptr};
    QLabel* rosTopicPublishModeValue_{nullptr};
    QPushButton* refreshRosTopicPublishingButton_{nullptr};
    QPushButton* applyRosTopicPublishingButton_{nullptr};
    QPushButton* restoreDefaultTopicPublishingButton_{nullptr};
    QLabel* airsimSettingsPathValue_{nullptr};
    QLabel* airsimBrightnessValueLabel_{nullptr};
    QLabel* sionnaScenePathValue_{nullptr};
    QLabel* sionnaPreviewHintValue_{nullptr};
    QGroupBox* sionnaPreviewControlsBox_{nullptr};
    QGroupBox* rosbagRecordBox_{nullptr};
    QLineEdit* rosbagRecordPathEdit_{nullptr};
    QCheckBox* rosbagRecordIncludeRfCheck_{nullptr};
    QCheckBox* rosbagRecordIncludeSysCheck_{nullptr};
    QCheckBox* rosbagRecordIncludeBeamCheck_{nullptr};
    QPushButton* rosbagRecordToggleButton_{nullptr};
    QLabel* rosbagRecordStatusValue_{nullptr};
    QPushButton* rosbagToolsButton_{nullptr};

    QMainWindow* rfDataWindow_{nullptr};
    QScrollArea* rfDataScroll_{nullptr};
    QWidget* rfDataCardsContainer_{nullptr};
    QGridLayout* rfDataCardsLayout_{nullptr};
    QMap<QString, RfCardView> rfDataCardViews_;
    int rfDataSpacerRow_{-1};

    QMainWindow* sysDataWindow_{nullptr};
    QLabel* sysDataStatusBodyLabel_{nullptr};
    QLabel* sysDataServingBodyLabel_{nullptr};
    QLabel* sysDataConfigBodyLabel_{nullptr};
    QTableWidget* sysDataTable_{nullptr};
    QPlainTextEdit* sysDataRawView_{nullptr};
    QTimer* sysDataRefreshTimer_{nullptr};
    bool sysDataRefreshPending_{false};
    bool sysDataTableLayoutInitialized_{false};
    QString sysDataLastRawJson_;

    QAction* openGuiConfigAction_{nullptr};
    QAction* saveGuiConfigAction_{nullptr};
    QAction* saveGuiConfigAsAction_{nullptr};
    QAction* loadSceneMeshAction_{nullptr};
    QAction* importOsmSceneAction_{nullptr};
    QAction* clearSceneAction_{nullptr};
    QAction* reloadConfigAction_{nullptr};
    QAction* simSettingsAction_{nullptr};
    QAction* coordinateTransformsAction_{nullptr};
    QAction* loadBaseStationsAction_{nullptr};
    QAction* saveBaseStationsAction_{nullptr};
    QAction* saveBaseStationsAsAction_{nullptr};
    QAction* addBaseStationAction_{nullptr};
    QAction* deleteBaseStationAction_{nullptr};
    QAction* undoBaseStationEditAction_{nullptr};
    QAction* showRfDataWindowAction_{nullptr};
    QAction* showSysDataWindowAction_{nullptr};
    QAction* showDeveloperToolsAction_{nullptr};
    QAction* stopAirSimLiveViewAction_{nullptr};
    QAction* showRosbagToolsAction_{nullptr};
    QAction* editorViewAction_{nullptr};
    QAction* liveViewAction_{nullptr};
    QAction* ckmMapViewAction_{nullptr};

    QMainWindow* rosbagToolsWindow_{nullptr};
    QTabWidget* rosbagToolsTabs_{nullptr};
    QLineEdit* rosbagPlaybackInputEdit_{nullptr};
    QLineEdit* rosbagPlaybackRfTopicEdit_{nullptr};
    QLineEdit* rosbagPlaybackSysTopicEdit_{nullptr};
    QLineEdit* rosbagPlaybackBeamTopicEdit_{nullptr};
    QLineEdit* rosbagPlaybackBeamCodebookTopicEdit_{nullptr};
    QCheckBox* rosbagPlaybackIncludeRfCheck_{nullptr};
    QCheckBox* rosbagPlaybackIncludeSysCheck_{nullptr};
    QCheckBox* rosbagPlaybackIncludeBeamCheck_{nullptr};
    QCheckBox* rosbagPlaybackClockCheck_{nullptr};
    QDoubleSpinBox* rosbagPlaybackRateSpin_{nullptr};
    QPushButton* rosbagPlaybackToggleButton_{nullptr};
    QLabel* rosbagPlaybackStatusValue_{nullptr};
    QLabel* rosbagPlaybackWirelessTopicsValue_{nullptr};
    QLineEdit* rosbagResimInputEdit_{nullptr};
    QLineEdit* rosbagResimOutputEdit_{nullptr};
    QLineEdit* rosbagResimPoseTopicEdit_{nullptr};
    QLineEdit* rosbagResimRfTopicEdit_{nullptr};
    QLineEdit* rosbagResimSysTopicEdit_{nullptr};
    QLineEdit* rosbagResimBeamTopicEdit_{nullptr};
    QLineEdit* rosbagResimBeamCodebookTopicEdit_{nullptr};
    QCheckBox* rosbagResimWriteRfCheck_{nullptr};
    QCheckBox* rosbagResimWriteSysCheck_{nullptr};
    QCheckBox* rosbagResimWriteBeamCheck_{nullptr};
    QDoubleSpinBox* rosbagResimMessageFrequencySpin_{nullptr};
    QPushButton* rosbagResimToggleButton_{nullptr};
    QProgressBar* rosbagResimProgressBar_{nullptr};
    QLabel* rosbagResimStatusValue_{nullptr};
    QLabel* rosbagResimWirelessTopicsValue_{nullptr};
    QPlainTextEdit* rosbagToolsLogView_{nullptr};

    bool suppressStationCameraFpsControls_{false};

};

}  // namespace airsim_gui
