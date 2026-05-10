#pragma once

#include <QDialog>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QString>
#include <QVariantMap>

class QLabel;
class QLineEdit;
class QListWidget;
class QListWidgetItem;
class QPushButton;
class QCheckBox;
class QComboBox;

namespace airsim_gui {

class OsmMapWidget;

struct OsmImportSelection {
    QString displayName;
    double southLat{0.0};
    double westLon{0.0};
    double northLat{0.0};
    double eastLon{0.0};
    bool includeTrees{true};
    bool includeStreetFurniture{true};
    bool includeGreenAreas{true};
    bool includeWater{true};
    bool useOsm2World{false};
    QString osm2worldExecutable;
    QString osm2worldJavaExecutable{"java"};
    QString osm2worldOutputFormat{"glb"};
    QString osm2worldOutputDirectory;
    QString osm2worldExtraArguments;
    bool keepDownloadedOsmCopy{true};

    bool isValid() const {
        return southLat < northLat && westLon < eastLon;
    }
};

class OsmImportDialog : public QDialog {
    Q_OBJECT
public:
    explicit OsmImportDialog(QWidget* parent = nullptr);

    OsmImportSelection selection() const { return selection_; }

private slots:
    void performSearch();
    void onSearchFinished();
    void onResultActivated(QListWidgetItem* item);
    void onSelectionChanged(double southLat, double westLon, double northLat, double eastLon);
    void toggleSelectionMode();
    void acceptSelection();
    void browseOsm2WorldExecutable();
    void browseOsm2WorldOutputDirectory();
    void browseJavaExecutable();
    void selectJavaExecutable();
    void updateOsm2WorldWidgets();

private:
    void updateSelectionLabel();
    void applySearchResult(const QVariantMap& result);
    QString displayNameForResult(const QVariantMap& result) const;

    QLineEdit* searchEdit_{nullptr};
    QPushButton* searchButton_{nullptr};
    QPushButton* selectionModeButton_{nullptr};
    QListWidget* searchResultsList_{nullptr};
    OsmMapWidget* mapWidget_{nullptr};
    QLabel* statusLabel_{nullptr};
    QLabel* selectionLabel_{nullptr};
    QCheckBox* includeTreesCheck_{nullptr};
    QCheckBox* includeStreetFurnitureCheck_{nullptr};
    QCheckBox* includeGreenAreasCheck_{nullptr};
    QCheckBox* includeWaterCheck_{nullptr};
    QCheckBox* useOsm2WorldCheck_{nullptr};
    QLineEdit* osm2worldExecutableEdit_{nullptr};
    QLineEdit* javaExecutableEdit_{nullptr};
    QPushButton* selectJavaButton_{nullptr};
    QComboBox* osm2worldFormatCombo_{nullptr};
    QLineEdit* osm2worldOutputDirEdit_{nullptr};
    QLineEdit* osm2worldExtraArgsEdit_{nullptr};
    QCheckBox* keepDownloadedOsmCheck_{nullptr};
    OsmImportSelection selection_;
    QNetworkAccessManager* network_{nullptr};
    QNetworkReply* pendingSearchReply_{nullptr};
};

}  // namespace airsim_gui
