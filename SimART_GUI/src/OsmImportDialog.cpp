#include "airsim_gui_UErealtime/OsmImportDialog.h"

#include "airsim_gui_UErealtime/OsmMapWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMap>
#include <QMessageBox>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QProcess>
#include <QPushButton>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QUrlQuery>
#include <QVBoxLayout>

#include <algorithm>
#include <set>

namespace airsim_gui {
namespace {

struct JavaRuntimeOption {
    QString name;
    QString version;
    QString path;
    QString kind;
    int majorVersion{0};
    bool recommended{false};
};

QString appUserAgent() {
    return QStringLiteral("AirSimGUI-UERealtime/OSMImport (+local desktop app)");
}

QVariantMap resultMapFromObject(const QJsonObject& obj) {
    QVariantMap map = obj.toVariantMap();
    if (obj.contains(QStringLiteral("boundingbox")) && obj.value(QStringLiteral("boundingbox")).isArray()) {
        map.insert(QStringLiteral("boundingbox"), obj.value(QStringLiteral("boundingbox")).toArray().toVariantList());
    }
    return map;
}

QString defaultExportDirectory() {
    QString dir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    if (dir.isEmpty()) {
        dir = QStandardPaths::writableLocation(QStandardPaths::DownloadLocation);
    }
    if (dir.isEmpty()) {
        dir = QDir::homePath();
    }
    return QDir(dir).absoluteFilePath(QStringLiteral("airsim_gui_osm2world_exports"));
}

QString runAndCaptureStdOut(const QString& program, const QStringList& args, int timeoutMs = 2500) {
    QProcess proc;
    proc.start(program, args);
    if (!proc.waitForStarted(timeoutMs)) {
        return QString();
    }
    proc.waitForFinished(timeoutMs);
    return QString::fromLocal8Bit(proc.readAllStandardOutput()).trimmed();
}

QString runAndCaptureMerged(const QString& program, const QStringList& args, int timeoutMs = 3000) {
    QProcess proc;
    proc.start(program, args);
    if (!proc.waitForStarted(timeoutMs)) {
        return QString();
    }
    proc.waitForFinished(timeoutMs);
    const QString out = QString::fromLocal8Bit(proc.readAllStandardOutput()).trimmed();
    const QString err = QString::fromLocal8Bit(proc.readAllStandardError()).trimmed();
    if (out.isEmpty()) return err;
    if (err.isEmpty()) return out;
    return (out + QStringLiteral("\n") + err).trimmed();
}

bool isExecutableFile(const QString& path) {
    const QFileInfo fi(path);
    return fi.exists() && fi.isFile() && fi.isExecutable();
}

QString parseJavaVersion(const QString& rawText) {
    QRegularExpression quotedVersion(QStringLiteral("version\\s+\"([^\"]+)\""));
    QRegularExpressionMatch quotedMatch = quotedVersion.match(rawText);
    if (quotedMatch.hasMatch()) {
        return quotedMatch.captured(1).trimmed();
    }

    QRegularExpression bareVersion(QStringLiteral("(?:openjdk|java)\\s+([0-9][0-9A-Za-z._+-]*)"));
    QRegularExpressionMatch bareMatch = bareVersion.match(rawText);
    if (bareMatch.hasMatch()) {
        return bareMatch.captured(1).trimmed();
    }

    QRegularExpression fallback(QStringLiteral("([0-9]+(?:\\.[0-9A-Za-z_+-]+)+)"));
    QRegularExpressionMatch fallbackMatch = fallback.match(rawText);
    if (fallbackMatch.hasMatch()) {
        return fallbackMatch.captured(1).trimmed();
    }
    return QString();
}

int javaMajorVersion(const QString& version) {
    if (version.isEmpty()) {
        return 0;
    }
    if (version.startsWith(QStringLiteral("1."))) {
        const QStringList parts = version.split('.');
        if (parts.size() >= 2) {
            return parts.at(1).toInt();
        }
    }
    QRegularExpression leadingNumber(QStringLiteral("^([0-9]+)"));
    QRegularExpressionMatch match = leadingNumber.match(version);
    return match.hasMatch() ? match.captured(1).toInt() : 0;
}

QString detectJavaVersion(const QString& path) {
    if (!isExecutableFile(path)) {
        return QString();
    }
    return parseJavaVersion(runAndCaptureMerged(path, {QStringLiteral("-version")}, 3000));
}

QString detectJavaKind(const QString& path) {
    const QString lower = QFileInfo(path).absoluteFilePath().toLower();
    if (lower.contains(QStringLiteral("/.sdkman/"))) {
        return QStringLiteral("SDKMAN");
    }
    if (lower.contains(QStringLiteral("/usr/lib/jvm/")) || lower.contains(QStringLiteral("/etc/alternatives/"))) {
        return QStringLiteral("System");
    }
    if (lower.contains(QStringLiteral("/envs/")) || lower.contains(QStringLiteral("miniconda")) ||
        lower.contains(QStringLiteral("anaconda")) || lower.contains(QStringLiteral("conda")) ||
        lower.contains(QStringLiteral("mambaforge")) || lower.contains(QStringLiteral("micromamba"))) {
        return QStringLiteral("Conda");
    }
    return QStringLiteral("Global");
}

QString javaDisplayName(const QString& path, const QString& kind) {
    const QString normalized = QFileInfo(path).absoluteFilePath();
    if (kind == QStringLiteral("System") || kind == QStringLiteral("SDKMAN")) {
        const QDir binDir = QFileInfo(normalized).dir();
        const QString runtimeDir = QFileInfo(binDir.absolutePath() + QStringLiteral("/..")).fileName();
        if (!runtimeDir.isEmpty()) {
            return runtimeDir;
        }
    }
    if (kind == QStringLiteral("Conda")) {
        const int envsIndex = normalized.indexOf(QStringLiteral("/envs/"));
        if (envsIndex >= 0) {
            const QString tail = normalized.mid(envsIndex + 6);
            const int slash = tail.indexOf('/');
            if (slash > 0) return tail.left(slash);
        }
    }
    return QFileInfo(normalized).fileName();
}

JavaRuntimeOption makeJavaRuntimeOption(const QString& path) {
    JavaRuntimeOption option;
    option.path = QFileInfo(path).absoluteFilePath();
    option.kind = detectJavaKind(option.path);
    option.version = detectJavaVersion(option.path);
    option.majorVersion = javaMajorVersion(option.version);
    option.recommended = option.majorVersion >= 17;
    option.name = javaDisplayName(option.path, option.kind);
    return option;
}

QList<JavaRuntimeOption> discoverJavaRuntimes() {
    std::set<QString> unique;
    QList<JavaRuntimeOption> results;

    auto addCandidate = [&](const QString& candidate) {
        const QString normalized = QFileInfo(candidate).absoluteFilePath();
        if (!isExecutableFile(normalized) || unique.count(normalized)) {
            return;
        }
        unique.insert(normalized);
        results.push_back(makeJavaRuntimeOption(normalized));
    };

    const QStringList directCandidates = {
        QStringLiteral("/usr/bin/java"),
        QStringLiteral("/usr/local/bin/java"),
        QStringLiteral("/bin/java"),
        QStringLiteral("/usr/lib/jvm/default-java/bin/java"),
        QStringLiteral("/usr/lib/jvm/java-17-openjdk-amd64/bin/java"),
        QStringLiteral("/usr/lib/jvm/java-21-openjdk-amd64/bin/java"),
        QDir::home().filePath(QStringLiteral(".sdkman/candidates/java/current/bin/java")),
        QDir::home().filePath(QStringLiteral("miniconda3/bin/java")),
        QDir::home().filePath(QStringLiteral("anaconda3/bin/java")),
        QDir::home().filePath(QStringLiteral("mambaforge/bin/java")),
        QDir::home().filePath(QStringLiteral("micromamba/bin/java"))
    };
    for (const QString& path : directCandidates) {
        addCandidate(path);
    }

    for (const QString& root : {QStringLiteral("/usr/lib/jvm"), QDir::home().filePath(QStringLiteral(".sdkman/candidates/java"))}) {
        QDir dir(root);
        if (!dir.exists()) continue;
        for (const QFileInfo& fi : dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot)) {
            addCandidate(fi.absoluteFilePath() + QStringLiteral("/bin/java"));
        }
    }

    const QString pathProbe = runAndCaptureStdOut(
        QStringLiteral("bash"),
        {QStringLiteral("-lc"), QStringLiteral("which -a java 2>/dev/null | awk '!seen[$0]++'")},
        2000);
    for (const QString& line : pathProbe.split(QStringLiteral("\n"), QString::SkipEmptyParts)) {
        addCandidate(line.trimmed());
    }

    const QString alternativesProbe = runAndCaptureStdOut(
        QStringLiteral("bash"),
        {QStringLiteral("-lc"), QStringLiteral("update-alternatives --list java 2>/dev/null | awk '!seen[$0]++'")},
        2000);
    for (const QString& line : alternativesProbe.split(QStringLiteral("\n"), QString::SkipEmptyParts)) {
        addCandidate(line.trimmed());
    }

    std::sort(results.begin(), results.end(), [](const JavaRuntimeOption& a, const JavaRuntimeOption& b) {
        if (a.recommended != b.recommended) return a.recommended > b.recommended;
        if (a.majorVersion != b.majorVersion) return a.majorVersion > b.majorVersion;
        const auto kindRank = [](const QString& kind) {
            if (kind == QStringLiteral("System")) return 0;
            if (kind == QStringLiteral("SDKMAN")) return 1;
            if (kind == QStringLiteral("Conda")) return 2;
            return 3;
        };
        if (kindRank(a.kind) != kindRank(b.kind)) return kindRank(a.kind) < kindRank(b.kind);
        if (a.name.toLower() != b.name.toLower()) return a.name.toLower() < b.name.toLower();
        return a.path.toLower() < b.path.toLower();
    });

    return results;
}

QString javaSummary(const JavaRuntimeOption& option) {
    const QString version = option.version.isEmpty() ? QStringLiteral("Java ?") : QStringLiteral("Java %1").arg(option.version);
    return QStringLiteral("%1  %2").arg(version, option.path);
}

bool filterJavaTree(QTreeWidget* tree, const QString& keyword) {
    const QString needle = keyword.trimmed().toLower();
    bool anyVisible = false;
    for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        auto* group = tree->topLevelItem(i);
        bool groupVisible = false;
        for (int j = 0; j < group->childCount(); ++j) {
            auto* child = group->child(j);
            const QString hay = (child->text(0) + QStringLiteral(" ") + child->text(1) + QStringLiteral(" ") + child->text(2)).toLower();
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

QString selectJavaRuntime(QWidget* parent, const QString& currentPath) {
    QDialog dialog(parent);
    dialog.setWindowTitle(QObject::tr("Select Java Runtime"));
    dialog.resize(860, 520);

    auto* layout = new QVBoxLayout(&dialog);
    auto* search = new QLineEdit(&dialog);
    search->setPlaceholderText(QObject::tr("Search runtimes by version, source, or path"));
    layout->addWidget(search);

    auto* tree = new QTreeWidget(&dialog);
    tree->setColumnCount(3);
    tree->setHeaderLabels({QObject::tr("Runtime"), QObject::tr("Executable"), QObject::tr("Source")});
    tree->header()->setStretchLastSection(false);
    tree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    tree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    tree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    tree->setRootIsDecorated(true);
    layout->addWidget(tree, 1);

    const QList<JavaRuntimeOption> runtimes = discoverJavaRuntimes();
    QMap<QString, QTreeWidgetItem*> groups;
    for (const QString& groupName : {QObject::tr("Recommended (Java 17+)"), QObject::tr("Other Java Runtimes")}) {
        auto* item = new QTreeWidgetItem(tree, QStringList{groupName});
        item->setFirstColumnSpanned(true);
        item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
        groups.insert(groupName, item);
    }

    auto sourceLabel = [](const QString& kind) {
        if (kind == QStringLiteral("System")) return QObject::tr("System");
        if (kind == QStringLiteral("SDKMAN")) return QObject::tr("SDKMAN");
        if (kind == QStringLiteral("Conda")) return QObject::tr("Conda");
        return QObject::tr("Global");
    };

    QTreeWidgetItem* selected = nullptr;
    for (const auto& runtime : runtimes) {
        auto* parentItem = groups.value(runtime.recommended ? QObject::tr("Recommended (Java 17+)") : QObject::tr("Other Java Runtimes"));
        const QString runtimeLabel = runtime.version.isEmpty()
                                         ? runtime.name
                                         : QStringLiteral("%1 (%2)").arg(runtime.name, runtime.version);
        auto* child = new QTreeWidgetItem(parentItem, QStringList{runtimeLabel, runtime.path, sourceLabel(runtime.kind)});
        child->setData(0, Qt::UserRole, runtime.path);
        if (QFileInfo(currentPath).absoluteFilePath() == QFileInfo(runtime.path).absoluteFilePath()) {
            selected = child;
        }
    }
    tree->expandAll();
    if (selected) tree->setCurrentItem(selected);

    auto* helperRow = new QHBoxLayout();
    auto* browse = new QPushButton(QObject::tr("Browse..."), &dialog);
    auto* note = new QLabel(
        QObject::tr("Tip: OSM2World works best with Java 17 or newer. Use this list to pick a detected runtime, or browse to a custom java executable."),
        &dialog);
    note->setWordWrap(true);
    helperRow->addWidget(browse);
    helperRow->addWidget(note, 1);
    layout->addLayout(helperRow);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    layout->addWidget(buttons);

    QObject::connect(search, &QLineEdit::textChanged, &dialog, [tree](const QString& text) {
        filterJavaTree(tree, text);
    });
    QObject::connect(browse, &QPushButton::clicked, &dialog, [&dialog, currentPath]() {
        const QString path = QFileDialog::getOpenFileName(
            &dialog,
            QObject::tr("Select Java executable"),
            currentPath.trimmed().isEmpty() ? QDir::homePath() : QFileInfo(currentPath).absolutePath(),
            QObject::tr("Executables (*)"));
        if (path.isEmpty()) return;
        if (!isExecutableFile(path)) {
            QMessageBox::warning(&dialog, QObject::tr("Java"), QObject::tr("The selected file is not executable."));
            return;
        }
        const QString version = detectJavaVersion(path);
        if (version.isEmpty()) {
            QMessageBox::warning(&dialog, QObject::tr("Java"), QObject::tr("The selected file does not appear to be a Java runtime."));
            return;
        }
        dialog.setProperty("selected_java_path", QFileInfo(path).absoluteFilePath());
        dialog.accept();
    });
    QObject::connect(tree, &QTreeWidget::itemDoubleClicked, &dialog, [&dialog](QTreeWidgetItem* item, int) {
        if (!item || item->childCount() > 0) return;
        dialog.setProperty("selected_java_path", item->data(0, Qt::UserRole).toString());
        dialog.accept();
    });
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&dialog, tree]() {
        auto* item = tree->currentItem();
        if (!item || item->childCount() > 0) {
            QMessageBox::information(&dialog, QObject::tr("Java"), QObject::tr("Please select one Java runtime."));
            return;
        }
        dialog.setProperty("selected_java_path", item->data(0, Qt::UserRole).toString());
        dialog.accept();
    });
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted) {
        return QString();
    }
    return dialog.property("selected_java_path").toString();
}

}  // namespace

OsmImportDialog::OsmImportDialog(QWidget* parent)
    : QDialog(parent),
      network_(new QNetworkAccessManager(this)) {
    setWindowTitle(tr("Search and Import OSM Area"));
    resize(1180, 820);

    auto* rootLayout = new QVBoxLayout(this);
    auto* topRow = new QHBoxLayout();
    searchEdit_ = new QLineEdit(this);
    searchEdit_->setPlaceholderText(tr("Search a place, address, district, or city..."));
    searchButton_ = new QPushButton(tr("Search"), this);
    selectionModeButton_ = new QPushButton(tr("Select Area"), this);
    selectionModeButton_->setCheckable(true);
    topRow->addWidget(searchEdit_, 1);
    topRow->addWidget(searchButton_);
    topRow->addWidget(selectionModeButton_);
    rootLayout->addLayout(topRow);

    auto* bodyLayout = new QHBoxLayout();
    mapWidget_ = new OsmMapWidget(this);
    bodyLayout->addWidget(mapWidget_, 1);

    auto* sidePanel = new QWidget(this);
    sidePanel->setMinimumWidth(360);
    auto* sideLayout = new QVBoxLayout(sidePanel);
    sideLayout->addWidget(new QLabel(tr("Search Results"), sidePanel));
    searchResultsList_ = new QListWidget(sidePanel);
    sideLayout->addWidget(searchResultsList_, 1);

    includeTreesCheck_ = new QCheckBox(tr("Trees"), sidePanel);
    includeStreetFurnitureCheck_ = new QCheckBox(tr("Street lamps / street furniture"), sidePanel);
    includeGreenAreasCheck_ = new QCheckBox(tr("Green areas"), sidePanel);
    includeWaterCheck_ = new QCheckBox(tr("Water areas"), sidePanel);
    includeTreesCheck_->setChecked(true);
    includeStreetFurnitureCheck_->setChecked(true);
    includeGreenAreasCheck_->setChecked(true);
    includeWaterCheck_->setChecked(true);

    auto* detailBox = new QGroupBox(tr("Import Content"), sidePanel);
    auto* detailLayout = new QVBoxLayout(detailBox);
    detailLayout->addWidget(includeTreesCheck_);
    detailLayout->addWidget(includeStreetFurnitureCheck_);
    detailLayout->addWidget(includeGreenAreasCheck_);
    detailLayout->addWidget(includeWaterCheck_);
    sideLayout->addWidget(detailBox);

    const QList<JavaRuntimeOption> detectedJavaRuntimes = discoverJavaRuntimes();
    const QString detectedJavaPath = detectedJavaRuntimes.isEmpty() ? QStringLiteral("java") : detectedJavaRuntimes.first().path;

    useOsm2WorldCheck_ = new QCheckBox(tr("Use OSM2World to generate a higher-quality mesh"), sidePanel);
    useOsm2WorldCheck_->setChecked(false);
    osm2worldExecutableEdit_ = new QLineEdit(sidePanel);
    osm2worldExecutableEdit_->setPlaceholderText(tr("Path to osm2world executable or OSM2World .jar (leave blank to use PATH)"));
    auto* browseOsm2WorldButton = new QPushButton(tr("Browse..."), sidePanel);
    javaExecutableEdit_ = new QLineEdit(detectedJavaPath, sidePanel);
    javaExecutableEdit_->setPlaceholderText(tr("Java executable for running an OSM2World .jar"));
    if (!detectedJavaRuntimes.isEmpty()) {
        javaExecutableEdit_->setToolTip(tr("Auto-detected runtime: %1").arg(javaSummary(detectedJavaRuntimes.first())));
    }
    selectJavaButton_ = new QPushButton(tr("Select..."), sidePanel);
    auto* browseJavaButton = new QPushButton(tr("Browse..."), sidePanel);
    osm2worldFormatCombo_ = new QComboBox(sidePanel);
    osm2worldFormatCombo_->addItem(QStringLiteral("glb"), QStringLiteral("glb"));
    osm2worldFormatCombo_->addItem(QStringLiteral("gltf"), QStringLiteral("gltf"));
    osm2worldFormatCombo_->addItem(QStringLiteral("obj"), QStringLiteral("obj"));
    osm2worldOutputDirEdit_ = new QLineEdit(defaultExportDirectory(), sidePanel);
    osm2worldOutputDirEdit_->setPlaceholderText(tr("Directory where generated meshes and optional OSM copies are written"));
    auto* browseOutputDirButton = new QPushButton(tr("Browse..."), sidePanel);
    osm2worldExtraArgsEdit_ = new QLineEdit(sidePanel);
    osm2worldExtraArgsEdit_->setPlaceholderText(tr("Optional extra OSM2World CLI arguments"));
    keepDownloadedOsmCheck_ = new QCheckBox(tr("Copy downloaded .osm into the export directory"), sidePanel);
    keepDownloadedOsmCheck_->setChecked(true);

    auto* osm2worldBox = new QGroupBox(tr("OSM2World Conversion"), sidePanel);
    auto* osm2worldLayout = new QVBoxLayout(osm2worldBox);
    osm2worldLayout->addWidget(useOsm2WorldCheck_);

    auto* exeRow = new QHBoxLayout();
    exeRow->addWidget(osm2worldExecutableEdit_, 1);
    exeRow->addWidget(browseOsm2WorldButton);
    osm2worldLayout->addLayout(exeRow);

    auto* javaRow = new QHBoxLayout();
    javaRow->addWidget(javaExecutableEdit_, 1);
    javaRow->addWidget(selectJavaButton_);
    javaRow->addWidget(browseJavaButton);
    osm2worldLayout->addLayout(javaRow);

    auto* javaTip = new QLabel(tr("Click Select... to scan installed Java runtimes and choose one. Java 17 or newer is recommended for OSM2World."), osm2worldBox);
    javaTip->setWordWrap(true);
    osm2worldLayout->addWidget(javaTip);

    auto* formatRow = new QHBoxLayout();
    formatRow->addWidget(new QLabel(tr("Output format"), osm2worldBox));
    formatRow->addWidget(osm2worldFormatCombo_, 1);
    osm2worldLayout->addLayout(formatRow);

    auto* outputRow = new QHBoxLayout();
    outputRow->addWidget(osm2worldOutputDirEdit_, 1);
    outputRow->addWidget(browseOutputDirButton);
    osm2worldLayout->addLayout(outputRow);
    osm2worldLayout->addWidget(osm2worldExtraArgsEdit_);
    osm2worldLayout->addWidget(keepDownloadedOsmCheck_);
    sideLayout->addWidget(osm2worldBox);

    selectionLabel_ = new QLabel(tr("No selected bounding box."), sidePanel);
    selectionLabel_->setWordWrap(true);
    statusLabel_ = new QLabel(tr("Search a place, then click Select Area and drag on the map."), sidePanel);
    statusLabel_->setWordWrap(true);
    sideLayout->addWidget(new QLabel(tr("Selection"), sidePanel));
    sideLayout->addWidget(selectionLabel_);
    sideLayout->addWidget(new QLabel(tr("Map Status"), sidePanel));
    sideLayout->addWidget(statusLabel_);
    bodyLayout->addWidget(sidePanel);

    rootLayout->addLayout(bodyLayout, 1);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Cancel, this);
    auto* importButton = buttons->addButton(tr("Download && Import"), QDialogButtonBox::AcceptRole);
    connect(importButton, &QPushButton::clicked, this, &OsmImportDialog::acceptSelection);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    rootLayout->addWidget(buttons);

    connect(searchButton_, &QPushButton::clicked, this, &OsmImportDialog::performSearch);
    connect(searchEdit_, &QLineEdit::returnPressed, this, &OsmImportDialog::performSearch);
    connect(searchResultsList_, &QListWidget::itemActivated, this, &OsmImportDialog::onResultActivated);
    connect(searchResultsList_, &QListWidget::itemClicked, this, &OsmImportDialog::onResultActivated);
    connect(selectionModeButton_, &QPushButton::clicked, this, &OsmImportDialog::toggleSelectionMode);
    connect(mapWidget_, &OsmMapWidget::selectionChanged, this, &OsmImportDialog::onSelectionChanged);
    connect(mapWidget_, &OsmMapWidget::statusTextChanged, statusLabel_, &QLabel::setText);
    connect(useOsm2WorldCheck_, &QCheckBox::toggled, this, &OsmImportDialog::updateOsm2WorldWidgets);
    connect(browseOsm2WorldButton, &QPushButton::clicked, this, &OsmImportDialog::browseOsm2WorldExecutable);
    connect(browseOutputDirButton, &QPushButton::clicked, this, &OsmImportDialog::browseOsm2WorldOutputDirectory);
    connect(selectJavaButton_, &QPushButton::clicked, this, &OsmImportDialog::selectJavaExecutable);
    connect(browseJavaButton, &QPushButton::clicked, this, &OsmImportDialog::browseJavaExecutable);

    updateOsm2WorldWidgets();
}

void OsmImportDialog::performSearch() {
    const QString query = searchEdit_->text().trimmed();
    if (query.isEmpty()) {
        QMessageBox::information(this, tr("OSM Search"), tr("Please enter a place name or address to search."));
        return;
    }

    if (pendingSearchReply_) {
        pendingSearchReply_->abort();
        pendingSearchReply_->deleteLater();
        pendingSearchReply_ = nullptr;
    }

    searchResultsList_->clear();
    statusLabel_->setText(tr("Searching..."));

    QUrl url(QStringLiteral("https://nominatim.openstreetmap.org/search"));
    QUrlQuery urlQuery;
    urlQuery.addQueryItem(QStringLiteral("q"), query);
    urlQuery.addQueryItem(QStringLiteral("format"), QStringLiteral("jsonv2"));
    urlQuery.addQueryItem(QStringLiteral("limit"), QStringLiteral("8"));
    urlQuery.addQueryItem(QStringLiteral("addressdetails"), QStringLiteral("1"));
    url.setQuery(urlQuery);

    QNetworkRequest request(url);
    request.setRawHeader("User-Agent", appUserAgent().toUtf8());
    request.setRawHeader("Accept", "application/json");
    pendingSearchReply_ = network_->get(request);
    connect(pendingSearchReply_, &QNetworkReply::finished, this, &OsmImportDialog::onSearchFinished);
}

void OsmImportDialog::onSearchFinished() {
    QNetworkReply* reply = pendingSearchReply_;
    pendingSearchReply_ = nullptr;
    if (!reply) {
        return;
    }

    if (reply->error() != QNetworkReply::NoError) {
        QMessageBox::warning(this, tr("OSM Search"), tr("Search request failed: %1").arg(reply->errorString()));
        statusLabel_->setText(tr("Search failed."));
        reply->deleteLater();
        return;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(reply->readAll());
    if (!doc.isArray()) {
        QMessageBox::warning(this, tr("OSM Search"), tr("Unexpected search response."));
        statusLabel_->setText(tr("Unexpected search response."));
        reply->deleteLater();
        return;
    }

    const QJsonArray array = doc.array();
    for (const QJsonValue& value : array) {
        if (!value.isObject()) {
            continue;
        }
        const QVariantMap result = resultMapFromObject(value.toObject());
        auto* item = new QListWidgetItem(displayNameForResult(result), searchResultsList_);
        item->setData(Qt::UserRole, result);
    }

    if (searchResultsList_->count() > 0) {
        searchResultsList_->setCurrentRow(0);
        onResultActivated(searchResultsList_->item(0));
        statusLabel_->setText(tr("Search complete. Drag a rectangle to choose an import area."));
    } else {
        statusLabel_->setText(tr("No search results."));
    }
    reply->deleteLater();
}

void OsmImportDialog::onResultActivated(QListWidgetItem* item) {
    if (!item) {
        return;
    }
    applySearchResult(item->data(Qt::UserRole).toMap());
}

void OsmImportDialog::onSelectionChanged(double southLat, double westLon, double northLat, double eastLon) {
    selection_.southLat = southLat;
    selection_.westLon = westLon;
    selection_.northLat = northLat;
    selection_.eastLon = eastLon;
    updateSelectionLabel();
}

void OsmImportDialog::toggleSelectionMode() {
    const bool enabled = selectionModeButton_->isChecked();
    mapWidget_->setSelectionMode(enabled);
    selectionModeButton_->setText(enabled ? tr("Exit Select Area") : tr("Select Area"));
}

void OsmImportDialog::acceptSelection() {
    if (!selection_.isValid()) {
        QMessageBox::information(this, tr("OSM Import"), tr("Please select an area on the map before importing."));
        return;
    }

    selection_.includeTrees = includeTreesCheck_->isChecked();
    selection_.includeStreetFurniture = includeStreetFurnitureCheck_->isChecked();
    selection_.includeGreenAreas = includeGreenAreasCheck_->isChecked();
    selection_.includeWater = includeWaterCheck_->isChecked();
    selection_.useOsm2World = useOsm2WorldCheck_->isChecked();
    selection_.osm2worldExecutable = osm2worldExecutableEdit_->text().trimmed();
    selection_.osm2worldJavaExecutable = javaExecutableEdit_->text().trimmed().isEmpty()
                                             ? QStringLiteral("java")
                                             : javaExecutableEdit_->text().trimmed();
    selection_.osm2worldOutputFormat = osm2worldFormatCombo_->currentData().toString();
    if (selection_.osm2worldOutputFormat.isEmpty()) {
        selection_.osm2worldOutputFormat = osm2worldFormatCombo_->currentText().trimmed().toLower();
    }
    selection_.osm2worldOutputDirectory = osm2worldOutputDirEdit_->text().trimmed();
    selection_.osm2worldExtraArguments = osm2worldExtraArgsEdit_->text().trimmed();
    selection_.keepDownloadedOsmCopy = keepDownloadedOsmCheck_->isChecked();

    if (selection_.useOsm2World && selection_.osm2worldOutputDirectory.isEmpty()) {
        QMessageBox::information(this, tr("OSM2World"), tr("Please choose an output directory for generated meshes."));
        return;
    }

    accept();
}

void OsmImportDialog::browseOsm2WorldExecutable() {
    const QString path = QFileDialog::getOpenFileName(
        this,
        tr("Select OSM2World Executable or JAR"),
        osm2worldExecutableEdit_->text().trimmed(),
        tr("OSM2World executable or jar (*.jar);;All files (*)"));
    if (!path.isEmpty()) {
        osm2worldExecutableEdit_->setText(path);
    }
}

void OsmImportDialog::browseOsm2WorldOutputDirectory() {
    const QString dir = QFileDialog::getExistingDirectory(
        this,
        tr("Choose OSM2World Export Directory"),
        osm2worldOutputDirEdit_->text().trimmed().isEmpty() ? defaultExportDirectory() : osm2worldOutputDirEdit_->text().trimmed());
    if (!dir.isEmpty()) {
        osm2worldOutputDirEdit_->setText(dir);
    }
}

void OsmImportDialog::browseJavaExecutable() {
    const QString path = QFileDialog::getOpenFileName(
        this,
        tr("Select Java Executable"),
        javaExecutableEdit_->text().trimmed().isEmpty() ? QDir::homePath() : QFileInfo(javaExecutableEdit_->text().trimmed()).absolutePath(),
        tr("Executables (*)"));
    if (!path.isEmpty()) {
        javaExecutableEdit_->setText(path);
    }
}

void OsmImportDialog::selectJavaExecutable() {
    const QString selected = selectJavaRuntime(this, javaExecutableEdit_->text().trimmed());
    if (!selected.isEmpty()) {
        javaExecutableEdit_->setText(selected);
    }
}

void OsmImportDialog::updateOsm2WorldWidgets() {
    const bool enabled = useOsm2WorldCheck_->isChecked();
    osm2worldExecutableEdit_->setEnabled(enabled);
    javaExecutableEdit_->setEnabled(enabled);
    if (selectJavaButton_) {
        selectJavaButton_->setEnabled(enabled);
    }
    osm2worldFormatCombo_->setEnabled(enabled);
    osm2worldOutputDirEdit_->setEnabled(enabled);
    osm2worldExtraArgsEdit_->setEnabled(enabled);
    keepDownloadedOsmCheck_->setEnabled(enabled);
}

void OsmImportDialog::updateSelectionLabel() {
    if (!selection_.isValid()) {
        selectionLabel_->setText(tr("No selected bounding box."));
        return;
    }
    selectionLabel_->setText(
        tr("South: %1\nWest: %2\nNorth: %3\nEast: %4")
            .arg(selection_.southLat, 0, 'f', 6)
            .arg(selection_.westLon, 0, 'f', 6)
            .arg(selection_.northLat, 0, 'f', 6)
            .arg(selection_.eastLon, 0, 'f', 6));
}

void OsmImportDialog::applySearchResult(const QVariantMap& result) {
    if (result.isEmpty()) {
        return;
    }

    const double lat = result.value(QStringLiteral("lat")).toString().toDouble();
    const double lon = result.value(QStringLiteral("lon")).toString().toDouble();
    mapWidget_->centerOn(lat, lon);

    const QVariantList bbox = result.value(QStringLiteral("boundingbox")).toList();
    selection_.displayName = displayNameForResult(result);
    if (bbox.size() >= 4) {
        const double southLat = bbox.at(0).toString().toDouble();
        const double northLat = bbox.at(1).toString().toDouble();
        const double westLon = bbox.at(2).toString().toDouble();
        const double eastLon = bbox.at(3).toString().toDouble();
        mapWidget_->setPreviewBoundingBox(southLat, westLon, northLat, eastLon);
        statusLabel_->setText(tr("Centered on search result. Click Select Area to draw your own import region."));
    } else {
        mapWidget_->clearPreviewBoundingBox();
    }
}

QString OsmImportDialog::displayNameForResult(const QVariantMap& result) const {
    const QString name = result.value(QStringLiteral("display_name")).toString().trimmed();
    if (!name.isEmpty()) {
        return name;
    }
    return result.value(QStringLiteral("name")).toString();
}

}  // namespace airsim_gui
