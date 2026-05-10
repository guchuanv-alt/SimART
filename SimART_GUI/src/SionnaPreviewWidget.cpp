#include "airsim_gui_UErealtime/SionnaPreviewWidget.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QLabel>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QTimer>
#include <QVBoxLayout>
#include <QWindow>

#ifdef Q_OS_LINUX
#include <signal.h>
#endif

namespace airsim_gui {
namespace {
QString trimmedOrDefault(const QString& value, const QString& fallback) {
    const QString trimmed = value.trimmed();
    return trimmed.isEmpty() ? fallback : trimmed;
}
}

SionnaPreviewWidget::SionnaPreviewWidget(QWidget* parent)
    : QWidget(parent) {
    rootLayout_ = new QVBoxLayout(this);
    rootLayout_->setContentsMargins(0, 0, 0, 0);
    rootLayout_->setSpacing(0);

    placeholderLabel_ = new QLabel(this);
    placeholderLabel_->setAlignment(Qt::AlignCenter);
    placeholderLabel_->setWordWrap(true);
    placeholderLabel_->setMinimumSize(320, 220);
    placeholderLabel_->setStyleSheet(QStringLiteral(
        "QLabel {"
        "background: #0f172a;"
        "color: #cbd5e1;"
        "border: 1px solid #1e293b;"
        "font-size: 14px;"
        "padding: 18px;"
        "}"
    ));
    rootLayout_->addWidget(placeholderLabel_, 1);
    setPlaceholderText(tr("Sionna Preview\n\nSelect a Scene XML in Simulation Settings, then open the preview window from Developer Tools."));

    previewProcess_ = new QProcess(this);
    previewProcess_->setProcessChannelMode(QProcess::SeparateChannels);
    connect(previewProcess_, &QProcess::started, this, &SionnaPreviewWidget::onProcessStarted);
    connect(previewProcess_, &QProcess::readyReadStandardOutput, this, &SionnaPreviewWidget::onProcessReadyReadStandardOutput);
    connect(previewProcess_, &QProcess::readyReadStandardError, this, &SionnaPreviewWidget::onProcessReadyReadStandardError);
    connect(previewProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &SionnaPreviewWidget::onProcessFinished);

    attachTimer_ = new QTimer(this);
    attachTimer_->setInterval(250);
    connect(attachTimer_, &QTimer::timeout, this, &SionnaPreviewWidget::tryAttachEmbeddedWindow);
}

SionnaPreviewWidget::~SionnaPreviewWidget() {
    stopProcess();
}

void SionnaPreviewWidget::setPythonExecutable(const QString& pythonExecutable) {
    const QString sanitized = trimmedOrDefault(pythonExecutable, QStringLiteral("python3"));
    if (pythonExecutable_ == sanitized) {
        return;
    }
    pythonExecutable_ = sanitized;
    pendingRestart_ = true;
    restartIfNeeded();
}

void SionnaPreviewWidget::setLauncherScriptPath(const QString& scriptPath) {
    const QString normalized = QFileInfo(scriptPath).absoluteFilePath();
    if (launcherScriptPath_ == normalized) {
        return;
    }
    launcherScriptPath_ = normalized;
    pendingRestart_ = true;
    restartIfNeeded();
}

void SionnaPreviewWidget::setScenePath(const QString& scenePath) {
    const QString normalized = scenePath.trimmed().isEmpty() ? QString() : QFileInfo(scenePath).absoluteFilePath();
    if (scenePath_ == normalized) {
        return;
    }
    scenePath_ = normalized;
    pendingRestart_ = true;
    restartIfNeeded();
}

void SionnaPreviewWidget::setEnvmapPath(const QString& envmapPath) {
    const QString normalized = envmapPath.trimmed().isEmpty() ? QString() : QFileInfo(envmapPath).absoluteFilePath();
    if (envmapPath_ == normalized) {
        return;
    }
    envmapPath_ = normalized;
    pendingRestart_ = true;
    restartIfNeeded();
}

void SionnaPreviewWidget::setPreviewActive(bool active) {
    if (previewActive_ == active) {
        if (active && pendingRestart_) {
            startProcess();
        }
        return;
    }
    previewActive_ = active;
    if (!previewActive_) {
        pauseProcess();
        return;
    }

    if (pendingRestart_ || previewProcess_->state() == QProcess::NotRunning) {
        startProcess();
    } else {
        resumeProcess();
    }
}

void SionnaPreviewWidget::restartPreview() {
    pendingRestart_ = true;
    restartIfNeeded();
}

void SionnaPreviewWidget::onProcessStarted() {
    processPaused_ = false;
    attachInProgress_ = true;
    attachTimer_->start();
    setPlaceholderText(tr("Launching embedded Sionna RT preview..."));
    emit statusMessage(tr("Launching embedded original sionna-rt-gui..."));
}

void SionnaPreviewWidget::onProcessReadyReadStandardOutput() {
    const QString text = QString::fromLocal8Bit(previewProcess_->readAllStandardOutput()).trimmed();
    if (!text.isEmpty()) {
        const QStringList lines = text.split(QRegularExpression(QStringLiteral("[\r\n]+")), QString::SkipEmptyParts);
        for (const QString& line : lines) {
            emit statusMessage(line);
        }
    }
}

void SionnaPreviewWidget::onProcessReadyReadStandardError() {
    const QString text = QString::fromLocal8Bit(previewProcess_->readAllStandardError()).trimmed();
    if (!text.isEmpty()) {
        const QStringList lines = text.split(QRegularExpression(QStringLiteral("[\r\n]+")), QString::SkipEmptyParts);
        for (const QString& line : lines) {
            emit statusMessage(line);
        }
    }
}

void SionnaPreviewWidget::onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    attachTimer_->stop();
    attachInProgress_ = false;
    clearEmbeddedWindow();
    processPaused_ = false;

    const QString reason = (exitStatus == QProcess::NormalExit)
        ? tr("Embedded Sionna preview exited (code %1)." ).arg(exitCode)
        : tr("Embedded Sionna preview crashed.");
    setPlaceholderText(reason);
    emit statusMessage(reason);

    if (previewActive_ && pendingRestart_) {
        QTimer::singleShot(0, this, [this]() { startProcess(); });
    }
}

void SionnaPreviewWidget::tryAttachEmbeddedWindow() {
    if (!attachInProgress_ || previewProcess_->state() == QProcess::NotRunning) {
        attachTimer_->stop();
        attachInProgress_ = false;
        return;
    }
    if (windowContainer_ != nullptr) {
        attachTimer_->stop();
        attachInProgress_ = false;
        return;
    }

    const WId wid = findWindowIdByTitle();
    if (wid == 0) {
        return;
    }

    foreignWindow_ = QWindow::fromWinId(wid);
    if (foreignWindow_ == nullptr) {
        emit statusMessage(tr("Found the Sionna RT window but failed to embed it."));
        return;
    }

    windowContainer_ = QWidget::createWindowContainer(foreignWindow_, this);
    windowContainer_->setFocusPolicy(Qt::StrongFocus);
    windowContainer_->setMinimumSize(320, 220);
    rootLayout_->addWidget(windowContainer_, 1);
    windowContainer_->show();
    placeholderLabel_->hide();
    attachTimer_->stop();
    attachInProgress_ = false;
    pendingRestart_ = false;
    emit statusMessage(tr("Embedded original sionna-rt-gui preview is ready."));
}

void SionnaPreviewWidget::startProcess() {
    pendingRestart_ = false;
    stopProcess();

    QString errorMessage;
    if (!ensureLaunchable(&errorMessage)) {
        setPlaceholderText(errorMessage);
        emit statusMessage(errorMessage);
        return;
    }
    if (!previewActive_) {
        return;
    }

    windowTitle_ = QStringLiteral("Sionna RT Embedded [%1]").arg(QString::number(QDateTime::currentMSecsSinceEpoch()));
    pendingRestart_ = false;
    setPlaceholderText(tr("Launching embedded Sionna RT preview..."));
    previewProcess_->start(pythonExecutable_, buildArguments());
    if (!previewProcess_->waitForStarted(3000)) {
        const QString failure = tr("Failed to start embedded Sionna preview: %1").arg(previewProcess_->errorString());
        setPlaceholderText(failure);
        emit statusMessage(failure);
        stopProcess();
    }
}

void SionnaPreviewWidget::stopProcess() {
    attachTimer_->stop();
    attachInProgress_ = false;
    clearEmbeddedWindow();

    if (previewProcess_->state() != QProcess::NotRunning) {
        previewProcess_->terminate();
        if (!previewProcess_->waitForFinished(1500)) {
            previewProcess_->kill();
            previewProcess_->waitForFinished(1500);
        }
    }
    processPaused_ = false;
}

void SionnaPreviewWidget::pauseProcess() {
#ifdef Q_OS_LINUX
    if (previewProcess_->state() == QProcess::Running && !processPaused_) {
        ::kill(static_cast<pid_t>(previewProcess_->processId()), SIGSTOP);
        processPaused_ = true;
    }
#else
    Q_UNUSED(this)
#endif
}

void SionnaPreviewWidget::resumeProcess() {
#ifdef Q_OS_LINUX
    if (previewProcess_->state() == QProcess::Running && processPaused_) {
        ::kill(static_cast<pid_t>(previewProcess_->processId()), SIGCONT);
        processPaused_ = false;
    }
#else
    Q_UNUSED(this)
#endif
}

void SionnaPreviewWidget::clearEmbeddedWindow() {
    if (windowContainer_ != nullptr) {
        rootLayout_->removeWidget(windowContainer_);
        delete windowContainer_;
        windowContainer_ = nullptr;
    }
    if (foreignWindow_ != nullptr) {
        delete foreignWindow_;
        foreignWindow_ = nullptr;
    }
    placeholderLabel_->show();
}

bool SionnaPreviewWidget::ensureLaunchable(QString* errorMessage) const {
    if (scenePath_.trimmed().isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = tr("No Sionna Scene XML is selected. Use Simulation Settings -> Scene XML first.");
        }
        return false;
    }
    if (!QFileInfo::exists(scenePath_)) {
        if (errorMessage != nullptr) {
            *errorMessage = tr("Scene XML not found: %1").arg(scenePath_);
        }
        return false;
    }
    if (launcherScriptPath_.trimmed().isEmpty() || !QFileInfo::exists(launcherScriptPath_)) {
        if (errorMessage != nullptr) {
            *errorMessage = tr("Embedded Sionna launcher script not found: %1").arg(launcherScriptPath_);
        }
        return false;
    }
    if (QStandardPaths::findExecutable(QStringLiteral("wmctrl")).isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = tr("wmctrl is required to embed the original sionna-rt-gui window on Linux.");
        }
        return false;
    }
    return true;
}

QStringList SionnaPreviewWidget::buildArguments() const {
    QStringList args;
    args << launcherScriptPath_
         << QStringLiteral("--scene") << scenePath_
         << QStringLiteral("--title") << windowTitle_;
    if (!envmapPath_.trimmed().isEmpty() && QFileInfo::exists(envmapPath_)) {
        args << QStringLiteral("--envmap") << envmapPath_;
    }
    return args;
}

void SionnaPreviewWidget::setPlaceholderText(const QString& text) {
    placeholderLabel_->setText(text);
}

WId SionnaPreviewWidget::findWindowIdByTitle() const {
    QProcess probe;
    probe.start(QStringLiteral("wmctrl"), {QStringLiteral("-lp")});
    if (!probe.waitForFinished(1000)) {
        return 0;
    }
    const QString output = QString::fromLocal8Bit(probe.readAllStandardOutput());
    const QStringList lines = output.split(QRegularExpression(QStringLiteral("[\r\n]+")), QString::SkipEmptyParts);
    for (const QString& line : lines) {
        if (!line.contains(windowTitle_)) {
            continue;
        }
        const QStringList parts = line.simplified().split(QLatin1Char(' '));
        if (parts.isEmpty()) {
            continue;
        }
        bool ok = false;
        const qulonglong value = parts.front().toULongLong(&ok, 16);
        if (ok) {
            return static_cast<WId>(value);
        }
    }
    return 0;
}

void SionnaPreviewWidget::restartIfNeeded() {
    if (!previewActive_) {
        setPlaceholderText(scenePath_.trimmed().isEmpty()
            ? tr("Sionna Preview\n\nSelect a Scene XML in Simulation Settings, then open the preview window from Developer Tools.")
            : tr("Sionna preview settings updated. Open the preview window from Developer Tools to launch the embedded original GUI."));
        return;
    }
    startProcess();
}

}  // namespace airsim_gui
