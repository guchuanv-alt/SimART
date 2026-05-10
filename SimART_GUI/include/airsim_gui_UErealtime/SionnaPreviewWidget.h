#pragma once

#include <QProcess>
#include <QString>
#include <QWidget>

class QLabel;
class QTimer;
class QVBoxLayout;
class QWindow;

namespace airsim_gui {

class SionnaPreviewWidget : public QWidget {
    Q_OBJECT
public:
    explicit SionnaPreviewWidget(QWidget* parent = nullptr);
    ~SionnaPreviewWidget() override;

    void setPythonExecutable(const QString& pythonExecutable);
    void setLauncherScriptPath(const QString& scriptPath);
    void setScenePath(const QString& scenePath);
    void setEnvmapPath(const QString& envmapPath);
    void setPreviewActive(bool active);

    void restartPreview();

signals:
    void statusMessage(const QString& text);

private slots:
    void onProcessStarted();
    void onProcessReadyReadStandardOutput();
    void onProcessReadyReadStandardError();
    void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void tryAttachEmbeddedWindow();

private:
    void startProcess();
    void stopProcess();
    void pauseProcess();
    void resumeProcess();
    void clearEmbeddedWindow();
    bool ensureLaunchable(QString* errorMessage = nullptr) const;
    QStringList buildArguments() const;
    void setPlaceholderText(const QString& text);
    WId findWindowIdByTitle() const;
    void restartIfNeeded();

    QLabel* placeholderLabel_{nullptr};
    QVBoxLayout* rootLayout_{nullptr};
    QProcess* previewProcess_{nullptr};
    QTimer* attachTimer_{nullptr};
    QWidget* windowContainer_{nullptr};
    QWindow* foreignWindow_{nullptr};

    QString pythonExecutable_{QStringLiteral("python3")};
    QString launcherScriptPath_;
    QString scenePath_;
    QString envmapPath_;
    QString windowTitle_;

    bool previewActive_{false};
    bool processPaused_{false};
    bool pendingRestart_{false};
    bool attachInProgress_{false};
};

}  // namespace airsim_gui
