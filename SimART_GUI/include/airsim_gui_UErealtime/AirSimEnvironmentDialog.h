#pragma once

#include <QDialog>

#include <array>
#include <atomic>
#include <thread>

class QCheckBox;
class QCloseEvent;
class QDateTimeEdit;
class QDialogButtonBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QPushButton;
class QSlider;

namespace airsim_gui {

class AirSimEnvironmentDialog : public QDialog {
    Q_OBJECT
public:
    explicit AirSimEnvironmentDialog(QWidget* parent = nullptr);
    ~AirSimEnvironmentDialog() override;

    void setEndpoint(const QString& host, int port);

signals:
    void environmentApplied(const QString& message);

protected:
    void closeEvent(QCloseEvent* event) override;
    void reject() override;

private slots:
    void applyEnvironment();
    void applyWeatherPreset(int index);
    void finishApply(bool success, const QString& message);

private:
    struct ApplyRequest {
        QString host;
        int port{41451};
        bool timeEnabled{true};
        QString dateTime;
        bool dst{false};
        float celestialClockSpeed{0.0f};
        float updateIntervalSecs{60.0f};
        bool moveSun{true};
        bool weatherEnabled{true};
        std::array<float, 8> weather{};
    };

    void buildUi();
    void loadState();
    void saveState() const;
    void setBusy(bool busy);
    void setWeatherValues(const std::array<double, 8>& values);
    ApplyRequest currentRequest() const;
    void runApply(ApplyRequest request);

    QString host_{QStringLiteral("127.0.0.1")};
    int port_{41451};
    std::atomic_bool busy_{false};
    std::thread worker_;

    QLabel* endpointLabel_{nullptr};
    QLabel* statusLabel_{nullptr};
    QGroupBox* timeGroup_{nullptr};
    QGroupBox* weatherGroup_{nullptr};
    QCheckBox* timeEnabledCheck_{nullptr};
    QDateTimeEdit* dateTimeEdit_{nullptr};
    QCheckBox* dstCheck_{nullptr};
    QDoubleSpinBox* clockSpeedSpin_{nullptr};
    QDoubleSpinBox* updateIntervalSpin_{nullptr};
    QCheckBox* moveSunCheck_{nullptr};
    QCheckBox* weatherEnabledCheck_{nullptr};
    std::array<QSlider*, 8> weatherSliders_{};
    std::array<QDoubleSpinBox*, 8> weatherSpins_{};
    QDialogButtonBox* buttonBox_{nullptr};
    QPushButton* applyButton_{nullptr};
};

} // namespace airsim_gui
