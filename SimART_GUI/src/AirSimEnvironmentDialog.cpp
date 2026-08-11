#include "airsim_gui_UErealtime/AirSimEnvironmentDialog.h"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDateTimeEdit>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QPushButton>
#include <QSettings>
#include <QSignalBlocker>
#include <QSlider>
#include <QTime>
#include <QVBoxLayout>

#ifdef AIRSIM_GUI_ENABLE_AIRSIM
#include "api/WorldSimApiBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#endif

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace airsim_gui {
namespace {

constexpr float kRpcTimeoutSeconds = 3.0f;

const std::array<const char*, 8> kWeatherSettingKeys = {{
    "rain",
    "roadWetness",
    "snow",
    "roadSnow",
    "mapleLeaf",
    "roadLeaf",
    "dust",
    "fog",
}};

double clampWeather(double value) {
    return std::max(0.0, std::min(1.0, value));
}

} // namespace

AirSimEnvironmentDialog::AirSimEnvironmentDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(tr("AirSim Environment"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setModal(false);
    buildUi();
    loadState();
    setEndpoint(host_, port_);
}

AirSimEnvironmentDialog::~AirSimEnvironmentDialog() {
    if (worker_.joinable()) {
        worker_.join();
    }
}

void AirSimEnvironmentDialog::setEndpoint(const QString& host, int port) {
    host_ = host.trimmed().isEmpty() ? QStringLiteral("127.0.0.1") : host.trimmed();
    port_ = (port > 0 && port <= 65535) ? port : 41451;
    if (endpointLabel_) {
        endpointLabel_->setText(tr("%1:%2").arg(host_).arg(port_));
    }
}

void AirSimEnvironmentDialog::buildUi() {
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(10);

    auto* endpointRow = new QHBoxLayout();
    endpointRow->addWidget(new QLabel(tr("AirSim RPC"), this));
    endpointLabel_ = new QLabel(this);
    endpointLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    endpointRow->addWidget(endpointLabel_);
    endpointRow->addStretch(1);
    rootLayout->addLayout(endpointRow);

    timeGroup_ = new QGroupBox(tr("Time of Day"), this);
    auto* timeLayout = new QGridLayout(timeGroup_);
    timeEnabledCheck_ = new QCheckBox(tr("Apply time settings"), timeGroup_);
    timeEnabledCheck_->setChecked(true);
    dateTimeEdit_ = new QDateTimeEdit(QDateTime::currentDateTime(), timeGroup_);
    dateTimeEdit_->setDisplayFormat(QStringLiteral("yyyy-MM-dd HH:mm:ss"));
    dateTimeEdit_->setCalendarPopup(true);
    dateTimeEdit_->setMinimumWidth(190);
    dstCheck_ = new QCheckBox(tr("DST"), timeGroup_);
    moveSunCheck_ = new QCheckBox(tr("Move sun"), timeGroup_);
    moveSunCheck_->setChecked(true);
    clockSpeedSpin_ = new QDoubleSpinBox(timeGroup_);
    clockSpeedSpin_->setRange(0.0, 100000.0);
    clockSpeedSpin_->setDecimals(2);
    clockSpeedSpin_->setSingleStep(1.0);
    clockSpeedSpin_->setValue(0.0);
    clockSpeedSpin_->setKeyboardTracking(false);
    updateIntervalSpin_ = new QDoubleSpinBox(timeGroup_);
    updateIntervalSpin_->setRange(0.01, 3600.0);
    updateIntervalSpin_->setDecimals(2);
    updateIntervalSpin_->setSingleStep(1.0);
    updateIntervalSpin_->setSuffix(tr(" s"));
    updateIntervalSpin_->setValue(60.0);
    updateIntervalSpin_->setKeyboardTracking(false);

    timeLayout->addWidget(timeEnabledCheck_, 0, 0, 1, 2);
    timeLayout->addWidget(new QLabel(tr("Date and time"), timeGroup_), 1, 0);
    timeLayout->addWidget(dateTimeEdit_, 1, 1);
    timeLayout->addWidget(dstCheck_, 1, 2);
    auto* quickTimes = new QWidget(timeGroup_);
    auto* quickTimesLayout = new QHBoxLayout(quickTimes);
    quickTimesLayout->setContentsMargins(0, 0, 0, 0);
    quickTimesLayout->setSpacing(6);
    const std::array<std::pair<QString, QTime>, 4> timePresets = {{
        {tr("Dawn"), QTime(6, 0)},
        {tr("Noon"), QTime(12, 0)},
        {tr("Dusk"), QTime(18, 0)},
        {tr("Midnight"), QTime(0, 0)},
    }};
    for (const auto& preset : timePresets) {
        auto* button = new QPushButton(preset.first, quickTimes);
        connect(button, &QPushButton::clicked, this, [this, time = preset.second]() {
            dateTimeEdit_->setTime(time);
        });
        quickTimesLayout->addWidget(button);
    }
    timeLayout->addWidget(new QLabel(tr("Quick times"), timeGroup_), 2, 0);
    timeLayout->addWidget(quickTimes, 2, 1, 1, 2);
    timeLayout->addWidget(new QLabel(tr("Celestial speed"), timeGroup_), 3, 0);
    timeLayout->addWidget(clockSpeedSpin_, 3, 1);
    timeLayout->addWidget(moveSunCheck_, 3, 2);
    timeLayout->addWidget(new QLabel(tr("Update interval"), timeGroup_), 4, 0);
    timeLayout->addWidget(updateIntervalSpin_, 4, 1);
    timeLayout->setColumnStretch(1, 1);
    rootLayout->addWidget(timeGroup_);

    weatherGroup_ = new QGroupBox(tr("Weather"), this);
    auto* weatherLayout = new QGridLayout(weatherGroup_);
    weatherEnabledCheck_ = new QCheckBox(tr("Apply weather settings"), weatherGroup_);
    weatherEnabledCheck_->setChecked(true);
    auto* presetCombo = new QComboBox(weatherGroup_);
    presetCombo->addItems({
        tr("Custom"),
        tr("Clear"),
        tr("Light rain"),
        tr("Heavy rain"),
        tr("Fog"),
        tr("Snow"),
        tr("Dust"),
        tr("Autumn leaves"),
    });
    weatherLayout->addWidget(weatherEnabledCheck_, 0, 0, 1, 3);
    weatherLayout->addWidget(new QLabel(tr("Preset"), weatherGroup_), 0, 3);
    weatherLayout->addWidget(presetCombo, 0, 4, 1, 2);

    const std::array<QString, 8> weatherLabels = {{
        tr("Rain"),
        tr("Road wetness"),
        tr("Snow"),
        tr("Road snow"),
        tr("Maple leaves"),
        tr("Road leaves"),
        tr("Dust"),
        tr("Fog"),
    }};
    for (int index = 0; index < static_cast<int>(weatherSliders_.size()); ++index) {
        const int columnGroup = index / 4;
        const int row = (index % 4) + 1;
        const int baseColumn = columnGroup * 3;
        auto* label = new QLabel(weatherLabels[static_cast<size_t>(index)], weatherGroup_);
        auto* slider = new QSlider(Qt::Horizontal, weatherGroup_);
        slider->setRange(0, 100);
        slider->setSingleStep(5);
        slider->setPageStep(10);
        slider->setMinimumWidth(125);
        auto* spin = new QDoubleSpinBox(weatherGroup_);
        spin->setRange(0.0, 1.0);
        spin->setDecimals(2);
        spin->setSingleStep(0.05);
        spin->setKeyboardTracking(false);
        spin->setFixedWidth(70);
        weatherSliders_[static_cast<size_t>(index)] = slider;
        weatherSpins_[static_cast<size_t>(index)] = spin;
        weatherLayout->addWidget(label, row, baseColumn);
        weatherLayout->addWidget(slider, row, baseColumn + 1);
        weatherLayout->addWidget(spin, row, baseColumn + 2);
        connect(slider, &QSlider::valueChanged, this, [spin](int value) {
            const QSignalBlocker blocker(spin);
            spin->setValue(static_cast<double>(value) / 100.0);
        });
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [slider](double value) {
            const QSignalBlocker blocker(slider);
            slider->setValue(static_cast<int>(std::lround(clampWeather(value) * 100.0)));
        });
    }
    weatherLayout->setColumnStretch(1, 1);
    weatherLayout->setColumnStretch(4, 1);
    connect(presetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &AirSimEnvironmentDialog::applyWeatherPreset);
    rootLayout->addWidget(weatherGroup_);

    statusLabel_ = new QLabel(tr("Ready"), this);
    statusLabel_->setWordWrap(true);
    rootLayout->addWidget(statusLabel_);

    buttonBox_ = new QDialogButtonBox(QDialogButtonBox::Close, this);
    applyButton_ = buttonBox_->addButton(tr("Apply to AirSim"), QDialogButtonBox::ApplyRole);
    connect(applyButton_, &QPushButton::clicked, this, &AirSimEnvironmentDialog::applyEnvironment);
    connect(buttonBox_, &QDialogButtonBox::rejected, this, &AirSimEnvironmentDialog::reject);
    rootLayout->addWidget(buttonBox_);

#ifndef AIRSIM_GUI_ENABLE_AIRSIM
    applyButton_->setEnabled(false);
    statusLabel_->setText(tr("AirSim C++ RPC support is disabled in this build."));
#endif

    setMinimumSize(700, 455);
    resize(760, 500);
}

void AirSimEnvironmentDialog::loadState() {
    QSettings settings(QStringLiteral("OpenAI"), QStringLiteral("airsim_gui_UErealtime"));
    settings.beginGroup(QStringLiteral("environment"));
    timeEnabledCheck_->setChecked(settings.value(QStringLiteral("timeEnabled"), true).toBool());
    const QDateTime savedDateTime = QDateTime::fromString(
        settings.value(QStringLiteral("dateTime")).toString(), Qt::ISODate);
    dateTimeEdit_->setDateTime(savedDateTime.isValid() ? savedDateTime : QDateTime::currentDateTime());
    dstCheck_->setChecked(settings.value(QStringLiteral("dst"), false).toBool());
    clockSpeedSpin_->setValue(settings.value(QStringLiteral("celestialClockSpeed"), 0.0).toDouble());
    updateIntervalSpin_->setValue(settings.value(QStringLiteral("updateIntervalSecs"), 60.0).toDouble());
    moveSunCheck_->setChecked(settings.value(QStringLiteral("moveSun"), true).toBool());
    weatherEnabledCheck_->setChecked(settings.value(QStringLiteral("weatherEnabled"), true).toBool());
    for (int index = 0; index < static_cast<int>(weatherSpins_.size()); ++index) {
        const QString key = QString::fromLatin1(kWeatherSettingKeys[static_cast<size_t>(index)]);
        const double defaultValue = (index == 0 || index == 1 || index == 7) ? 1.0 : 0.0;
        weatherSpins_[static_cast<size_t>(index)]->setValue(
            clampWeather(settings.value(key, defaultValue).toDouble()));
    }
    settings.endGroup();
}

void AirSimEnvironmentDialog::saveState() const {
    QSettings settings(QStringLiteral("OpenAI"), QStringLiteral("airsim_gui_UErealtime"));
    settings.beginGroup(QStringLiteral("environment"));
    settings.setValue(QStringLiteral("timeEnabled"), timeEnabledCheck_->isChecked());
    settings.setValue(QStringLiteral("dateTime"), dateTimeEdit_->dateTime().toString(Qt::ISODate));
    settings.setValue(QStringLiteral("dst"), dstCheck_->isChecked());
    settings.setValue(QStringLiteral("celestialClockSpeed"), clockSpeedSpin_->value());
    settings.setValue(QStringLiteral("updateIntervalSecs"), updateIntervalSpin_->value());
    settings.setValue(QStringLiteral("moveSun"), moveSunCheck_->isChecked());
    settings.setValue(QStringLiteral("weatherEnabled"), weatherEnabledCheck_->isChecked());
    for (int index = 0; index < static_cast<int>(weatherSpins_.size()); ++index) {
        settings.setValue(QString::fromLatin1(kWeatherSettingKeys[static_cast<size_t>(index)]),
                          weatherSpins_[static_cast<size_t>(index)]->value());
    }
    settings.endGroup();
}

void AirSimEnvironmentDialog::setBusy(bool busy) {
    busy_.store(busy);
    timeGroup_->setEnabled(!busy);
    weatherGroup_->setEnabled(!busy);
    applyButton_->setEnabled(!busy);
    if (QPushButton* closeButton = buttonBox_->button(QDialogButtonBox::Close)) {
        closeButton->setEnabled(!busy);
    }
}

void AirSimEnvironmentDialog::setWeatherValues(const std::array<double, 8>& values) {
    for (int index = 0; index < static_cast<int>(weatherSpins_.size()); ++index) {
        weatherSpins_[static_cast<size_t>(index)]->setValue(
            clampWeather(values[static_cast<size_t>(index)]));
    }
}

void AirSimEnvironmentDialog::applyWeatherPreset(int index) {
    switch (index) {
    case 1:
        setWeatherValues({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
        break;
    case 2:
        setWeatherValues({{0.35, 0.45, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15}});
        break;
    case 3:
        setWeatherValues({{1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.65}});
        break;
    case 4:
        setWeatherValues({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8}});
        break;
    case 5:
        setWeatherValues({{0.0, 0.0, 0.85, 0.75, 0.0, 0.0, 0.0, 0.2}});
        break;
    case 6:
        setWeatherValues({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.85, 0.25}});
        break;
    case 7:
        setWeatherValues({{0.0, 0.0, 0.0, 0.0, 0.85, 0.7, 0.0, 0.1}});
        break;
    default:
        break;
    }
}

AirSimEnvironmentDialog::ApplyRequest AirSimEnvironmentDialog::currentRequest() const {
    ApplyRequest request;
    request.host = host_;
    request.port = port_;
    request.timeEnabled = timeEnabledCheck_->isChecked();
    request.dateTime = dateTimeEdit_->dateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss"));
    request.dst = dstCheck_->isChecked();
    request.celestialClockSpeed = static_cast<float>(clockSpeedSpin_->value());
    request.updateIntervalSecs = static_cast<float>(updateIntervalSpin_->value());
    request.moveSun = moveSunCheck_->isChecked();
    request.weatherEnabled = weatherEnabledCheck_->isChecked();
    for (int index = 0; index < static_cast<int>(request.weather.size()); ++index) {
        request.weather[static_cast<size_t>(index)] =
            static_cast<float>(clampWeather(weatherSpins_[static_cast<size_t>(index)]->value()));
    }
    return request;
}

void AirSimEnvironmentDialog::applyEnvironment() {
    if (busy_.load()) {
        return;
    }
    if (worker_.joinable()) {
        worker_.join();
    }

    clockSpeedSpin_->interpretText();
    updateIntervalSpin_->interpretText();
    for (QDoubleSpinBox* spin : weatherSpins_) {
        spin->interpretText();
    }
    const ApplyRequest request = currentRequest();
    saveState();
    setBusy(true);
    statusLabel_->setStyleSheet(QString());
    statusLabel_->setText(tr("Applying environment settings to %1:%2...")
                              .arg(request.host).arg(request.port));
    worker_ = std::thread(&AirSimEnvironmentDialog::runApply, this, request);
}

void AirSimEnvironmentDialog::runApply(ApplyRequest request) {
    bool success = false;
    QString message;
#ifdef AIRSIM_GUI_ENABLE_AIRSIM
    try {
        using msr::airlib::MultirotorRpcLibClient;
        using WeatherParameter = msr::airlib::WorldSimApiBase::WeatherParameter;
        MultirotorRpcLibClient client(request.host.toStdString(),
                                      static_cast<uint16_t>(request.port),
                                      kRpcTimeoutSeconds);
        if (!client.ping()) {
            throw std::runtime_error("AirSim did not respond to the RPC ping.");
        }
        if (request.timeEnabled) {
            client.simSetTimeOfDay(true,
                                   request.dateTime.toStdString(),
                                   request.dst,
                                   request.celestialClockSpeed,
                                   request.updateIntervalSecs,
                                   request.moveSun);
        } else {
            client.simSetTimeOfDay(false);
        }
        const std::array<WeatherParameter, 8> parameters = {{
            WeatherParameter::Rain,
            WeatherParameter::Roadwetness,
            WeatherParameter::Snow,
            WeatherParameter::RoadSnow,
            WeatherParameter::MapleLeaf,
            WeatherParameter::RoadLeaf,
            WeatherParameter::Dust,
            WeatherParameter::Fog,
        }};
        if (request.weatherEnabled) {
            client.simEnableWeather(true);
            for (int index = 0; index < static_cast<int>(parameters.size()); ++index) {
                client.simSetWeatherParameter(parameters[static_cast<size_t>(index)],
                                              request.weather[static_cast<size_t>(index)]);
            }
        } else {
            for (const WeatherParameter parameter : parameters) {
                client.simSetWeatherParameter(parameter, 0.0f);
            }
            client.simEnableWeather(false);
        }
        success = true;
        if (!request.timeEnabled && !request.weatherEnabled) {
            message = tr("Default AirSim lighting and weather restored at %1:%2.")
                          .arg(request.host).arg(request.port);
        } else if (!request.timeEnabled) {
            message = tr("Default AirSim lighting restored and weather updated at %1:%2.")
                          .arg(request.host).arg(request.port);
        } else if (!request.weatherEnabled) {
            message = tr("AirSim time updated and default weather restored at %1:%2.")
                          .arg(request.host).arg(request.port);
        } else {
            message = tr("AirSim environment updated at %1:%2.").arg(request.host).arg(request.port);
        }
    } catch (const std::exception& error) {
        message = tr("Could not update AirSim environment: %1")
                      .arg(QString::fromUtf8(error.what()));
    } catch (...) {
        message = tr("Could not update AirSim environment: unknown RPC error.");
    }
#else
    Q_UNUSED(request)
    message = tr("AirSim C++ RPC support is disabled in this build.");
#endif
    QMetaObject::invokeMethod(this, [this, success, message]() {
        finishApply(success, message);
    }, Qt::QueuedConnection);
}

void AirSimEnvironmentDialog::finishApply(bool success, const QString& message) {
    if (worker_.joinable()) {
        worker_.join();
    }
    setBusy(false);
    statusLabel_->setStyleSheet(success
        ? QStringLiteral("color: #166534;")
        : QStringLiteral("color: #b91c1c;"));
    statusLabel_->setText(message);
    if (success) {
        emit environmentApplied(message);
    }
}

void AirSimEnvironmentDialog::closeEvent(QCloseEvent* event) {
    if (busy_.load()) {
        event->ignore();
        return;
    }
    QDialog::closeEvent(event);
}

void AirSimEnvironmentDialog::reject() {
    if (!busy_.load()) {
        QDialog::reject();
    }
}

} // namespace airsim_gui
