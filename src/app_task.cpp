/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"

#include "light_switch.h"

#include "app/matter_init.h"
#include "app/task_executor.h"
#include "board/board.h"
#include "clusters/identify.h"

#include <setup_payload/OnboardingCodesUtil.h>

#include <zephyr/logging/log.h>
#include <cmath>

#include <app/server/Server.h>
#include <platform/CHIPDeviceLayer.h>

#include <dk_buttons_and_leds.h>

#ifdef CONFIG_MATTER_BUTTON_MODE_2
#include <app/clusters/switch-server/switch-server.h>
#endif

#ifdef CONFIG_SHT4X
#include <app-common/zap-generated/attributes/Accessors.h>
#endif

#include <app/util/endpoint-config-api.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::DeviceLayer;

/* --- Raw SPI for accelerometer ------------------------------------------- */
#include <zephyr/drivers/spi.h>

static const struct spi_dt_spec sAccelBus =
	SPI_DT_SPEC_GET(DT_ALIAS(accel0), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);

static void WriteAccelRegister(uint8_t reg, uint8_t val)
{
	uint8_t tx_buf[2] = { reg, val };
	struct spi_buf tx_spi_buf = { .buf = tx_buf, .len = 2 };
	struct spi_buf_set tx_spi_buf_set = { .buffers = &tx_spi_buf, .count = 1 };
	int err = spi_write_dt(&sAccelBus, &tx_spi_buf_set);
	if (err) {
		LOG_ERR("Accel SPI write failed (reg 0x%02X): %d", reg, err);
	}
}

/* --- LPS22HB power-down (25015 only) ------------------------------------- */
/* No native SPI driver exists for LPS22HB in NCS; pressure sensing is not
 * implemented. The sensor is explicitly placed in power-down mode at boot
 * since it defaults to ODR=0 on a cold power-up but may retain state across
 * soft resets. */
#ifdef CONFIG_SHT4X

static const struct spi_dt_spec sLps22hbBus = {
	.bus = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lps22hb))),
	.config = {
		.frequency = 10000000U,
		.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_LINES_SINGLE,
		.slave    = DT_REG_ADDR(DT_NODELABEL(lps22hb)),
		.cs       = SPI_CS_CONTROL_INIT(DT_NODELABEL(lps22hb), 0),
	},
};

static void WriteLps22hbRegister(uint8_t reg, uint8_t val)
{
	uint8_t tx_buf[2] = { (uint8_t)(reg & 0x3F), val };
	struct spi_buf tx_spi_buf = { .buf = tx_buf, .len = 2 };
	struct spi_buf_set tx_set = { .buffers = &tx_spi_buf, .count = 1 };
	spi_write_dt(&sLps22hbBus, &tx_set);
}

#endif /* CONFIG_SHT4X */

/* --- Constants and module-level state ------------------------------------ */
namespace
{
constexpr EndpointId kLightSwitchEndpointId = 1;
constexpr EndpointId kLightEndpointId = 1;
constexpr EndpointId kGenericSwitchEndpointId = 2;
constexpr EndpointId kTempEndpointId = 3;
constexpr EndpointId kHumEndpointId = 4;
constexpr EndpointId kPressureEndpointId = 5;

/* Mode 1: accelerometer dimming */
constexpr uint32_t kAccelPollInterval = 50;
constexpr uint32_t kMovementSettlingTimeMs = 150;
constexpr uint32_t kTapWakeDurationMs = 10000;
constexpr uint32_t kRotationWakeExtensionMs = 3000;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRotationToLevelMultiplier = 40.0;

/* Mode 2: GPIO long-press dimming */
constexpr uint32_t kDimmerTriggeredTimeout = 500;
constexpr uint32_t kDimmerInterval = 300;

/* Mode 3: generic switch long-press threshold */
constexpr uint32_t kLongPressThresholdMs = 500;

/* Factory reset: 12s warning, 15s trigger */
constexpr uint32_t kFactoryResetWarningMs = 12000;
constexpr uint32_t kFactoryResetTriggerMs = 15000;

/* LED blink: slow (50ms on / 950ms off) and fast (100ms on/off) */
constexpr uint32_t kSlowBlinkOnMs = 50;
constexpr uint32_t kSlowBlinkOffMs = 950;
constexpr uint32_t kFastBlinkMs = 100;
constexpr uint32_t kFactoryWarningWhiteMs = 500; /* white flash duration — matches DK library blink rate */

/* Sensor poll: every 5 minutes */
constexpr uint32_t kSensorPollIntervalMs = 300000;

/* --- Timers --- */
k_timer sSingleTapTimer;
k_timer sAccelPollTimer;
k_timer sAccelSleepTimer;
k_timer sDimmerTriggerTimer;
k_timer sDimmerTimer;
k_timer sLongPressTimer;
k_timer sFactoryWarningTimer;
k_timer sFactoryResetTimer;
k_timer sStatusLedBlinkTimer;
k_timer sSensorPollTimer;

Nrf::Matter::IdentifyCluster sIdentifyCluster(kLightEndpointId);

/* --- Mode 1 state --- */
#ifdef CONFIG_MATTER_BUTTON_MODE_1
const struct device *sAccelDevice = nullptr;
double sLastAngle = 0.0;
bool sFirstPollAfterWake = false;
uint32_t sLastButtonPressTime = 0;
double sAccumulatedBrightness = 127.0;
uint8_t sLastSentBrightness = 127;
uint32_t sLastMovementTime = 0;
bool sDimmerTriggered = false;
#endif

/* --- Mode 2 state --- */
#ifdef CONFIG_MATTER_BUTTON_MODE_2
bool sLongPressTriggered = false;
#endif

/* --- Factory reset state --- */
bool sInFactoryWarning = false;
uint8_t sFactoryWarningBlinksRemaining = 0; /* counts remaining 500ms half-cycles */

/* --- LED state --- */
enum class LedState { SlowBlink, FastBlink, Off };
LedState sLedState = LedState::SlowBlink;
bool sStatusLedOn = false;

/* --- Sensor devices (25015 only) --- */
#ifdef CONFIG_SHT4X
const struct device *sSht40Device = nullptr;
bool sIsSensorTimerStarted = false;
#endif

/* Button mask: on 25015 the physical button is sw0 (DK_BTN1), on 25008 it is sw1 (DK_BTN2) */
#if defined(CONFIG_BOARD_HOLYIOT_25015_NRF54L15_CPUAPP)
#define APPLICATION_BUTTON_MASK DK_BTN1_MSK
#else
#define APPLICATION_BUTTON_MASK DK_BTN2_MSK
#endif

#ifdef CONFIG_CHIP_ICD_UAT_SUPPORT
#define UAT_BUTTON_MASK DK_BTN3_MSK
#endif
} /* namespace */

/* --- LED management ------------------------------------------------------ */

void AppTask::UpdateStatusLed()
{
	if (sInFactoryWarning) {
		return; /* LEDs controlled by factory reset warning logic */
	}

	CancelTimer(Timer::StatusLedBlink);

	switch (sLedState) {
	case LedState::Off:
		dk_set_led_off(DK_LED1);
		break;
	case LedState::FastBlink:
		sStatusLedOn = false;
		StartTimer(Timer::StatusLedBlink, kFastBlinkMs);
		break;
	case LedState::SlowBlink:
		sStatusLedOn = false;
		dk_set_led_off(DK_LED1);
		StartTimer(Timer::StatusLedBlink, kSlowBlinkOffMs);
		break;
	}
}

void AppTask::MatterEventHandler(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t)
{
	static bool sIsBleConnected = false;

	switch (event->Type) {
	case DeviceEventType::kCHIPoBLEAdvertisingChange:
		sIsBleConnected = ConnectivityMgr().NumBLEConnections() != 0;
		break;
	case DeviceEventType::kCommissioningComplete:
#ifdef CONFIG_SHT4X
		if (sSht40Device && !sIsSensorTimerStarted) {
			LOG_INF("Commissioning complete — starting sensor measurements.");
			Nrf::PostTask([] { SensorUpdateEventHandler(); });
			k_timer_start(&sSensorPollTimer, K_MSEC(kSensorPollIntervalMs), K_MSEC(kSensorPollIntervalMs));
			sIsSensorTimerStarted = true;
		}
#endif
		break;
	case DeviceEventType::kThreadStateChange:
		break;
	default:
		return;
	}

	/* Always read live from FabricTable — never cache commissioned state.
	 * A cached bool starts false and a BLE advertising event on a commissioned
	 * reboot would incorrectly start the blink timer before kThreadStateChange
	 * corrects it, preventing ICD deep sleep. */
	bool isCommissioned = Server::GetInstance().GetFabricTable().FabricCount() > 0;

	if (isCommissioned) {
		sLedState = LedState::Off;
	} else if (sIsBleConnected) {
		sLedState = LedState::FastBlink;
	} else {
		sLedState = LedState::SlowBlink;
	}
	Instance().UpdateStatusLed();
}

/* --- Mode 1: accelerometer poll ------------------------------------------ */

#ifdef CONFIG_MATTER_BUTTON_MODE_1
void AppTask::AccelPollEventHandler()
{
	if (!sAccelDevice) {
		return;
	}

	if (chip::Server::GetInstance().GetCommissioningWindowManager().IsCommissioningWindowOpen()) {
		return;
	}

	struct sensor_value accel[3];
	if (sensor_sample_fetch(sAccelDevice) < 0 ||
	    sensor_channel_get(sAccelDevice, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) {
		return;
	}

	double x = sensor_value_to_double(&accel[0]);
	double y = sensor_value_to_double(&accel[1]);
	double z = sensor_value_to_double(&accel[2]);

	if (std::abs(z) > 7.5) {
		LOG_INF("Horizontal tilt — stopping dimming.");
		uint8_t targetLevel = (uint8_t)sAccumulatedBrightness;
		if (targetLevel != sLastSentBrightness) {
			LightSwitch::GetInstance().SetBrightnessLevel(targetLevel);
			sLastSentBrightness = targetLevel;
		}
		Instance().CancelTimer(Timer::AccelPoll);
		Instance().CancelTimer(Timer::AccelSleep);
		WriteAccelRegister(0x20, 0x00); /* Power-down until next double-tap */
		return;
	}

	double magnitude_sq = (x * x) + (y * y);
	if (magnitude_sq < 9.0) {
		sLastAngle = std::atan2(y, x);
		sLastMovementTime = k_uptime_get_32();
		return;
	}

	double currentAngle = std::atan2(y, x);

	if (sFirstPollAfterWake) {
		sLastAngle = currentAngle;
		sFirstPollAfterWake = false;
		return;
	}

	double deltaAngle = currentAngle - sLastAngle;
	if (deltaAngle > kPi) {
		deltaAngle -= 2.0 * kPi;
	} else if (deltaAngle < -kPi) {
		deltaAngle += 2.0 * kPi;
	}

	uint32_t now = k_uptime_get_32();

	if (std::abs(deltaAngle) >= 0.05) {
		sAccumulatedBrightness += (deltaAngle * kRotationToLevelMultiplier);
		if (sAccumulatedBrightness > 254.0) {
			sAccumulatedBrightness = 254.0;
		}
		if (sAccumulatedBrightness < 1.0) {
			sAccumulatedBrightness = 1.0;
		}
		sLastAngle = currentAngle;
		sLastMovementTime = now;
		Instance().StartTimer(Timer::AccelSleep, kRotationWakeExtensionMs);
	} else {
		if ((now - sLastMovementTime) > kMovementSettlingTimeMs) {
			uint8_t targetLevel = (uint8_t)sAccumulatedBrightness;
			if (targetLevel != sLastSentBrightness) {
				LOG_INF("Movement settled — brightness: %d", targetLevel);
				LightSwitch::GetInstance().SetBrightnessLevel(targetLevel);
				sLastSentBrightness = targetLevel;
			}
		}
	}
}
#else
void AppTask::AccelPollEventHandler() {}
#endif /* CONFIG_MATTER_BUTTON_MODE_1 */

/* --- 25015 sensor reading ------------------------------------------------ */

void AppTask::SensorUpdateEventHandler()
{
#ifdef CONFIG_SHT4X
	if (!sSht40Device) {
		return;
	}

	struct sensor_value temp_val, hum_val;

	if (sensor_sample_fetch(sSht40Device) < 0) {
		LOG_ERR("SHT40 fetch failed");
		return;
	}
	sensor_channel_get(sSht40Device, SENSOR_CHAN_AMBIENT_TEMP, &temp_val);
	sensor_channel_get(sSht40Device, SENSOR_CHAN_HUMIDITY, &hum_val);

	int16_t temp_matter = (int16_t)(sensor_value_to_double(&temp_val) * 100.0);
	uint16_t hum_matter = (uint16_t)(sensor_value_to_double(&hum_val) * 100.0);

	LOG_INF("Sensors: temp=%d (x100 C), hum=%u (x100 %%)", temp_matter, hum_matter);

	chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(kTempEndpointId, temp_matter);
	chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(kHumEndpointId, hum_matter);
#endif /* CONFIG_SHT4X */
}

/* --- Timer event dispatcher ---------------------------------------------- */

void AppTask::TimerEventHandler(const Timer &timerType)
{
	switch (timerType) {

#ifdef CONFIG_MATTER_BUTTON_MODE_1
	case Timer::SingleTap:
		/* Only toggle if the button is no longer held — if still held, DimmerTrigger handles it */
		if (!(dk_get_buttons() & APPLICATION_BUTTON_MASK)) {
			LOG_INF("Single tap — toggling light.");
			LightSwitch::GetInstance().InitiateActionSwitch(LightSwitch::Action::Toggle);
		}
		break;

	case Timer::AccelPoll:
		AccelPollEventHandler();
		break;

	case Timer::AccelSleep:
		LOG_INF("Accel inactivity timeout — stopping dimming.");
		k_timer_stop(&sAccelPollTimer);
		WriteAccelRegister(0x20, 0x00); /* Power-down until next double-tap */
		break;

	case Timer::DimmerTrigger:
		LOG_INF("Long press — starting brightness cycle.");
		Instance().CancelTimer(Timer::SingleTap);
		sDimmerTriggered = true;
		LightSwitch::GetInstance().InitiateActionSwitch(LightSwitch::Action::On);
		Instance().StartTimer(Timer::Dimmer, kDimmerInterval);
		break;

	case Timer::Dimmer:
		LightSwitch::GetInstance().DimmerChangeBrightness();
		break;
#endif /* CONFIG_MATTER_BUTTON_MODE_1 */

#ifdef CONFIG_MATTER_BUTTON_MODE_2
	case Timer::LongPressDetect:
		sLongPressTriggered = true;
		chip::app::Clusters::SwitchServer::Instance().OnLongPress(kGenericSwitchEndpointId, 1);
		break;
#endif /* CONFIG_MATTER_BUTTON_MODE_2 */

	case Timer::FactoryWarning:
		LOG_INF("Factory reset warning — release to cancel, or hold to reset.");
		sInFactoryWarning = true;
		sFactoryWarningBlinksRemaining = 5; /* 5 half-cycles → 3 on-periods at 500ms = 3 white blinks */
		sStatusLedOn = true;
		dk_set_led_on(DK_LED1);
		dk_set_led_on(DK_LED2);
		dk_set_led_on(DK_LED3);
		StartTimer(Timer::StatusLedBlink, kFactoryWarningWhiteMs);
		break;

	case Timer::FactoryReset:
		LOG_INF("Factory reset triggered.");
		chip::Server::GetInstance().ScheduleFactoryReset();
		break;

	case Timer::StatusLedBlink:
		if (sInFactoryWarning) {
			/* Toggle all 3 LEDs (white blink) */
			sStatusLedOn = !sStatusLedOn;
			if (sStatusLedOn) {
				dk_set_led_on(DK_LED1);
				dk_set_led_on(DK_LED2);
				dk_set_led_on(DK_LED3);
			} else {
				dk_set_led_off(DK_LED1);
				dk_set_led_off(DK_LED2);
				dk_set_led_off(DK_LED3);
			}
			sFactoryWarningBlinksRemaining--;
			if (sFactoryWarningBlinksRemaining > 0) {
				StartTimer(Timer::StatusLedBlink, kFactoryWarningWhiteMs);
			} else {
				/* Blink sequence done — LEDs are off; restore commissioning indicator */
				sInFactoryWarning = false;
				Instance().UpdateStatusLed();
			}
		} else if (sLedState == LedState::FastBlink) {
			sStatusLedOn = !sStatusLedOn;
			sStatusLedOn ? dk_set_led_on(DK_LED1) : dk_set_led_off(DK_LED1);
			StartTimer(Timer::StatusLedBlink, kFastBlinkMs);
		} else if (sLedState == LedState::SlowBlink) {
			sStatusLedOn = !sStatusLedOn;
			sStatusLedOn ? dk_set_led_on(DK_LED1) : dk_set_led_off(DK_LED1);
			StartTimer(Timer::StatusLedBlink, sStatusLedOn ? kSlowBlinkOnMs : kSlowBlinkOffMs);
		}
		break;

	case Timer::SensorPoll:
		SensorUpdateEventHandler();
		break;

	default:
		break;
	}
}

/* --- Button handler ------------------------------------------------------ */

void AppTask::ButtonEventHandler(Nrf::ButtonState state, Nrf::ButtonMask hasChanged)
{
	if (APPLICATION_BUTTON_MASK & hasChanged) {
		if (APPLICATION_BUTTON_MASK & state) {
			/* Button pressed */

			/* Always restart factory reset timers on each press */
			sInFactoryWarning = false;
			Instance().CancelTimer(Timer::FactoryWarning);
			Instance().CancelTimer(Timer::FactoryReset);
			Instance().StartTimer(Timer::FactoryWarning, kFactoryResetWarningMs);
			Instance().StartTimer(Timer::FactoryReset, kFactoryResetTriggerMs);

#ifdef CONFIG_MATTER_BUTTON_MODE_1
			uint32_t now = k_uptime_get_32();
			if (now - sLastButtonPressTime < 400) {
				/* Double tap — cancel pending timers, wake accel */
				Instance().CancelTimer(Timer::SingleTap);
				Instance().CancelTimer(Timer::DimmerTrigger);
				LOG_INF("GPIO double tap — starting accel dimming.");
				WriteAccelRegister(0x20, 0x77); /* 400Hz normal mode — powered down at all other times */
				sFirstPollAfterWake = true;
				sLastMovementTime = now;
				Instance().StartTimer(Timer::AccelPoll, kAccelPollInterval);
				Instance().StartTimer(Timer::AccelSleep, kTapWakeDurationMs);
			} else {
				/* First press — wait for double tap or long press */
				sDimmerTriggered = false;
				Instance().StartTimer(Timer::SingleTap, 400);
				Instance().StartTimer(Timer::DimmerTrigger, kDimmerTriggeredTimeout);
			}
			sLastButtonPressTime = now;
#endif

#ifdef CONFIG_MATTER_BUTTON_MODE_2
			sLongPressTriggered = false;
			chip::app::Clusters::SwitchServer::Instance().OnInitialPress(kGenericSwitchEndpointId, 1);
			Instance().StartTimer(Timer::LongPressDetect, kLongPressThresholdMs);
#endif
		} else {
			/* Button released */
			Instance().CancelTimer(Timer::FactoryWarning);
			Instance().CancelTimer(Timer::FactoryReset);

			if (sInFactoryWarning) {
				/* Released during warning window — cancel factory reset */
				LOG_INF("Factory reset cancelled.");
				sInFactoryWarning = false;
				sFactoryWarningBlinksRemaining = 0;
				dk_set_led_off(DK_LED2);
				dk_set_led_off(DK_LED3);
				Instance().UpdateStatusLed();
				return;
			}

#ifdef CONFIG_MATTER_BUTTON_MODE_1
			Instance().CancelTimer(Timer::DimmerTrigger);
			if (sDimmerTriggered) {
				Instance().CancelTimer(Timer::Dimmer);
				sDimmerTriggered = false;
			}
			/* Short press: SingleTap timer is still running and will fire the toggle */
#endif

#ifdef CONFIG_MATTER_BUTTON_MODE_2
			Instance().CancelTimer(Timer::LongPressDetect);
			if (sLongPressTriggered) {
				chip::app::Clusters::SwitchServer::Instance().OnLongRelease(kGenericSwitchEndpointId, 1);
				sLongPressTriggered = false;
			} else {
				chip::app::Clusters::SwitchServer::Instance().OnShortRelease(kGenericSwitchEndpointId, 1);
			}
#endif
		}

#ifdef CONFIG_CHIP_ICD_UAT_SUPPORT
	} else if ((UAT_BUTTON_MASK & state & hasChanged)) {
		Server::GetInstance().GetICDManager().OnNetworkActivity();
#endif
	}
}

/* --- Timer start / cancel / callback ------------------------------------- */

void AppTask::StartTimer(Timer timer, uint32_t timeoutMs)
{
	switch (timer) {
	case Timer::SingleTap:
		k_timer_start(&sSingleTapTimer, K_MSEC(timeoutMs), K_NO_WAIT);
		break;
	case Timer::AccelPoll:
		k_timer_start(&sAccelPollTimer, K_MSEC(timeoutMs), K_MSEC(timeoutMs));
		break;
	case Timer::AccelSleep:
		k_timer_start(&sAccelSleepTimer, K_MSEC(timeoutMs), K_NO_WAIT);
		break;
	case Timer::DimmerTrigger:
		k_timer_start(&sDimmerTriggerTimer, K_MSEC(timeoutMs), K_NO_WAIT);
		break;
	case Timer::Dimmer:
		k_timer_start(&sDimmerTimer, K_MSEC(timeoutMs), K_MSEC(timeoutMs));
		break;
	case Timer::LongPressDetect:
		k_timer_start(&sLongPressTimer, K_MSEC(timeoutMs), K_NO_WAIT);
		break;
	case Timer::FactoryWarning:
		k_timer_start(&sFactoryWarningTimer, K_MSEC(timeoutMs), K_NO_WAIT);
		break;
	case Timer::FactoryReset:
		k_timer_start(&sFactoryResetTimer, K_MSEC(timeoutMs), K_NO_WAIT);
		break;
	case Timer::StatusLedBlink:
		k_timer_start(&sStatusLedBlinkTimer, K_MSEC(timeoutMs), K_NO_WAIT);
		break;
	case Timer::SensorPoll:
		k_timer_start(&sSensorPollTimer, K_MSEC(timeoutMs), K_MSEC(timeoutMs));
		break;
	default:
		break;
	}
}

void AppTask::CancelTimer(Timer timer)
{
	switch (timer) {
	case Timer::SingleTap:
		k_timer_stop(&sSingleTapTimer);
		break;
	case Timer::AccelPoll:
		k_timer_stop(&sAccelPollTimer);
		break;
	case Timer::AccelSleep:
		k_timer_stop(&sAccelSleepTimer);
		break;
	case Timer::DimmerTrigger:
		k_timer_stop(&sDimmerTriggerTimer);
		break;
	case Timer::Dimmer:
		k_timer_stop(&sDimmerTimer);
		break;
	case Timer::LongPressDetect:
		k_timer_stop(&sLongPressTimer);
		break;
	case Timer::FactoryWarning:
		k_timer_stop(&sFactoryWarningTimer);
		break;
	case Timer::FactoryReset:
		k_timer_stop(&sFactoryResetTimer);
		break;
	case Timer::StatusLedBlink:
		k_timer_stop(&sStatusLedBlinkTimer);
		break;
	case Timer::SensorPoll:
		k_timer_stop(&sSensorPollTimer);
		break;
	default:
		break;
	}
}

void AppTask::UserTimerTimeoutCallback(k_timer *timer)
{
	if (!timer) {
		return;
	}

	Timer timerType;
	if (timer == &sSingleTapTimer)         timerType = Timer::SingleTap;
	else if (timer == &sAccelPollTimer)    timerType = Timer::AccelPoll;
	else if (timer == &sAccelSleepTimer)   timerType = Timer::AccelSleep;
	else if (timer == &sDimmerTriggerTimer) timerType = Timer::DimmerTrigger;
	else if (timer == &sDimmerTimer)        timerType = Timer::Dimmer;
	else if (timer == &sLongPressTimer)    timerType = Timer::LongPressDetect;
	else if (timer == &sFactoryWarningTimer) timerType = Timer::FactoryWarning;
	else if (timer == &sFactoryResetTimer) timerType = Timer::FactoryReset;
	else if (timer == &sStatusLedBlinkTimer) timerType = Timer::StatusLedBlink;
	else if (timer == &sSensorPollTimer)   timerType = Timer::SensorPoll;
	else return;

	Nrf::PostTask([timerType]() { TimerEventHandler(timerType); });
}

/* --- Init and StartApp --------------------------------------------------- */

CHIP_ERROR AppTask::Init()
{
	ReturnErrorOnFailure(Nrf::Matter::PrepareServer(Nrf::Matter::InitData{ .mPostServerInitClbk = [] {
		LightSwitch::GetInstance().Init(kLightSwitchEndpointId);
		return CHIP_NO_ERROR;
	} }));

	/* Init all timers before any code that might start them */
	k_timer_init(&sSingleTapTimer,       AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sAccelPollTimer,       AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sAccelSleepTimer,      AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sDimmerTriggerTimer,   AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sDimmerTimer,          AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sLongPressTimer,       AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sFactoryWarningTimer,  AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sFactoryResetTimer,    AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sStatusLedBlinkTimer,  AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sSensorPollTimer,      AppTask::UserTimerTimeoutCallback, nullptr);

	/* Accelerometer: active in Mode 1, powered down in Mode 2 */
#ifdef CONFIG_MATTER_BUTTON_MODE_1
	sAccelDevice = DEVICE_DT_GET(DT_ALIAS(accel0));
	if (!device_is_ready(sAccelDevice)) {
		LOG_ERR("Accelerometer not ready.");
		sAccelDevice = nullptr;
	} else {
		WriteAccelRegister(0x20, 0x00); /* CTRL_REG1: power-down — woken on demand at double-tap */
	}
#else
	/* Power down the accelerometer to save current */
	WriteAccelRegister(0x20, 0x00); /* CTRL_REG1: power-down mode */
#endif

	/* Environmental sensors (25015 only) */
#ifdef CONFIG_SHT4X
	sSht40Device = DEVICE_DT_GET(DT_ALIAS(env0));
	if (!device_is_ready(sSht40Device)) {
		LOG_ERR("SHT40 not ready.");
		sSht40Device = nullptr;
	} else {
		/* LPS22HB: force power-down (ODR=0) — not used, but may retain state across soft resets */
		WriteLps22hbRegister(0x10, 0x00); /* CTRL_REG1: power-down mode */
		/* Sensor poll timer started only after commissioning — see MatterEventHandler */
	}
#endif

	/* Init LED GPIOs — the board module skips this when NCS_SAMPLE_MATTER_LEDS=n */
	int led_err = dk_leds_init();
	if (led_err) {
		LOG_ERR("LED init failed: %d", led_err);
	}

	if (!Nrf::GetBoard().Init(ButtonEventHandler)) {
		LOG_ERR("UI init failed.");
		return CHIP_ERROR_INCORRECT_STATE;
	}

	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(Nrf::Board::DefaultMatterEventHandler, 0));
	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(AppTask::MatterEventHandler, 0));
	ReturnErrorOnFailure(sIdentifyCluster.Init());

	/* Start the Matter server first so the FabricTable is loaded from persistent storage */
	CHIP_ERROR err = Nrf::Matter::StartServer();
	if (err != CHIP_NO_ERROR) {
		return err;
	}

	/* Disable endpoints that are unused in the selected operating mode */
#ifdef CONFIG_MATTER_BUTTON_MODE_1
	emberAfEndpointEnableDisable(kGenericSwitchEndpointId, false);
#endif
#ifdef CONFIG_MATTER_BUTTON_MODE_2
	emberAfEndpointEnableDisable(kLightSwitchEndpointId, false);
#endif

	/* Disable sensor endpoints on boards without environmental sensors */
#ifndef CONFIG_SHT4X
	emberAfEndpointEnableDisable(kTempEndpointId, false);
	emberAfEndpointEnableDisable(kHumEndpointId, false);
#endif
	/* Pressure sensor (LPS22HB) not implemented — always disable EP5 */
	emberAfEndpointEnableDisable(kPressureEndpointId, false);

	bool commissioned = Server::GetInstance().GetFabricTable().FabricCount() > 0;

	/* On reboot, if already commissioned, fire the sensor timer immediately (K_NO_WAIT)
	 * then repeat every 5 minutes.  On a fresh unpaired device the timer stays stopped
	 * until kCommissioningComplete fires in MatterEventHandler. */
#ifdef CONFIG_SHT4X
	if (commissioned && sSht40Device && !sIsSensorTimerStarted) {
		LOG_INF("Device already commissioned — starting sensor measurements.");
		k_timer_start(&sSensorPollTimer, K_NO_WAIT, K_MSEC(kSensorPollIntervalMs));
		sIsSensorTimerStarted = true;
	}
#endif

	/* Set initial LED state based on commissioning status */
	sLedState = commissioned ? LedState::Off : LedState::SlowBlink;
	UpdateStatusLed();

	return CHIP_NO_ERROR;
}

void AppTask::UpdateClusterState() {}

CHIP_ERROR AppTask::StartApp()
{
	ReturnErrorOnFailure(Init());

	while (true) {
		Nrf::DispatchNextTask();
	}

	return CHIP_NO_ERROR;
}
