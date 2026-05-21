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

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::DeviceLayer;

// --- BARE METAL SPI INJECTION ---
#include <zephyr/drivers/spi.h>

// Fetch the SPI bus details directly from your device tree
static const struct spi_dt_spec sAccelBus = SPI_DT_SPEC_GET(DT_ALIAS(accel0), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);

static void WriteAccelRegister(uint8_t reg, uint8_t val) {
    uint8_t tx_buf[2] = { reg, val };
    struct spi_buf tx_spi_buf = { .buf = tx_buf, .len = 2 };
    struct spi_buf_set tx_spi_buf_set = { .buffers = &tx_spi_buf, .count = 1 };

    int err = spi_write_dt(&sAccelBus, &tx_spi_buf_set);
    if (err) {
        LOG_ERR("Raw SPI Write Failed on Reg 0x%02X: %d", reg, err);
    }
}


namespace
{
constexpr EndpointId kLightSwitchEndpointId = 1;
constexpr EndpointId kLightEndpointId = 1;

constexpr uint32_t kAccelPollInterval = 50;
constexpr uint32_t kMovementSettlingTimeMs = 150;
constexpr uint32_t kTapWakeDurationMs = 10000;
constexpr uint32_t kRotationWakeExtensionMs = 3000;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRotationToLevelMultiplier = 40.0;

k_timer sSingleTapTimer;
k_timer sAccelPollTimer;
k_timer sAccelSleepTimer;

Nrf::Matter::IdentifyCluster sIdentifyCluster(kLightEndpointId);

const struct device *sAccelDevice = nullptr;
double sLastAngle = 0.0;
bool sFirstPollAfterWake = false;
uint32_t sLastButtonPressTime = 0;

double sAccumulatedBrightness = 127.0;
uint8_t sLastSentBrightness = 127;
uint32_t sLastMovementTime = 0;

#define APPLICATION_BUTTON_MASK DK_BTN2_MSK

#ifdef CONFIG_CHIP_ICD_UAT_SUPPORT
#define UAT_BUTTON_MASK DK_BTN3_MSK
#endif
} /* namespace */


void AppTask::AccelPollEventHandler()
{
    if (!sAccelDevice) return;

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
        LOG_INF("Horizontal tilt detected! Turning off rotation listener.");
        
        uint8_t targetLevel = (uint8_t)sAccumulatedBrightness;
        if (targetLevel != sLastSentBrightness) {
            LightSwitch::GetInstance().SetBrightnessLevel(targetLevel);
            sLastSentBrightness = targetLevel;
        }

        Instance().CancelTimer(Timer::AccelPoll);
        Instance().CancelTimer(Timer::AccelSleep);
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
    
    if (deltaAngle > kPi) deltaAngle -= 2.0 * kPi;
    else if (deltaAngle < -kPi) deltaAngle += 2.0 * kPi;

    uint32_t now = k_uptime_get_32();

    if (std::abs(deltaAngle) >= 0.05) {
        sAccumulatedBrightness += (deltaAngle * kRotationToLevelMultiplier);
        
        if (sAccumulatedBrightness > 254.0) sAccumulatedBrightness = 254.0;
        if (sAccumulatedBrightness < 1.0) sAccumulatedBrightness = 1.0;

        sLastAngle = currentAngle; 
        sLastMovementTime = now;
        
        Instance().StartTimer(Timer::AccelSleep, kRotationWakeExtensionMs);
        
    } else {
        if ((now - sLastMovementTime) > kMovementSettlingTimeMs) {
            uint8_t targetLevel = (uint8_t)sAccumulatedBrightness;
            
            if (targetLevel != sLastSentBrightness) {
                LOG_INF("Movement Settled. Flushing Net Change -> Target: %d", targetLevel);
                LightSwitch::GetInstance().SetBrightnessLevel(targetLevel);
                sLastSentBrightness = targetLevel;
            }
        }
    }
}

void AppTask::TimerEventHandler(const Timer &timerType)
{
	switch (timerType) {
	case Timer::SingleTap:
		LOG_INF("Single Tap confirmed. Toggling Light.");
		LightSwitch::GetInstance().InitiateActionSwitch(LightSwitch::Action::Toggle);
		break;
	case Timer::AccelPoll:
		AccelPollEventHandler();
		break;
	case Timer::AccelSleep:
		LOG_INF("Inactivity timeout. Stopping dimming.");
		k_timer_stop(&sAccelPollTimer);
		break;
	default:
		break;
	}
}

void AppTask::ButtonEventHandler(Nrf::ButtonState state, Nrf::ButtonMask hasChanged)
{
	if (APPLICATION_BUTTON_MASK & hasChanged) {
        if (APPLICATION_BUTTON_MASK & state) {
            uint32_t now = k_uptime_get_32();
            
            if (now - sLastButtonPressTime < 400) {
                Instance().CancelTimer(Timer::SingleTap);
                LOG_INF("GPIO double tap — starting dimming.");
                sFirstPollAfterWake = true;
                sLastMovementTime = now;
                Instance().StartTimer(Timer::AccelPoll, kAccelPollInterval);
                Instance().StartTimer(Timer::AccelSleep, kTapWakeDurationMs);
            } else {
                Instance().StartTimer(Timer::SingleTap, 400);
            }
            sLastButtonPressTime = now;
        }
#ifdef CONFIG_CHIP_ICD_UAT_SUPPORT
	} else if ((UAT_BUTTON_MASK & state & hasChanged)) {
		Server::GetInstance().GetICDManager().OnNetworkActivity();
#endif
	}
}

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
	default:
		break;
	}
}

void AppTask::UserTimerTimeoutCallback(k_timer *timer)
{
	if (!timer) return;

	Timer timerType;
	if (timer == &sSingleTapTimer)       timerType = Timer::SingleTap;
	else if (timer == &sAccelPollTimer)  timerType = Timer::AccelPoll;
	else if (timer == &sAccelSleepTimer) timerType = Timer::AccelSleep;
	else return;

	Nrf::PostTask([timerType]() { TimerEventHandler(timerType); });
}

CHIP_ERROR AppTask::Init()
{
	ReturnErrorOnFailure(Nrf::Matter::PrepareServer(Nrf::Matter::InitData{ .mPostServerInitClbk = [] {
		LightSwitch::GetInstance().Init(kLightSwitchEndpointId);
		return CHIP_NO_ERROR;
	} }));

	// Init timers before any timer-starting code (e.g. SetAccelerometerPower)
	k_timer_init(&sSingleTapTimer, AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sAccelPollTimer, AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sAccelSleepTimer, AppTask::UserTimerTimeoutCallback, nullptr);

	sAccelDevice = DEVICE_DT_GET(DT_ALIAS(accel0));

	if (!device_is_ready(sAccelDevice)) {
		LOG_ERR("Accelerometer not ready.");
		sAccelDevice = nullptr;
	} else {
		WriteAccelRegister(0x20, 0x77); // CTRL_REG1: 400Hz normal mode, X/Y/Z
	}

	if (!Nrf::GetBoard().Init(ButtonEventHandler)) {
		LOG_ERR("UI init failed.");
		return CHIP_ERROR_INCORRECT_STATE;
	}

	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(Nrf::Board::DefaultMatterEventHandler, 0));
	ReturnErrorOnFailure(sIdentifyCluster.Init());

	return Nrf::Matter::StartServer();
}

CHIP_ERROR AppTask::StartApp()
{
	ReturnErrorOnFailure(Init());

	while (true) {
		Nrf::DispatchNextTask();
	}

	return CHIP_NO_ERROR;
}