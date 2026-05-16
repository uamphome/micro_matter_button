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

namespace
{
constexpr EndpointId kLightSwitchEndpointId = 1;
constexpr EndpointId kLightEndpointId = 1;

// --- Accelerometer Configuration ---
constexpr uint32_t kAccelPollInterval = 50;         // 20Hz Polling
constexpr uint32_t kTapWakeDurationMs = 10000;      // 10 second sleep timeout
constexpr uint32_t kRotationWakeExtensionMs = 3000; // 3 second extension on movement
constexpr double kPi = 3.14159265358979323846;

// Maps 1 full rotation to roughly 0-100% brightness
constexpr double kRotationToLevelMultiplier = 40.0; 

k_timer sSingleTapTimer;
k_timer sAccelPollTimer;
k_timer sAccelSleepTimer;

Nrf::Matter::IdentifyCluster sIdentifyCluster(kLightEndpointId);

const struct device *sAccelDevice = nullptr;
double sLastAngle = 0.0;
bool sFirstPollAfterWake = false;
uint32_t sLastButtonPressTime = 0;

// Internal states for the 1-second settling logic
double sAccumulatedBrightness = 127.0; 
uint8_t sLastSentBrightness = 127;
uint32_t sLastMovementTime = 0; 

#define APPLICATION_BUTTON_MASK DK_BTN2_MSK

#ifdef CONFIG_CHIP_ICD_UAT_SUPPORT
#define UAT_BUTTON_MASK DK_BTN3_MSK
#endif
} /* namespace */

void AppTask::SetAccelerometerPower(bool wake)
{
    if (!sAccelDevice) return;

    struct sensor_value odr;
    if (wake) {
        LOG_INF("Waking sensor (25Hz ODR).");
        odr.val1 = 25; 
        odr.val2 = 0;
        sFirstPollAfterWake = true;
        sLastMovementTime = k_uptime_get_32(); // Reset the settling timer on wake
    } else {
        LOG_INF("Sleeping sensor (0Hz ODR).");
        odr.val1 = 0; 
        odr.val2 = 0;
    }
    
    sensor_attr_set(sAccelDevice, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
}

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

    // GEOMETRY NOISE REJECTION: Check horizontal magnitude. 
    double magnitude_sq = (x * x) + (y * y);
    if (magnitude_sq < 9.0) {
        sLastAngle = std::atan2(y, x); 
        return;
    }

    double currentAngle = std::atan2(y, x);

    if (sFirstPollAfterWake) {
        sLastAngle = currentAngle;
        sFirstPollAfterWake = false;
        return; 
    }

    double deltaAngle = currentAngle - sLastAngle;
    
    // Handle wrap-around
    if (deltaAngle > kPi) deltaAngle -= 2.0 * kPi;
    else if (deltaAngle < -kPi) deltaAngle += 2.0 * kPi;

    uint32_t now = k_uptime_get_32();

    // Check if movement is larger than the noise floor
    if (std::abs(deltaAngle) >= 0.05) {
        
        // 1. Accumulate the rotation
        sAccumulatedBrightness += (deltaAngle * kRotationToLevelMultiplier);
        
        // 2. Clamp to valid Matter bounds
        if (sAccumulatedBrightness > 254.0) sAccumulatedBrightness = 254.0;
        if (sAccumulatedBrightness < 1.0) sAccumulatedBrightness = 1.0;

        // 3. Update baselines and reset the 1-second settling timer
        sLastAngle = currentAngle; 
        sLastMovementTime = now;
        
        // 4. Extend the overall hardware sleep timer
        Instance().StartTimer(Timer::AccelSleep, kRotationWakeExtensionMs);
        
    } else {
        // No significant physical movement detected on this 50ms tick.
        // Check if it has been strictly > 300ms since the LAST movement.
        if ((now - sLastMovementTime) > 300) {
            
            uint8_t targetLevel = (uint8_t)sAccumulatedBrightness;
            
            // Only send if we actually have a new value to flush
            if (targetLevel != sLastSentBrightness) {
                LOG_INF("Movement Settled (1s timeout). Flushing Net Change -> Target: %d", targetLevel);
                
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
        LOG_INF("Inactivity timeout. Putting accelerometer to sleep.");
        k_timer_stop(&sAccelPollTimer);
		SetAccelerometerPower(false);
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
                // DOUBLE TAP
                Instance().CancelTimer(Timer::SingleTap);
                LOG_INF("Double tap detected! Starting 20Hz polling.");
                SetAccelerometerPower(true);
                
                Instance().StartTimer(Timer::AccelPoll, kAccelPollInterval);
                Instance().StartTimer(Timer::AccelSleep, kTapWakeDurationMs);
            } else {
                // SINGLE TAP
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
	if (timer == &sSingleTapTimer) timerType = Timer::SingleTap;
    else if (timer == &sAccelPollTimer) timerType = Timer::AccelPoll;
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

	sAccelDevice = DEVICE_DT_GET(DT_ALIAS(accel0));
    
	if (!device_is_ready(sAccelDevice)) {
		LOG_ERR("Accelerometer not ready.");
		sAccelDevice = nullptr;
	} else {
		LOG_INF("Accelerometer initialized.");
        SetAccelerometerPower(false);
	}

	k_timer_init(&sSingleTapTimer, AppTask::UserTimerTimeoutCallback, nullptr);
    k_timer_init(&sAccelPollTimer, AppTask::UserTimerTimeoutCallback, nullptr);
	k_timer_init(&sAccelSleepTimer, AppTask::UserTimerTimeoutCallback, nullptr);

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