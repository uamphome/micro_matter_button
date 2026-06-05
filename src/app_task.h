/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include "board/board.h"

#include <platform/CHIPDeviceLayer.h>
#include <zephyr/drivers/sensor.h>

struct k_timer;
struct Identify;

class AppTask {
public:
	static AppTask &Instance()
	{
		static AppTask sAppTask;
		return sAppTask;
	};

	CHIP_ERROR StartApp();

	void UpdateClusterState();
	static void UpdateStatusLed();

private:
	enum Timer : uint8_t {
		/* Mode 1: accelerometer dimming */
		SingleTap,
		AccelPoll,
		AccelSleep,
		/* Mode 2: GPIO long-press dimming */
		DimmerTrigger,
		Dimmer,
		/* Mode 3: generic switch */
		LongPressDetect,
		/* All modes: factory reset */
		FactoryWarning,
		FactoryReset,
		/* All modes: commissioning LED */
		StatusLedBlink,
		/* 25015 only: environmental sensor poll */
		SensorPoll,
	};

	CHIP_ERROR Init();

	static void TimerEventHandler(const Timer &event);
	static void ButtonEventHandler(Nrf::ButtonState state, Nrf::ButtonMask hasChanged);

	static void AccelPollEventHandler();
	static void SensorUpdateEventHandler();

	static void StartTimer(Timer, uint32_t);
	static void CancelTimer(Timer);
	static void UserTimerTimeoutCallback(k_timer *timer);

	static void MatterEventHandler(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t arg);
};
