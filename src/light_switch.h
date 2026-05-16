/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once
#include <app/util/basic-types.h>
#include <lib/core/CHIPError.h>

#include "binding/binding_handler.h"

#include <atomic>

/** @class LightSwitch
 *  @brief Class for controlling a CHIP light bulb over a Thread network
 *
 *  Features:
 *  - discovering a CHIP light bulb which advertises itself by sending Thread multicast packets
 *  - toggling and dimming the connected CHIP light bulb by sending appropriate CHIP messages
 */
class LightSwitch {
public:
	enum class Action : uint8_t {
		Toggle,
		On,
		Off 
	};

	void Init(chip::EndpointId lightSwitchEndpoint);
	void InitiateActionSwitch(Action action);
	
	// Updated signature to accept direction
	void DimmerChangeBrightness(bool increase);
	
	chip::EndpointId GetLightSwitchEndpointId() { return mLightSwitchEndpoint; }
	static void SwitchChangedHandler(const chip::app::Clusters::Binding::TableEntry &binding,
					 chip::OperationalDeviceProxy *deviceProxy,
					 Nrf::Matter::BindingHandler::BindingData &bindingData);

	static LightSwitch &GetInstance()
	{
		static LightSwitch sLightSwitch;
		return sLightSwitch;
	}

private:
	static void OnOffProcessCommand(chip::CommandId commandId,
					const chip::app::Clusters::Binding::TableEntry &binding,
					chip::OperationalDeviceProxy *device,
					Nrf::Matter::BindingHandler::BindingData &bindingData);
	static void LevelControlProcessCommand(chip::CommandId commandId,
					       const chip::app::Clusters::Binding::TableEntry &binding,
					       chip::OperationalDeviceProxy *device,
					       Nrf::Matter::BindingHandler::BindingData &bindingData);

	chip::EndpointId mLightSwitchEndpoint;
};
