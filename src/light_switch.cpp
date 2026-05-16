/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "light_switch.h"
#include "binding/binding_handler.h"

#ifdef CONFIG_CHIP_LIB_SHELL
#include "shell_commands.h"
#endif

#include <app/clusters/bindings/binding-table.h>
#include <app/server/Server.h>
#include <controller/InvokeInteraction.h>
#include <platform/CHIPDeviceLayer.h>
#include <zephyr/logging/log.h>

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

void LightSwitch::Init(chip::EndpointId lightSwitchEndpoint)
{
	Nrf::Matter::BindingHandler::Init();
#ifdef CONFIG_CHIP_LIB_SHELL
	SwitchCommands::RegisterSwitchCommands();
#endif
	mLightSwitchEndpoint = lightSwitchEndpoint;
}

void LightSwitch::InitiateActionSwitch(Action action)
{
	Nrf::Matter::BindingHandler::BindingData *data = Platform::New<Nrf::Matter::BindingHandler::BindingData>();
	if (data) {
		data->EndpointId = mLightSwitchEndpoint;
		data->ClusterId = Clusters::OnOff::Id;
		data->InvokeCommandFunc = SwitchChangedHandler;
		switch (action) {
		case Action::Toggle:
			data->CommandId = Clusters::OnOff::Commands::Toggle::Id;
			break;
		case Action::On:
			data->CommandId = Clusters::OnOff::Commands::On::Id;
			break;
		case Action::Off:
			data->CommandId = Clusters::OnOff::Commands::Off::Id;
			break;
		default:
			Platform::Delete(data);
			return;
		}
		Nrf::Matter::BindingHandler::RunBoundClusterAction(data);
	}
}

void LightSwitch::DimmerChangeBrightness(bool increase)
{
    // Track the absolute brightness locally to force the bulb to obey
    static int16_t currentLevel = 127; // Start in the middle
    int step = 15; // ~6% per tick

    if (increase) {
        currentLevel += step;
    } else {
        currentLevel -= step;
    }

    // Clamp to valid Matter LevelControl bounds (1 to 254)
    if (currentLevel > 254) currentLevel = 254;
    if (currentLevel < 1) currentLevel = 1;

	Nrf::Matter::BindingHandler::BindingData *data = Platform::New<Nrf::Matter::BindingHandler::BindingData>();
	if (data) {
		data->EndpointId = mLightSwitchEndpoint;
		
		// Use the absolute MoveToLevelWithOnOff command
		data->CommandId = Clusters::LevelControl::Commands::MoveToLevelWithOnOff::Id;
		data->ClusterId = Clusters::LevelControl::Id;
		data->InvokeCommandFunc = SwitchChangedHandler;
		
		// Pass the calculated absolute level to the payload
		data->Value = (uint8_t)currentLevel;
		
		Nrf::Matter::BindingHandler::RunBoundClusterAction(data);
	}
}

void LightSwitch::SwitchChangedHandler(const Binding::TableEntry &binding, OperationalDeviceProxy *deviceProxy,
				       Nrf::Matter::BindingHandler::BindingData &bindingData)
{
	if (binding.type == Binding::MATTER_MULTICAST_BINDING) {
		switch (bindingData.ClusterId) {
		case Clusters::OnOff::Id:
			OnOffProcessCommand(bindingData.CommandId, binding, nullptr, bindingData);
			break;
		case Clusters::LevelControl::Id:
			LevelControlProcessCommand(bindingData.CommandId, binding, nullptr, bindingData);
			break;
		default:
			ChipLogError(NotSpecified, "Invalid binding group command data");
			break;
		}
	} else if (binding.type == Binding::MATTER_UNICAST_BINDING) {
		switch (bindingData.ClusterId) {
		case Clusters::OnOff::Id:
			OnOffProcessCommand(bindingData.CommandId, binding, deviceProxy, bindingData);
			break;
		case Clusters::LevelControl::Id:
			LevelControlProcessCommand(bindingData.CommandId, binding, deviceProxy, bindingData);
			break;
		default:
			ChipLogError(NotSpecified, "Invalid binding unicast command data");
			break;
		}
	}
}

void LightSwitch::OnOffProcessCommand(CommandId commandId, const Binding::TableEntry &binding,
				      OperationalDeviceProxy *device,
				      Nrf::Matter::BindingHandler::BindingData &bindingData)
{
	CHIP_ERROR ret = CHIP_NO_ERROR;

	auto onSuccess = [dataPointer = Platform::New<Nrf::Matter::BindingHandler::BindingData>(bindingData)](
				 const ConcreteCommandPath &commandPath, const StatusIB &status,
				 const auto &dataResponse) {
		Nrf::Matter::BindingHandler::OnInvokeCommandSucces(dataPointer);
	};

	auto onFailure = [dataPointer = Platform::New<Nrf::Matter::BindingHandler::BindingData>(bindingData)](
				 CHIP_ERROR aError) mutable {
		Nrf::Matter::BindingHandler::OnInvokeCommandFailure(dataPointer, aError);
	};

	if (device) {
		VerifyOrDie(device->ConnectionReady());
	}

	switch (commandId) {
	case Clusters::OnOff::Commands::Toggle::Id:
		Clusters::OnOff::Commands::Toggle::Type toggleCommand;
		if (device) {
			ret = Controller::InvokeCommandRequest(device->GetExchangeManager(),
							       device->GetSecureSession().Value(), binding.remote,
							       toggleCommand, onSuccess, onFailure);
		} else {
			Messaging::ExchangeManager &exchangeMgr = Server::GetInstance().GetExchangeManager();
			ret = Controller::InvokeGroupCommandRequest(&exchangeMgr, binding.fabricIndex, binding.groupId,
								    toggleCommand);
		}
		break;

	case Clusters::OnOff::Commands::On::Id:
		Clusters::OnOff::Commands::On::Type onCommand;
		if (device) {
			ret = Controller::InvokeCommandRequest(device->GetExchangeManager(),
							       device->GetSecureSession().Value(), binding.remote,
							       onCommand, onSuccess, onFailure);
		} else {
			Messaging::ExchangeManager &exchangeMgr = Server::GetInstance().GetExchangeManager();
			ret = Controller::InvokeGroupCommandRequest(&exchangeMgr, binding.fabricIndex, binding.groupId,
								    onCommand);
		}
		break;

	case Clusters::OnOff::Commands::Off::Id:
		Clusters::OnOff::Commands::Off::Type offCommand;
		if (device) {
			ret = Controller::InvokeCommandRequest(device->GetExchangeManager(),
							       device->GetSecureSession().Value(), binding.remote,
							       offCommand, onSuccess, onFailure);
		} else {
			Messaging::ExchangeManager &exchangeMgr = Server::GetInstance().GetExchangeManager();
			ret = Controller::InvokeGroupCommandRequest(&exchangeMgr, binding.fabricIndex, binding.groupId,
								    offCommand);
		}
		break;
	default:
		LOG_DBG("Invalid binding command data - commandId is not supported");
		break;
	}
	if (CHIP_NO_ERROR != ret) {
		LOG_ERR("Invoke OnOff Command Request ERROR: %s", ErrorStr(ret));
	}
}

void LightSwitch::LevelControlProcessCommand(CommandId commandId, const Binding::TableEntry &binding,
					     OperationalDeviceProxy *device,
					     Nrf::Matter::BindingHandler::BindingData &bindingData)
{
	auto onSuccess = [dataPointer = Platform::New<Nrf::Matter::BindingHandler::BindingData>(bindingData)](
				 const ConcreteCommandPath &commandPath, const StatusIB &status,
				 const auto &dataResponse) {
		Nrf::Matter::BindingHandler::OnInvokeCommandSucces(dataPointer);
	};

	auto onFailure = [dataPointer = Platform::New<Nrf::Matter::BindingHandler::BindingData>(bindingData)](
				 CHIP_ERROR aError) mutable {
		Nrf::Matter::BindingHandler::OnInvokeCommandFailure(dataPointer, aError);
	};

	CHIP_ERROR ret = CHIP_NO_ERROR;

	if (device) {
		VerifyOrDie(device->ConnectionReady());
	}

	switch (commandId) {
	case Clusters::LevelControl::Commands::MoveToLevelWithOnOff::Id: {
		Clusters::LevelControl::Commands::MoveToLevelWithOnOff::Type moveToLevelCommand;
		
		moveToLevelCommand.level = bindingData.Value;
		moveToLevelCommand.transitionTime.SetNonNull(0); // Ensure an instant snap

		if (device) {
			ret = Controller::InvokeCommandRequest(device->GetExchangeManager(),
							       device->GetSecureSession().Value(), binding.remote,
							       moveToLevelCommand, onSuccess, onFailure);
		} else {
			Messaging::ExchangeManager &exchangeMgr = Server::GetInstance().GetExchangeManager();
			ret = Controller::InvokeGroupCommandRequest(&exchangeMgr, binding.fabricIndex, binding.groupId,
								    moveToLevelCommand);
		}
	} break;
	default:
		LOG_DBG("Invalid binding command data - commandId is not supported");
		break;
	}
	if (CHIP_NO_ERROR != ret) {
		LOG_ERR("Invoke Group Command Request ERROR: %s", ErrorStr(ret));
	}
}
