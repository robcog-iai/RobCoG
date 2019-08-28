// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "UElectricalDevicesEdCommands.h"

#define LOCTEXT_NAMESPACE "FUElectricalDevicesEdModule"

void FUElectricalDevicesEdCommands::RegisterCommands()
{
	UI_COMMAND(PluginAction, "UElectricalDevicesEd", "Execute UElectricalDevicesEd action", EUserInterfaceActionType::Button, FInputGesture());
}

#undef LOCTEXT_NAMESPACE
