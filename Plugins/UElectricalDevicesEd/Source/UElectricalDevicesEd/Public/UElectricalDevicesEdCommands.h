// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Framework/Commands/Commands.h"
#include "UElectricalDevicesEdStyle.h"

class FUElectricalDevicesEdCommands : public TCommands<FUElectricalDevicesEdCommands>
{
public:

	FUElectricalDevicesEdCommands()
		: TCommands<FUElectricalDevicesEdCommands>(TEXT("UElectricalDevicesEd"), NSLOCTEXT("Contexts", "UElectricalDevicesEd", "UElectricalDevicesEd Plugin"), NAME_None, FUElectricalDevicesEdStyle::GetStyleSetName())
	{
	}

	// TCommands<> interface
	virtual void RegisterCommands() override;

public:
	TSharedPtr< FUICommandInfo > PluginAction;
};
