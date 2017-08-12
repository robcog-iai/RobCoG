// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "EngineUtils.h"
#include "Engine.h"
#include "RRobot.h"
#include "ROSBridgeHandler.h"
#include "ROSBridgePublisher.h"
#include "RobCoGGameModeBase.generated.h"

UCLASS()
class ROBCOG_API ARobCoGGameModeBase : public AGameModeBase
{
	GENERATED_BODY()
	
public:
    TSharedPtr<FROSBridgeHandler> Handler;
    TSharedPtr<FROSBridgePublisher> TimePublisher;

    UPROPERTY()
    FString ROSBridgeServerIPAddr;

    UPROPERTY()
    uint32 ROSBridgeServerPort;

	ARobCoGGameModeBase();

    void BeginPlay() override;

    void Tick(float DeltaSeconds) override;

    void Logout(AController *Exiting) override;
	
	
};
