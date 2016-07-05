// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/GameMode.h"
#include "RCGGameMode.generated.h"

/**
 * 
 */
UCLASS()
class ROCOG_API ARCGGameMode : public AGameMode
{
	GENERATED_BODY()
	
public:
	// Constructor
	ARCGGameMode(const FObjectInitializer& ObjectInitializer);

	// Called when the game starts or when spawned
	virtual void StartPlay() override;
};
