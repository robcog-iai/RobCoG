// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/GameMode.h"
#include "RGameMode.generated.h"

/**
 * 
 */
UCLASS()
class ROBCOG_API ARGameMode : public AGameMode
{
	GENERATED_BODY()
	
public:
	// Constructor
	ARGameMode(const FObjectInitializer& ObjectInitializer);

	// Called when the game starts or when spawned
	virtual void StartPlay() override;
};
