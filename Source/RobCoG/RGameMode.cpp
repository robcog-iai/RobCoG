// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RGameMode.h"

// Constructor
ARGameMode::ARGameMode(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// Set default pawn/character class
	DefaultPawnClass = ARGameMode::StaticClass();
}

// Transitions to WaitingToStart and calls BeginPlay on actors
void ARGameMode::StartPlay()
{
	Super::StartPlay();

	StartMatch();
}
