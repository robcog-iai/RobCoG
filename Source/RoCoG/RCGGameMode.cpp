// Fill out your copyright notice in the Description page of Project Settings.

#include "RoCoG.h"
#include "RCGGameMode.h"

// Constructor
ARCGGameMode::ARCGGameMode(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// Set default pawn/character class
	DefaultPawnClass = ARCGGameMode::StaticClass();
}

// Transitions to WaitingToStart and calls BeginPlay on actors
void ARCGGameMode::StartPlay()
{
	Super::StartPlay();

	StartMatch();
}
