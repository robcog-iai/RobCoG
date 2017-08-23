// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "Enums/GraspType.h"
#include "GameFramework/Actor.h"


struct FLogInfo
{
	// Default constructor
	FLogInfo() : GraspType(EGraspType::FullGrasp) {}

	EGraspType GraspType;
	TArray<float> OrientationGrasp;
	TArray<float> VelocityGrasp;
};

class ROBCOG_API HandLogger : public AActor
{

public:
	// Constructor
	HandLogger();


	// Saves the Itemname and the chosen 
	TMap<FString, FLogInfo> ItemToGraspInfoMap;

	// The start countdown timer
	FTimerHandle StartTimerHandle;

	// The game timer to log the 
	FTimerHandle GameTimerHandle;

	void UpdateTimer();

	void TimerHasFinished();

};
