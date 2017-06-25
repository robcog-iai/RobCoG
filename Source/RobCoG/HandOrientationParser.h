// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"

#include "GraspType.h"
#include "Structs/HandOrientation.h"


/**
 * 
 */
class ROBCOG_API HandOrientationParser
{

public:
	HandOrientationParser();
	~HandOrientationParser();

	FHandOrientation GetInitialHandOrientationForGraspType(EGraspType GraspType);
	FHandOrientation GetClosedHandOrientationForGraspType(EGraspType GraspType);

	/*
	 TODO: Implement an Interface for this

	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "HandOrientationParser")
		FHandOrientation GetHandOrientationForGraspType(EGraspType GraspType);
	virtual FHandOrientation GetHandOrientationForGraspType_Implementation(EGraspType GraspType) override;
	*/
};
