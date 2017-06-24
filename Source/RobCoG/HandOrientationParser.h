// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
//#include "Interfaces/HandOrientationReadable.h"
#include "Structs/HandOrientation.h"
#include "Grasp.h"

#include "HandOrientationParser.generated.h"

/**
 * 
 */
UCLASS()
class ROBCOG_API UHandOrientationParser : public UObject //, public IHandOrientationReadable
{
	GENERATED_BODY()

public:
	UHandOrientationParser();

	FHandOrientation GetInitialHandOrientationForGraspType(EGraspType GraspType);
	FHandOrientation GetClosedHandOrientationForGraspType(EGraspType GraspType);

	/*
	 TODO: Implement an Interface for this

	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "HandOrientationParser")
		FHandOrientation GetHandOrientationForGraspType(EGraspType GraspType);
	virtual FHandOrientation GetHandOrientationForGraspType_Implementation(EGraspType GraspType) override;
	*/
};
