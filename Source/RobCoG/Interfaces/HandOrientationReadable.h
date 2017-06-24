// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "Grasp.h"
#include "Structs/HandOrientation.h"

#include "HandOrientationReadable.generated.h"

// This class does not need to be modified.
UINTERFACE(MinimalAPI)
class UHandOrientationReadable : public UInterface
{
	GENERATED_UINTERFACE_BODY()
};

/**
 * 
 */
class ROBCOG_API IHandOrientationReadable
{
	GENERATED_IINTERFACE_BODY()

	// Add interface functions to this class. This is the class that will be inherited to implment this interface.
public:
	
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "HandOrientationReadable")
		FHandOrientation GetHandOrientationForGraspType(EGraspType GraspType);
	
};
