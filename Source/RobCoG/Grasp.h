// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"

#include "Structs/Finger.h"
#include "Structs/HandOrientation.h"

class AHand;

/** Enum indicating the grasp movement type */
UENUM(BlueprintType)
enum class EGraspType : uint8
{
	FullGrasp			UMETA(DisplayName = "FullGrasp"),
	PinchGrasp			UMETA(DisplayName = "PinchGrasp"),
	PinchThreeGrasp		UMETA(DisplayName = "PinchThreeGrasp"),
};

/**
 * 
 */
class ROBCOG_API Grasp
{
public:
	Grasp();

	~Grasp();

	void DriveToHandOrientation(const FHandOrientation & HandOrientation, AHand* const Hand);

	void DriveToFingerOrientation(const FFingerOrientation & FingerOrientation, const FFinger & Finger);

	// Set grasp type
	void SetCurrentGraspType(const EGraspType InGraspType);

	//Get grasp type
	EGraspType GetCurrentGraspType() const { return CurrentGraspType; };

private: 

	EGraspType CurrentGraspType;


};
