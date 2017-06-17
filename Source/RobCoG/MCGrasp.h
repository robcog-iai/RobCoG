// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "CoreMinimal.h"
#include "MCHand.h"

/** Enum indicating the grasp movement type */
UENUM(BlueprintType)
enum class EGraspType : uint8
{
	AllFingers		UMETA(DisplayName = "AllFingers"),
	Pinch			UMETA(DisplayName = "Pinch"),
};

/**
 * 
 */
class ROBCOG_API MCGrasp
{
public:
	// Default constructor
	MCGrasp();
	
	// Destructor
	~MCGrasp();

	//// Set grasp fingers
	//void SetFingers(
	//	const FFinger& InThumb,
	//	const FFinger& InIndex,
	//	const FFinger& InMiddle,
	//	const FFinger& InRing,
	//	const FFinger& InPinky);

	// Set grasp type
	void SetGraspType(const EGraspType InGraspType);

	//Get grasp type
	EGraspType GetGraspType() const { return GraspType; };

	// Update grasp
	void Update(const float Goal);

protected:
	// Grasp type
	EGraspType GraspType;

	//// Thumb finger
	//FFinger Thumb;

	//// Index finger 
	//FFinger Index;

	//// Middle finger
	//FFinger Middle;

	//// Ring finger
	//FFinger Ring;

	//// Pinky finger
	//FFinger Pinky;
};
