// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "CoreMinimal.h"

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
	// Constructor
	MCGrasp();

	// Destructor
	~MCGrasp();

	// Update grasp
	void Update(const float Goal);

	// Grasp type
	EGraspType GraspType;
};
