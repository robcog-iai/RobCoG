// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once
#include "ObjectMacros.h"

/*
 * This enum defines several grasp types
 */
UENUM(BlueprintType)
enum class EGraspType : uint8
{
	FullGrasp			UMETA(DisplayName = "FullGrasp"),
	PinchGrasp			UMETA(DisplayName = "PinchGrasp"),
	PinchThreeGrasp		UMETA(DisplayName = "PinchThreeGrasp"),
};