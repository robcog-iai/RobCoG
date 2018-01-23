// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once
#include "ObjectMacros.h"

/*
 * This enum defines several grasp types
 */
UENUM(BlueprintType)
enum class EGraspType : uint8
{
	LargeDiameter		UMETA(DisplayName = "LargeDiameter"),
	Ring				UMETA(DisplayName = "Ring"),
	PalmarPinch			UMETA(DisplayName = "PalmarPinch"),
	ParallelExtension	UMETA(DisplayName = "ParallelExtension"),
	Lateral				UMETA(DisplayName = "Lateral"),
	Tripod				UMETA(DisplayName = "Tripod"),
};