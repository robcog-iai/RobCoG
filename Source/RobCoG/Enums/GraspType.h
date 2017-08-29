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
	LargeDiameter			UMETA(DisplayName = "LargeDiameter"),
	Ring			UMETA(DisplayName = "Ring"),
	PalmarPinch			UMETA(DisplayName = "PalmarPinch"),
	SphereThreeFinger		UMETA(DisplayName = "SphereThreeFinger"),
	Lateral		UMETA(DisplayName = "Lateral"),
	ParallelExtension		UMETA(DisplayName = "ParallelExtension")
};