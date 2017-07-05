// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Marcel Meier

#pragma once

#include "CoreMinimal.h"
#include "HandOrientation.generated.h"

/*
 * Orientation of a joint
 */
USTRUCT()
struct FJointOrientation
{
	GENERATED_USTRUCT_BODY()

public:
	// Default constructor
	FJointOrientation() : Orientation(FRotator(0, 0, 0))
	{}

	UPROPERTY(EditAnywhere)
		FRotator Orientation;

};

/*
 * Orientation of all Joints of a finger
 */
USTRUCT()
struct FFingerOrientation
{
	GENERATED_USTRUCT_BODY()

public:
	// Default constructor
	FFingerOrientation()
	{}

	UPROPERTY(EditAnywhere)
		FJointOrientation MetacarpalOrientation;

	UPROPERTY(EditAnywhere)
		FJointOrientation ProximalOrientation;

	UPROPERTY(EditAnywhere)
		FJointOrientation IntermediateOrientation;

	UPROPERTY(EditAnywhere)
		FJointOrientation DistalOrientation;

};

/*
 * Orientation of all fingers of a hand
 */
USTRUCT()
struct FHandOrientation
{
	GENERATED_USTRUCT_BODY()

public:
	// Default constructor
	FHandOrientation()
	{}

	UPROPERTY(EditAnywhere)
		FFingerOrientation IndexOrientation;

	UPROPERTY(EditAnywhere)
		FFingerOrientation MiddleOrientation;

	UPROPERTY(EditAnywhere)
		FFingerOrientation RingOrientation;

	UPROPERTY(EditAnywhere)
		FFingerOrientation PinkyOrientation;

	UPROPERTY(EditAnywhere)
		FFingerOrientation ThumbOrientation;

};