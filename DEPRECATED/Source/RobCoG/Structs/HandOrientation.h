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

	bool Equals(const FJointOrientation & JointOrientation, const float Delta)
	{
		return Orientation.Equals(JointOrientation.Orientation, Delta);
	}

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

	bool Equals(const FFingerOrientation & FingerOrientation, const float Delta)
	{
		if (!MetacarpalOrientation.Equals(FingerOrientation.MetacarpalOrientation, Delta))
		{
			return false;
		}
		if (!ProximalOrientation.Equals(FingerOrientation.ProximalOrientation, Delta))
		{
			return false;
		}
		if (!IntermediateOrientation.Equals(FingerOrientation.IntermediateOrientation, Delta))
		{
			return false;
		}
		if (!DistalOrientation.Equals(FingerOrientation.DistalOrientation, Delta))
		{
			return false;
		}

		return true;
	}

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

	// Checks two Orientations
	bool Equals(const FHandOrientation & HandOrientation, const float Delta)
	{
		if (!IndexOrientation.Equals(HandOrientation.IndexOrientation, Delta))
			return false;
		if (!MiddleOrientation.Equals(HandOrientation.MiddleOrientation, Delta))
			return false;
		if (!RingOrientation.Equals(HandOrientation.RingOrientation, Delta))
			return false;
		if (!PinkyOrientation.Equals(HandOrientation.PinkyOrientation, Delta))
			return false;
		if (!ThumbOrientation.Equals(HandOrientation.ThumbOrientation, Delta))
			return false;

		return true;
	}

};