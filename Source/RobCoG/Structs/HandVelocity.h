// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Marcel Meier

#pragma once

#include "CoreMinimal.h"
#include "HandVelocity.generated.h"

/*
 * Velocity of a joint
 */
USTRUCT()
struct FJointVelocity
{
	GENERATED_USTRUCT_BODY()

public:
	// Default constructor
	FJointVelocity() : Velocity(FVector(0, 0, 0))
	{}

	UPROPERTY(EditAnywhere)
		FVector Velocity;

	FORCEINLINE bool Equals(const FJointVelocity & JointVelocity, const float Delta)
	{
		return Velocity.Equals(JointVelocity.Velocity, Delta);
	}

};

/*
 * Velocity of all Joints of a finger
 */
USTRUCT()
struct FFingerVelocity
{
	GENERATED_USTRUCT_BODY()

public:
	// Default constructor
	FFingerVelocity()
	{}

	UPROPERTY(EditAnywhere)
		FJointVelocity MetacarpalVelocity;

	UPROPERTY(EditAnywhere)
		FJointVelocity ProximalVelocity;

	UPROPERTY(EditAnywhere)
		FJointVelocity IntermediateVelocity;

	UPROPERTY(EditAnywhere)
		FJointVelocity DistalVelocity;

	bool Equals(const FFingerVelocity & FingerVelocity, const float Delta)
	{
		if (!MetacarpalVelocity.Equals(FingerVelocity.MetacarpalVelocity, Delta))
		{
			return false;
		}
		if (!ProximalVelocity.Equals(FingerVelocity.ProximalVelocity, Delta))
		{
			return false;
		}
		if (!IntermediateVelocity.Equals(FingerVelocity.IntermediateVelocity, Delta))
		{
			return false;
		}
		if (!DistalVelocity.Equals(FingerVelocity.DistalVelocity, Delta))
		{
			return false;
		}

		return true;
	}


};

/*
 * Velocity of all fingers of a hand
 */
USTRUCT()
struct FHandVelocity
{
	GENERATED_USTRUCT_BODY()

public:
	// Default constructor
	FHandVelocity()
	{}

	UPROPERTY(EditAnywhere)
		FFingerVelocity IndexVelocity;

	UPROPERTY(EditAnywhere)
		FFingerVelocity MiddleVelocity;

	UPROPERTY(EditAnywhere)
		FFingerVelocity RingVelocity;

	UPROPERTY(EditAnywhere)
		FFingerVelocity PinkyVelocity;

	UPROPERTY(EditAnywhere)
		FFingerVelocity ThumbVelocity;

	// Checks two Velocities
	FORCEINLINE bool Equals(const FHandVelocity & HandVelocity, const float Delta)
	{
		if (!IndexVelocity.Equals(HandVelocity.IndexVelocity, Delta))
			return false;
		if (!MiddleVelocity.Equals(HandVelocity.MiddleVelocity, Delta))
			return false;
		if (!RingVelocity.Equals(HandVelocity.RingVelocity, Delta))
			return false;
		if (!PinkyVelocity.Equals(HandVelocity.PinkyVelocity, Delta))
			return false;
		if (!ThumbVelocity.Equals(HandVelocity.ThumbVelocity, Delta))
			return false;

		return true;
	}

	FORCEINLINE	FHandVelocity operator*(const float & Value)
	{
		FHandVelocity HandVelocitiy;

		HandVelocitiy.IndexVelocity.DistalVelocity.Velocity = this->IndexVelocity.DistalVelocity.Velocity*Value;
		HandVelocitiy.IndexVelocity.IntermediateVelocity.Velocity = this->IndexVelocity.IntermediateVelocity.Velocity*Value;
		HandVelocitiy.IndexVelocity.ProximalVelocity.Velocity = this->IndexVelocity.ProximalVelocity.Velocity*Value;
		HandVelocitiy.IndexVelocity.MetacarpalVelocity.Velocity = this->IndexVelocity.MetacarpalVelocity.Velocity*Value;

		HandVelocitiy.MiddleVelocity.DistalVelocity.Velocity = this->MiddleVelocity.DistalVelocity.Velocity*Value;
		HandVelocitiy.MiddleVelocity.IntermediateVelocity.Velocity = this->MiddleVelocity.IntermediateVelocity.Velocity*Value;
		HandVelocitiy.MiddleVelocity.ProximalVelocity.Velocity = this->MiddleVelocity.ProximalVelocity.Velocity*Value;
		HandVelocitiy.MiddleVelocity.MetacarpalVelocity.Velocity = this->MiddleVelocity.MetacarpalVelocity.Velocity*Value;

		HandVelocitiy.RingVelocity.DistalVelocity.Velocity = this->RingVelocity.DistalVelocity.Velocity*Value;
		HandVelocitiy.RingVelocity.IntermediateVelocity.Velocity = this->RingVelocity.IntermediateVelocity.Velocity*Value;
		HandVelocitiy.RingVelocity.ProximalVelocity.Velocity = this->RingVelocity.ProximalVelocity.Velocity*Value;
		HandVelocitiy.RingVelocity.MetacarpalVelocity.Velocity = this->RingVelocity.MetacarpalVelocity.Velocity*Value;

		HandVelocitiy.PinkyVelocity.DistalVelocity.Velocity = this->PinkyVelocity.DistalVelocity.Velocity*Value;
		HandVelocitiy.PinkyVelocity.IntermediateVelocity.Velocity = this->PinkyVelocity.IntermediateVelocity.Velocity*Value;
		HandVelocitiy.PinkyVelocity.ProximalVelocity.Velocity = this->PinkyVelocity.ProximalVelocity.Velocity*Value;
		HandVelocitiy.PinkyVelocity.MetacarpalVelocity.Velocity = this->PinkyVelocity.MetacarpalVelocity.Velocity*Value;

		HandVelocitiy.ThumbVelocity.DistalVelocity.Velocity = this->ThumbVelocity.DistalVelocity.Velocity*Value;
		HandVelocitiy.ThumbVelocity.IntermediateVelocity.Velocity = this->ThumbVelocity.IntermediateVelocity.Velocity*Value;
		HandVelocitiy.ThumbVelocity.ProximalVelocity.Velocity = this->ThumbVelocity.ProximalVelocity.Velocity*Value;
		HandVelocitiy.ThumbVelocity.MetacarpalVelocity.Velocity = this->ThumbVelocity.MetacarpalVelocity.Velocity*Value;

		return HandVelocitiy;

	}

};