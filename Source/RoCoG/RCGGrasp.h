// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "RCGUtils.h"

/** Enum indicating the finger type */
UENUM(BlueprintType)
enum class ERCGHandLimb : uint8
{
	Thumb		UMETA(DisplayName = "Thumb"),
	Index		UMETA(DisplayName = "Index"),
	Middle		UMETA(DisplayName = "Middle"),
	Ring		UMETA(DisplayName = "Ring"),
	Pinky		UMETA(DisplayName = "Pinky"),
	Palm		UMETA(DisplayName = "Palm")
};

/** Enum indicating the grasp state */
enum class EGraspState : uint8
{
	Opened		UMETA(DisplayName = "Opened"),
	Opening		UMETA(DisplayName = "Opening"),
	Closed		UMETA(DisplayName = "Closed"),
	Closing		UMETA(DisplayName = "Closed"),
	Free		UMETA(DisplayName = "Free"),
};

/**
 * Base class for grasping, opens and closes the hand with a given step size
 */
class ROCOG_API FRCGGrasp
{
public:
	// Constructor
	FRCGGrasp();

	// Constructor with access to the fingers and the constraints
	FRCGGrasp(TMultiMap<ERCGHandLimb, FConstraintInstance*>& /*FingerTypeToConstrs*/);
	
	// Destructor
	~FRCGGrasp();

	// Update the grasping (open/close fingers with the given step)
	virtual void Update(const float Step);

	// Add finger to the blocked ones (grasping will have no effect on it)
	void BlockFinger(ERCGHandLimb Finger);

	// Remove finger from the blocked ones (grasping will effect it)
	void FreeFinger(ERCGHandLimb Finger);

	// Free all fingers
	void FreeFingers();	

	// Grasping state
	EGraspState State;

protected:
	// Finger types and their constraints as multi map (e.g Index : index_01_l, index_02_l)
	TMultiMap<ERCGHandLimb, FConstraintInstance*> FingerTypeToConstraintsMMap;

	// Map hand fingers to their target
	TMap<ERCGHandLimb, float> FingerToTargetMap;

	// Map hand fingers to their limits
	TMap<ERCGHandLimb, float> FingerToUpperLimitMap;

	// Map hand fingers to their limits
	TMap<ERCGHandLimb, float> FingerToLowerLimitMap;

	// Blocked fingers
	TSet<ERCGHandLimb> BlockedFingers;

private:
	// Store the value of the previous step
	float PrevStep;

	// 

};

