// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "RCGUtils.h"

/**
 * Base class for grasping, opens and closes the hand with a given step
 */
class ROCOG_API FRCGGrasp
{
public:
	/** Enum indicating the grasp state */
	enum class EGraspState : uint8
	{
		Opened		UMETA(DisplayName = "Opened"),
		Closed		UMETA(DisplayName = "Closed"),
		Free		UMETA(DisplayName = "Free"),
	};

	// Constructor
	FRCGGrasp();

	// Constructor with access to the fingers and the constraints
	FRCGGrasp(TMultiMap<ERCGHandLimb, FConstraintInstance*>& /*FingerTypeToConstrs*/);
	
	// Destructor
	~FRCGGrasp();

	// Update the grasping (open/close fingers with the given step)
	virtual void Update(TMultiMap<ERCGHandLimb, FConstraintInstance*>& FingerTypeToConstrs,
		const float Step);

	// Add finger to the blocked ones (grasping will have no effect on it)
	void BlockFinger(ERCGHandLimb Finger);

	// Remove finger from the blocked ones (grasping will effect it)
	void FreeFinger(ERCGHandLimb Finger);

	// Free all fingers
	void FreeFingers();
	
	// Blocked fingers
	TSet<ERCGHandLimb> BlockedFingers;

private:
	// Grasping state
	EGraspState State;

	// Constraints target
	float Target;

};

