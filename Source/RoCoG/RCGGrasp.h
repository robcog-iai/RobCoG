// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "RCGUtils.h"



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

	// Set the state of the grasp
	void SetState(const ERCGGraspState State);

	// Get the state of the grasp
	ERCGGraspState GetState();

	// Add finger to the blocked ones (grasping will have no effect on it)
	void BlockFinger(const ERCGHandLimb Finger);

	// Remove finger from the blocked ones (grasping will effect it)
	void FreeFinger(const ERCGHandLimb Finger);

	// Free all fingers
	void FreeFingers();

	// Check if finger is free
	bool IsFingerBlocked(const ERCGHandLimb Finger);

	// Return the grasp state as string
	FString GetStateAsString();

protected:
	// Grasping state
	ERCGGraspState GraspState;

	// Finger types and their constraints as multi map (e.g Index : index_01_l, index_02_l)
	TMultiMap<ERCGHandLimb, FConstraintInstance*> FingerTypeToConstraintsMMap;

	// Map hand fingers to their target
	TMap<ERCGHandLimb, float> FingerToTargetMap;

	// Blocked fingers
	TArray<ERCGHandLimb> BlockedFingers;

private:
	// Store the value of the previous step
	float PrevStep;
};

