// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "RUtils.h"

/**
 * Base class for grasping, opens and closes the hand with a given step size
 */
class ROBCOG_API FRGrasp
{
public:
	// Constructor
	FRGrasp();

	// Constructor with access to the fingers and the constraints
	FRGrasp(TMultiMap<ERHandLimb, FConstraintInstance*>& /*FingerTypeToConstrs*/);

	// Destructor
	~FRGrasp();

	// Update the grasping (open/close fingers with the given step)
	virtual void Update(const float Step);

	// Set the state of the grasp
	void SetState(const ERGraspState State);

	// Get the state of the grasp
	ERGraspState GetState();

	// Add finger to the blocked ones (grasping will have no effect on it)
	void BlockFinger(const ERHandLimb Finger);

	// Remove finger from the blocked ones (grasping will effect it)
	void FreeFinger(const ERHandLimb Finger);

	// Free all fingers
	void FreeFingers();

	// Check if finger is free
	bool IsFingerBlocked(const ERHandLimb Finger);

	// Return the grasp state as string
	FString GetStateAsString();

protected:
	// Grasping state
	ERGraspState GraspState;

	// Finger types and their constraints as multi map (e.g Index : index_01_l, index_02_l)
	TMultiMap<ERHandLimb, FConstraintInstance*> FingerTypeToConstraintsMMap;

	// Map hand fingers to their target
	TMap<ERHandLimb, float> FingerToTargetMap;

	// Blocked fingers
	TArray<ERHandLimb> BlockedFingers;

private:
	// Store the value of the previous step
	float PrevStep;
};

