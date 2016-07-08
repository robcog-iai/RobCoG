// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "RCGGrasp.h"

/**
 * Grasp type testing, closing and opening every finger one at a time
 */
class ROCOG_API FRCGGraspPiano : public FRCGGrasp
{
public:
	// Constructor
	FRCGGraspPiano();

	// Constructor with access to the fingers and the constraints
	FRCGGraspPiano(TMultiMap<ERCGHandLimb, FConstraintInstance*>& FingerTypeToConstrs);
	
	// Destructor
	~FRCGGraspPiano();
	
	// Update the grasping (open/close fingers with the given step)
	virtual void Update(const float Step) override;

private:
	// Set active finger, return true if succesfull
	bool SetActiveFinger(uint8 ActiveFingerIdx);

	// Grasping state
	ERCGGraspState State;

	// Fingers targets map
	TMap<ERCGHandLimb, float> FingerToTargetMap;

	// Finger states map
	TMap<ERCGHandLimb, ERCGGraspState> FingerToStateMap;

	// Array of the finger types //TODO use list
	TArray<ERCGHandLimb> FingerTypesArr;

	// Current active finger index
	uint8 ActiveFingerIdx;

	// Current active finger
	ERCGHandLimb ActiveFinger;
};
