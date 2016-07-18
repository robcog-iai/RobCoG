// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "RGrasp.h"

/**
 * Grasp type testing, closing and opening every finger one at a time
 */
class ROBCOG_API FRGraspPiano : public FRGrasp
{
public:
	// Constructor
	FRGraspPiano();

	// Constructor with access to the fingers and the constraints
	FRGraspPiano(TMultiMap<ERHandLimb, FConstraintInstance*>& FingerTypeToConstrs);
	
	// Destructor
	~FRGraspPiano();
	
	// Update the grasping (open/close fingers with the given step)
	virtual void Update(const float Step) override;

private:
	// Set active finger, return true if succesfull
	bool SetActiveFinger(uint8 ActiveFingerIdx);

	// Grasping state
	ERGraspState State;

	// Fingers targets map
	TMap<ERHandLimb, float> FingerToTargetMap;

	// Finger states map
	TMap<ERHandLimb, ERGraspState> FingerToStateMap;

	// Array of the finger types //TODO use list
	TArray<ERHandLimb> FingerTypesArr;

	// Current active finger index
	uint8 ActiveFingerIdx;

	// Current active finger
	ERHandLimb ActiveFinger;
};
