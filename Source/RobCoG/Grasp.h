// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "GraspType.h"
#include "HandOrientationParser.h"
#include "Structs/Finger.h"
#include "Structs/HandOrientation.h"

class AHand;

/**
 * 
 */
class ROBCOG_API Grasp
{
public:
	Grasp();
	~Grasp();

	// Sets the InitialHandOrientation
	void SetInitialHandOrientation(FHandOrientation InitialHandOrientation);
	// Sets the ClosedHandOrientation
	void SetClosedHandOrientation(FHandOrientation ClosedHandOrientation);

	// Moves the given Hand to the given HandOrientation
	void DriveToHandOrientation(const FHandOrientation & HandOrientation,const AHand* const Hand);
	// Moves the given Finger to the given FingerOrientation
	void DriveToFingerOrientation(const FFingerOrientation & FingerOrientation, const FFinger & Finger);

	// Drives the given Hand to the InitialHandOrientation
	void DriveToInitialOrientation(const AHand * const Hand);

	// Updates the Grasp Orientation of the gven Hand
	void UpdateGrasp(const float Alpha,const AHand * const Hand);

	void SwitchGrasp(const AHand * const Hand);

private:
	// The initial HandOrientation
	FHandOrientation InitialHandOrientation;
	// The closed HandOrientation
	FHandOrientation ClosedHandOrientation;
	// The Current Grasp Position
	EGraspType CurrentGraspType;

	// Parser of the ini files
	TSharedPtr<HandOrientationParser> HandOrientationParserPtr;
	
	// Linear Interpolation between the given InitialHandOrientation and the given ClosedHandOrientation from 0-1
	FHandOrientation LerpHandOrientation(FHandOrientation InitialHandOrientation, FHandOrientation ClosedHandOrientation, float Alpha);
	// Linear Interpolation between the given InitialFingerOrientation and the given ClosedFingerOrientation from 0-1
	FFingerOrientation LerpFingerOrientation(FFingerOrientation InitialFingerOrientation, FFingerOrientation ClosedFingerOrientation, float Alpha);

};
