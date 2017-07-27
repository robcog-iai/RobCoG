// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "Enums/GraspType.h"
#include "Utilities/HandOrientationParser.h"
#include "Structs/Finger.h"
#include "Structs/HandOrientation.h"

class AHand;

/** Enum indicating the hand type */
UENUM(BlueprintType)
enum class EGraspProcess : uint8
{
	SLERP_Orientation			UMETA(DisplayName = "SLERP_Orientation"), 
	SLERP_Velocity				UMETA(DisplayName = "SLERP_Velocity"),
	TwistAndSwing_Orientation	UMETA(DisplayName = "TwistAndSwing_Orientation"),
	TwistAndSwing_Velocity		UMETA(DisplayName = "TwistAndSwing_Velocity"),
};

/** Enum indicating the hand type */
UENUM(BlueprintType)
enum class EGraspStatus : uint8
{
	Started			UMETA(DisplayName = "Started"),
	Orientation		UMETA(DisplayName = "Orientation"),
	Velocity		UMETA(DisplayName = "Velocity"),
	Stopped			UMETA(DisplayName = "Stopped"),
};


/**
 * This class deals with the grasping of a hand
 */
class ROBCOG_API Grasp
{
public:
	Grasp();
	~Grasp();

	// Sets the InitialHandOrientation
	void SetInitialHandOrientation(const FHandOrientation & InitialHandOrientation);
	// Sets the ClosedHandOrientation
	void SetClosedHandOrientation(const FHandOrientation & ClosedHandOrientation);

	// Moves the given Hand to the given HandOrientation
	void DriveToHandOrientationTarget(const FHandOrientation & HandOrientation, const AHand* const Hand);
	// Moves the given Finger to the given FingerOrientation
	void DriveToFingerOrientationTarget(const FFingerOrientation & FingerOrientation, const FFinger & Finger);

	// Moves the given Hand to the given HandOrientation
	void DriveToHandVelocityTarget(const FHandOrientation & HandOrientation, const AHand * const Hand);
	// Moves the given Finger to the given FingerOrientation
	void DriveToFingerVelocityTarget(const FFingerOrientation & FingerOrientation, const FFinger & Finger);

	// Drives the given Hand to the InitialHandOrientation
	void DriveToInitialOrientation(const AHand * const Hand);

	// Updates the Grasp Orientation of the gven Hand
	void UpdateGrasp(const float Alpha, const float VelocityThreshold, AHand * const Hand);

	// Switches the Grasping Style
	void SwitchGraspStyle(const AHand * const Hand);

	// Switches the Grasping Process
	void SwitchGraspProcess(AHand * const Hand, const float InSpring, const float InDamping, const float ForceLimit);

	// Print The Fore 
	void PrintHandInfo(const AHand * const Hand);


private:
	// The current status of the grasp process
	EGraspStatus GraspStatus;
	// The initial HandOrientation
	FHandOrientation InitialHandOrientation;
	// The closed HandOrientation
	FHandOrientation ClosedHandOrientation;
	// The HandOrientation of the last Tick
	FHandOrientation LastHandOrientation;
	// The Current Grasp Position
	EGraspType CurrentGraspType;
	// Current Grasp Process
	EGraspProcess CurrentGraspProcess;

	// Parser of the ini files
	TSharedPtr<HandOrientationParser> HandOrientationParserPtr;

	// Linear Interpolation between the given InitialHandOrientation and the given ClosedHandOrientation from 0-1
	FHandOrientation LerpHandOrientation(const FHandOrientation & InitialHandOrientation, const FHandOrientation & ClosedHandOrientation, const float Alpha);
	// Linear Interpolation between the given InitialFingerOrientation and the given ClosedFingerOrientation from 0-1
	FFingerOrientation LerpFingerOrientation(const FFingerOrientation & InitialFingerOrientation, const FFingerOrientation & ClosedFingerOrientation, const float Alpha);

	bool CheckDistalVelocity(const AHand* const Hand, const float VelocityThreshold);
	bool ForceOfAllConstraintsSmaler(const AHand* const Hand, const float ForceThreshold);
	bool ForceOfAllFingerConstraintsSmaler(const FFinger & Finger, const float ForceThreshold);
	bool ForceOfAllConstraintsBigger(const AHand* const Hand, const float ForceThreshold);
	bool ForceOfAllFingerConstraintsBigger(const FFinger & Finger, const float ForceThreshold);
};
