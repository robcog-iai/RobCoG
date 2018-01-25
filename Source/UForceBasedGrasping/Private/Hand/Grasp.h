// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "Enums/GraspType.h"
#include "Utilities/HandInformationParser.h"
#include "Structs/Finger.h"
#include "Structs/HandOrientation.h"
#include "Structs/HandVelocity.h"
#include "Utilities/GraspingGame.h"

class AHand;

/** Enum indicating the grasp status */
UENUM(BlueprintType)
enum class EGraspStatus : uint8
{
	Started		UMETA(DisplayName = "Started"),
	Orientation	UMETA(DisplayName = "Orientation"),
	Velocity	UMETA(DisplayName = "Velocity"),
	Stopped		UMETA(DisplayName = "Stopped"),
};

/** Enum indicating the comparison type */
UENUM(BlueprintType)
enum class EComparison : uint8
{
	Bigger	UMETA(DisplayName = "Bigger"),
	Smaller	UMETA(DisplayName = "Smaller"),
	Equals	UMETA(DisplayName = "Equals"),
};

/**
 * This class deals with the grasping of a hand
 */
class UFORCEBASEDGRASPING_API Grasp
{
public:
	// Constructor
	Grasp();

	// Destructor
	~Grasp();

	// Updates the Grasp Orientation of the gven Hand
	void UpdateGrasp(const float Alpha, const float VelocityThreshold, AHand * const Hand);

	// Switches the Grasping Type
	void SwitchGraspType(const AHand * const Hand, EGraspType GraspType);

	// Switches the Grasping Type
	void SwitchToPreviousGraspType(const AHand * const Hand, FText & GraspTypeName);

	// Switches the Grasping Type
	void SwitchToNextGraspType(const AHand * const Hand, FText & GraspTypeName);

	// Switches the Grasping Process
	void SwitchGraspProcess(AHand * const Hand, const float InSpring, const float InDamping, const float ForceLimit);

	// Print The Fore 
	void PrintHandInfo(const AHand * const Hand) const;
	
	// The current status of the grasp process
	EGraspStatus GraspStatus;

	// The Current Grasp type
	EGraspType CurrentGraspType;

private:
	// The initial HandOrientation
	FHandOrientation InitialHandOrientation;

	// The closed HandOrientation
	FHandOrientation ClosedHandOrientation;

	// The HandVelocity after grasping
	FHandVelocity HandVelocity;

	// The HandOrientation of the last Tick
	FHandOrientation LastHandOrientation;

	// Current Grasp Process
	TEnumAsByte<EAngularDriveMode::Type> CurrentAngularDriveMode;

	// Parser of the ini files
	TSharedPtr<HandInformationParser> HandInformationParserPtr;

	// Linear Interpolation between the given InitialHandOrientation and the given ClosedHandOrientation from 0-1
	void LerpHandOrientation(FHandOrientation & TargetHandOrientation, const FHandOrientation & InitialHandOrientation, const FHandOrientation & ClosedHandOrientation, const float Alpha);
	
	// Linear Interpolation between the given InitialFingerOrientation and the given ClosedFingerOrientation from 0-1
	void LerpFingerOrientation(FFingerOrientation & TargetFingerOrientation, const FFingerOrientation & InitialFingerOrientation, const FFingerOrientation & ClosedFingerOrientation, const float Alpha);

	// Moves the given Hand to the given HandOrientation
	void DriveToHandOrientationTarget(const FHandOrientation & HandOrientation, const AHand* const Hand);
	
	// Moves the given Finger to the given FingerOrientation
	void DriveToFingerOrientationTarget(const FFingerOrientation & FingerOrientation, const FFinger & Finger);

	// Moves the given Hand to the given HandOrientation
	void DriveToHandVelocityTarget(const FHandVelocity & HandVelocity, const AHand * const Hand);
	
	// Moves the given Finger to the given FingerOrientation
	void DriveToFingerVelocityTarget(const FFingerVelocity & FingerVelocity, const FFinger & Finger);

	// Drives the given Hand to the InitialHandOrientation
	void DriveToInitialOrientation(const AHand * const Hand);

	// Checks the Distal Velocity to be higher,lower,equals the threshold
	bool CheckDistalVelocity(const AHand* const Hand, const float VelocityThreshold, const EComparison Comparison);

	// Locks the angular rotation of a Constraint
	void LockConstraint(FConstraintInstance* Constraint);

	//Unlocks the angular rotation of a constraint
	void UnlockConstraint(FConstraintInstance* Constraint);
};
