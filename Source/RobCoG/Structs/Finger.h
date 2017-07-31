// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "CoreMinimal.h"
#include "HandOrientation.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "PhysicsEngine/BodySetup.h"

#include "Finger.generated.h"

/** Enum indicating the finger type */
UENUM(BlueprintType)
enum class EFingerType : uint8
{
	Thumb		UMETA(DisplayName = "Thumb"),
	Index		UMETA(DisplayName = "Index"),
	Middle		UMETA(DisplayName = "Middle"),
	Ring		UMETA(DisplayName = "Ring"),
	Pinky		UMETA(DisplayName = "Pinky")
};

/** 
* Enum indicating the finger parts 
* https://en.wikipedia.org/wiki/Phalanx_bone
*/
UENUM(BlueprintType)
enum class EFingerPart : uint8
{
	Metacarpal		UMETA(DisplayName = "Metacarpal"),
	Proximal		UMETA(DisplayName = "ProximalConstraint"),
	Intermediate	UMETA(DisplayName = "IntermediateConstraint"),
	Distal			UMETA(DisplayName = "DistalConstraint")
};

UENUM(BlueprintType)
enum class EAngularDriveType : uint8
{
	Orientation		UMETA(DisplayName = "Orientation"),
	Velocity		UMETA(DisplayName = "Velocity"),
};

/**
*
*/
USTRUCT()
struct FFinger
{
	GENERATED_USTRUCT_BODY()

	// Default constructor
	FFinger() :
		FingerType(EFingerType::Thumb)
	{}

	// Finger type
	UPROPERTY(EditAnywhere, Category = "Finger")
	EFingerType FingerType;

	// Map of finger part to skeletal bone name
	UPROPERTY(EditAnywhere, Category = "Finger")
		TMap<EFingerPart, FString> FingerPartToBoneName;

	// Map of finger part to skeletal bone 
	TMap<EFingerPart, FBodyInstance*> FingerPartToBone;

	// Map of finger part to constraint
	TMap<EFingerPart, FConstraintInstance*> FingerPartToConstraint;

	// Set finger part to constraint from bone names
	bool SetFingerPartsConstraints(TArray<FConstraintInstance*>& Constraints)
	{
		// Iterate the bone names
		for (const auto& MapItr : FingerPartToBoneName)
		{
			// TODO null ptr exception if the names do not match
			// Check if bone name match with the constraint joint name
			FConstraintInstance* FingerPartConstraint = *Constraints.FindByPredicate(
				[&MapItr](FConstraintInstance* ConstrInst)
			{return ConstrInst->JointName.ToString() == MapItr.Value; }
			);
			// If constraint has been found, add to map
			if (FingerPartConstraint)
			{
				FingerPartToConstraint.Add(MapItr.Key, FingerPartConstraint);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("Finger: Bone %s has no constraint!"), *MapItr.Value);
				return false;
			}
		}
		return true;
	}

	// Set finger part to constraint from bone names
	bool SetFingerPartsBones(TArray<FBodyInstance*>& Bodies)
	{
		// Iterate the bone names
		for (const auto& MapItr : FingerPartToBoneName)
		{
			// TODO null ptr exception if the names do not match
			// Check if bone name match with the constraint joint name
			FBodyInstance* FingerPartBone = *Bodies.FindByPredicate(
				[&MapItr](FBodyInstance* BodyInst)
			{return BodyInst->BodySetup->BoneName.ToString() == MapItr.Value; }
			);
			// If constraint has been found, add to map
			if (FingerPartBone)
			{
				FingerPartToBone.Add(MapItr.Key, FingerPartBone);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("Finger: Bone %s has no bone!"), *MapItr.Value);
				return false;
			}
		}
		return true;
	}

	// Set constraint drive mode
	void SetFingerDriveMode(
		const EAngularDriveMode::Type DriveMode,
		const EAngularDriveType DriveType,
		const float InSpring,
		const float InDamping,
		const float InForceLimit)
	{
		for (const auto& MapItr : FingerPartToConstraint)
		{
			MapItr.Value->SetAngularDriveMode(DriveMode);
			if (DriveMode == EAngularDriveMode::TwistAndSwing)
			{
				if(DriveType == EAngularDriveType::Orientation)
				{
					MapItr.Value->SetOrientationDriveSLERP(false);
					MapItr.Value->SetAngularVelocityDriveSLERP(false);
					MapItr.Value->SetAngularVelocityDriveTwistAndSwing(false, false);
					MapItr.Value->SetOrientationDriveTwistAndSwing(true, true);
				}
				else if(DriveType == EAngularDriveType::Velocity)
				{
					MapItr.Value->SetOrientationDriveSLERP(false);
					MapItr.Value->SetAngularVelocityDriveSLERP(false);
					MapItr.Value->SetOrientationDriveTwistAndSwing(false, false);
					MapItr.Value->SetAngularVelocityDriveTwistAndSwing(true, true);
				}
			}
			else if (DriveMode == EAngularDriveMode::SLERP)
			{
				if (DriveType == EAngularDriveType::Orientation)
				{
					MapItr.Value->SetOrientationDriveTwistAndSwing(false, false);
					MapItr.Value->SetAngularVelocityDriveTwistAndSwing(false, false);
					MapItr.Value->SetAngularVelocityDriveSLERP(false);
					MapItr.Value->SetOrientationDriveSLERP(true);
				}
				else if (DriveType == EAngularDriveType::Velocity)
				{
					MapItr.Value->SetOrientationDriveTwistAndSwing(false, false);
					MapItr.Value->SetAngularVelocityDriveTwistAndSwing(false, false);
					MapItr.Value->SetOrientationDriveSLERP(false);
					MapItr.Value->SetAngularVelocityDriveSLERP(true);
				}
			}
			MapItr.Value->SetAngularDriveParams(InSpring, InDamping, InForceLimit);
		}
	}

	FFingerOrientation GetCurrentFingerOrientation()
	{

		FFingerOrientation FingerOrientation;

		FingerOrientation.DistalOrientation.Orientation = FingerPartToConstraint[EFingerPart::Distal]->AngularRotationOffset;
		FingerOrientation.IntermediateOrientation.Orientation = FingerPartToConstraint[EFingerPart::Intermediate]->AngularRotationOffset;
		FingerOrientation.ProximalOrientation.Orientation = FingerPartToConstraint[EFingerPart::Proximal]->AngularRotationOffset;
		//FingerOrientation.MetacarpalOrientation.Orientation = FingerPartToConstraint[EFingerPart::Metacarpal]->AngularRotationOffset;

		return FingerOrientation;
	}

};