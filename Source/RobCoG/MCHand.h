// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "CoreMinimal.h"
#include "Animation/SkeletalMeshActor.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "MCGrasp.h"
#include "MCHand.generated.h"

/** Enum indicating the hand type */
UENUM(BlueprintType)
enum class EHandType : uint8
{
	Left		UMETA(DisplayName = "Left"),
	Right		UMETA(DisplayName = "Right")
};

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
	Proximal		UMETA(DisplayName = "Proximal"),
	Intermediate	UMETA(DisplayName = "Intermediate"),
	Distal			UMETA(DisplayName = "Distal")
};

/**
*
*/
USTRUCT()
struct FFinger
{
	GENERATED_USTRUCT_BODY()

	// Default constructor
	FFinger()
	{}

	// Finger type
	UPROPERTY(EditAnywhere, Category = "Finger")
	EFingerType FingerType;

	// Map of finger part to skeletal bone name
	UPROPERTY(EditAnywhere, Category = "Finger")
	TMap<EFingerPart, FString> FingerPartToBoneName;

	// Map of finger part to constraint
	TMap<EFingerPart, FConstraintInstance*> FingerPartToConstraint;

	// Set finger part to constraint from bone names
	bool SetFingerPartsConstraints(TArray<FConstraintInstance*>& Constraints)
	{
		// Iterate the bone names
		for (const auto& MapItr : FingerPartToBoneName)
		{
			// Check if bone name match with the constraint joint name
			FConstraintInstance* FingerPartConstraint = *Constraints.FindByPredicate(
				[&MapItr](FConstraintInstance* ConstrInst)
				{return ConstrInst->JointName.ToString() == MapItr.Value;}
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

	// Set constraint drive mode
	void SetFingerDriveMode(
		const EAngularDriveMode::Type DriveMode,
		const float InSpring,
		const float InDamping,
		const float InForceLimit)
	{
		for (const auto& MapItr : FingerPartToConstraint)
		{
			MapItr.Value->SetAngularDriveMode(DriveMode);
			if (DriveMode == EAngularDriveMode::TwistAndSwing)
			{
				MapItr.Value->SetOrientationDriveTwistAndSwing(true, true);
			}
			else if (DriveMode == EAngularDriveMode::SLERP)
			{
				MapItr.Value->SetOrientationDriveSLERP(true);
			}			
			MapItr.Value->SetAngularDriveParams(InSpring, InDamping, InForceLimit);
		}
	}
};

/**
 * 
 */
UCLASS()
class ROBCOG_API AMCHand : public ASkeletalMeshActor
{
	GENERATED_BODY()
	
public:
	// Sets default values for this actor
	AMCHand();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	// Update the grasp //TODO state, power, step
	void UpdateGrasp(const float Goal);

	// Attach to hand
	void AttachToHand();

	// Detach from hand
	void DetachFromHand();
	
protected:
	// Post edit change property callback
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent);

	//	// Callback on collision
	//	UFUNCTION()
	//	void OnFingerHit(UPrimitiveComponent* SelfComp, AActor* OtherActor, UPrimitiveComponent* OtherComp,
	//		FVector NormalImpulse, const FHitResult& Hit);


	// Hand type
	UPROPERTY(EditAnywhere, Category = "MC|Hand")
	EHandType HandType;

	// Thumb finger skeletal bone names
	UPROPERTY(EditAnywhere, Category = "MC|Hand")
	FFinger Thumb;

	// Index finger skeletal bone names
	UPROPERTY(EditAnywhere, Category = "MC|Hand")
	FFinger Index;

	// Middle finger skeletal bone names
	UPROPERTY(EditAnywhere, Category = "MC|Hand")
	FFinger Middle;

	// Ring finger skeletal bone names
	UPROPERTY(EditAnywhere, Category = "MC|Hand")
	FFinger Ring;

	// Pinky finger skeletal bone names
	UPROPERTY(EditAnywhere, Category = "MC|Hand")
	FFinger Pinky;

	// Spring value to apply to the angular drive (Position strength)
	UPROPERTY(EditAnywhere, Category = "MC|Drive Parameters")
	float Spring;

	// Damping value to apply to the angular drive (Velocity strength) 
	UPROPERTY(EditAnywhere, Category = "MC|Drive Parameters")
	float Damping;

	// Limit of the force that the angular drive can apply
	UPROPERTY(EditAnywhere, Category = "MC|Drive Parameters")
	float ForceLimit;
	
private:
	// Setup hand default values
	FORCEINLINE void SetupHandDefaultValues(EHandType HandType);

	// Setup skeletal mesh default values
	FORCEINLINE void SetupSkeletalDefaultValues(USkeletalMeshComponent* InSkeletalMeshComponent);

	// Setup fingers angular drive values
	FORCEINLINE void SetupAngularDriveValues(EAngularDriveMode::Type DriveMode);

	//// Grasp
	//MCGrasp Grasp;
};
