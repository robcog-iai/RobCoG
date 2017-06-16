// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "CoreMinimal.h"
#include "Animation/SkeletalMeshActor.h"
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

	// Constructor
	FFinger() : bInCollision(false)
	{
	}

	// Finger type
	UPROPERTY(EditAnywhere, Category = "Finger")
	EFingerType FingerType;

	// Map of finger part to skeletal bone name
	UPROPERTY(EditAnywhere, Category = "Finger")
	TMap<EFingerPart, FString> FingerPartBoneNames;

	// Finger in collision
	bool bInCollision;
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

	// Grasp
	MCGrasp Grasp;

//	// Fingers type to constraints Map
//	TMultiMap<ERHandLimb, FConstraintInstance*> FingerTypeToConstrs;
//
//	// Fingers type name to finger body (collision)
//	TMap<FName, FBodyInstance*> FingerBoneNameToBody;
//
//	// Bone name to finger type
//	TMap<FName, ERHandLimb> BoneNameToFingerTypeMap;
//
//	// Finger hit events enable flag
//	bool bFingerHitEvents;
//
//	// Store a map of components in contact with fingers,
//	// used for reasoning on what objects to attach to the hand
//	// (e.g. Cup : Index, Middle, Pinky, Thumb; Table : Palm; -> Attach Cup )
//	TMultiMap<AActor*, ERHandLimb> HitActorToFingerMMap;
//
//	// Currently grasped component (to enable/disable gravity when grasped)
//	UStaticMeshComponent* GraspedComponent;
};
