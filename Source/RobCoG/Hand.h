// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "CoreMinimal.h"
#include "Grasp.h"
#include "Animation/SkeletalMeshActor.h"
#include "Components/SphereComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Structs/Finger.h"
#include "HandOrientationParser.h"

#include "Hand.generated.h"

/** Number of hands constants */
enum
{ 
	HAND_NONE = 0,
	HAND_ONE = 1,
	HAND_TWO = 2
};

/** Enum indicating the hand type */
UENUM(BlueprintType)
enum class EHandType : uint8
{
	Left		UMETA(DisplayName = "Left"),
	Right		UMETA(DisplayName = "Right")
};

/**
 * 
 */
UCLASS()
class ROBCOG_API AHand : public ASkeletalMeshActor
{
	GENERATED_BODY()
	
public:
	// Sets default values for this actor
	AHand();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	// Update the grasp //TODO state, power, step
	void UpdateGrasp(const float Goal);

	// Update the grasp with the mannequin hand
	void UpdateGrasp2(const float Alpha);

	// Switch the grasping style
	void SwitchGrasp();

	// Attach to hand
	bool AttachToHand();

	// Detach from hand
	bool DetachFromHand();

	// Attach to hand
	bool TwoHandAttach();

	// Get possible two hand grasp object
	AStaticMeshActor* GetPossibleTwoHandGraspObject() const { return PossibleTwoHandGraspObject; };
	
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

protected:
	// Post edit change property callback
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent);

	// Object in reach for grasping
	UFUNCTION()
	void OnAttachmentCollisionBeginOverlap(class UPrimitiveComponent* HitComp, class AActor* OtherActor,
		class UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult & SweepResult);

	// Object out of reach for grasping
	UFUNCTION()
	void OnAttachmentCollisionEndOverlap(class UPrimitiveComponent* HitComp, class AActor* OtherActor,
		class UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);

	// Enable grasping with fixation
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp")
	bool bEnableFixationGrasp;

	// Collision component used for attaching grasped objects
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp", meta = (editcondition = "bEnableFixationGrasp"))
	USphereComponent* AttachmentCollision;

	// Maximum mass (kg) of an object that can be attached to the hand
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp", meta = (editcondition = "bEnableFixationGrasp"), meta = (ClampMin = 0))
	float OneHandFixationMaximumMass;

	// Maximum length (cm) of an object that can be attached to the hand
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp", meta = (editcondition = "bEnableFixationGrasp"), meta = (ClampMin = 0))
	float OneHandFixationMaximumLength;

	// Maximum mass (kg) of an object that can be attached to two hands
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp", meta = (editcondition = "bEnableFixationGrasp"), meta = (ClampMin = 0))
	float TwoHandsFixationMaximumMass;

	// Maximum length (cm) of an object that can be attached to two hands
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp", meta = (editcondition = "bEnableFixationGrasp"), meta = (ClampMin = 0))
	float TwoHandsFixationMaximumLength;

	// Spring value to apply to the angular drive (Position strength)
	UPROPERTY(EditAnywhere, Category = "MC|Drive Parameters")
	TEnumAsByte<EAngularDriveMode::Type> AngularDriveMode;;

	// Spring value to apply to the angular drive (Position strength)
	UPROPERTY(EditAnywhere, Category = "MC|Drive Parameters", meta = (ClampMin = 0))
	float Spring;

	// Damping value to apply to the angular drive (Velocity strength) 
	UPROPERTY(EditAnywhere, Category = "MC|Drive Parameters", meta = (ClampMin = 0))
	float Damping;

	// Limit of the force that the angular drive can apply
	UPROPERTY(EditAnywhere, Category = "MC|Drive Parameters", meta = (ClampMin = 0))
	float ForceLimit;

private:
	// Check if object is graspable, return the number of hands (0, 1, 2)
	uint8 IsGraspable(AActor* InActor);

	// Hold grasp in the current position
	void HoldGrasp();

	// Setup hand default values
	FORCEINLINE void SetupHandDefaultValues(EHandType HandType);

	// Setup skeletal mesh default values
	FORCEINLINE void SetupSkeletalDefaultValues(USkeletalMeshComponent* InSkeletalMeshComponent);

	// Setup fingers angular drive values
	FORCEINLINE void SetupAngularDriveValues(EAngularDriveMode::Type DriveMode);

	// Objects that are in reach to be grasped by one hand
	TArray<AStaticMeshActor*> OneHandGraspableObjects;

	// Objects that are in reach to be grasped by two hands
	TArray<AStaticMeshActor*> TwoHandsGraspableObjects;

	// Pointer to the grasped object
	AStaticMeshActor* GraspedObject;

	// Pointer to the object graspable with two hands
	AStaticMeshActor* PossibleTwoHandGraspObject;

	// Mark that the grasp has been held, avoid reinitializing the finger drivers
	bool bGraspHeld;

	// Two hand grasp active
	bool bTwoHandGraspActive;

	// Grasp
	TSharedPtr<Grasp> GraspPtr;
};
