// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "CoreMinimal.h"
#include "Grasp.h"
#include "Animation/SkeletalMeshActor.h"
#include "Components/SphereComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Structs/Finger.h"

#include "Hand.generated.h"

/** Number of hands constants */
enum
{
	NOT_GRASPABLE = 0,
	ONE_HAND_GRASPABLE = 1,
	TWO_HANDS_GRASPABLE = 2
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
	// Tick value for debuging
	float TickValue;

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
	void SwitchGraspStyle();

	// Switch the grasping process
	void SwitchGraspProcess();

	// Fixation grasp via attachment of the object to the hand
	bool TryOneHandFixationGrasp();

	// Fixation grasp of two hands attachment
	bool TryTwoHandsFixationGrasp();

	// Fixation grasp of two hands attachment (triggered by other hand)
	void TwoHandsFixationGraspFromOther();

	// Detach fixation grasp from hand
	bool DetachFixationGrasp();

	// Detach fixation grasp from hand (triggered by the other hand)
	bool DetachTwoHandFixationGraspFromOther();

	// Get possible two hand grasp object
	AStaticMeshActor* GetTwoHandsGraspableObject() const { return TwoHandsGraspableObject; };

	// Check if the two hand grasp is still valid (the hands have not moved away from each other)
	bool IsTwoHandGraspStillValid();

	// Set pointer to other hand, used for two hand fixation grasp
	void SetOtherHand(AHand* InOtherHand);

	void ResetAngularDriveValues(EAngularDriveMode::Type DriveMode, EAngularDriveType DriveType);

protected:
	// Collision component used for attaching grasped objects
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp", meta = (editcondition = "bEnableFixationGrasp"))
		USphereComponent* AttachmentCollision;

	// Maximum mass (kg) of an object that can be attached to the hand
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp", meta = (editcondition = "bEnableFixationGrasp"))
		float MaxAttachMass;

	// Maximum length of an object that can be attached to the hand
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp", meta = (editcondition = "bEnableFixationGrasp"))
		float MaxAttachLength;

	// Flag showing that the hand is ready for a two hands grasp
	bool bReadyForTwoHandsGrasp;

	// Post edit change property callback
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;

	// Check if the object in reach is one-, two-hand(s), or not graspable
	UFUNCTION()
		void OnFixationGraspAreaBeginOverlap(class UPrimitiveComponent* HitComp, class AActor* OtherActor,
			class UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult & SweepResult);

	// Object out or grasping reach, remove as possible grasp object
	UFUNCTION()
		void OnFixationGraspAreaEndOverlap(class UPrimitiveComponent* HitComp, class AActor* OtherActor,
			class UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);

private:
	// Collision component used for attaching grasped objects
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp", meta = (editcondition = "bEnableFixationGrasp"))
		USphereComponent* FixationGraspArea;

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
		float Spring;

	// Damping value to apply to the angular drive (Velocity strength) 
	UPROPERTY(EditAnywhere, Category = "MC|Drive Parameters")
		float Damping;

	// Limit of the force that the angular drive can apply
	UPROPERTY(EditAnywhere, Category = "MC|Drive Parameters")
		float ForceLimit;

	// Limit of the force that the angular drive must apply before changiing to velocity
	UPROPERTY(EditAnywhere, Category = "MC|Drive Parameters")
		float VelocityThreshold;

	// Enable grasping with fixation
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp")
		bool bFixationGraspEnabled;

	// Enable two hand grasping with fixation
	UPROPERTY(EditAnywhere, Category = "MC|Fixation Grasp", meta = (editcondition = "bEnableFixationGrasp"))
		bool bTwoHandsFixationGraspEnabled;

	// Check if object is graspable, return the number of hands (0, 1, 2)
	uint8 CheckObjectGraspableType(AActor* InActor);

	// Hold grasp in the current position
	void MaintainFingerPositions();

	// Setup hand default values
	void SetupHandDefaultValues(EHandType HandType);

	// Setup skeletal mesh default values
	void SetupSkeletalDefaultValues(USkeletalMeshComponent* InSkeletalMeshComponent);

	//	// Callback on collision
	//	UFUNCTION()
	//	void OnFingerHit(UPrimitiveComponent* SelfComp, AActor* OtherActor, UPrimitiveComponent* OtherComp,
	//		FVector NormalImpulse, const FHitResult& Hit);

private:
	// Pointer to objects in reach for grasping
	TArray<AStaticMeshActor*> GraspableObjects;

	TEnumAsByte<EAngularDriveMode::Type> AngularDriveMode;;

	// Objects that are in reach to be grasped by one hand
	TArray<AStaticMeshActor*> OneHandGraspableObjects;

	// Pointer to the grasped object
	AStaticMeshActor* OneHandGraspedObject;

	// Object that is in reach, and is two hand graspable
	AStaticMeshActor* TwoHandsGraspableObject;

	// Pointer to the object grasped by with two hands
	AStaticMeshActor* TwoHandsGraspedObject;

	// Pointer to the other hand (used for two hand fixation grasp)
	AHand* OtherHand;

	// If the hand is mimicking movements in the two hand fixation grasp case (no actual attachment)
	bool bMovementMimickingHand;

	// Movement mimicking relative location from the other hand
	FVector MimickingRelativeLocation;

	// Movement mimicking relative rotation from the other hand
	FQuat MimickingRelativeRotation;

	// Mark that the grasp has been held, avoid reinitializing the finger drivers
	bool bGraspHeld;

	// Grasp
	TSharedPtr<Grasp> GraspPtr;

	// Setup fingers angular drive values
	FORCEINLINE void SetupAngularDriveValues(EAngularDriveMode::Type DriveMode, EAngularDriveType DriveType);

	// Setup finger bones
	FORCEINLINE void SetupBones();
};
