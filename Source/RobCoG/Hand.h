// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "CoreMinimal.h"
#include "Animation/SkeletalMeshActor.h"
#include "Components/SphereComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Finger.h"
#include "Hand.generated.h"

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

	// Attach to hand
	void AttachToHand();

	// Detach from hand
	void DetachFromHand();
	
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

	// Collision component used for attaching grasped objects
	UPROPERTY(EditAnywhere, Category = "MC|Hand")
	USphereComponent* AttachmentCollision;

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

	// Check if object is graspable
	FORCEINLINE bool IsGraspable(AActor* InActor);
	
	// Pointer to objects in reach for grasping
	TArray<AStaticMeshActor*> GraspableObjects;

	// Pointer to the grasped object
	AStaticMeshActor* GraspedObject;

	//// Grasp
	//MCGrasp Grasp;
};
