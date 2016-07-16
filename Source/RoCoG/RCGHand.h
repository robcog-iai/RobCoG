// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Animation/SkeletalMeshActor.h"
#include "MotionControllerComponent.h"
#include "RCGUtils.h"
#include "RCGPid3d.h"
#include "RCGGrasp.h"
#include "RCGHand.generated.h"

/**
*
*/
UCLASS()
class ROCOG_API ARCGHand : public ASkeletalMeshActor
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	ARCGHand();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	// Hand type
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	EControllerHand HandType;

	// Body name to apply forces on
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	FName ControlBoneName;

	// PID controller proportional argument
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	float PGain;

	// PID controller integral argument
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	float IGain;

	// PID controller derivative argument
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	float DGain;

	// PID controller maximum output
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	float PIDMaxOutput;

	// PID controller minimum output
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	float PIDMinOutput;

	// Hand rotation controller output strength (multiply output velocity)
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	float RotOutStrength;

	// Finger types of the hand (make sure the skeletal bone name is part of the finger name)
	UPROPERTY(EditAnywhere, Category = "Joint Controller")
	TArray<ERCGHandLimb> FingerTypes;

	// Finger collision bone name (used for collision detection)
	UPROPERTY(EditAnywhere, Category = "Joint Controller")
	TArray<FName> CollisionBoneNames;

	// Spring value to apply to the angular drive (Position strength)
	UPROPERTY(EditAnywhere, Category = "Joint Controller")
	float Spring;

	// Damping value to apply to the angular drive (Velocity strength) 
	UPROPERTY(EditAnywhere, Category = "Joint Controller")
	float Damping;

	// Limit of the force that the angular drive can apply
	UPROPERTY(EditAnywhere, Category = "Joint Controller")
	float ForceLimit;

	// Initial velocity
	UPROPERTY(EditAnywhere, Category = "Joint Controller")
	float Velocity;

	// Fixating grasp constraint instance
	UPROPERTY(EditAnywhere, Category = "Joint Controller")
	FConstraintInstance FixatingGraspConstraintInstance;

	// Handles closing the hand
	UFUNCTION()
	void CloseHand(const float Val);

	// Attach grasped object to hand
	UFUNCTION()
	void AttachToHand();

	// Handles opening the left hand
	UFUNCTION()
	void OpenHand(const float Val);

	// Callback on collision
	UFUNCTION()
	void OnFingerHit(UPrimitiveComponent* SelfComp, AActor* OtherActor, UPrimitiveComponent* OtherComp,
		FVector NormalImpulse, const FHitResult& Hit);

private:
	// Motion controller component for the hand to follow
	UMotionControllerComponent* MCComponent;

	// 3D PID controller for all axis
	FRCGPid3d HandPID3D;
	
	// Current location of the control body
	FVector CurrLoc;

	// Current rotation of the control body
	FQuat CurrQuat;

	// Control body
	FBodyInstance* ControlBody;

	// Fingers type to constraints Map
	TMultiMap<ERCGHandLimb, FConstraintInstance*> FingerTypeToConstrs;

	// Fingers type name to finger body (collision)
	TMap<FName, FBodyInstance*> FingerBoneNameToBody;

	// Grasp base class
	FRCGGrasp* Grasp;

	// Bone name to finger type
	TMap<FName, ERCGHandLimb> BoneNameToFingerTypeMap;

	// Finger hit events enable flag
	bool bFingerHitEvents;

	// Store a map of components in contact with fingers,
	// used for reasoning on what objects to attach to the hand
	// (e.g. Cup : Index, Middle, Pinky, Thumb; Table : Palm; -> Attach Cup )
	TMultiMap<AActor*, ERCGHandLimb> HitActorToFingerMMap;

	// Grasp physics constraint component (fixating grasp case)
	UPhysicsConstraintComponent* GraspFixatingConstraint;

	// Dummy static mesh actor (workaround for the contraint attachment directly to the skeleton issue)
	AStaticMeshActor* DummyStaticMeshActor;

	// Dummy static mesh (workaround for the contraint attachment directly to the skeleton issue)
	UStaticMeshComponent* DummyStaticMesh;

	// Currently grasped component (to enable/disable gravity when grasped)
	UStaticMeshComponent* GraspedComponent;

	// Player controller to apply force feedback and enable input
	APlayerController* PC;
};
