// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "Camera/CameraComponent.h"
#include "Animation/SkeletalMeshActor.h"
#include "MotionControllerComponent.h"
#include "PIDController3D.h"
#include "Hand.h"
#include "MCCharacter.generated.h"

UCLASS()
class ROBCOG_API AMCCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	AMCCharacter();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

protected:
	// Left hand skeletal mesh
	UPROPERTY(EditAnywhere, Category = "MC|Hands")
	ASkeletalMeshActor* LeftSkelActor;

	// Right hand skeletal mesh
	UPROPERTY(EditAnywhere, Category = "MC|Hands")
	ASkeletalMeshActor* RightSkelActor;
	
	// Flag to apply rotation offset to hands
	UPROPERTY(EditAnywhere, Category = "MC|Hands")
	bool bUseHandsInitialRotationAsOffset;

	// Show motion controller pose arrows
	UPROPERTY(EditAnywhere, Category = "MC|Hands")
	bool bShowTargetArrows;

	// If hands enabled, try fixation grasp
	UPROPERTY(EditAnywhere, Category = "MC|Hands")
	bool bTryFixationGrasp;

	// If hands enable it, try two hands fixation grasp
	UPROPERTY(EditAnywhere, Category = "MC|Hands", meta = (editcondition = "bEnableFixationGrasp"))
	bool bTryTwoHandsFixationGrasp;

	// PID controller proportional argument
	UPROPERTY(EditAnywhere, Category = "MC|Control")
	float PGain;

	// PID controller integral argument
	UPROPERTY(EditAnywhere, Category = "MC|Control")
	float IGain;
	
	// PID controller derivative argument
	UPROPERTY(EditAnywhere, Category = "MC|Control")
	float DGain;
	
	// PID controller maximum output (absolute value)
	UPROPERTY(EditAnywhere, Category = "MC|Control")
	float MaxOutput;

	// Hand rotation controller boost
	UPROPERTY(EditAnywhere, Category = "MC|Control")
	float RotationBoost;
	
	// Character camera
	UPROPERTY(EditAnywhere)
	UCameraComponent* CharCamera;
	
	// Handles moving forward/backward
	void MoveForward(const float Val);

	// Handles strafing Left/Right
	void MoveRight(const float Val);

	// Move hands when not in VR up and down
	void MoveHandsOnZ(const float Value);

	// Update hand positions
	FORCEINLINE void UpdateHandLocationAndRotation(
		UMotionControllerComponent* MC,
		const FQuat& RotOffset,
		USkeletalMeshComponent* SkelMesh,
		PIDController3D& PIDController,
		const float DeltaTime);

	// Switch the curent grasping style
	void SwitchGraspStyle();

	// For testing several grasping processes
	void SwitchGraspProcess();

	// Update left hand grasp
	void GraspWithLeftHand(const float Val);

	// Update right hand grasp
	void GraspWithRightHand(const float Val);

	// Attach to left hand
	void TryLeftFixationGrasp();

	// Attach to right hand
	void TryRightFixationGrasp();

	// Detach from left hand
	void TryLeftGraspDetach();

	// Detach from right hand
	void TryRightGraspDetach();

	// Motion controller origin parent
	USceneComponent* MCOriginComponent;

	// Left hand motion controller
	UMotionControllerComponent* MCLeft;

	// Right hand motion controller
	UMotionControllerComponent* MCRight;

	// Left target arrow visual
	UArrowComponent* LeftTargetArrow;

	// Right target arrow visual
	UArrowComponent* RightTargetArrow;

	// Left hand controller
	PIDController3D LeftPIDController;

	// Right hand controller
	PIDController3D RightPIDController;

	// Left MC hand // TODO look into delegates to avoid dynamic casting
	AHand* LeftHand;

	// Right MC hand
	AHand* RightHand;

	// Offset to add to the hand in order to tracked in the selected position (world rotation at start time)
	FQuat LeftHandRotationOffset;

	// Offset to add to the hand in order to tracked in the selected position (world rotation at start time)
	FQuat RightHandRotationOffset;
};
