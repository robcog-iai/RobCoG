// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "Camera/CameraComponent.h"
#include "Animation/SkeletalMeshActor.h"
#include "MotionControllerComponent.h"
#include "PIDController3D.h"
#include "MCCharacter.generated.h"

UCLASS()
class ROBCOG_API AMCCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	AMCCharacter(const FObjectInitializer& ObjectInitializer);

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

protected:
	// Show motion controller pose arrows
	UPROPERTY(EditAnywhere, Category = "MC")
	bool bShowTargetArrows;

	// Left hand skeletal mesh
	UPROPERTY(EditAnywhere, Category = "MC")
	ASkeletalMeshActor* LeftHand;

	// Right hand skeletal mesh
	UPROPERTY(EditAnywhere, Category = "MC")
	ASkeletalMeshActor* RightHand;

	// PID controller proportional argument
	UPROPERTY(EditAnywhere, Category = "MC")
	float PGain;
	
	// PID controller integral argument
	UPROPERTY(EditAnywhere, Category = "MC")
	float IGain;
	
	// PID controller derivative argument
	UPROPERTY(EditAnywhere, Category = "MC")
	float DGain;
	
	// PID controller maximum output (absolute value)
	UPROPERTY(EditAnywhere, Category = "MC")
	float PIDMaxAbsOutput;

	// Hand rotation controller boost
	UPROPERTY(EditAnywhere, Category = "MC")
	float RotationBoost;
	
	// Handles moving forward/backward
	void MoveForward(const float Val);

	// Handles strafing Left/Right
	void MoveRight(const float Val);

	// Update hand positions
	FORCEINLINE void UpdateHandLocationAndRotation(
		UMotionControllerComponent* MC,
		USkeletalMeshComponent* SkelMesh,
		PIDController3D& PIDController,
		const float DeltaTime);

	// Character camera
	UCameraComponent* CharCamera;

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

	// Left hand control body
	FBodyInstance* LeftControlBody;

	// Right hand control body
	FBodyInstance* RightControlBody;
};
