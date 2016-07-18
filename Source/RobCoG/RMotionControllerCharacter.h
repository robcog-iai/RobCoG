// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Character.h"
#include "RMotionControllerCharacter.generated.h"

UCLASS()
class ROBCOG_API ARMotionControllerCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	ARMotionControllerCharacter(const FObjectInitializer& ObjectInitializer);

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* InputComponent) override;

	// Get the Motion Controller Component
	UMotionControllerComponent* GetMotionController(EControllerHand HandType);

	// Get Motion Controller calibration offset location
	FVector GetMCTrackingOffsetLoc(EControllerHand HandType);

	// Get Motion Controller calibration offset orientation
	FQuat GetMCTrackingOffsetRot(EControllerHand HandType);

protected:
	// Visualise or not target arrows
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	bool bVisTargetArrows;

	// Handles moving forward/backward
	void MoveForward(const float Val);

	// Handles strafing Left/Right
	void MoveRight(const float Val);

private:
	// Character camera
	UCameraComponent* CharCamera;

	// Motion controller origin parent
	USceneComponent* MCOriginComponent;

	// Left hand motion controller
	UMotionControllerComponent* LeftMC;

	// Right hand motion controller
	UMotionControllerComponent* RightMC;

	// Left target arrow visual
	UArrowComponent* LeftTargetArrow;

	// Right target arrow visual
	UArrowComponent* RightTargetArrow;
};
