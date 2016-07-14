// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Character.h"
#include "MotionControllerComponent.h"
#include "RCGMotionControllerCharacter.generated.h"

UCLASS()
class ROCOG_API ARCGMotionControllerCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	ARCGMotionControllerCharacter(const FObjectInitializer& ObjectInitializer);

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* InputComponent) override;

	// Visualise or not target arrows
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	bool bVisTargetArrows;

	// Handles moving forward/backward
	UFUNCTION()
	void MoveForward(const float Val);

	// Handles strafing Left/Right
	UFUNCTION()
	void MoveRight(const float Val);

private:
	// Character camera
	UCameraComponent* CharCamera;

	// Motion controller offset parent
	USceneComponent* MCOffset;

	// Left hand motion controller
	UMotionControllerComponent* LeftMC;

	// Right hand motion controller
	UMotionControllerComponent* RightMC;

	// Left target arrow visual
	UArrowComponent* LeftTargetArrow;

	// Right target arrow visual
	UArrowComponent* RightTargetArrow;
};
