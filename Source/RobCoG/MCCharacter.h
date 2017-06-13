// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "MotionControllerComponent.h"
#include "Camera/CameraComponent.h"
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

	// Handles moving forward/backward
	void MoveForward(const float Val);

	// Handles strafing Left/Right
	void MoveRight(const float Val);

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
