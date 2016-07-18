// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Character.h"
#include "RMotionControllerCharacterVR.generated.h"

UCLASS()
class ROBCOG_API ARMotionControllerCharacterVR : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	ARMotionControllerCharacterVR(const FObjectInitializer& ObjectInitializer);

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* InputComponent) override;

	// Get the Motion Controller Component
	UMotionControllerComponent* GetMotionController(EControllerHand HandType);

protected:
	// Visualise or not target arrows
	UPROPERTY(EditAnywhere, Category = "Motion Controller")
	bool bVisTargetArrows;

	// Enable HMD tracking
	UPROPERTY(EditDefaultsOnly, Category = "Motion Controller")
	bool bPositionalHeadTracking;

private:
	// Character camera
	UCameraComponent* CharCamera;

	// Motion controller origin
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
