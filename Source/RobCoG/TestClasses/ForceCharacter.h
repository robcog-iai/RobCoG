// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "PhysicsEngine/ConstraintInstance.h"

#include "ForceCharacter.generated.h"

UCLASS()
class ROBCOG_API AForceCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	// Spring value to apply to the angular drive (Position strength)
	UPROPERTY(EditAnywhere, Category = "Drive Parameters")
		float Spring;

	// Damping value to apply to the angular drive (Velocity strength) 
	UPROPERTY(EditAnywhere, Category = "Drive Parameters")
		float Damping;

	// Limit of the force that the angular drive can apply
	UPROPERTY(EditAnywhere, Category = "Drive Parameters")
		float ForceLimit;

	// Sets default values for this character's properties
	AForceCharacter();
	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	void MoveForward(const float Value);
	void MoveRight(const float Value);
	void MoveHandsOnZ(const float Value);
	void Open(const float Value);
	void Close(const float Value);

private:
	FRotator CurrentRotation;

	FConstraintInstance* LeftConstraint;

	FConstraintInstance* RightConstraint;

};
