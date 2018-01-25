// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "Engine/TriggerSphere.h"
#include "GameFramework/Character.h"

#include "ForceSimpleHand.h"

#include "ForceCharacter.generated.h"

UCLASS()
class UFORCEBASEDGRASPING_API AForceCharacter : public ACharacter
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere, Category = "SkeletalMeshActor")
	AForceSimpleHand* ForceSimpleHand;

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
	void Open(const float Value);
	void Close(const float Value);

private:
	FQuat ForceSimpleHandOffset;

};
