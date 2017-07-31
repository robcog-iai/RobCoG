// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "ForceCharacter.h"

#include "Components/CapsuleComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/CharacterMovementComponent.h"


// Sets default values
AForceCharacter::AForceCharacter()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	// Set this pawn to be controlled by the lowest-numbered player
	AutoPossessPlayer = EAutoReceiveInput::Player0;
	// Make the capsule thin, and set it only to collide with static objects (VR Mode)
	GetCapsuleComponent()->SetCapsuleRadius(10);
	GetCapsuleComponent()->SetCollisionProfileName(TEXT("BlockAll"));

}

// Called when the game starts or when spawned
void AForceCharacter::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AForceCharacter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

// Called to bind functionality to input
void AForceCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	// Set up gameplay key bindings
	PlayerInputComponent->BindAxis("MoveForward", this, &AForceCharacter::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &AForceCharacter::MoveRight);

	// Default Camera view bindings
	PlayerInputComponent->BindAxis("CameraPitch", this, &AForceCharacter::AddControllerPitchInput);
	PlayerInputComponent->BindAxis("CameraYaw", this, &AForceCharacter::AddControllerYawInput);

	// Hand control binding
	PlayerInputComponent->BindAxis("GraspWithLeftHand", this, &AForceCharacter::Open);
	PlayerInputComponent->BindAxis("GraspWithRightHand", this, &AForceCharacter::Close);
}

// Handles moving forward/backward
void AForceCharacter::MoveForward(const float Value)
{
	if ((Controller != nullptr) && (Value != 0.0f))
	{
		// find out which way is forward
		FRotator Rotation = Controller->GetControlRotation();
		// Limit pitch when walking or falling
		if (GetCharacterMovement()->IsMovingOnGround() || GetCharacterMovement()->IsFalling())
		{
			Rotation.Pitch = 0.0f;
		}
		// add movement in that direction
		const FVector Direction = FRotationMatrix(Rotation).GetScaledAxis(EAxis::X);
		AddMovementInput(Direction, Value);
	}
}

// Handles moving right/left
void AForceCharacter::MoveRight(const float Value)
{
	if ((Controller != nullptr) && (Value != 0.0f))
	{
		// find out which way is right
		const FRotator Rotation = Controller->GetControlRotation();
		const FVector Direction = FRotationMatrix(Rotation).GetScaledAxis(EAxis::Y);
		// add movement in that direction
		AddMovementInput(Direction, Value);
	}
}

void AForceCharacter::Open(const float Value)
{
	if (ForceSimpleHand != nullptr)
		ForceSimpleHand->StartGrasp(Value);
}

void AForceCharacter::Close(const float Value)
{
	if (ForceSimpleHand != nullptr)
		ForceSimpleHand->StopGrasp(Value);
}