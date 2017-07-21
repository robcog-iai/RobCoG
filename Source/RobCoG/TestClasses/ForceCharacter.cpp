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
	GetCapsuleComponent()->SetCollisionProfileName(TEXT("Spectator"));

	CurrentRotation = FRotator(10, 10, 10);

	Spring = 50.0f;
	Damping = 50.0f;
	ForceLimit = 0.0f;

}

// Called when the game starts or when spawned
void AForceCharacter::BeginPlay()
{
	Super::BeginPlay();
	
	USkeletalMeshComponent* SkeletalMeshComponent = GetMesh();

	if(SkeletalMeshComponent != nullptr)
	{
		//SkeletalMeshComponent->SetEnableGravity(false);
		//SkeletalMeshComponent->SetSimulatePhysics(true);

		LeftConstraint = SkeletalMeshComponent->Constraints[0];

		LeftConstraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
		LeftConstraint->SetOrientationDriveTwistAndSwing(true, true);
		LeftConstraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
		LeftConstraint->SetAngularOrientationTarget(CurrentRotation.Quaternion());
		
		RightConstraint = SkeletalMeshComponent->Constraints[1];
		RightConstraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
		RightConstraint->SetOrientationDriveTwistAndSwing(true, true);
		RightConstraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
		RightConstraint->SetAngularOrientationTarget(CurrentRotation.Quaternion());
	}
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
	PlayerInputComponent->BindAxis("MoveHandsOnZ", this, &AForceCharacter::MoveHandsOnZ);

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

// Move hands when not in VR up and down
void AForceCharacter::MoveHandsOnZ(const float Value)
{
	if (Value != 0)
	{
		GetMesh()->AddLocalOffset(FVector(0.f, 0.f, Value),false,nullptr,ETeleportType::TeleportPhysics);
	}
}

void AForceCharacter::Open(const float Value)
{
	if (Value != 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Value: %f | Rotation: %s"), Value, *CurrentRotation.ToString());

		CurrentRotation.Add(1,1,1);
		LeftConstraint->SetAngularOrientationTarget(CurrentRotation.Quaternion());

		FRotator Rotation = CurrentRotation * -1;
		RightConstraint->SetAngularOrientationTarget(Rotation.Quaternion());
	}
}

void AForceCharacter::Close(const float Value)
{
	if (Value != 0)
	{

	}
}