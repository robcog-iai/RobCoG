// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "MCCharacter.h"
#include "Components/CapsuleComponent.h"
#include "Components/ArrowComponent.h"
#include "Components/InputComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/Controller.h"
#include "HeadMountedDisplay.h"
#include "IHeadMountedDisplay.h"

// Sets default values
AMCCharacter::AMCCharacter(const FObjectInitializer& ObjectInitializer)
{
 	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Set flag default values
	bShowTargetArrows = true;

	// Make the capsule thin
	GetCapsuleComponent()->SetCapsuleRadius(1);

	// Set this pawn to be controlled by the lowest-numbered player
	AutoPossessPlayer = EAutoReceiveInput::Player0;

	// Create the motion controller offset (hands in front of the character), attach to root component
	MCOriginComponent = CreateDefaultSubobject<USceneComponent>(TEXT("MCOriginComponent"));
	MCOriginComponent->SetupAttachment(GetRootComponent());
	//MCOriginComponent->RelativeLocation = FVector(
	//	0.0f, 0.0f, - GetCapsuleComponent()->GetScaledCapsuleHalfHeight());

	// Create a CameraComponent, attach to capsule
	CharCamera = ObjectInitializer.CreateDefaultSubobject<UCameraComponent>(this, TEXT("MCCharacterCamera"));
	CharCamera->SetupAttachment(MCOriginComponent);
	CharCamera->RelativeLocation = FVector(0.0f, 0.0f, BaseEyeHeight);
	CharCamera->bUsePawnControlRotation = true;

	// Create left/right motion controller
	LeftMC = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("MCLeft"));
	LeftMC->SetupAttachment(MCOriginComponent);
	LeftMC->Hand = EControllerHand::Left;
	RightMC = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("MCRight"));
	RightMC->SetupAttachment(MCOriginComponent);
	RightMC->Hand = EControllerHand::Right;

	// Create left/right target vis arrows, attached to the MC
	LeftTargetArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("MCLeftTargetArrow"));
	LeftTargetArrow->ArrowSize = 0.1;
	LeftTargetArrow->SetupAttachment(LeftMC);
	RightTargetArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("MCRightTargetArrow"));
	RightTargetArrow->ArrowSize = 0.1;
	RightTargetArrow->SetupAttachment(RightMC);
}

// Called when the game starts or when spawned
void AMCCharacter::BeginPlay()
{
	Super::BeginPlay();

	// Set target arrow visibility at runtime
	LeftTargetArrow->SetHiddenInGame(!bShowTargetArrows);
	RightTargetArrow->SetHiddenInGame(!bShowTargetArrows);

	// Check if VR is enabled
	IHeadMountedDisplay* HMD = (IHeadMountedDisplay*)(GEngine->HMDDevice.Get());
	if (HMD && HMD->IsStereoEnabled())
	{
		UE_LOG(LogTemp, Error, TEXT(" !! !! vr enabled !! !! !! "));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT(" !! !! vr disabled !! !! !! "));
	}
}

// Called every frame
void AMCCharacter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

// Called to bind functionality to input
void AMCCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	// Set up gameplay key bindings
	PlayerInputComponent->BindAxis("MoveForward", this, &AMCCharacter::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &AMCCharacter::MoveRight);
	// Default Camera view bindings
	PlayerInputComponent->BindAxis("CameraPitch", this, &AMCCharacter::AddControllerPitchInput);
	PlayerInputComponent->BindAxis("CameraYaw", this, &AMCCharacter::AddControllerYawInput);
}

// Handles moving forward/backward
void AMCCharacter::MoveForward(const float Value)
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
void AMCCharacter::MoveRight(const float Value)
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

