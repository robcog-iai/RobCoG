// Fill out your copyright notice in the Description page of Project Settings.

#include "RoCoG.h"
#include "RCGUtils.h"
#include "RCGMotionControllerCharacter.h"


// Sets default values
ARCGMotionControllerCharacter::ARCGMotionControllerCharacter(const FObjectInitializer& ObjectInitializer)
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	// Visualize motion controller debug arrows
	bVisTargetArrows = true;

	// Make the capsule thin
	GetCapsuleComponent()->SetCapsuleRadius(5);	

	// Set this pawn to be controlled by the lowest-numbered player
	AutoPossessPlayer = EAutoReceiveInput::Player0;

	// Create a CameraComponent, attach to capsule
	CharCamera = ObjectInitializer.CreateDefaultSubobject<UCameraComponent>(this, TEXT("CharacterCamera"));
	CharCamera->SetupAttachment(GetCapsuleComponent());
	// Position the camera
	CharCamera->RelativeLocation = FVector(0.0f, 0.0f, BaseEyeHeight);
	// Allow the pawn to control the camera rotation
	CharCamera->bUsePawnControlRotation = true;

	// Get root component
	RootComponent = GetRootComponent();
	// Create the motion controller offset (hands in front of the character)
	MCOffsetComponent = CreateDefaultSubobject<USceneComponent>(TEXT("MCOffsetComponent"));
	// Attach Offset to root
	MCOffsetComponent->SetupAttachment(RootComponent);
	// Position of the offset
	MCOffsetComponent->RelativeLocation = FVector(
		0.0f, 0.0f, - GetCapsuleComponent()->GetScaledCapsuleHalfHeight());

	// Create left/right motion controller
	LeftMC = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("LeftMotionController"));
	RightMC = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("RightMotionController"));
	// Attach controllers to root component
	LeftMC->SetupAttachment(MCOffsetComponent);
	RightMC->SetupAttachment(MCOffsetComponent);
	// Set the mapped hand (from the Motion Controller)
	LeftMC->Hand = EControllerHand::Left;
	RightMC->Hand = EControllerHand::Right;

	// Create left/right target vis arrows
	LeftTargetArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("LeftVisArrow"));
	RightTargetArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("RightVisArrow"));
	// Set arrow sizes
	LeftTargetArrow->ArrowSize = 0.1;
	RightTargetArrow->ArrowSize = 0.1;
	// Attach vis arrow to motion controllers
	LeftTargetArrow->SetupAttachment(LeftMC);
	RightTargetArrow->SetupAttachment(RightMC);
}

// Called when the game starts or when spawned
void ARCGMotionControllerCharacter::BeginPlay()
{
	Super::BeginPlay();

	// Set arrows to visible
	if (bVisTargetArrows)
	{
		LeftTargetArrow->SetHiddenInGame(false);
		RightTargetArrow->SetHiddenInGame(false);
	}
}

// Called every frame
void ARCGMotionControllerCharacter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

// Called to bind functionality to input
void ARCGMotionControllerCharacter::SetupPlayerInputComponent(class UInputComponent* InputComponent)
{
	Super::SetupPlayerInputComponent(InputComponent);

	// Set up gameplay key bindings
	InputComponent->BindAxis("MoveForward", this, &ARCGMotionControllerCharacter::MoveForward);
	InputComponent->BindAxis("MoveRight", this, &ARCGMotionControllerCharacter::MoveRight);
	// Default Camera view bindings
	InputComponent->BindAxis("CameraPitch", this, &ARCGMotionControllerCharacter::AddControllerPitchInput);
	InputComponent->BindAxis("CameraYaw", this, &ARCGMotionControllerCharacter::AddControllerYawInput);

}

// Get the Motion Controller Component
UMotionControllerComponent* ARCGMotionControllerCharacter::GetMotionController(EControllerHand HandType)
{
	if (HandType == EControllerHand::Left)
	{
		return LeftMC;
	}
	else if (HandType == EControllerHand::Right)
	{
		return RightMC;
	}
	else
	{
		return nullptr;
	}
}

// Get Motion Controller calibration offset location
FVector ARCGMotionControllerCharacter::GetMCTrackingOffsetLoc(EControllerHand HandType)
{
	if (HandType == EControllerHand::Left)
	{
		return LeftMC->RelativeLocation;
	}
	else if (HandType == EControllerHand::Right)
	{
		return RightMC->RelativeLocation;
	}
	else
	{
		return FVector(0.0f);
	}
}

// Get Motion Controller calibration offset orientation
FQuat ARCGMotionControllerCharacter::GetMCTrackingOffsetRot(EControllerHand HandType)
{
	if (HandType == EControllerHand::Left)
	{
		return LeftMC->RelativeRotation.Quaternion();
	}
	else if (HandType == EControllerHand::Right)
	{
		return RightMC->RelativeRotation.Quaternion();
	}
	else
	{
		return FQuat::Identity;
	}
}

// Handles moving forward/backward
void ARCGMotionControllerCharacter::MoveForward(const float Value)
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
void ARCGMotionControllerCharacter::MoveRight(const float Value)
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
