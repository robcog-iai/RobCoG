// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#include "MCCharacter.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/Controller.h"
#include "Components/CapsuleComponent.h"
#include "Components/ArrowComponent.h"
#include "Components/InputComponent.h"
#include "HeadMountedDisplay.h"
#include "IHeadMountedDisplay.h"

// Sets default values
AMCCharacter::AMCCharacter()
{
 	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	// Set this pawn to be controlled by the lowest-numbered player
	AutoPossessPlayer = EAutoReceiveInput::Player0;
	// Make the capsule thin, and set it only to collide with static objects (VR Mode)
	GetCapsuleComponent()->SetCapsuleRadius(10);
	GetCapsuleComponent()->SetCollisionProfileName(TEXT("Spectator"));

	// Set flag default values
	bShowTargetArrows = true;
	bUseHandsInitialRotationAsOffset = true;
	bTryFixationGrasp = true;
	bTryTwoHandsFixationGrasp = true;

	// Create the motion controller offset (hands in front of the character), attach to root component
	MCOriginComponent = CreateDefaultSubobject<USceneComponent>(TEXT("MCOriginComponent"));
	MCOriginComponent->SetupAttachment(GetRootComponent());
	MCOriginComponent->SetRelativeLocation(FVector(0.0f, 0.0f, -GetCapsuleComponent()->GetScaledCapsuleHalfHeight()));

	// Create a CameraComponent, attach to capsule
	CharCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("MCCharacterCamera"));
	CharCamera->SetupAttachment(MCOriginComponent);
	// Default camera for VR use -- relative location at the floor 
	//CharCamera->SetRelativeLocation(FVector(0.0f, 0.0f, -GetCapsuleComponent()->GetScaledCapsuleHalfHeight()));
	//CharCamera->SetRelativeLocation(FVector(0.0f, 0.0f, BaseEyeHeight));

	// Create left/right motion controller
	MCLeft = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("MCLeft"));
	MCLeft->SetupAttachment(MCOriginComponent);
	MCLeft->Hand = EControllerHand::Left;
	MCRight = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("MCRight"));
	MCRight->SetupAttachment(MCOriginComponent);
	MCRight->Hand = EControllerHand::Right;

	// Create left/right target vis arrows, attached to the MC
	LeftTargetArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("MCLeftTargetArrow"));
	LeftTargetArrow->ArrowSize = 0.1;
	LeftTargetArrow->SetupAttachment(MCLeft);
	RightTargetArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("MCRightTargetArrow"));
	RightTargetArrow->ArrowSize = 0.1;
	RightTargetArrow->SetupAttachment(MCRight);

	// PID params
	PGain = 700.0f;
	IGain = 0.0f;
	DGain = 50.0f;
	MaxOutput = 350000.0f;
	RotationBoost = 12000.f;

	// Init rotation offset
	LeftHandRotationOffset = FQuat::Identity;
	RightHandRotationOffset = FQuat::Identity;
}

// Called when the game starts or when spawned
void AMCCharacter::BeginPlay()
{
	Super::BeginPlay();

	// Set target arrow visibility at runtime
	LeftTargetArrow->SetHiddenInGame(!bShowTargetArrows);
	RightTargetArrow->SetHiddenInGame(!bShowTargetArrows);

	// Set the hand PID controller values
	LeftPIDController.SetValues(PGain, IGain, DGain, MaxOutput, -MaxOutput);
	RightPIDController.SetValues(PGain, IGain, DGain, MaxOutput, -MaxOutput);

	// Check if VR is enabled
	IHeadMountedDisplay* HMD = (IHeadMountedDisplay*)(GEngine->HMDDevice.Get());
	if (HMD && HMD->IsStereoEnabled())
	{		
		// VR MODE
		//CharCamera->SetRelativeLocation(FVector(0.0f, 0.0f, -GetCapsuleComponent()->GetScaledCapsuleHalfHeight()));
	}
	else
	{		
		GetCapsuleComponent()->SetCollisionProfileName(TEXT("Pawn"));
		CharCamera->SetRelativeLocation(FVector(0.0f, 0.0f, BaseEyeHeight));
		CharCamera->bUsePawnControlRotation = true;
		MCOriginComponent->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
		MCLeft->SetRelativeLocation(FVector(75.f, -30.f, 30.f));
		MCRight->SetRelativeLocation(FVector(75.f, 30.f, 30.f));
	}

	if (LeftSkelActor)
	{	
		// Cast the hands to AHand
		LeftHand = Cast<AHand>(LeftSkelActor);

		// Set hand offsets
		if (bUseHandsInitialRotationAsOffset)
		{
			LeftHandRotationOffset = LeftSkelActor->GetSkeletalMeshComponent()->GetComponentQuat();
		}

		// Teleport hands to the current position of the motion controllers
		LeftSkelActor->SetActorLocationAndRotation(MCLeft->GetComponentLocation(),
			MCLeft->GetComponentQuat() * LeftHandRotationOffset,
			false, (FHitResult*)nullptr, ETeleportType::TeleportPhysics);
	}

	if (RightSkelActor)
	{
		// Cast the hands to AHand
		RightHand = Cast<AHand>(RightSkelActor);

		// Set hand offsets
		if (bUseHandsInitialRotationAsOffset)
		{
			RightHandRotationOffset = RightSkelActor->GetSkeletalMeshComponent()->GetComponentQuat();
		}

		// Teleport hands to the current position of the motion controllers
		RightSkelActor->SetActorLocationAndRotation(MCRight->GetComponentLocation(),
			MCRight->GetComponentQuat() * RightHandRotationOffset,
			false, (FHitResult*)nullptr,ETeleportType::TeleportPhysics);
	}

	// If two hands are available, let them know about each other (for two hands fixation grasp)
	bTryTwoHandsFixationGrasp = (bTryTwoHandsFixationGrasp && LeftHand && RightHand);
	if (bTryTwoHandsFixationGrasp)
	{
		LeftHand->SetOtherHand(RightHand);
		RightHand->SetOtherHand(LeftHand);
	}
}

// Called every frame
void AMCCharacter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Force based movement of the hands to target location and rotation
	if (LeftSkelActor)
	{
		AMCCharacter::UpdateHandLocationAndRotation(
			MCLeft, LeftHandRotationOffset, LeftSkelActor->GetSkeletalMeshComponent(), LeftPIDController, DeltaTime);
	}
	if (RightSkelActor)
	{
		AMCCharacter::UpdateHandLocationAndRotation(
			MCRight, RightHandRotationOffset, RightSkelActor->GetSkeletalMeshComponent(), RightPIDController, DeltaTime);
	}
}

// Called to bind functionality to input
void AMCCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	// Set up gameplay key bindings
	PlayerInputComponent->BindAxis("MoveForward", this, &AMCCharacter::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &AMCCharacter::MoveRight);
	PlayerInputComponent->BindAxis("MoveHandsOnZ", this, &AMCCharacter::MoveHandsOnZ);
	// Default Camera view bindings
	PlayerInputComponent->BindAxis("CameraPitch", this, &AMCCharacter::AddControllerPitchInput);
	PlayerInputComponent->BindAxis("CameraYaw", this, &AMCCharacter::AddControllerYawInput);
	// Hand control binding
	PlayerInputComponent->BindAxis("GraspWithLeftHand", this, &AMCCharacter::GraspWithLeftHand);
	PlayerInputComponent->BindAxis("GraspWithRightHand", this, &AMCCharacter::GraspWithRightHand);

	PlayerInputComponent->BindAction("SwitchGraspStyle", IE_Pressed, this, &AMCCharacter::SwitchGraspStyle);
	PlayerInputComponent->BindAction("SwitchGraspProcess", IE_Pressed, this, &AMCCharacter::SwitchGraspProcess);

	// Hand action binding
	PlayerInputComponent->BindAction("AttachToLeftHand", IE_Pressed, this, &AMCCharacter::TryLeftFixationGrasp);
	PlayerInputComponent->BindAction("AttachToRightHand", IE_Pressed, this, &AMCCharacter::TryRightFixationGrasp);
	PlayerInputComponent->BindAction("AttachToLeftHand", IE_Released, this, &AMCCharacter::TryLeftGraspDetach);
	PlayerInputComponent->BindAction("AttachToRightHand", IE_Released, this, &AMCCharacter::TryRightGraspDetach);
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

// Move hands when not in VR up and down
void AMCCharacter::MoveHandsOnZ(const float Value)
{
	if (Value != 0)
	{
		// Check if VR is enabled
		IHeadMountedDisplay* HMD = (IHeadMountedDisplay*)(GEngine->HMDDevice.Get());
		if (!(HMD && HMD->IsStereoEnabled()))
		{
			MCLeft->AddLocalOffset(FVector(0.f, 0.f, Value));
			MCRight->AddLocalOffset(FVector(0.f, 0.f, Value));
		}

	}
}

// Update hand positions
FORCEINLINE void AMCCharacter::UpdateHandLocationAndRotation(
	UMotionControllerComponent* MC,
	const FQuat& RotOffset,
	USkeletalMeshComponent* SkelMesh,
	PIDController3D& PIDController,
	const float DeltaTime)
{
	//// Location
	const FVector Error = MC->GetComponentLocation() - SkelMesh->GetComponentLocation();
	const FVector LocOutput = PIDController.UpdateAsPD(Error, DeltaTime);
	SkelMesh->AddForceToAllBodiesBelow(LocOutput, NAME_None, true, true);
	//// Velocity based control
	//const FVector LocOutput = PIDController.UpdateAsP(Error, DeltaTime);
	//SkelMesh->SetAllPhysicsLinearVelocity(LocOutput);

	//// Rotation
	const FQuat TargetQuat = MC->GetComponentQuat() * RotOffset;
	FQuat CurrQuat = SkelMesh->GetComponentQuat();

	// Dot product to get cos theta
	const float CosTheta = TargetQuat | CurrQuat;
	// Avoid taking the long path around the sphere
	if (CosTheta < 0)
	{
		CurrQuat *= -1.f;
	}
	// Use the xyz part of the quat as the rotation velocity
	const FQuat OutputFromQuat = TargetQuat * CurrQuat.Inverse();
	const FVector RotOutput = FVector(OutputFromQuat.X, OutputFromQuat.Y, OutputFromQuat.Z) * RotationBoost;
	SkelMesh->SetAllPhysicsAngularVelocity(RotOutput);
}

// Switch Grasp style
void AMCCharacter::SwitchGraspStyle()
{
	if (RightHand)
	{
		RightHand->SwitchGraspStyle();
	}

	if (LeftHand)
	{
		LeftHand->SwitchGraspStyle();
	}
}

// Switch Grasp process
void AMCCharacter::SwitchGraspProcess()
{
	if (RightHand)
	{
		RightHand->SwitchGraspProcess();
	}

	if (LeftHand)
	{
		LeftHand->SwitchGraspProcess();
	}
}

// Update left hand grasp
void AMCCharacter::GraspWithLeftHand(const float Val)
{
	if (LeftHand)
	{
		LeftHand->UpdateGrasp(Val);
	}
}

// Update right hand grasp
void AMCCharacter::GraspWithRightHand(const float Val)
{
	if (RightHand)
	{
		//RightHand->UpdateGrasp(Val);
		// TODO For the realisitc grasping part
		RightHand->UpdateGrasp2(Val); 
	}
}

// Attach to left hand
void AMCCharacter::TryLeftFixationGrasp()
{
	if (bTryFixationGrasp && LeftHand)
	{
		// If one hand attachment is not possible, check for two hands
		if (!LeftHand->TryOneHandFixationGrasp())
		{
			// If other hand is set and two hand grasp is enabled
			if (bTryTwoHandsFixationGrasp && RightHand)
			{
				// Try grasping with two hands
				LeftHand->TryTwoHandsFixationGrasp();
			}
		}
	}
}

// Attach to right hand
void AMCCharacter::TryRightFixationGrasp()
{
	if (bTryFixationGrasp && RightHand)
	{
		// If one hand attachment is not possible, check for two hands
		if (!RightHand->TryOneHandFixationGrasp())
		{
			if (bTryTwoHandsFixationGrasp && LeftHand)
			{
				// Check if two hand fixation is possible
				RightHand->TryTwoHandsFixationGrasp();
			}
		}
	}
}

// Detach from left hand
void AMCCharacter::TryLeftGraspDetach()
{
	if (LeftHand)
	{
		LeftHand->DetachFixationGrasp();
	}
}

// Detach from right hand
void AMCCharacter::TryRightGraspDetach()
{
	if (RightHand)
	{
		RightHand->DetachFixationGrasp();
	}
}
