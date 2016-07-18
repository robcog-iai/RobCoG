// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "HeadMountedDisplay.h"
#include "MotionControllerComponent.h"
#include "RMotionControllerCharacterVR.h"


// Sets default values
ARMotionControllerCharacterVR::ARMotionControllerCharacterVR(const FObjectInitializer& ObjectInitializer)
{
 	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	// Visualize motion controller debug arrows
	bVisTargetArrows = true;

	// Positional head tracking
	bPositionalHeadTracking = false;

	// Make the capsule thin
	GetCapsuleComponent()->SetCapsuleRadius(5);

	// Set this pawn to be controlled by the lowest-numbered player
	AutoPossessPlayer = EAutoReceiveInput::Player0;

	// Get root component
	RootComponent = GetRootComponent();

	// Create the motion controller offset (hands in front of the character)
	MCOriginComponent = CreateDefaultSubobject<USceneComponent>(TEXT("MCOriginComponent"));
	// Attach Offset to root
	MCOriginComponent->SetupAttachment(RootComponent);
	//// Position of the offset
	//MCOriginComponent->RelativeLocation = FVector(
	//	0.0f, 0.0f, -GetCapsuleComponent()->GetScaledCapsuleHalfHeight());

	// Create a CameraComponent, attach to the MC origin component
	CharCamera = ObjectInitializer.CreateDefaultSubobject<UCameraComponent>(this, TEXT("CharacterCamera"));
	CharCamera->SetupAttachment(MCOriginComponent);

	// Position the camera
	CharCamera->RelativeLocation = FVector(0.0f, 0.0f, BaseEyeHeight);
	// Allow the pawn to control the camera rotation
	CharCamera->bUsePawnControlRotation = true;

	// Create left/right motion controller
	LeftMC = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("LeftMotionController"));
	RightMC = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("RightMotionController"));
	// Set the mapped hand (from the Motion Controller)
	LeftMC->Hand = EControllerHand::Left;
	RightMC->Hand = EControllerHand::Right;
	// Attach controllers to root component
	LeftMC->SetupAttachment(MCOriginComponent);
	RightMC->SetupAttachment(MCOriginComponent);

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
void ARMotionControllerCharacterVR::BeginPlay()
{
	Super::BeginPlay();
	
	// Set arrows to visible
	if (bVisTargetArrows)
	{
		LeftTargetArrow->SetHiddenInGame(false);
		RightTargetArrow->SetHiddenInGame(false);
	}

	IHeadMountedDisplay* HMD = (IHeadMountedDisplay*)(GEngine->HMDDevice.Get());
	if (HMD && HMD->IsStereoEnabled())
	{
		// Disable/Enable positional movement to pin camera translation
		HMD->EnablePositionalTracking(bPositionalHeadTracking);

		// Remove any translation when disabling positional head tracking
		if (!bPositionalHeadTracking)
		{
			CharCamera->SetRelativeLocation(FVector(0, 0, 0));
		}
	}
}

// Called every frame
void ARMotionControllerCharacterVR::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

// Called to bind functionality to input
void ARMotionControllerCharacterVR::SetupPlayerInputComponent(class UInputComponent* InputComponent)
{
	Super::SetupPlayerInputComponent(InputComponent);
}

// Get the Motion Controller Component
UMotionControllerComponent* ARMotionControllerCharacterVR::GetMotionController(EControllerHand HandType)
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
