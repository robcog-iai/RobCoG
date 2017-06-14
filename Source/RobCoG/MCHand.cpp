// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#include "MCHand.h"
#include "Components/SkeletalMeshComponent.h"

// Sets default values
AMCHand::AMCHand()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Set default as left hand
	HandType = EHandType::Left;

	// Set up fingers and their bone names
	AMCHand::SetupHandDefaultValues(HandType);

	// Simulate physics
	GetSkeletalMeshComponent()->SetSimulatePhysics(true);
	// Disable gravity
	GetSkeletalMeshComponent()->SetEnableGravity(false);
	// Collision default
	GetSkeletalMeshComponent()->SetCollisionProfileName(TEXT("BlockAll"));
	// Generate overlap events
	GetSkeletalMeshComponent()->bGenerateOverlapEvents = true;

	//// Flag showing if the finger collisions events are enabled or disabled
	//bFingerHitEvents = false;

	//// Finger types
	//FingerTypes.Add(ERHandLimb::Index);
	//FingerTypes.Add(ERHandLimb::Middle);
	//FingerTypes.Add(ERHandLimb::Ring);
	//FingerTypes.Add(ERHandLimb::Pinky);
	//FingerTypes.Add(ERHandLimb::Thumb);

	//// Finger collision bone names (used for collision detection)
	//CollisionBoneNames.Add(FName("index_03_l"));
	//CollisionBoneNames.Add(FName("middle_03_l"));
	//CollisionBoneNames.Add(FName("ring_03_l"));
	//CollisionBoneNames.Add(FName("pinky_03_l"));
	//CollisionBoneNames.Add(FName("thumb_03_l"));

	// Joint control params
	Spring = 59950000.0f;
	Damping = 59950000.0f;
	ForceLimit = 0.0f;
	Velocity = 0.1f;
}

// Called when the game starts or when spawned
void AMCHand::BeginPlay()
{
	Super::BeginPlay();

	// Hand joint velocity drive
	GetSkeletalMeshComponent()->SetAllMotorsAngularPositionDrive(true, true);
	// Set drive parameters
	GetSkeletalMeshComponent()->SetAllMotorsAngularDriveParams(Spring, Damping, ForceLimit);

	//// Set finger types to constraints map
	//for (FConstraintInstance* Constr : GetSkeletalMeshComponent()->Constraints)
	//{
	//	// Current constraint joint name
	//	const FString ConstrName = Constr->JointName.ToString();
	//	// Iterate hand limbs type
	//	for (const ERHandLimb Type : FingerTypes)
	//	{
	//		// Get the enum type as string
	//		const FString TypeName = FRUtils::GetEnumValueToString<ERHandLimb>("ERHandLimb", Type);
	//		// Add to finger map if the constraint name matches the finger type (name)
	//		if (ConstrName.Contains(TypeName))
	//		{
	//			FingerTypeToConstrs.Add(Type, Constr);
	//		}
	//	}
	//}

	//// Get the hands finger collisions map, disable hit events,
	//// create finger bone names to finger type Map
	//for (const auto BoneName : CollisionBoneNames)
	//{
	//	// Get the body instance
	//	FBodyInstance* BoneBody = GetSkeletalMeshComponent()->GetBodyInstance(BoneName);
	//	// Disable collision notification for now (active only during grasping)
	//	BoneBody->SetInstanceNotifyRBCollision(false);
	//	// Add the body instance to the map
	//	FingerBoneNameToBody.Add(BoneName, GetSkeletalMeshComponent()->GetBodyInstance(BoneName));

	//	// Get bone name as string
	//	const FString NameStr = BoneName.ToString();
	//	// Iterate hand limbs type
	//	for (const ERHandLimb Type : FingerTypes)
	//	{
	//		// Get the enum type as string
	//		const FString TypeName = FRUtils::GetEnumValueToString<ERHandLimb>("ERHandLimb", Type);
	//		// Add to finger map if the constraint name matches the finger type (name)
	//		if (NameStr.Contains(TypeName))
	//		{
	//			BoneNameToFingerTypeMap.Add(BoneName, Type);
	//		}
	//	}
	//}


	//// Collision callbacks for the hands
	//GetSkeletalMeshComponent()->OnComponentHit.AddDynamic(this, &AMCHand::OnFingerHit);

	//// Get the player controller, enable and bind inputs
	//PC = GetWorld()->GetFirstPlayerController();
	//// Enable input
	//EnableInput(PC);
	//// Set up hand bindings
	//if (HandType == EControllerHand::Left)
	//{
	//	PC->InputComponent->BindAxis("OpenHandLeft", this, &ARHand::OpenHand);
	//	PC->InputComponent->BindAxis("CloseHandLeft", this, &ARHand::CloseHand);
	//	PC->InputComponent->BindAction("AttachHandLeft", IE_Released, this, &ARHand::AttachToHand);
	//}
	//else if (HandType == EControllerHand::Right)
	//{
	//	PC->InputComponent->BindAxis("OpenHandRight", this, &ARHand::OpenHand);
	//	PC->InputComponent->BindAxis("CloseHandRight", this, &ARHand::CloseHand);
	//	PC->InputComponent->BindAction("AttachHandRight", IE_Released, this, &ARHand::AttachToHand);
	//}
	//PC->InputComponent->BindAction("SetTrackingOffset", IE_Pressed, this, &ARHand::SetMCTrackingOffset);
}

// Called every frame, used for motion control
void AMCHand::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

#if WITH_EDITOR  
void AMCHand::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	// Call the base class version  
	Super::PostEditChangeProperty(PropertyChangedEvent);

	AMCHand::SetupHandDefaultValues(HandType);
}
#endif  

// Setup hand default values
FORCEINLINE void AMCHand::SetupHandDefaultValues(EHandType InHandType)
{
	if (InHandType == EHandType::Left)
	{
		Thumb.FingerType = EFingerType::Thumb;
		Thumb.FingerPartBoneNames.Add(EFingerPart::Proximal, "thumb_01_l");
		Thumb.FingerPartBoneNames.Add(EFingerPart::Intermediate, "thumb_02_l");
		Thumb.FingerPartBoneNames.Add(EFingerPart::Distal, "thumb_03_l");

		Index.FingerType = EFingerType::Index;
		Index.FingerPartBoneNames.Add(EFingerPart::Proximal, "index_01_l");
		Index.FingerPartBoneNames.Add(EFingerPart::Intermediate, "index_02_l");
		Index.FingerPartBoneNames.Add(EFingerPart::Distal, "index_03_l");

		Middle.FingerType = EFingerType::Middle;
		Middle.FingerPartBoneNames.Add(EFingerPart::Proximal, "middle_01_l");
		Middle.FingerPartBoneNames.Add(EFingerPart::Intermediate, "middle_02_l");
		Middle.FingerPartBoneNames.Add(EFingerPart::Distal, "middle_03_l");

		Ring.FingerType = EFingerType::Ring;
		Ring.FingerPartBoneNames.Add(EFingerPart::Proximal, "ring_01_l");
		Ring.FingerPartBoneNames.Add(EFingerPart::Intermediate, "ring_02_l");
		Ring.FingerPartBoneNames.Add(EFingerPart::Distal, "ring_03_l");

		Pinky.FingerType = EFingerType::Ring;
		Pinky.FingerPartBoneNames.Add(EFingerPart::Proximal, "pinky_01_l");
		Pinky.FingerPartBoneNames.Add(EFingerPart::Intermediate, "pinky_02_l");
		Pinky.FingerPartBoneNames.Add(EFingerPart::Distal, "pinky_03_l");
	}
	else if (InHandType == EHandType::Right)
	{
		Thumb.FingerType = EFingerType::Thumb;
		Thumb.FingerPartBoneNames.Add(EFingerPart::Proximal, "thumb_01_r");
		Thumb.FingerPartBoneNames.Add(EFingerPart::Intermediate, "thumb_02_r");
		Thumb.FingerPartBoneNames.Add(EFingerPart::Distal, "thumb_03_r");

		Index.FingerType = EFingerType::Index;
		Index.FingerPartBoneNames.Add(EFingerPart::Proximal, "index_01_r");
		Index.FingerPartBoneNames.Add(EFingerPart::Intermediate, "index_02_r");
		Index.FingerPartBoneNames.Add(EFingerPart::Distal, "index_03_r");

		Middle.FingerType = EFingerType::Middle;
		Middle.FingerPartBoneNames.Add(EFingerPart::Proximal, "middle_01_r");
		Middle.FingerPartBoneNames.Add(EFingerPart::Intermediate, "middle_02_r");
		Middle.FingerPartBoneNames.Add(EFingerPart::Distal, "middle_03_r");

		Ring.FingerType = EFingerType::Ring;
		Ring.FingerPartBoneNames.Add(EFingerPart::Proximal, "ring_01_r");
		Ring.FingerPartBoneNames.Add(EFingerPart::Intermediate, "ring_02_r");
		Ring.FingerPartBoneNames.Add(EFingerPart::Distal, "ring_03_r");

		Pinky.FingerType = EFingerType::Ring;
		Pinky.FingerPartBoneNames.Add(EFingerPart::Proximal, "pinky_01_r");
		Pinky.FingerPartBoneNames.Add(EFingerPart::Intermediate, "pinky_02_r");
		Pinky.FingerPartBoneNames.Add(EFingerPart::Distal, "pinky_03_r");
	}
}

//// Close hand fingers
//void AMCHand::CloseHand(const float AxisValue)
//{
//	if (AxisValue == 0)
//	{
//		// Disable finger hit events
//		if (bFingerHitEvents)
//		{
//			for (auto BoneName : CollisionBoneNames)
//			{
//				GetSkeletalMeshComponent()->GetBodyInstance(BoneName)->SetInstanceNotifyRBCollision(false);
//			}
//			// Set finger notification flag
//			bFingerHitEvents = false;
//		}
//		return;
//	}
//
//	// Enable finger hit events
//	if (!bFingerHitEvents)
//	{
//		for (auto BoneName : CollisionBoneNames)
//		{
//			GetSkeletalMeshComponent()->GetBodyInstance(BoneName)->SetInstanceNotifyRBCollision(true);
//		}
//		// Set finger notification flag
//		bFingerHitEvents = true;
//	}
//}
//
//// Attach grasped object to hand
//void AMCHand::AttachToHand()
//{
//
//}
//
//// Open hand fingers
//void AMCHand::OpenHand(const float AxisValue)
//{
//
//}
//

