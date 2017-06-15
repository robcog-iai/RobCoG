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

	// Set skeletal mesh default physics related values
	GetSkeletalMeshComponent()->SetSimulatePhysics(true);
	GetSkeletalMeshComponent()->SetEnableGravity(false);
	GetSkeletalMeshComponent()->SetCollisionProfileName(TEXT("BlockAll"));
	GetSkeletalMeshComponent()->bGenerateOverlapEvents = true;

	// Joint control params
	Spring = 59950000.0f;
	Damping = 59950000.0f;
	ForceLimit = 0.0f;
	Velocity = 0.1f;

	// Set fingers and their bone names default values
	AMCHand::SetupHandDefaultValues(HandType);

	// Set skeletal default values
	AMCHand::SetupSkeletalDefaultValues(GetSkeletalMeshComponent());
}



// Called when the game starts or when spawned
void AMCHand::BeginPlay()
{
	Super::BeginPlay();
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

	// Get the name of the property that was changed  
	FName PropertyName = (PropertyChangedEvent.Property != nullptr) ? PropertyChangedEvent.Property->GetFName() : NAME_None;

	// If hand type has been changed
	if ((PropertyName == GET_MEMBER_NAME_CHECKED(AMCHand, HandType)))
	{
		AMCHand::SetupHandDefaultValues(HandType);
	}

	// If the skeletal mesh has been changed
	if ((PropertyName == GET_MEMBER_NAME_CHECKED(AMCHand, SkeletalMeshComponent)))
	{
		AMCHand::SetupSkeletalDefaultValues(GetSkeletalMeshComponent());
	}

	UE_LOG(LogTemp, Warning, TEXT("Selected property name: %s"), *PropertyName.ToString());
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

// Setup skeletal mesh default values
FORCEINLINE void AMCHand::SetupSkeletalDefaultValues(USkeletalMeshComponent* InSkeletalMeshComponent)
{
	if (InSkeletalMeshComponent->GetPhysicsAsset())
	{
		// Hand joint velocity drive
		InSkeletalMeshComponent->SetAllMotorsAngularPositionDrive(true, true);

		// Set drive parameters
		InSkeletalMeshComponent->SetAllMotorsAngularDriveParams(Spring, Damping, ForceLimit);

		UE_LOG(LogTemp, Error, TEXT("AMCHand: SkeletalMeshComponent's angular motors set!"));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("AMCHand: SkeletalMeshComponent's has no PhysicsAsset set!"));
	}
}

// Close hand
void AMCHand::CloseHand(const float Goal)
{

}

// Open hand fingers
void AMCHand::OpenHand(const float Goal)
{

}

// Attach grasped object to hand
void AMCHand::AttachToHand()
{
	//if (Grasped != nullptr) {
	//	Grasped->SetSimulatePhysics(false);
	//	Grasped->AttachToComponent(RootComponent, FAttachmentTransformRules(EAttachmentRule::KeepWorld, EAttachmentRule::KeepWorld, EAttachmentRule::KeepWorld, true));
	//}
}

// Detach grasped object from hand
void AMCHand::DetachFromHand()
{
	//if (Grasped != nullptr) {
	//	Grasped->SetSimulatePhysics(true);
	//	Grasped->DetachFromComponent(FDetachmentTransformRules(EDetachmentRule::KeepWorld, EDetachmentRule::KeepWorld, EDetachmentRule::KeepWorld, true));
	//	Grasped = nullptr;
	//}
}
