// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#include "Hand.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "Components/SkeletalMeshComponent.h"

// Sets default values
AHand::AHand()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Fixation grasp parameters	
	bEnableFixationGrasp = true;
	bGraspHeld = false;
	MaxAttachMass = 4.5f;
	MaxAttachLength = 50.f;
	// Set attachement collision component
	AttachmentCollision = CreateDefaultSubobject<USphereComponent>(TEXT("AttachmentCollision"));
	AttachmentCollision->SetupAttachment(GetRootComponent());
	AttachmentCollision->InitSphereRadius(5.f);

	// Set default as left hand
	HandType = EHandType::Left;

	// Set skeletal mesh default physics related values
	USkeletalMeshComponent* const SkelComp = GetSkeletalMeshComponent();
	SkelComp->SetSimulatePhysics(true);
	SkelComp->SetEnableGravity(false);
	SkelComp->SetCollisionProfileName(TEXT("BlockAll"));
	SkelComp->bGenerateOverlapEvents = true;

	// Angular drive default values
	//AngularDriveMode = EAngularDriveMode::SLERP;
	Spring = 950000.0f;
	Damping = 950000.0f;
	ForceLimit = 0.0f;
	
	// Set fingers and their bone names default values
	AHand::SetupHandDefaultValues(HandType);

	// Set skeletal default values
	//AHand::SetupSkeletalDefaultValues(GetSkeletalMeshComponent());

	GraspPtr = MakeShareable(new Grasp());
}

// Called when the game starts or when spawned
void AHand::BeginPlay()
{
	Super::BeginPlay();

	AttachmentCollision->OnComponentBeginOverlap.AddDynamic(this, &AHand::OnAttachmentCollisionBeginOverlap);
	AttachmentCollision->OnComponentEndOverlap.AddDynamic(this, &AHand::OnAttachmentCollisionEndOverlap);

	// Setup the values for controlling the hand fingers
	AHand::SetupAngularDriveValues(EAngularDriveMode::TwistAndSwing);

	FHandOrientation HandOrient;
	GraspPtr->DriveToHandOrientation(HandOrient, this);

	FFingerOrientation FingerOrient;
	GraspPtr->DriveToFingerOrientation(FingerOrient, Pinky);
}

// Called every frame, used for motion control
void AHand::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

// Update default values if properties have been changed in the editor
#if WITH_EDITOR
void AHand::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	// Call the base class version  
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// Get the name of the property that was changed  
	FName PropertyName = (PropertyChangedEvent.Property != nullptr) ? PropertyChangedEvent.Property->GetFName() : NAME_None;

	// If hand type has been changed
	if ((PropertyName == GET_MEMBER_NAME_CHECKED(AHand, HandType)))
	{
		AHand::SetupHandDefaultValues(HandType);
	}

	// If the skeletal mesh has been changed
	if ((PropertyName == GET_MEMBER_NAME_CHECKED(AHand, GetSkeletalMeshComponent())))
	{
		//AHand::SetupSkeletalDefaultValues(GetSkeletalMeshComponent());
	}

	UE_LOG(LogTemp, Warning, TEXT("Selected property name: %s"), *PropertyName.ToString());
}
#endif  

// Object in reach for grasping
void AHand::OnAttachmentCollisionBeginOverlap(class UPrimitiveComponent* HitComp, class AActor* OtherActor,
	class UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult & SweepResult)
{
	// Object in reach
	if (!GraspedObject)
	{
		// Hand is free, check if object is graspable
		if (IsGraspable(OtherActor))
		{
			GraspableObjects.Emplace(Cast<AStaticMeshActor>(OtherActor));		
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("Overlap begin: %s | GraspableObjects size: %i"),
		*OtherActor->GetName(), GraspableObjects.Num());
}

// Object out of reach for grasping
void AHand::OnAttachmentCollisionEndOverlap(class UPrimitiveComponent* HitComp, class AActor* OtherActor,
	class UPrimitiveComponent* OtherComp, int32 OtherBodyIndex)
{	
	// Object out of reach
	if (!GraspedObject)
	{
		// If present, remove from the graspable objects
		GraspableObjects.Remove(Cast<AStaticMeshActor>(OtherActor));
	}
	UE_LOG(LogTemp, Warning, TEXT("Overlap end: %s | GraspableObjects size: %i"), 
		*OtherActor->GetName(), GraspableObjects.Num());
}

// Check if object is graspable
bool AHand::IsGraspable(AActor* InActor)
{
	// Check if the static mesh actor can be grasped
	AStaticMeshActor* const SMActor = Cast<AStaticMeshActor>(InActor);
	if (SMActor)
	{
		UStaticMeshComponent* const SMComp = SMActor->GetStaticMeshComponent();
		if ((SMActor->IsRootComponentMovable()) && 
			(SMComp) &&
			(SMComp->GetMass() < MaxAttachMass) &&
			(SMActor->GetComponentsBoundingBox().GetSize().Size() < MaxAttachLength))
		{
			// Actor is movable
			// Actor has a static mesh component
			// Actor is not too heavy
			// Actor is not too large
			// Object can be attached
			return true;
		}
	}
	// Actor cannot be attached
	return false;
}

// Hold grasp in the current position
FORCEINLINE void AHand::HoldGrasp()
{
	for (const auto& ConstrMapItr : Thumb.FingerPartToConstraint)
	{
		ConstrMapItr.Value->SetAngularOrientationTarget(FQuat(FRotator(
			ConstrMapItr.Value->GetCurrentSwing2(),
			ConstrMapItr.Value->GetCurrentSwing1(),
			ConstrMapItr.Value->GetCurrentTwist())));
	}
	for (const auto& ConstrMapItr : Index.FingerPartToConstraint)
	{
		ConstrMapItr.Value->SetAngularOrientationTarget(FQuat(FRotator(
			ConstrMapItr.Value->GetCurrentSwing2(),
			ConstrMapItr.Value->GetCurrentSwing1(),
			ConstrMapItr.Value->GetCurrentTwist())));
	}
	for (const auto& ConstrMapItr : Middle.FingerPartToConstraint)
	{
		ConstrMapItr.Value->SetAngularOrientationTarget(FQuat(FRotator(
			ConstrMapItr.Value->GetCurrentSwing2(),
			ConstrMapItr.Value->GetCurrentSwing1(),
			ConstrMapItr.Value->GetCurrentTwist())));
	}
	for (const auto& ConstrMapItr : Ring.FingerPartToConstraint)
	{
		ConstrMapItr.Value->SetAngularOrientationTarget(FQuat(FRotator(
			ConstrMapItr.Value->GetCurrentSwing2(),
			ConstrMapItr.Value->GetCurrentSwing1(),
			ConstrMapItr.Value->GetCurrentTwist())));
	}
	for (const auto& ConstrMapItr : Pinky.FingerPartToConstraint)
	{
		ConstrMapItr.Value->SetAngularOrientationTarget(FQuat(FRotator(
			ConstrMapItr.Value->GetCurrentSwing2(),
			ConstrMapItr.Value->GetCurrentSwing1(),
			ConstrMapItr.Value->GetCurrentTwist())));
	}

	bGraspHeld = true;
}

// Setup hand default values
FORCEINLINE void AHand::SetupHandDefaultValues(EHandType InHandType)
{
	if (InHandType == EHandType::Left)
	{
		Thumb.FingerType = EFingerType::Thumb;
		Thumb.FingerPartToBoneName.Add(EFingerPart::Proximal, "thumb_01_l");
		Thumb.FingerPartToBoneName.Add(EFingerPart::Intermediate, "thumb_02_l");
		Thumb.FingerPartToBoneName.Add(EFingerPart::Distal, "thumb_03_l");

		Index.FingerType = EFingerType::Index;
		Index.FingerPartToBoneName.Add(EFingerPart::Proximal, "index_01_l");
		Index.FingerPartToBoneName.Add(EFingerPart::Intermediate, "index_02_l");
		Index.FingerPartToBoneName.Add(EFingerPart::Distal, "index_03_l");

		Middle.FingerType = EFingerType::Middle;
		Middle.FingerPartToBoneName.Add(EFingerPart::Proximal, "middle_01_l");
		Middle.FingerPartToBoneName.Add(EFingerPart::Intermediate, "middle_02_l");
		Middle.FingerPartToBoneName.Add(EFingerPart::Distal, "middle_03_l");

		Ring.FingerType = EFingerType::Ring;
		Ring.FingerPartToBoneName.Add(EFingerPart::Proximal, "ring_01_l");
		Ring.FingerPartToBoneName.Add(EFingerPart::Intermediate, "ring_02_l");
		Ring.FingerPartToBoneName.Add(EFingerPart::Distal, "ring_03_l");

		Pinky.FingerType = EFingerType::Pinky;
		Pinky.FingerPartToBoneName.Add(EFingerPart::Proximal, "pinky_01_l");
		Pinky.FingerPartToBoneName.Add(EFingerPart::Intermediate, "pinky_02_l");
		Pinky.FingerPartToBoneName.Add(EFingerPart::Distal, "pinky_03_l");
	}
	else if (InHandType == EHandType::Right)
	{
		Thumb.FingerType = EFingerType::Thumb;
		Thumb.FingerPartToBoneName.Add(EFingerPart::Proximal, "thumb_01_r");
		Thumb.FingerPartToBoneName.Add(EFingerPart::Intermediate, "thumb_02_r");
		Thumb.FingerPartToBoneName.Add(EFingerPart::Distal, "thumb_03_r");

		Index.FingerType = EFingerType::Index;
		Index.FingerPartToBoneName.Add(EFingerPart::Proximal, "index_01_r");
		Index.FingerPartToBoneName.Add(EFingerPart::Intermediate, "index_02_r");
		Index.FingerPartToBoneName.Add(EFingerPart::Distal, "index_03_r");

		Middle.FingerType = EFingerType::Middle;
		Middle.FingerPartToBoneName.Add(EFingerPart::Proximal, "middle_01_r");
		Middle.FingerPartToBoneName.Add(EFingerPart::Intermediate, "middle_02_r");
		Middle.FingerPartToBoneName.Add(EFingerPart::Distal, "middle_03_r");

		Ring.FingerType = EFingerType::Ring;
		Ring.FingerPartToBoneName.Add(EFingerPart::Proximal, "ring_01_r");
		Ring.FingerPartToBoneName.Add(EFingerPart::Intermediate, "ring_02_r");
		Ring.FingerPartToBoneName.Add(EFingerPart::Distal, "ring_03_r");

		Pinky.FingerType = EFingerType::Pinky;
		Pinky.FingerPartToBoneName.Add(EFingerPart::Proximal, "pinky_01_r");
		Pinky.FingerPartToBoneName.Add(EFingerPart::Intermediate, "pinky_02_r");
		Pinky.FingerPartToBoneName.Add(EFingerPart::Distal, "pinky_03_r");
	}
}

// Setup skeletal mesh default values
FORCEINLINE void AHand::SetupSkeletalDefaultValues(USkeletalMeshComponent* InSkeletalMeshComponent)
{
	if (InSkeletalMeshComponent->GetPhysicsAsset())
	{
		// Hand joint velocity drive
		InSkeletalMeshComponent->SetAllMotorsAngularPositionDrive(true, true);

		// Set drive parameters
		InSkeletalMeshComponent->SetAllMotorsAngularDriveParams(Spring, Damping, ForceLimit);

		UE_LOG(LogTemp, Error, TEXT("AHand: SkeletalMeshComponent's angular motors set!"));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("AHand: SkeletalMeshComponent's has no PhysicsAsset set!"));
	}
}

// Setup fingers angular drive values
FORCEINLINE void AHand::SetupAngularDriveValues(EAngularDriveMode::Type DriveMode)
{
	USkeletalMeshComponent* const SkelMeshComp = GetSkeletalMeshComponent();
	if (Thumb.SetFingerPartsConstraints(SkelMeshComp->Constraints))
	{
		Thumb.SetFingerDriveMode(DriveMode, Spring, Damping, ForceLimit);
	}
	if (Index.SetFingerPartsConstraints(SkelMeshComp->Constraints))
	{
		Index.SetFingerDriveMode(DriveMode, Spring, Damping, ForceLimit);
	}
	if (Middle.SetFingerPartsConstraints(SkelMeshComp->Constraints))
	{
		Middle.SetFingerDriveMode(DriveMode, Spring, Damping, ForceLimit);
	}
	if (Ring.SetFingerPartsConstraints(SkelMeshComp->Constraints))
	{
		Ring.SetFingerDriveMode(DriveMode, Spring, Damping, ForceLimit);
	}
	if (Pinky.SetFingerPartsConstraints(SkelMeshComp->Constraints))
	{
		Pinky.SetFingerDriveMode(DriveMode, Spring, Damping, ForceLimit);
	}
}

// Update the grasp pose
void AHand::UpdateGrasp(const float Goal)
{
	if (!GraspedObject)
	{
		for (const auto& ConstrMapItr : Thumb.FingerPartToConstraint)
		{
			ConstrMapItr.Value->SetAngularOrientationTarget(FQuat(FRotator(0.f, 0.f, Goal * 100.f)));
		}
		for (const auto& ConstrMapItr : Index.FingerPartToConstraint)
		{
			ConstrMapItr.Value->SetAngularOrientationTarget(FQuat(FRotator(0.f, 0.f, Goal * 100.f)));
		}
		for (const auto& ConstrMapItr : Middle.FingerPartToConstraint)
		{
			ConstrMapItr.Value->SetAngularOrientationTarget(FQuat(FRotator(0.f, 0.f, Goal * 100.f)));
		}
		for (const auto& ConstrMapItr : Ring.FingerPartToConstraint)
		{
			ConstrMapItr.Value->SetAngularOrientationTarget(FQuat(FRotator(0.f, 0.f, Goal * 100.f)));
		}
		for (const auto& ConstrMapItr : Pinky.FingerPartToConstraint)
		{
			ConstrMapItr.Value->SetAngularOrientationTarget(FQuat(FRotator(0.f, 0.f, Goal * 100.f)));
		}
	}
	else if(!bGraspHeld)
	{
		AHand::HoldGrasp();
	}
}

// Attach grasped object to hand
void AHand::AttachToHand()
{
	if ((!GraspedObject) && (GraspableObjects.Num() > 0))
	{		
		GraspedObject = GraspableObjects.Pop();
		GraspedObject->GetStaticMeshComponent()->SetSimulatePhysics(false);
		GraspedObject->AttachToComponent(GetRootComponent(), FAttachmentTransformRules(
			EAttachmentRule::KeepWorld, EAttachmentRule::KeepWorld, EAttachmentRule::KeepWorld, true));
	}
	UE_LOG(LogTemp, Warning, TEXT("Attached to hand"));
}

// Detach grasped object from hand
void AHand::DetachFromHand()
{
	if (GraspedObject)
	{
		GraspedObject->GetStaticMeshComponent()->SetSimulatePhysics(true);
		GraspedObject->GetStaticMeshComponent()->DetachFromComponent(FDetachmentTransformRules(
			EDetachmentRule::KeepWorld, EDetachmentRule::KeepWorld, EDetachmentRule::KeepWorld, true));
		GraspedObject = nullptr;
		bGraspHeld = false;
	}
	UE_LOG(LogTemp, Warning, TEXT("Detach from hand"));
}
