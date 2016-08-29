// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "MotionControllerComponent.h"
#include "RMotionControllerCharacter.h"
#include "RMotionControllerCharacterVR.h"
#include "SemLog/RSemEventsExporterSingl.h"
#include "HeadMountedDisplay.h"
#include "RHand.h"

// Sets default values
ARHand::ARHand()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Set default as left hand
	HandType = EControllerHand::Left;

	// Set the default skeletal mesh of the hand
	static ConstructorHelpers::FObjectFinder<USkeletalMesh> LeftSkelMesh(TEXT(
		"SkeletalMesh'/Game/Models/Hands/LeftHand/LeftHand.LeftHand'"));
	// Add skeletal mesh to component
	GetSkeletalMeshComponent()->SetSkeletalMesh(LeftSkelMesh.Object);
	// Simulate physics
	GetSkeletalMeshComponent()->SetSimulatePhysics(true);
	// Disable gravity
	GetSkeletalMeshComponent()->SetEnableGravity(false);
	// Collision default
	GetSkeletalMeshComponent()->SetCollisionProfileName(TEXT("BLockAll"));
	// Generate overlap events
	GetSkeletalMeshComponent()->bGenerateOverlapEvents = true;
	
	// Control bone name
	ControlBoneName = FName("palm_l");
	// PID params
	PGain = 140.0f;
	IGain = 0.0f;
	DGain = 20.0f;
	PIDMaxOutput = 1500.0f;
	PIDMinOutput = -1500.0f;
	// Rotation control param (angular movement strength)
	RotOutStrength = 1000.0f;

	// Flag showing if the finger collisions events are enabled or disabled
	bFingerHitEvents = false;

	// Finger types
	FingerTypes.Add(ERHandLimb::Index);
	FingerTypes.Add(ERHandLimb::Middle);
	FingerTypes.Add(ERHandLimb::Ring);
	FingerTypes.Add(ERHandLimb::Pinky);
	FingerTypes.Add(ERHandLimb::Thumb);

	// Finger collision bone names (used for collision detection)
	CollisionBoneNames.Add(FName("index_03_l"));
	CollisionBoneNames.Add(FName("middle_03_l"));
	CollisionBoneNames.Add(FName("ring_03_l"));
	CollisionBoneNames.Add(FName("pinky_03_l"));
	CollisionBoneNames.Add(FName("thumb_03_l"));

	// Joint control params
	Spring = 59950000.0f;
	Damping = 59950000.0f;
	ForceLimit = 0.0f;
	Velocity = 0.1f;

	// Set default values for the constraint instance
	FixatingGraspConstraintInstance.SetDisableCollision(true);
	FixatingGraspConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
	FixatingGraspConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
	FixatingGraspConstraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);
	FixatingGraspConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
	FixatingGraspConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
	FixatingGraspConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0);

	// Make sure collision hit events are enabled on the hand
	GetSkeletalMeshComponent()->SetNotifyRigidBodyCollision(true);
	
	// Create the dummy static mesh
	DummyStaticMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DummyStaticMesh"));
	// Get a sphere static mesh asset
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMeshAsset(TEXT("StaticMesh'/Engine/BasicShapes/Sphere.Sphere'"));
	if (SphereMeshAsset.Succeeded())
	{
		DummyStaticMesh->SetStaticMesh(SphereMeshAsset.Object);
	}
	// Disable gravity
	DummyStaticMesh->SetEnableGravity(false);
	// Disable mesh collision
	DummyStaticMesh->SetCollisionProfileName(TEXT("NoCollision"));
	DummyStaticMesh->SetWorldScale3D(FVector(0.01f));
	DummyStaticMesh->SetupAttachment(RootComponent);	
}

// Called when the game starts or when spawned
void ARHand::BeginPlay()
{
	Super::BeginPlay();

	//TODO check for the character type not the play mode (e.g. stereo eabled)
	IHeadMountedDisplay* HMD = (IHeadMountedDisplay*)(GEngine->HMDDevice.Get());
	if (HMD && HMD->IsStereoEnabled())
	{
		// Get the motion controller of the character
		for (TActorIterator<ARMotionControllerCharacterVR> CharItr(GetWorld()); CharItr; ++CharItr)
		{
			// Pointer to the motion controller component
			MCComponent = CharItr->GetMotionController(HandType);
			// Stop the loop since we only need the first character (there should be only one)
			break;
		}
	}
	else
	{
		// Get the motion controller of the character
		for (TActorIterator<ARMotionControllerCharacter> CharItr(GetWorld()); CharItr; ++CharItr)
		{
			// Pointer to the motion controller component
			MCComponent = CharItr->GetMotionController(HandType);
			// Stop the loop since we only need the first character (there should be only one)
			break;
		}
	}

	// Disable Tick if no motion controller component or character has been found
	if ((!MCComponent))
	{
		SetActorTickEnabled(false);
		UE_LOG(LogTemp, Error, 
			TEXT("No motion controller (character or component)! Tick disabled for: %s "), *GetName());
	}

	// Get the body to apply forces on for pose control
	ControlBody = GetSkeletalMeshComponent()->GetBodyInstance(ControlBoneName);
	// Check if control body is available
	if (ControlBody)
	{
		// Linear movement PIDs
		HandPID3D = FRPid3d(PGain, IGain, DGain, PIDMaxOutput, PIDMinOutput);
		// Init location of the control body
		CurrLoc = GetSkeletalMeshComponent()->GetBoneLocation(ControlBoneName);
		CurrQuat = GetSkeletalMeshComponent()->GetBoneQuaternion(ControlBoneName);
	}
	else
	{
		SetActorTickEnabled(false);
		UE_LOG(LogTemp, Error, TEXT("No control body was found! Tick disabled for: %s "),
			*GetName());
	}

	///////////////////////////////////////////////////////////////////////
	// Hand joint velocity drive
	GetSkeletalMeshComponent()->SetAllMotorsAngularPositionDrive(true, true);
	// Set drive parameters
	GetSkeletalMeshComponent()->SetAllMotorsAngularDriveParams(Spring, Damping, ForceLimit);
	
	// Set finger types to constraints map
	for (FConstraintInstance* Constr : GetSkeletalMeshComponent()->Constraints)
	{
		// Current constraint joint name
		const FString ConstrName = Constr->JointName.ToString();
		// Iterate hand limbs type
		for (const ERHandLimb Type : FingerTypes)
		{
			// Get the enum type as string
			const FString TypeName = FRUtils::GetEnumValueToString<ERHandLimb>("ERHandLimb", Type);
			// Add to finger map if the constraint name matches the finger type (name)
			if (ConstrName.Contains(TypeName))
			{
				FingerTypeToConstrs.Add(Type, Constr);
			}
		}
	}

	// Get the hands finger collisions map, disable hit events,
	// create finger bone names to finger type Map
	for (const auto BoneName : CollisionBoneNames)
	{
		// Get the body instance
		FBodyInstance* BoneBody = GetSkeletalMeshComponent()->GetBodyInstance(BoneName);
		// Disable collision notification for now (active only during grasping)
		BoneBody->SetInstanceNotifyRBCollision(false);
		// Add the body instance to the map
		FingerBoneNameToBody.Add(BoneName, GetSkeletalMeshComponent()->GetBodyInstance(BoneName));

		// Get bone name as string
		const FString NameStr = BoneName.ToString();
		// Iterate hand limbs type
		for (const ERHandLimb Type : FingerTypes)
		{
			// Get the enum type as string
			const FString TypeName = FRUtils::GetEnumValueToString<ERHandLimb>("ERHandLimb", Type);
			// Add to finger map if the constraint name matches the finger type (name)
			if (NameStr.Contains(TypeName))
			{
				BoneNameToFingerTypeMap.Add(BoneName, Type);
			}
		}
	}

	// Initialize grasp type
	Grasp = new FRGrasp(FingerTypeToConstrs);

	///////////////////////////////////////////////////////////////////////
	// Create grasp fixating constraint
	GraspFixatingConstraint = NewObject<UPhysicsConstraintComponent>(this, UPhysicsConstraintComponent::StaticClass());
	//GraspFixatingConstraint->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepWorldTransform);// TODO needed?

	// Set the grasp constraint instance
	GraspFixatingConstraint->ConstraintInstance = FixatingGraspConstraintInstance;

	// Create dummy actor to attach the grasped object to
	DummyStaticMeshActor = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass());
	DummyStaticMeshActor->SetMobility(EComponentMobility::Movable);
	DummyStaticMeshActor->GetStaticMeshComponent()->SetStaticMesh(DummyStaticMesh->StaticMesh);
	DummyStaticMeshActor->SetRootComponent(DummyStaticMeshActor->GetStaticMeshComponent());
	DummyStaticMeshActor->GetStaticMeshComponent()->SetWorldScale3D(FVector(0.01f));
	DummyStaticMeshActor->SetActorEnableCollision(false);
	DummyStaticMeshActor->AttachToActor(this, FAttachmentTransformRules::SnapToTargetNotIncludingScale);

	// Collision callbacks for the hands
	GetSkeletalMeshComponent()->OnComponentHit.AddDynamic(this, &ARHand::OnFingerHit);

	// Get the player controller, enable and bind inputs
	PC = GetWorld()->GetFirstPlayerController();
	// Enable input
	EnableInput(PC);
	// Set up hand bindings
	if (HandType == EControllerHand::Left)
	{
		PC->InputComponent->BindAxis("OpenHandLeft", this, &ARHand::OpenHand);
		PC->InputComponent->BindAxis("CloseHandLeft", this, &ARHand::CloseHand);
		PC->InputComponent->BindAction("AttachHandLeft", IE_Released, this, &ARHand::AttachToHand);
	}
	else if(HandType == EControllerHand::Right)
	{
		PC->InputComponent->BindAxis("OpenHandRight", this, &ARHand::OpenHand);
		PC->InputComponent->BindAxis("CloseHandRight", this, &ARHand::CloseHand);
		PC->InputComponent->BindAction("AttachHandRight", IE_Released, this, &ARHand::AttachToHand);
	}
	PC->InputComponent->BindAction("SetTrackingOffset", IE_Pressed, this, &ARHand::SetMCTrackingOffset);
}

// Called every frame, used for motion control
void ARHand::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Get the target (motion controller) location and rotation
	const FVector TargetLoc = MCComponent->GetComponentLocation();
	const FQuat TargetQuat = MCComponent->GetComponentQuat();

	//// Location
	// Get the pos errors
	const FVector LocError = TargetLoc - CurrLoc;
	// Compute the pos output
	const FVector LocOutput = HandPID3D.Update(LocError, DeltaTime);
	// Apply force to the hands control body 
	ControlBody->AddForce(LocOutput);
	// Set the current location of the control bodies
	CurrLoc = GetSkeletalMeshComponent()->GetBoneLocation(ControlBoneName);

	//// Rotation
	// Dot product to get costheta
	const float CosTheta = TargetQuat | CurrQuat;
	// Avoid taking the long path around the sphere
	if (CosTheta < 0)
	{
		CurrQuat = CurrQuat * (-1.0f);
	}
	// Use the xyz part of the quat as the rotation velocity
	const FQuat OutputAsQuat = TargetQuat * CurrQuat.Inverse();
	// Get the rotation output
	FVector RotOutput = FVector(OutputAsQuat.X, OutputAsQuat.Y, OutputAsQuat.Z) * RotOutStrength;
	// Apply torque/angularvel to the hands control body 
	ControlBody->SetAngularVelocity(RotOutput, false);
	// Set the current rotation of the control bodies
	CurrQuat = GetSkeletalMeshComponent()->GetBoneQuaternion(ControlBoneName);
}

// Close hand fingers
void ARHand::CloseHand(const float AxisValue)
{
	if (AxisValue == 0)
	{
		// Disable finger hit events
		if (bFingerHitEvents)
		{
			for (auto BoneName : CollisionBoneNames)
			{
				GetSkeletalMeshComponent()->GetBodyInstance(BoneName)->SetInstanceNotifyRBCollision(false);
			}
			// Set finger notification flag
			bFingerHitEvents = false;
		}
		return;
	}

	// Enable finger hit events
	if (!bFingerHitEvents)
	{
		for (auto BoneName : CollisionBoneNames)
		{
			GetSkeletalMeshComponent()->GetBodyInstance(BoneName)->SetInstanceNotifyRBCollision(true);
		}
		// Set finger notification flag
		bFingerHitEvents = true;
	}

	// Update grasping
	Grasp->Update(0.5 * AxisValue);
}

// Attach grasped object to hand
void ARHand::AttachToHand()
{
	// Attach object only if the state is blocked
	// and we have objects colliding with fingers
	if ((Grasp->GetState() == ERGraspState::Blocked) &&
		(HitActorToFingerMMap.Num() > 0))
	{
		// Map colliding component with the number of appearance
		TMap<AActor*, uint8> ActorToCount;

		// Iterate and count the colliding actors
		for (const auto ActorToFinger : HitActorToFingerMMap)
		{
			// If the map already contains the actor
			if (ActorToCount.Contains(ActorToFinger.Key))
			{
				// Increase the count
				ActorToCount[ActorToFinger.Key]++;
			}
			else
			{
				// Add actor to map, set count to 1
				ActorToCount.Add(ActorToFinger.Key, 1);
			}
		}

		// Sort the map (most frequent actor first)
		ActorToCount.ValueSort([](const uint8 A, const uint8 B){return A > B;});

		// If the count is enough, attach actor to hand
		for (const auto ActToCountItr : ActorToCount)
		{
			// TODO make sure hand to hand attachment is avoided
			if (ActToCountItr.Value > 2
				&& !ActToCountItr.Key->GetName().Contains("Hand")
				&& !ActToCountItr.Key->GetName().Contains("Drawer")
				&& !ActToCountItr.Key->GetName().Contains("Area"))
			{


				// Set the grasped component
				GraspedComponent = Cast<AStaticMeshActor>(ActToCountItr.Key)->GetStaticMeshComponent();

				// Disable gravity on the grasped model
				GraspedComponent->SetEnableGravity(false);
				// Set a very small mass scale
				GraspedComponent->SetAllMassScale(0.0000001f);
				GraspedComponent->SetLinearDamping(0.0f);
				GraspedComponent->SetAngularDamping(0.0f);
				// Constrain components
				GraspFixatingConstraint->SetConstrainedComponents(
					DummyStaticMeshActor->GetStaticMeshComponent(), NAME_None,
					GraspedComponent, NAME_None);

				// Set state to attached
				Grasp->SetState(ERGraspState::Attached);

				// Check if semantic event logger is initialized
				if (FRSemEventsExporterSingl::Get().IsInit())
				{
					FRSemEventsExporterSingl::Get().BeginGraspingEvent(
						this, ActToCountItr.Key, GetWorld()->GetTimeSeconds());
				}

				// Attachment constroller feedback
				FLatentActionInfo ActionInfo;
				ActionInfo.CallbackTarget = this;
				if (HandType == EControllerHand::Left)
				{
					PC->PlayDynamicForceFeedback(1.0f, 0.05f, true, false, false, false, EDynamicForceFeedbackAction::Start, ActionInfo);
				}
				else if (HandType == EControllerHand::Right)
				{
					PC->PlayDynamicForceFeedback(1.0f, 0.05f, false, false, true, true, EDynamicForceFeedbackAction::Start, ActionInfo);
				}
			}
			// We only need the first value with the most contact counts
			break;
		}
		// Free the finger collisions
		HitActorToFingerMMap.Empty();
	}
}

// Open hand fingers
void ARHand::OpenHand(const float AxisValue)
{
	if (AxisValue == 0)
	{
		return;
	}

	// Free the colliding fingers map
	if (HitActorToFingerMMap.Num() != 0)
	{
		HitActorToFingerMMap.Empty();
	}

	// Detach object if case
	if (Grasp->GetState() == ERGraspState::Attached)
	{
		if (GraspedComponent)
		{
			// Enable gravity to the grasped component
			GraspedComponent->SetEnableGravity(true);
			// Set mass scale back
			GraspedComponent->SetAllMassScale(1.0f);
			GraspedComponent->SetLinearDamping(0.1f);
			// Break constraint
			GraspFixatingConstraint->BreakConstraint();

			// Check if semantic event logger is initialized
			if (FRSemEventsExporterSingl::Get().IsInit())
			{
				FRSemEventsExporterSingl::Get().EndGraspingEvent(
					this, GraspedComponent->GetAttachmentRootActor(), GetWorld()->GetTimeSeconds());
			}

			FLatentActionInfo ActionInfo;
			ActionInfo.CallbackTarget = this;
			if (HandType == EControllerHand::Left)
			{
				PC->PlayDynamicForceFeedback(1.0f, 0.05f, true, false, false, false, EDynamicForceFeedbackAction::Start, ActionInfo);
			}
			else if (HandType == EControllerHand::Right)
			{
				PC->PlayDynamicForceFeedback(1.0f, 0.05f, false, false, true, true, EDynamicForceFeedbackAction::Start, ActionInfo);
			}
		}
		// Set state to free
		Grasp->SetState(ERGraspState::Free);
	}

	// Update grasping
	Grasp->Update( - AxisValue);
}

// Set tracking offset
void ARHand::SetMCTrackingOffset()
{
}

// Hand collision callback
void ARHand::OnFingerHit(UPrimitiveComponent* SelfComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	// Check collisions if grasp state is free, 
	if (Grasp->GetState() == ERGraspState::Free)
	{
		// Get the finger in collision
		const ERHandLimb* Finger = BoneNameToFingerTypeMap.Find(Hit.BoneName);

		// Block finger if free, and not colliding with self
		if ((Finger != nullptr) && (!Grasp->IsFingerBlocked(*Finger)) && (SelfComp != OtherComp))
		{
			// Block finger
			Grasp->BlockFinger(*Finger);
			// Add colliding component to the map
			HitActorToFingerMMap.Add(OtherActor, *Finger);

			FLatentActionInfo ActionInfo;
			ActionInfo.CallbackTarget = this;
			if (HandType == EControllerHand::Left)
			{
				PC->PlayDynamicForceFeedback(0.3f, 0.05f, true, false, false, false, EDynamicForceFeedbackAction::Start, ActionInfo);
			}
			else if (HandType == EControllerHand::Right)
			{
				PC->PlayDynamicForceFeedback(0.3f, 0.05f, false, false, true, true, EDynamicForceFeedbackAction::Start, ActionInfo);
			}
		}
	}
}
