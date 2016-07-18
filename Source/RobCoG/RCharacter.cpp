// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RUtils.h"
#include "RGraspPiano.h" //TODO remove this?
#include "RCharacter.h"

// Sets default values
ARCharacter::ARCharacter(const FObjectInitializer& ObjectInitializer)
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	//TODO separate control and grasp into two classes
	// set all their values here in local variables and then load in their constructor

	// Visualize motion controller debug arrows
	bVisTargetArrows = true;

	// Flag showing if the finger collisions events are enabled or disabled
	bLeftFingerHitEvents = false;
	bRightFingerHitEvents = false;

	// Bone names
	LeftControlBoneName = FName("palm_l");
	RightControlBoneName = FName("palm_r");

	// TODO check that bones exist (in begin play)
	// Left hands finger collision (sensor) names
	LeftCollisionBoneNames.Add(FName("palm_l"));//TODO check if palm needed
	LeftCollisionBoneNames.Add(FName("index_03_l"));
	LeftCollisionBoneNames.Add(FName("middle_03_l"));
	LeftCollisionBoneNames.Add(FName("ring_03_l"));
	LeftCollisionBoneNames.Add(FName("pinky_03_l"));
	LeftCollisionBoneNames.Add(FName("thumb_03_l"));
	// Right hands finger collision (sensor) names
	RightCollisionBoneNames.Add(FName("palm_r"));
	RightCollisionBoneNames.Add(FName("index_03_r"));
	RightCollisionBoneNames.Add(FName("middle_03_r"));
	RightCollisionBoneNames.Add(FName("ring_03_r"));
	RightCollisionBoneNames.Add(FName("pinky_03_r"));
	RightCollisionBoneNames.Add(FName("thumb_03_r"));

	// PID params
	PGain = 140.0f;
	IGain = 0.0f;
	DGain = 20.0f;
	PIDMaxOutput = 1500.0f;
	PIDMinOutput = -1500.0f;

	// Rotation control param
	RotOutStrength = 1000.0f;

	// Joint control params
	Spring = 59950000.0f;
	Damping = 59950000.0f;
	ForceLimit = 0.0f;
	Velocity = 0.1f;

	// Finger types
	FingerTypes.Add(ERHandLimb::Index);
	FingerTypes.Add(ERHandLimb::Middle);
	FingerTypes.Add(ERHandLimb::Ring);
	FingerTypes.Add(ERHandLimb::Pinky);
	FingerTypes.Add(ERHandLimb::Thumb);

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
	MCOriginComponent = CreateDefaultSubobject<USceneComponent>(TEXT("MCOriginComponent"));
	// Attach Offset to root
	MCOriginComponent->SetupAttachment(RootComponent);
	// Position of the offset
	MCOriginComponent->RelativeLocation = FVector(80.0f, 0.0f, 0.0f);

	// Create left/right motion controller
	LeftMC = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("LeftMotionController"));
	RightMC = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("RightMotionController"));
	// Attach controllers to root component
	LeftMC->SetupAttachment(MCOriginComponent);
	RightMC->SetupAttachment(MCOriginComponent);
	// Set the mapped hand (from the Motion Controller)
	LeftMC->Hand = EControllerHand::Left;
	RightMC->Hand = EControllerHand::Right;

	// TODO check if arrows should be created in begin play
	// Create left/right target vis arrows
	LeftTargetArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("LeftVisArrow"));
	RightTargetArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("RightVisArrow"));
	// Set arrow sizes
	LeftTargetArrow->ArrowSize = 0.1;
	RightTargetArrow->ArrowSize = 0.1;
	// Attach vis arrow to motion controllers
	LeftTargetArrow->SetupAttachment(LeftMC);
	RightTargetArrow->SetupAttachment(RightMC);

	// Default type of the hands
	LeftHand = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("LeftSkeletalMesh"));
	RightHand = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("RightSkeletalMesh"));
	// Get default skeletal mesh of the right / left hands
	static ConstructorHelpers::FObjectFinder<USkeletalMesh> LeftSkelMesh(TEXT(
		"SkeletalMesh'/Game/Models/Hands/LeftHand/LeftHand.LeftHand'"));
	static ConstructorHelpers::FObjectFinder<USkeletalMesh> RightSkelMesh(TEXT(
		"SkeletalMesh'/Game/Models/Hands/RightHand/RightHand.RightHand'"));
	// Set default skeletal mesh
	LeftHand->SetSkeletalMesh(LeftSkelMesh.Object);
	RightHand->SetSkeletalMesh(RightSkelMesh.Object);
	// Simulate physics
	LeftHand->SetSimulatePhysics(true);
	RightHand->SetSimulatePhysics(true);
	// Disable gravity
	LeftHand->SetEnableGravity(false);
	RightHand->SetEnableGravity(false);
	// Collision default
	LeftHand->SetCollisionProfileName(TEXT("BLockAll"));
	RightHand->SetCollisionProfileName(TEXT("BLockAll"));
	// Attach hands to the motion controller offset parent
	LeftHand->SetupAttachment(MCOriginComponent);
	RightHand->SetupAttachment(MCOriginComponent);

	// Set default values for the constraint instance
	FixatingGraspConstraintInstance.SetDisableCollision(true);
	FixatingGraspConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
	FixatingGraspConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
	FixatingGraspConstraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);
	FixatingGraspConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
	FixatingGraspConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
	FixatingGraspConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0);

	// TODO check if this should be created in begin play 
	// (if we want a flag to allow fixation grasp)
	// (then use NewObject<UPhysicsConstraintComponent>(LeftHand)
	// Create the fixating grasp constraint components
	LeftGraspFixatingConstraint = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LeftFixatingGraspConstr"));
	RightGraspFixatingConstraint = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RightFixatingGraspConstr"));
	///////////////////////////////////////////////////
	// .CPP
	// CONSTRUCTOR
	// Create the physics constraint
	MConstraintComp = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("MFixatingGraspConstr"));
	bAttached = false;
	///////////////////////////////////////////////////
	// Attach the fixating grasp components to the hands
	LeftGraspFixatingConstraint->SetupAttachment(LeftHand);
	RightGraspFixatingConstraint->SetupAttachment(RightHand);

	// Create the static mesh components as fixating grasp helpers
	LeftFixatingGraspStaticMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LeftFixatingGraspStaticMesh"));
	RightFixatingGraspStaticMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RightFixatingGraspStaticMesh"));
	GraspHelperStaticMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("GraspHelperStaticMesh"));
	// Get the static mesh asset
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMeshAsset(TEXT("StaticMesh'/Engine/BasicShapes/Sphere.Sphere'"));

	// Attach sphere mesh asset to the components
	if (SphereMeshAsset.Succeeded())
	{
		LeftFixatingGraspStaticMesh->SetStaticMesh(SphereMeshAsset.Object);
		LeftFixatingGraspStaticMesh->SetWorldScale3D(FVector(0.01f));
		RightFixatingGraspStaticMesh->SetStaticMesh(SphereMeshAsset.Object);
		RightFixatingGraspStaticMesh->SetWorldScale3D(FVector(0.01f));
		GraspHelperStaticMesh->SetStaticMesh(SphereMeshAsset.Object);
		GraspHelperStaticMesh->SetWorldScale3D(FVector(0.3f));
	}
	// Attach components to hand
	LeftFixatingGraspStaticMesh->SetupAttachment(LeftHand);
	RightFixatingGraspStaticMesh->SetupAttachment(RightHand);
	
	// Make sure collision hit events are enabled on the hands
	LeftHand->SetNotifyRigidBodyCollision(true);
	RightHand->SetNotifyRigidBodyCollision(true);
}

// Called when the game starts or when spawned
void ARCharacter::BeginPlay()
{
	Super::BeginPlay();

	//TODO check that a physics asset is present
	//TODO check that all bones exist (return error if not)

	if (bVisTargetArrows)
	{
		// Set arrows to visible
		LeftTargetArrow->SetHiddenInGame(false);
		RightTargetArrow->SetHiddenInGame(false);
	}

	// Linear movement PIDs
	LeftPID3D = FRPid3d(PGain, IGain, DGain, PIDMaxOutput, PIDMinOutput);
	RightPID3D = FRPid3d(PGain, IGain, DGain, PIDMaxOutput, PIDMinOutput);

	// Get the bodies to apply forces on for pose control
	LeftControlBody = LeftHand->GetBodyInstance(LeftControlBoneName);
	RightControlBody = RightHand->GetBodyInstance(RightControlBoneName);
	// Check if the bodies not set
	if (!LeftControlBody || !RightControlBody)
	{
		UE_LOG(LogTemp, Error,
			TEXT("LeftControlBody or RightControlBody not set, check if [%s] or [%s] exists!"),
			*LeftControlBoneName.ToString(), *RightControlBoneName.ToString());
	}
	
	// Hand joint velocity drive
	LeftHand->SetAllMotorsAngularPositionDrive(true, true);
	RightHand->SetAllMotorsAngularPositionDrive(true, true);

	// Set drive parameters
	LeftHand->SetAllMotorsAngularDriveParams(Spring, Damping, ForceLimit);
	RightHand->SetAllMotorsAngularDriveParams(Spring, Damping, ForceLimit);
	
	// Lambda to map the hand fingers constraints to its type (index, middle etc.)
	auto GetFingerToConstrLambda = [&](TArray<FConstraintInstance*>& Constraints)
	{
		// Finger type to contraint map
		TMultiMap<ERHandLimb, FConstraintInstance*> FingersMap;
		// Iterate constraints
		for (FConstraintInstance* Constr : Constraints)
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
					FingersMap.Add(Type, Constr);
				}
			}
		}
		return FingersMap;
	};

	// Get the left and right hand finger constraints
	LFingerTypeToConstrs = GetFingerToConstrLambda(LeftHand->Constraints);
	RFingerTypeToConstrs = GetFingerToConstrLambda(RightHand->Constraints);
	
	// Get the hands finger collisions map, make sure hit events are set
	auto GetFingerCollisionBonesLambda = [](USkeletalMeshComponent* Hand, TArray<FName>& CollisionBoneNames)
	{
		// Hand finger bone name to its body Map
		TMap<FName, FBodyInstance*> FingerBoneNameToBodyMap;
		// Iterate the bone names
		for (auto BoneName : CollisionBoneNames)
		{
			// Get the body instance
			FBodyInstance* BoneBody = Hand->GetBodyInstance(BoneName);
			// Disable collision notification for now (active only during grasping)
			BoneBody->SetInstanceNotifyRBCollision(false);
			// Add the body instance to the map
			FingerBoneNameToBodyMap.Add(BoneName, Hand->GetBodyInstance(BoneName));
		}
		// Return the map
		return FingerBoneNameToBodyMap;
	};

	// Get the left / right hand finger sensor collision map
	LFingerBoneNameToBody = GetFingerCollisionBonesLambda(LeftHand, LeftCollisionBoneNames);
	RFingerBoneNameToBody = GetFingerCollisionBonesLambda(RightHand, RightCollisionBoneNames);
	
	// Add bone names to limb type Map
	auto AddBoneNamesToLimbsLambda = [&](TArray<FName> BoneNames, TMap<FName, ERHandLimb>& NameToLimbMap)
	{
		// Iterate the bone names
		for (auto Name : BoneNames)
		{
			// Get bone name as string
			const FString NameStr = Name.ToString();

			// Iterate hand limbs type
			for (const ERHandLimb Type : FingerTypes)
			{
				// Get the enum type as string
				const FString TypeName = FRUtils::GetEnumValueToString<ERHandLimb>("ERHandLimb", Type);
				// Add to finger map if the constraint name matches the finger type (name)
				if (NameStr.Contains(TypeName))
				{
					NameToLimbMap.Add(Name, Type);
				}
			}
		}
	};

	// Create finger bone names to hand limb type Map
	AddBoneNamesToLimbsLambda(LeftCollisionBoneNames, BoneNameToLimbMap);
	AddBoneNamesToLimbsLambda(RightCollisionBoneNames, BoneNameToLimbMap);

	// Set init location of the control body
	LeftCurrLoc = LeftHand->GetBoneLocation(LeftControlBoneName);
	LeftCurrQuat = LeftHand->GetBoneQuaternion(LeftControlBoneName);
	RightCurrLoc = RightHand->GetBoneLocation(RightControlBoneName);
	RightCurrQuat = RightHand->GetBoneQuaternion(RightControlBoneName);

	// Initialize grasp type
	LeftGrasp = new FRGrasp(LFingerTypeToConstrs);
	RightGrasp = new FRGrasp(RFingerTypeToConstrs);

	// Set the grasp constraint instance
	LeftGraspFixatingConstraint->ConstraintInstance = FixatingGraspConstraintInstance;
	RightGraspFixatingConstraint->ConstraintInstance = FixatingGraspConstraintInstance;

	// Collision callbacks for the hands
	LeftHand->OnComponentHit.AddDynamic(this, &ARCharacter::OnHitLeft);
	RightHand->OnComponentHit.AddDynamic(this, &ARCharacter::OnHitRight);


	/////////////////////////////////////////////////////////////////
	for (TActorIterator<AStaticMeshActor> ActorItr(GetWorld()); ActorItr; ++ActorItr)
	{
		if (ActorItr->GetName().Equals("CAct1"))
		{
			StaticMeshAct1 = *ActorItr;
			UE_LOG(LogTemp, Warning, TEXT("StaticMeshAct1: %s SET"), *StaticMeshAct1->GetName());
		}
		else if (ActorItr->GetName().Equals("CAct2"))
		{
			StaticMeshAct2 = *ActorItr;
			UE_LOG(LogTemp, Warning, TEXT("StaticMeshAct2: %s SET"), *StaticMeshAct2->GetName());
		}
		else if (ActorItr->GetName().Equals("GraspCone"))
		{
			AttachedConeAct = *ActorItr;
			UE_LOG(LogTemp, Warning, TEXT("AttachedCone: %s SET"), *AttachedConeAct->GetName());
		}
	}
	for (TActorIterator<ASkeletalMeshActor> SkelActorItr(GetWorld()); SkelActorItr; ++SkelActorItr)
	{
		
		if (SkelActorItr->GetName().Equals("LeftHand_2"))
		{
			HandSkelAct = *SkelActorItr;
			UE_LOG(LogTemp, Warning, TEXT("HandSkelAct: %s SET"), *HandSkelAct->GetName());
		}
	}

	//Set Constraint Instance!
	MConstraintComp->ConstraintInstance = FixatingGraspConstraintInstance;
	FVector NewLocation = LeftHand->GetComponentLocation() + FVector(0.f, 0.f, 300.f);
	FRotator Rot = FRotator::ZeroRotator;

	FTransform transf = FTransform();
	transf.SetLocation(NewLocation);

	GraspHelperStaticMeshAct = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass());
	//GraspHelperStaticMeshAct = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), transf);
	//GraspHelperStaticMeshAct = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), &NewLocation, &Rot);

	GraspHelperStaticMeshAct->SetMobility(EComponentMobility::Movable);
	GraspHelperStaticMeshAct->StaticMeshComponent = GraspHelperStaticMesh;
	//GraspHelperStaticMeshAct->GetStaticMeshComponent()->SetSimulatePhysics(true);
	GraspHelperStaticMeshAct->GetStaticMeshComponent()->SetCollisionProfileName(TEXT("BLockAllDynamic"));


	GraspHelperStaticMeshAct->AttachToComponent(LeftHand, FAttachmentTransformRules::KeepWorldTransform);
	//GraspHelperStaticMeshAct->AttachToActor(HandSkelAct, FAttachmentTransformRules::SnapToTargetIncludingScale);
	//GraspHelperStaticMeshAct->GetStaticMeshComponent()->SetStaticMesh(MeshAsset);
	/////////////////////////////////////////////////////////////////

	//PrintLong(TEXT("PhysicsConstraint between ASkeletalMeshActor <-> AStaticMeshActor"));

}

// Called every frame
void ARCharacter::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

	//// Compute location
	// Get the target location (motion controller loc)
	const FVector LeftTargetLoc = LeftMC->GetComponentLocation();
	const FVector RightTargetLoc = RightMC->GetComponentLocation();

	// Get the loc errors
	const FVector LeftError = LeftTargetLoc - LeftCurrLoc;
	const FVector RightError = RightTargetLoc - RightCurrLoc;

	// Compute the pos output
	const FVector LeftLocOutput = LeftPID3D.Update(LeftError, DeltaTime);
	const FVector RightLocOutput = RightPID3D.Update(RightError, DeltaTime);

	// Apply force to the hands control body 
	LeftControlBody->AddForce(FVector(LeftLocOutput));
	RightControlBody->AddForce(FVector(RightLocOutput));

	// Set the current location of the control bodies
	LeftCurrLoc = LeftHand->GetBoneLocation(LeftControlBoneName);
	RightCurrLoc = RightHand->GetBoneLocation(RightControlBoneName);

	//// Compute rotation
	// Get the target rotation (motion controller loc)
	const FQuat LeftTargetQuat = LeftMC->GetComponentQuat();
	const FQuat RightTargetQuat = RightMC->GetComponentQuat();

	// Lambda to compute the rotational output velocity
	auto RotOutputLambda = [&](const FQuat& TargetQuat, FQuat CurrQuat)
	{
		// Dot product to get costheta
		const float CosTheta = TargetQuat | CurrQuat;
		// Avoid taking the long path around the sphere
		if (CosTheta < 0)
		{
			CurrQuat = CurrQuat * (-1.0f);
		}
		// Use the xyz part of the quat as the rotation velocity
		const FQuat OutputAsQuat = TargetQuat * CurrQuat.Inverse();
		return FVector(OutputAsQuat.X, OutputAsQuat.Y, OutputAsQuat.Z) * RotOutStrength;
	};

	// Compute rotation output
	const FVector LeftOuputRot = RotOutputLambda(LeftTargetQuat, LeftCurrQuat);
	const FVector RightOuputRot = RotOutputLambda(RightTargetQuat, RightCurrQuat);

	// Apply torque/angularvel to the hands control body 
	LeftControlBody->SetAngularVelocity(LeftOuputRot, false);
	RightControlBody->SetAngularVelocity(RightOuputRot, false);

	// Set the current rotation of the control bodies
	LeftCurrQuat = LeftHand->GetBoneQuaternion(LeftControlBoneName);
	RightCurrQuat = RightHand->GetBoneQuaternion(RightControlBoneName);
}

// Called to bind functionality to input
void ARCharacter::SetupPlayerInputComponent(class UInputComponent* InputComponent)
{
	Super::SetupPlayerInputComponent(InputComponent);

	// Set up gameplay key bindings
	InputComponent->BindAxis("MoveForward", this, &ARCharacter::MoveForward);
	InputComponent->BindAxis("MoveRight", this, &ARCharacter::MoveRight);
	// Default Camera view bindings
	InputComponent->BindAxis("CameraPitch", this, &ARCharacter::AddControllerPitchInput);
	InputComponent->BindAxis("CameraYaw", this, &ARCharacter::AddControllerYawInput);
	// Set up hand bindings
	InputComponent->BindAxis("OpenHandLeft", this, &ARCharacter::OpenHandLeft);
	InputComponent->BindAxis("CloseHandLeft", this, &ARCharacter::CloseHandLeft);
	InputComponent->BindAxis("OpenHandRight", this, &ARCharacter::OpenHandRight);
	InputComponent->BindAxis("CloseHandRight", this, &ARCharacter::CloseHandRight);
	// Bind hand attach
	InputComponent->BindAction("AttachHandLeft", IE_Released, this, &ARCharacter::AttachHandLeft);
	InputComponent->BindAction("AttachHandRight", IE_Released, this, &ARCharacter::AttachHandRight);
	// TODO TEST
	// Set up grasp switch
	InputComponent->BindAction("SwitchGrasp", IE_Pressed, this, &ARCharacter::OnSwitchGrasp);
	InputComponent->BindAction("ConstraintTest", IE_Pressed, this, &ARCharacter::OnCreateConstraint);
}

// Handles moving forward/backward
void ARCharacter::MoveForward(const float Value)
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
void ARCharacter::MoveRight(const float Value)
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

// Called to bind functionality to input
void ARCharacter::CloseHandLeft(const float AxisValue)
{
	if (AxisValue == 0)
	{		
		// Disable finger hit events
		if (bLeftFingerHitEvents)
		{
			for (auto BoneName : LeftCollisionBoneNames)
			{
				LeftHand->GetBodyInstance(BoneName)->SetInstanceNotifyRBCollision(false);
			}
			// Set finger notification flag
			bLeftFingerHitEvents = false;
		}		
		return;
	}

	// Enable finger hit events
	if (!bLeftFingerHitEvents)
	{
		for (auto BoneName : LeftCollisionBoneNames)
		{
			LeftHand->GetBodyInstance(BoneName)->SetInstanceNotifyRBCollision(true);
		}
		// Set finger notification flag
		bLeftFingerHitEvents = true;
	}
	
	// Update grasping
	LeftGrasp->Update(0.5 * AxisValue);
}

// Called to bind functionality to input
void ARCharacter::OpenHandLeft(const float AxisValue)
{
	if (AxisValue == 0)
	{
		return;
	}
	
	// Free the colliding fingers map
	if (LeftHitActorToFingerMMap.Num() != 0)
	{
		LeftHitActorToFingerMMap.Empty();
	}

	// Detach object if case
	if (LeftGrasp->GetState() == ERGraspState::Attached)
	{
		// Break constraint
		LeftGraspFixatingConstraint->BreakConstraint();

		// Set state to free
		LeftGrasp->SetState(ERGraspState::Free);
	}

	// Update grasping
	LeftGrasp->Update(- AxisValue );
}

// Attach grasped object to hand
void ARCharacter::AttachHandLeft()
{
	// Attach object only if the state is blocked
	if (LeftGrasp->GetState() == ERGraspState::Blocked)
	{
		// Map colliding component with the number of appearance
		TMap<AActor*, uint8> ActorToCount;

		// Iterate and count the colliding actors
		for (const auto ActorToFinger : LeftHitActorToFingerMMap)
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
		ActorToCount.ValueSort([](const uint8 A, const uint8 B)
		{
			return A > B;
		});

		// If the count is enough, attach actor to hand
		for (const auto ActToCount : ActorToCount)
		{
			if (ActToCount.Value > 2)
			{
				LeftGraspFixatingConstraint->SetWorldLocation(ActToCount.Key->GetActorLocation());

				//LeftGraspFixatingConstraint->SetWorldLocation(LeftHand->GetComponentLocation());

				//for (TActorIterator<AStaticMeshActor> ActorItr(GetWorld()); ActorItr; ++ActorItr)
				//{
				//	UE_LOG(LogTemp, Warning, TEXT("ActorItr: %s"), *ActorItr->GetName());
				//	if (ActorItr->GetName().Equals("SCube3"))
				//	{
				//		Sphere = *ActorItr;
				//		break;
				//	}
				//}

				//if (Sphere)
				//{
				//	UE_LOG(LogTemp, Warning, TEXT("Sphere name: %s"), *Sphere->GetName());
				//}

				LeftGraspFixatingConstraint->SetConstrainedComponents(
					LeftFixatingGraspStaticMesh, NAME_None,
					Cast<AStaticMeshActor>(ActToCount.Key)->GetStaticMeshComponent(), NAME_None);
			}
			break;
		}
		
		// Set state to attached
		LeftGrasp->SetState(ERGraspState::Attached);
		
		
		// Free the finger collisions
		LeftHitActorToFingerMMap.Empty();
	}
}

// Called to bind functionality to input
void ARCharacter::CloseHandRight(const float AxisValue)
{
	if (AxisValue == 0)
	{
		// Disable finger hit events
		if (bRightFingerHitEvents)
		{
			for (auto BoneName : RightCollisionBoneNames)
			{
				RightHand->GetBodyInstance(BoneName)->SetInstanceNotifyRBCollision(false);
			}
			// Set finger notification flag
			bRightFingerHitEvents = false;
		}
		return;
	}

	// Enable finger hit events
	if (!bRightFingerHitEvents)
	{
		for (auto BoneName : RightCollisionBoneNames)
		{
			RightHand->GetBodyInstance(BoneName)->SetInstanceNotifyRBCollision(true);
		}
		// Set finger notification flag
		bRightFingerHitEvents = true;
	}

	// Update grasping
	RightGrasp->Update(0.5 * AxisValue);
}

// Called to bind functionality to input
void ARCharacter::OpenHandRight(const float AxisValue)
{
	if (AxisValue == 0)
	{
		return;
	}

	// Free the colliding fingers map
	if (RightHitActorToFingerMMap.Num() != 0)
	{
		RightHitActorToFingerMMap.Empty();
	}

	// Detach object if case
	if (RightGrasp->GetState() == ERGraspState::Attached)
	{
		// Break constraint
		RightGraspFixatingConstraint->BreakConstraint();

		// Set state to free
		RightGrasp->SetState(ERGraspState::Free);
	}

	// Update grasping
	RightGrasp->Update(- AxisValue);
}

// Attach grasped object to hand
void ARCharacter::AttachHandRight()
{
	// Attach object only if the state is blocked
	if (RightGrasp->GetState() == ERGraspState::Blocked)
	{
		// Map colliding component with the number of appearance
		TMap<AActor*, uint8> ActorToCount;

		// Iterate and count the colliding actors
		for (const auto ActorToFinger : RightHitActorToFingerMMap)
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
		ActorToCount.ValueSort([](const uint8 A, const uint8 B)
		{
			return A > B;
		});

		// If the count is enough, attach actor to hand
		for (const auto ActToCount : ActorToCount)
		{
			if (ActToCount.Value > 2)
			{
				RightGraspFixatingConstraint->SetConstrainedComponents(
					Cast<UPrimitiveComponent>(ActToCount.Key->GetRootComponent()), NAME_None,
					Cast<UPrimitiveComponent>(RightHand), NAME_None);
			}
			break;
		}

		// Set state to attached
		RightGrasp->SetState(ERGraspState::Attached);

		// Free the finger collisions
		RightHitActorToFingerMMap.Empty();
	}
}

// Hand collision callback
void ARCharacter::OnHitLeft(UPrimitiveComponent* SelfComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{	
	// Check collisions if grasp state is free, 
	if (LeftGrasp->GetState() == ERGraspState::Free)
	{		
		// Get the finger in collision
		const ERHandLimb* Finger = BoneNameToLimbMap.Find(Hit.BoneName);

		// Block finger if free, and not colliding with self
		if ((Finger != nullptr) && (!LeftGrasp->IsFingerBlocked(*Finger)) && (SelfComp != OtherComp))
		{
			// Block finger
			LeftGrasp->BlockFinger(*Finger);

			// Add colliding component to the map
			LeftHitActorToFingerMMap.Add(OtherActor, *Finger);
		}
	}
}

// Hand collision callback
void ARCharacter::OnHitRight(UPrimitiveComponent* SelfComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	// Check collisions if grasp state is free, 
	if (RightGrasp->GetState() == ERGraspState::Free)
	{
		// Get the finger in collision
		const ERHandLimb* Finger = BoneNameToLimbMap.Find(Hit.BoneName);

		// Block finger if free, and not colliding with self
		if ((Finger != nullptr) && (!RightGrasp->IsFingerBlocked(*Finger)) && (SelfComp != OtherComp))
		{
			// Block finger
			RightGrasp->BlockFinger(*Finger);

			// Add colliding component to the map
			RightHitActorToFingerMMap.Add(OtherActor, *Finger);
		}
	}
}

// Swith between grasping styles
void ARCharacter::OnSwitchGrasp()
{
	UE_LOG(LogTemp, Warning, TEXT("Switch!"));

	//LeftGrasp = dynamic_cast<FRGrasp*>(new FRGraspPiano(LFingerTypeToConstrs));
	//RightGrasp = dynamic_cast<FRGrasp*>(new FRGraspPiano(RFingerTypeToConstrs));
}

// Swith between grasping styles
void ARCharacter::OnCreateConstraint()
{
	// Action binding function
	if (!bAttached)
	{
		//Set World Location
		//MConstraintComp->SetWorldLocation(LeftFixatingGraspStaticMesh->GetComponentLocation());
		MConstraintComp->SetWorldLocation(HandSkelAct->GetActorLocation());

		//Attach to Root!
		//MConstraintComp->AttachToComponent(LeftFixatingGraspStaticMesh, FAttachmentTransformRules::KeepWorldTransform);
		MConstraintComp->AttachToComponent(HandSkelAct->GetRootComponent(), FAttachmentTransformRules::KeepWorldTransform);

		//MConstraintComp->RegisterComponent();



		MConstraintComp->SetConstrainedComponents(
			//AttachedConeAct->GetStaticMeshComponent(), NAME_None,
			HandSkelAct->GetSkeletalMeshComponent(), NAME_None,
			//StaticMeshAct1->GetStaticMeshComponent(), NAME_None,
			//LeftFixatingGraspStaticMesh, NAME_None,
			//GraspHelperStaticMeshAct->GetStaticMeshComponent(), NAME_None,
			//LeftHand, NAME_None,

			//Cast<UPrimitiveComponent>(SkelAct1->SkeletalMeshComponent), NAME_None,
			//Cast<UPrimitiveComponent>(LeftControlBody), NAME_None,
			//Cast<UPrimitiveComponent>(Act2->GetStaticMeshComponent()), NAME_None);

			StaticMeshAct2->GetStaticMeshComponent(), NAME_None);


		UE_LOG(LogTemp, Warning, TEXT("Attach!"));
		bAttached = true;
		PrintGreen(TEXT("Attached!"));
	}
	else if (bAttached)
	{

		MConstraintComp->BreakConstraint();
		UE_LOG(LogTemp, Warning, TEXT("Detach!"));
		bAttached = false;
		PrintRed(TEXT("Detached!"));
	}

}