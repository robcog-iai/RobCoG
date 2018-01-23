// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "ForceFinger.h"
#include "Utilities/ForceFileWriter.h"
#include "Paths.h"

AForceFinger::AForceFinger()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	Spring = 50.0;
	Damping = 10.0;
	ForceLimit = 0.0;

	bLogForceIntoFile = false;
	bShowForceArrows = true;

	USkeletalMeshComponent* SkeletalMesh = GetSkeletalMeshComponent();
	if (SkeletalMesh == nullptr)
		return;

	/*
	 * INITIALIZE FORCE ARROWS
	 */
	ProximalForceArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("ProximalForceArrow"));
	ProximalForceArrow->SetupAttachment(RootComponent);

	IntermediateForceArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("IntermediateForceArrow"));
	IntermediateForceArrow->SetupAttachment(RootComponent);

	DistalForceArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("DistalForceArrow"));
	DistalForceArrow->SetupAttachment(RootComponent);

	/*
	* INITIALIZE BODY ARROWS
	*/
	MetacarpalBodyArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("MetacarpalBodyArrow"));
	MetacarpalBodyArrow->SetupAttachment(RootComponent);

	ProximalBodyArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("ProximalBodyArrow"));
	ProximalBodyArrow->SetupAttachment(RootComponent);

	IntermediateBodyArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("IntermediateBodyArrow"));
	IntermediateBodyArrow->SetupAttachment(RootComponent);

	DistalBodyArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("DistalBodyArrow"));
	DistalBodyArrow->SetupAttachment(RootComponent);

	ForceFileWriterPtr = MakeShareable(new ForceFileWriter());

}

// Called when the game starts or when spawned
void AForceFinger::BeginPlay()
{
	Super::BeginPlay();

	USkeletalMeshComponent* SkeletalMesh = GetSkeletalMeshComponent();
	if (SkeletalMesh == nullptr)
		return;

	SkeletalMesh->SetEnableGravity(false);
	SkeletalMesh->SetSimulatePhysics(true);
	/*
	 * INITIALIZE BODYPARTS
	 */

	MetacarpalBody = SkeletalMesh->GetBodyInstance("Metacarpal");
	ProximalBody = SkeletalMesh->GetBodyInstance("Proximal");
	IntermediateBody = SkeletalMesh->GetBodyInstance("Intermediate");
	DistalBody = SkeletalMesh->GetBodyInstance("Distal");

	/**
	 * INITIALIZE CONSTRAINTS
	 */
	for (auto Constraint : SkeletalMesh->Constraints)
	{

		if (Constraint->JointName.ToString() == "Proximal")
		{
			ProximalConstraint = Constraint;
			ProximalForceArrow->SetWorldLocation(ProximalConstraint->GetConstraintLocation());
			ProximalForceArrow->SetHiddenInGame(!bShowForceArrows);

			InitializeOrientationTwistAndSwing(ProximalConstraint, FRotator(0, 0, 0).Quaternion());
		}
		else if (Constraint->JointName.ToString() == "Intermediate")
		{
			IntermediateConstraint = Constraint;
			IntermediateForceArrow->SetWorldLocation(IntermediateConstraint->GetConstraintLocation());
			IntermediateForceArrow->SetHiddenInGame(!bShowForceArrows);

			InitializeOrientationTwistAndSwing(IntermediateConstraint, FRotator(0, 0, 0).Quaternion());
		}
		else if (Constraint->JointName.ToString() == "Distal")
		{
			DistalConstraint = Constraint;
			DistalForceArrow->SetWorldLocation(DistalConstraint->GetConstraintLocation());
			DistalForceArrow->SetHiddenInGame(!bShowForceArrows);

			InitializeOrientationTwistAndSwing(DistalConstraint, FRotator(0, 45, 0).Quaternion());
		}
	}

}

// Called every frame, used for motion control
void AForceFinger::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	/*
	if (DistalConstraint != nullptr && DistalForceArrow != nullptr)
		UpdateConstraintArrow(DistalConstraint, DistalForceArrow);

	if (IntermediateConstraint != nullptr && IntermediateForceArrow != nullptr)
		UpdateConstraintArrow(IntermediateConstraint, IntermediateForceArrow);

	if (ProximalConstraint != nullptr && ProximalForceArrow != nullptr)
		UpdateConstraintArrow(ProximalConstraint, ProximalForceArrow);
	*/

}

void AForceFinger::InitializeOrientationTwistAndSwing(FConstraintInstance* Constraint, const FQuat & Quaternion)
{
	Constraint->SetDisableCollision(true);
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetOrientationDriveTwistAndSwing(true, true);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularOrientationTarget(Quaternion);
}

void AForceFinger::InitializeVelocityTwistAndSwing(FConstraintInstance* Constraint, const FVector & Vector)
{
	Constraint->SetDisableCollision(true);
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularVelocityDriveTwistAndSwing(true, true);
	Constraint->SetAngularVelocityTarget(Vector);
}

void AForceFinger::UpdateConstraintArrow(FConstraintInstance* const Constraint, UArrowComponent* const Arrow)
{
	FVector OutLinearForce;
	FVector OutAngularForce;

	Constraint->GetConstraintForce(OutLinearForce, OutAngularForce);
	Arrow->ArrowSize = 1;// OutAngularForce.Size();
	Arrow->SetWorldLocationAndRotation(Constraint->GetConstraintLocation(), OutAngularForce.Rotation());

	if (Constraint == DistalConstraint)
	{
		if (ForceFileWriterPtr.IsValid() && bLogForceIntoFile)
		{
			//ForceFileWriterPtr->AppendFloatToFile(OutAngularForce.Size(), FPaths::GameSavedDir() + "Force.csv");
		}
		UE_LOG(LogTemp, Warning, TEXT("AngularForce: %s"), *OutAngularForce.ToString());
		UE_LOG(LogTemp, Warning, TEXT("AngularForce.Size: %f"), OutAngularForce.Size());
	}

}

void AForceFinger::UpdateBodyArrow(FBodyInstance* const Body, UArrowComponent* const Arrow, const float DeltaTime)
{

	Arrow->ArrowSize = 1;
	Arrow->SetWorldLocationAndRotation(Body->GetUnrealWorldTransform().GetLocation(), Body->GetUnrealWorldTransform().GetRotation());

	/*
	if (Body == DistalBody)
	{

		FVector Acceleration = (Body->GetUnrealWorldAngularVelocity() - LastVelocity) / DeltaTime;
		FVector Force = Body->GetBodyMass() * Acceleration;
		UE_LOG(LogTemp, Warning, TEXT("Force: %s"), Body->Get*Body->GetBodyMass());
		UE_LOG(LogTemp, Warning, TEXT("AngularForce.Size: %f"), OutAngularForce.Size());

	}
	//*/

	LastVelocity = Body->GetUnrealWorldAngularVelocity();
}

