// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "TestForceActor.h"

ATestForceActor::ATestForceActor()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	Spring = 50.0;
	Damping = 10.0;
	ForceLimit = 0.0;

	USkeletalMeshComponent* SkeletalMesh = GetSkeletalMeshComponent();
	if (SkeletalMesh == nullptr)
		return;

	ProximalForceArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("ProximalForceArrow"));
	ProximalForceArrow->SetupAttachment(RootComponent);

	IntermediateForceArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("IntermediateForceArrow"));
	IntermediateForceArrow->SetupAttachment(RootComponent);

	DistalForceArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("DistalForceArrow"));
	DistalForceArrow->SetupAttachment(RootComponent);

}

// Called when the game starts or when spawned
void ATestForceActor::BeginPlay()
{
	Super::BeginPlay();
	
	USkeletalMeshComponent* SkeletalMesh = GetSkeletalMeshComponent();
	if (SkeletalMesh == nullptr)
		return;

	SkeletalMesh->SetSimulatePhysics(true);
	
	FVector OutLinearForce;
	FVector OutAngularForce;

	for (auto Constraint : SkeletalMesh->Constraints)
	{
		Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
		Constraint->SetOrientationDriveTwistAndSwing(true, true);
		Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
		Constraint->SetAngularOrientationTarget(FRotator(0, 0, 0).Quaternion());

		//InitializeOrientationTwistAndSwing(Constraint);
		//InitializeVelocityTwistAndSwing(Constraint);

		if (Constraint->JointName.ToString() == "Proximal")
		{
			Proximal = Constraint;
			Proximal->GetConstraintForce(OutLinearForce, OutAngularForce);
			ProximalForceArrow->SetWorldLocation(Proximal->GetConstraintLocation());
			ProximalForceArrow->ArrowSize = OutAngularForce.Size();
			ProximalForceArrow->SetHiddenInGame(!bShowForceArrows);
		}

		if (Constraint->JointName.ToString() == "Intermediate")
		{
			Intermediate = Constraint;
			Intermediate->GetConstraintForce(OutLinearForce, OutAngularForce);
			IntermediateForceArrow->SetWorldLocation(Intermediate->GetConstraintLocation());
			IntermediateForceArrow->ArrowSize = OutAngularForce.Size();
			IntermediateForceArrow->SetHiddenInGame(!bShowForceArrows);
		}

		if (Constraint->JointName.ToString() == "Distal")
		{
			Distal = Constraint;
			InitializeVelocityTwistAndSwing(Distal);
			Distal->GetConstraintForce(OutLinearForce, OutAngularForce);
			DistalForceArrow->SetWorldLocation(Distal->GetConstraintLocation());
			DistalForceArrow->ArrowSize = OutAngularForce.Size();
			DistalForceArrow->SetHiddenInGame(!bShowForceArrows);
		}
	}

}

// Called every frame, used for motion control
void ATestForceActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	FVector OutLinearForce;
	FVector OutAngularForce;

	Distal->GetConstraintForce(OutLinearForce, OutAngularForce);
	DistalForceArrow->ArrowSize = 1;
	DistalForceArrow->SetWorldLocationAndRotation(Distal->GetConstraintLocation(), OutAngularForce.Rotation());
	
	UE_LOG(LogTemp, Warning, TEXT("AngularForce: %s"), *OutAngularForce.ToString());
	UE_LOG(LogTemp, Warning, TEXT("AngularForce.Size: %f"), OutAngularForce.Size());

}

void ATestForceActor::InitializeOrientationTwistAndSwing(FConstraintInstance* Constraint)
{
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetOrientationDriveTwistAndSwing(true, true);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularOrientationTarget(FRotator(0, -50, 0).Quaternion());
}

void ATestForceActor::InitializeVelocityTwistAndSwing(FConstraintInstance* Constraint)
{
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularVelocityDriveTwistAndSwing(true, true);
	Constraint->SetAngularVelocityTarget(FRotator(50, 0, 0).Vector());
}
