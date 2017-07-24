// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "ForceSimpleHand.h"
#include "Paths.h"

AForceSimpleHand::AForceSimpleHand()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	Spring = 50.0;
	Damping = 10.0;
	ForceLimit = 0.0;


	USkeletalMeshComponent* SkeletalMesh = GetSkeletalMeshComponent();
	if (SkeletalMesh == nullptr)
		return;

	SkeletalMesh->SetEnableGravity(false);
	SkeletalMesh->SetSimulatePhysics(true);

}

// Called when the game starts or when spawned
void AForceSimpleHand::BeginPlay()
{
	Super::BeginPlay();

	USkeletalMeshComponent* SkeletalMesh = GetSkeletalMeshComponent();

	if (SkeletalMesh != nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("SkeletalMesh assigned"));

		LeftConstraint = SkeletalMesh->FindConstraintInstance("Left");
		if (LeftConstraint)
		{
			UE_LOG(LogTemp, Warning, TEXT("Left Constraint Found"));
			//ProximalForceArrow->SetWorldLocation(LeftConstraint->GetConstraintLocation());
			//ProximalForceArrow->SetHiddenInGame(!bShowForceArrows);

			//InitializeOrientationTwistAndSwing(LeftConstraint, FRotator(0, 0, 0).Quaternion());
		}

		RightConstraint = SkeletalMesh->FindConstraintInstance("Right");
		if (RightConstraint)
		{
			UE_LOG(LogTemp, Warning, TEXT("Right Constraint Found"));
			//IntermediateForceArrow->SetWorldLocation(LeftConstraint->GetConstraintLocation());
			//IntermediateForceArrow->SetHiddenInGame(!bShowForceArrows);

			//InitializeOrientationTwistAndSwing(RightConstraint, FRotator(0, 0, 0).Quaternion());
		}
	}
}

// Called every frame, used for motion control
void AForceSimpleHand::Tick(float DeltaTime)
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

void AForceSimpleHand::Open(const float Value) const
{
	if (Value != 0)
	{
		if (LeftConstraint != nullptr && RightConstraint != nullptr)
		{
			UE_LOG(LogTemp, Warning, TEXT("Open - Value: %f | Rotation: %s"), Value, *FRotator(0, 0, 0).ToString());
			LeftConstraint->SetAngularOrientationTarget(FRotator(0, 0, 0).Quaternion());

			RightConstraint->SetAngularOrientationTarget(FRotator(0, 0, 0).Quaternion());
		}
	}
}

void AForceSimpleHand::Close(const float Value) const
{
	if (Value != 0)
	{
		if (LeftConstraint != nullptr && RightConstraint != nullptr)
		{
			UE_LOG(LogTemp, Warning, TEXT("Close - Value: %f | Rotation: %s"), Value, *FRotator(0, 0, 20).ToString());
			LeftConstraint->SetAngularOrientationTarget(FRotator(-20, -20, -20).Quaternion());

			RightConstraint->SetAngularOrientationTarget(FRotator(20, 20, 20).Quaternion());
		}
	}
}

void AForceSimpleHand::InitializeOrientationTwistAndSwing(FConstraintInstance* Constraint, const FQuat & Quaternion)
{
	Constraint->SetDisableCollision(true);
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetOrientationDriveTwistAndSwing(true, true);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularOrientationTarget(Quaternion);
}

void AForceSimpleHand::InitializeVelocityTwistAndSwing(FConstraintInstance* Constraint, const FVector & Vector)
{
	Constraint->SetDisableCollision(true);
	Constraint->SetAngularVelocityDriveTwistAndSwing(true, true);
	Constraint->SetAngularVelocityTarget(Vector);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
}