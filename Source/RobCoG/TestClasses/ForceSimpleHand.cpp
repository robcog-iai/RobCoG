// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "ForceSimpleHand.h"
#include "Utilities/ForceFileWriter.h"
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

		for (FConstraintInstance* Constraint : SkeletalMesh->Constraints)
		{

			if (Constraint->JointName.ToString() == "Left")
			{
				UE_LOG(LogTemp, Warning, TEXT("Left Constraint Found"));
				LeftConstraint = Constraint;
				//ProximalForceArrow->SetWorldLocation(LeftConstraint->GetConstraintLocation());
				//ProximalForceArrow->SetHiddenInGame(!bShowForceArrows);

				InitializeVelocityTwistAndSwing(LeftConstraint, FRotator(0, 0, 0).Vector());
			}
			else if (Constraint->JointName.ToString() == "Right")
			{
				UE_LOG(LogTemp, Warning, TEXT("Right Constraint Found"));
				RightConstraint = Constraint;
				//IntermediateForceArrow->SetWorldLocation(LeftConstraint->GetConstraintLocation());
				//IntermediateForceArrow->SetHiddenInGame(!bShowForceArrows);

				InitializeVelocityTwistAndSwing(RightConstraint, FRotator(0, 0, 0).Vector());
			}
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
			LeftConstraint->SetAngularVelocityTarget(FRotator(0, 0, 0).Vector());

			RightConstraint->SetAngularVelocityTarget(FRotator(0, 0, 0).Vector());
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
			LeftConstraint->SetAngularVelocityTarget(FRotator(-20, -20, -20).Vector());

			RightConstraint->SetAngularVelocityTarget(FRotator(20, 20, 20).Vector());
		}

	}
}

void AForceSimpleHand::InitializeOrientationTwistAndSwing(FConstraintInstance* const Constraint, const FQuat & Quaternion) const
{
	Constraint->SetDisableCollision(true);
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetOrientationDriveTwistAndSwing(true, true);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularOrientationTarget(Quaternion);
}

void AForceSimpleHand::InitializeVelocityTwistAndSwing(FConstraintInstance* const  Constraint, const FVector & Vector) const
{
	Constraint->SetDisableCollision(true);
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularVelocityDriveTwistAndSwing(true, true);
	Constraint->SetAngularVelocityTarget(Vector);
}