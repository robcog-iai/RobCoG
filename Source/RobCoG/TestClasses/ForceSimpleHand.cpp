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

}

// Called when the game starts or when spawned
void AForceSimpleHand::BeginPlay()
{
	Super::BeginPlay();

	USkeletalMeshComponent* SkeletalMesh = GetSkeletalMeshComponent();

	if (SkeletalMesh != nullptr)
	{
		SkeletalMesh->SetEnableGravity(false);
		SkeletalMesh->SetSimulatePhysics(true);

		for (auto Constraint : SkeletalMesh->Constraints)
		{

			if (Constraint->JointName.ToString() == "Left")
			{
				LeftConstraint = Constraint;
				//ProximalForceArrow->SetWorldLocation(LeftConstraint->GetConstraintLocation());
				//ProximalForceArrow->SetHiddenInGame(!bShowForceArrows);

				InitializeOrientationTwistAndSwing(LeftConstraint, FRotator(0, 0, 0).Quaternion());
			}
			else if (Constraint->JointName.ToString() == "Right")
			{
				RightConstraint = Constraint;
				//IntermediateForceArrow->SetWorldLocation(LeftConstraint->GetConstraintLocation());
				//IntermediateForceArrow->SetHiddenInGame(!bShowForceArrows);

				InitializeOrientationTwistAndSwing(RightConstraint, FRotator(0, 0, 0).Quaternion());
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

void AForceSimpleHand::Open(const float Value)
{
	if (Value != 0)
	{
		//UE_LOG(LogTemp, Warning, TEXT("Value: %f | Rotation: %s"), Value, *CurrentRotation.ToString());

		/*
		CurrentRotation.Add(1, 1, 1);
		LeftConstraint->SetAngularOrientationTarget(CurrentRotation.Quaternion());

		FRotator Rotation = CurrentRotation * -1;
		RightConstraint->SetAngularOrientationTarget(Rotation.Quaternion());
		*/
	}
}

void AForceSimpleHand::Close(const float Value)
{
	if (Value != 0)
	{

	}
}

void AForceSimpleHand::InitializeOrientationTwistAndSwing(FConstraintInstance* const Constraint, const FQuat & Quaternion) const
{
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetOrientationDriveTwistAndSwing(true, true);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularOrientationTarget(Quaternion);
}

void AForceSimpleHand::InitializeVelocityTwistAndSwing(FConstraintInstance* const  Constraint, const FVector & Vector) const
{
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularVelocityDriveTwistAndSwing(true, true);
	Constraint->SetAngularVelocityTarget(Vector);
}