// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "ForceSimpleHand.h"
#include "Paths.h"
#include "Engine/Engine.h"

AForceSimpleHand::AForceSimpleHand()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	Spring = 50.0;
	Damping = 10.0;
	ForceLimit = 0.0;

	bGraspRunning = false;
	bLeftChangeable = false;
	bRightChangeable = false;

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
		}

		RightConstraint = SkeletalMesh->FindConstraintInstance("Right");
		if (RightConstraint)
		{
			UE_LOG(LogTemp, Warning, TEXT("Right Constraint Found"));
		}
	}
}

// Called every frame, used for motion control
void AForceSimpleHand::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (bGraspRunning)
	{
		FVector OutAngularForce;
		FVector OutLinearForce;

		// LEFT

		LeftConstraint->GetConstraintForce(OutLinearForce, OutAngularForce);
		if (OutAngularForce.Size() > ForceThreshold)
			bLeftChangeable = true;

		if (bLeftChangeable)
		{
			if (OutAngularForce.Size() < 1)
			{
				if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Green, "Change Left to Velocity");
				InitializeVelocityTwistAndSwing(LeftConstraint, FVector(10, 0, 0));
			}
		}
		UE_LOG(LogTemp, Warning, TEXT("Left Force: %f"), OutAngularForce.Size());


		//RIGHT

		RightConstraint->GetConstraintForce(OutLinearForce, OutAngularForce);
		if (OutAngularForce.Size() > ForceThreshold)
			bRightChangeable = true;

		if (bRightChangeable)
		{

			if (OutAngularForce.Size() < 1)
			{
				if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Green, "Change Right to Velocity");
				InitializeVelocityTwistAndSwing(RightConstraint, FVector(10, 0, 0));
			}
		}
		UE_LOG(LogTemp, Warning, TEXT("Right Force: %f"), OutAngularForce.Size());
	}
}

void AForceSimpleHand::StartGrasp(const float Value)
{
	if (Value != 0)
	{
		if (LeftConstraint != nullptr && RightConstraint != nullptr && !bGraspRunning)
		{
			if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Green, "StartGrasp");
			bGraspRunning = true;

			ResetConstraint(LeftConstraint);
			ResetConstraint(RightConstraint);

			InitializeOrientationTwistAndSwing(LeftConstraint, FRotator(0, -15, 0).Quaternion());
			InitializeOrientationTwistAndSwing(RightConstraint, FRotator(0, 15, 0).Quaternion());
		}
	}
}

void AForceSimpleHand::StopGrasp(const float Value)
{
	if (Value != 0)
	{
		if (LeftConstraint != nullptr && RightConstraint != nullptr && bGraspRunning)
		{
			if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Green, "StopGrasp");
			bGraspRunning = false;
			bLeftChangeable = false;
			bRightChangeable = false;

			ResetConstraint(LeftConstraint);
			ResetConstraint(RightConstraint);

			InitializeOrientationTwistAndSwing(LeftConstraint, FRotator(0, 0, 0).Quaternion());
			InitializeOrientationTwistAndSwing(RightConstraint, FRotator(0, 0, 0).Quaternion());
		}
	}
}

void AForceSimpleHand::ResetConstraint(FConstraintInstance* Constraint)
{
	Constraint->SetOrientationDriveTwistAndSwing(false, false);
	Constraint->SetAngularVelocityDriveTwistAndSwing(false, false);
	Constraint->SetAngularVelocityDriveSLERP(false);
	Constraint->SetAngularVelocityDriveSLERP(false);
}

void AForceSimpleHand::InitializeOrientationTwistAndSwing(FConstraintInstance* Constraint, const FQuat& Quaternion)
{
	Constraint->SetDisableCollision(true);
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetOrientationDriveTwistAndSwing(true, true);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularOrientationTarget(Quaternion);
}

void AForceSimpleHand::InitializeVelocityTwistAndSwing(FConstraintInstance* Constraint, const FVector& Vector)
{
	Constraint->SetDisableCollision(true);
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Constraint->SetAngularVelocityDriveTwistAndSwing(true, true);
	Constraint->SetAngularDriveParams(Spring, Damping, ForceLimit);
	Constraint->SetAngularVelocityTarget(Vector);
}
