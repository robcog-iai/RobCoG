// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "Animation/SkeletalMeshActor.h" 
#include "Components/ArrowComponent.h"
#include "PhysicsEngine/ConstraintInstance.h"

#include "ForceSimpleHand.generated.h"

/**
 *
 */
UCLASS()
class UFORCEBASEDGRASPING_API AForceSimpleHand : public ASkeletalMeshActor
{
	GENERATED_BODY()

public:
	// Spring value to apply to the angular drive (Position strength)
	UPROPERTY(EditAnywhere, Category = "Drive Parameters")
		float Spring;

	// Damping value to apply to the angular drive (Velocity strength) 
	UPROPERTY(EditAnywhere, Category = "Drive Parameters")
		float Damping;

	// Limit of the force that the angular drive can apply
	UPROPERTY(EditAnywhere, Category = "Drive Parameters")
		float ForceLimit;

	UPROPERTY(EditAnywhere, Category = "Drive Parameters")
		float ForceThreshold;

	AForceSimpleHand();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	void StartGrasp(const float Value);
	void StopGrasp(const float Value);

private:
	bool bGraspRunning;
	bool bLeftChangeable;
	bool bRightChangeable;

	FConstraintInstance* LeftConstraint;

	FConstraintInstance* RightConstraint;

	void ResetConstraint(FConstraintInstance* Constraint);

	void InitializeOrientationTwistAndSwing(FConstraintInstance* Constraint, const FQuat & Quaternion);
	void InitializeVelocityTwistAndSwing(FConstraintInstance* Constraint, const FVector & Vector);

};
