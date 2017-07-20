// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "Animation/SkeletalMeshActor.h"
#include "Components/ArrowComponent.h"
#include "Utilities/ForceFileWriter.h"

#include "TestForceActor.generated.h"

/**
 *
 */
UCLASS()
class ROBCOG_API ATestForceActor : public ASkeletalMeshActor
{
	GENERATED_BODY()

public:
	// Spring value to apply to the angular drive (Position strength)
	UPROPERTY(EditAnywhere, Category = "Finger|Drive Parameters")
		float Spring;

	// Damping value to apply to the angular drive (Velocity strength) 
	UPROPERTY(EditAnywhere, Category = "Finger|Drive Parameters")
		float Damping;

	// Limit of the force that the angular drive can apply
	UPROPERTY(EditAnywhere, Category = "Finger|Drive Parameters")
		float ForceLimit;

	UPROPERTY(EditAnywhere, Category = "Finger|Force Parameters")
		bool bShowForceArrows;

	UPROPERTY(EditAnywhere, Category = "Finger|Force Logging")
		int32 NumberOfValuesToBeWritten;

	UPROPERTY(EditAnywhere, Category = "Finger|Force Logging")
		bool bLogForceIntoFile;

	ATestForceActor();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

private:
	FBodyInstance* MetacarpalBody;

	FBodyInstance* ProximalBody;
	FConstraintInstance* ProximalConstraint;
	UArrowComponent* ProximalForceArrow;

	FBodyInstance* IntermediateBody;
	FConstraintInstance* IntermediateConstraint;
	UArrowComponent* IntermediateForceArrow;

	FBodyInstance* DistalBody;
	FConstraintInstance* DistalConstraint;
	UArrowComponent* DistalForceArrow;

	TSharedPtr<ForceFileWriter> ForceFileWriterPtr;

	void InitializeOrientationTwistAndSwing(FConstraintInstance* Constraint, const FQuat & Quaternion);
	void InitializeVelocityTwistAndSwing(FConstraintInstance* Constraint, const FVector & Vector);

	void UpdateConstraintArrow(FConstraintInstance* const Constraint, UArrowComponent* const Arrow);
	void UpdateBodyArrow(FBodyInstance* const Body, UArrowComponent* const Arrow);

};
