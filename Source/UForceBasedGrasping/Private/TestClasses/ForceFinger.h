// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "Animation/SkeletalMeshActor.h" 
#include "Components/ArrowComponent.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "Utilities/ForceFileWriter.h"

#include "ForceFinger.generated.h"

/**
 *
 */
UCLASS()
class UFORCEBASEDGRASPING_API AForceFinger : public ASkeletalMeshActor
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

	AForceFinger();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

private:
	FVector LastVelocity;

	FBodyInstance* MetacarpalBody;
	UArrowComponent* MetacarpalBodyArrow;

	FBodyInstance* ProximalBody;
	UArrowComponent* ProximalBodyArrow;
	FConstraintInstance* ProximalConstraint;
	UArrowComponent* ProximalForceArrow;

	FBodyInstance* IntermediateBody;
	UArrowComponent* IntermediateBodyArrow;
	FConstraintInstance* IntermediateConstraint;
	UArrowComponent* IntermediateForceArrow;

	FBodyInstance* DistalBody;
	UArrowComponent* DistalBodyArrow;
	FConstraintInstance* DistalConstraint;
	UArrowComponent* DistalForceArrow;

	TSharedPtr<ForceFileWriter> ForceFileWriterPtr;

	void InitializeOrientationTwistAndSwing(FConstraintInstance* Constraint, const FQuat & Quaternion);
	void InitializeVelocityTwistAndSwing(FConstraintInstance* Constraint, const FVector & Vector);

	void UpdateConstraintArrow(FConstraintInstance* const Constraint, UArrowComponent* const Arrow);
	void UpdateBodyArrow(FBodyInstance* const Body, UArrowComponent* const Arrow, const float DeltaTime);

};
