// Fill out your copyright notice in the Description page of Project Settings.

#include "RoCoG.h"
#include "RCGGrasp.h"

// Set default values
FRCGGrasp::FRCGGrasp()
{
	// Set default state
	State = EGraspState::Free;

	// Set constraints target
	Target = 0.0f;
}

// Set default values 
FRCGGrasp::FRCGGrasp(TMultiMap<ERCGHandLimb, FConstraintInstance*>& /*FingerTypeToConstrs*/)
{
	// Set default state
	State = EGraspState::Free;

	// Set constraints target
	Target = 0.0f;
}

// Destructor
FRCGGrasp::~FRCGGrasp()
{
}

// Update grasping
void FRCGGrasp::Update(TMultiMap<ERCGHandLimb, FConstraintInstance*>& FingerTypeToConstrs,
	const float Step)
{
	// Lambda function for updating grasping
	auto GraspUpdateLambda = [&](TMultiMap<ERCGHandLimb, FConstraintInstance*>& FingerTypeToConstrsLambda)
	{
		// Iterate the finger type to constraints (joints) map
		for (const auto& TypeToConstr : FingerTypeToConstrsLambda)
		{
			// Get finger type
			const ERCGHandLimb Type = TypeToConstr.Key;

			// Skip iteration if finger is blocked 
			if (BlockedFingers.Contains(Type))
			{
				UE_LOG(LogTemp, Warning, TEXT("Blocked Skipping"));
				continue;
			}

			// Get finger constraints array
			TArray<FConstraintInstance*> FingerConstrArr;
			FingerTypeToConstrsLambda.MultiFind(Type, FingerConstrArr);

			// Apply position to the current finger types constraints
			for (FConstraintInstance* Constr : FingerConstrArr)
			{
				// get the limit of the constraint
				const float ConstrLimit = Constr->TwistLimitAngle;
				// Apply orientation target if constraints are not violated
				if (Target > ConstrLimit)
				{
					State = EGraspState::Closed;
				}
				else if (Target < - ConstrLimit)
				{
					State = EGraspState::Opened;
				}
				else
				{
					Constr->SetAngularOrientationTarget(FQuat(FRotator(Target, 0.0f, 0.0f)));
				}
			}
		}
	};
	

	// Check grasping states in order to apply/or not the relevant movement
	if (State == EGraspState::Free)
	{
		// Update target
		Target += Step;

		// Apply any movement
		GraspUpdateLambda(FingerTypeToConstrs);
	}
	else if (State == EGraspState::Closed && Step < 0)
	{
		// Update target
		Target += Step;

		// Set state to free and apply opening movement
		State = EGraspState::Free;
		GraspUpdateLambda(FingerTypeToConstrs);

	}
	else if (State == EGraspState::Opened && Step > 0)
	{
		// Update target
		Target += Step;

		// Set state to free and apply closing movement
		State = EGraspState::Free;
		GraspUpdateLambda(FingerTypeToConstrs);
	}
}

// Add finger to the blocked ones (grasping will have no effect on it)
void FRCGGrasp::BlockFinger(ERCGHandLimb Finger)
{
	UE_LOG(LogTemp, Warning, TEXT("Block test"));

	// Add unique
	if (!BlockedFingers.Contains(Finger))
	{
		UE_LOG(LogTemp, Warning, TEXT("ADD BLOCK FINGER"));
		BlockedFingers.Add(Finger);
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("Is already in the blocked fingers container"));
	}
}

// Remove finger from the blocked ones (grasping will effect it)
void FRCGGrasp::FreeFinger(ERCGHandLimb Finger)
{
	BlockedFingers.Remove(Finger);
}

// Free all fingers
void FRCGGrasp::FreeFingers()
{
	BlockedFingers.Empty();
}