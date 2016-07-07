// Fill out your copyright notice in the Description page of Project Settings.

#include "RoCoG.h"
#include "RCGGrasp.h"

// Set default values
FRCGGrasp::FRCGGrasp()
{
	// Set default state
	State = EGraspState::Free;
}

// Set default values 
FRCGGrasp::FRCGGrasp(TMultiMap<ERCGHandLimb, FConstraintInstance*>& FingerTypeToConstrs)
{
	// Set the fingers to control
	FingerTypeToConstraintsMMap = FingerTypeToConstrs;

	// Set default state
	State = EGraspState::Free;

	// Set every finger default target
	// This iterates every finger multiple types, since it is a multimap
	for (const auto FingerToConstr : FingerTypeToConstraintsMMap)
	{
		// Get the finger type
		const ERCGHandLimb FingerType = FingerToConstr.Key;
		// Set finger target
		FingerToTargetMap.Emplace(FingerType, 0.0f);
	}

	// Set prev step as a positive value (opening hand)
	PrevStep = 0.1;
}

// Destructor
FRCGGrasp::~FRCGGrasp()
{
}

// Update grasping
void FRCGGrasp::Update(const float Step)
{
	auto GraspUpdateLambda = [&](const float LambStep)
	{
		// Iterate through all the fingers
		for (auto FingerToConstrs : FingerToTargetMap)
		{
			// Finger type
			const ERCGHandLimb Finger = FingerToConstrs.Key;

			// Skip iteration if finger is blocked 
			if (BlockedFingers.Contains(Finger))
			{
				continue;
			}

			// Get and update finger target
			const float Target = FingerToTargetMap[Finger] += LambStep;

			// Get the fingers constraints array
			TArray<FConstraintInstance*> FingerConstrArr;
			FingerTypeToConstraintsMMap.MultiFind(Finger, FingerConstrArr);

			// Apply position to the current finger's constraints
			for (FConstraintInstance* Constr : FingerConstrArr)
			{
				// Get the limit of the constraint
				const float ConstrLimit = Constr->Swing2LimitAngle;
				// Apply orientation target if constraints are not violated
				if (Target > ConstrLimit)
				{
					// The finger reached it's limit, mark it blocked
					FRCGGrasp::BlockFinger(Finger);
					// Set it's taget to the contsraint limit,
					// otherwise the value might still be over the limit in the next tick,
					// thus blocking the fingers again
					FingerToTargetMap[Finger] = ConstrLimit;
				}
				else if (Target < - ConstrLimit)
				{
					// The finger reached it's limit, mark it blocked
					FRCGGrasp::BlockFinger(Finger);
					// Set it's taget to the contsraint limit,
					// otherwise the value might still be over the limit in the next tick,
					// thus blocking the fingers again
					FingerToTargetMap[Finger] = - ConstrLimit;
				}
				else
				{
					Constr->SetAngularOrientationTarget(FQuat(FRotator(0.0f, 0.0f, Target)));
				}
			}
		}
	};

	// Check grasping states in order to apply/or not the relevant movement
	if (State == EGraspState::Free)
	{
		// If the current and the previous step are positive (did not change)
		if (Step * PrevStep >= 0)
		{
			//UE_LOG(LogTemp, Warning, TEXT("\tPOS %f"), Step * PrevStep);
			// Update target
			GraspUpdateLambda(Step);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("\tNEG %f"), Step * PrevStep);

			// The orientation of the movement changed, free blocked fingers
			FRCGGrasp::FreeFingers();
			// Update target
			GraspUpdateLambda(Step);
		}
	}
	//else if (State == EGraspState::Closed && Step < 0)
	//{
	//	// Set state to free and apply opening movement
	//	State = EGraspState::Free;

	//	// Update target
	//	GraspUpdateLambda(Step);
	//}
	//else if (State == EGraspState::Opened && Step > 0)
	//{
	//	// Set state to free and apply opening movement
	//	State = EGraspState::Free;

	//	// Update target
	//	GraspUpdateLambda(Step);
	//}

	// Update the prevous step (used for checking open/close variation)
	PrevStep = Step;
}

// Block the given finger at its current target
void FRCGGrasp::BlockFinger(ERCGHandLimb Finger)
{
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

// Free finger
void FRCGGrasp::FreeFinger(ERCGHandLimb Finger)
{	
	BlockedFingers.Remove(Finger);
}

// Free all fingers
void FRCGGrasp::FreeFingers()
{
	State = EGraspState::Free;
	BlockedFingers.Empty();
}