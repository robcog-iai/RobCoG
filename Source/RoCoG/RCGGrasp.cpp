// Fill out your copyright notice in the Description page of Project Settings.

#include "RoCoG.h"
#include "RCGGrasp.h"

// Set default values
FRCGGrasp::FRCGGrasp()
{
	// Set default state
	SetState(ERCGGraspState::Free);
}

// Set default values 
FRCGGrasp::FRCGGrasp(TMultiMap<ERCGHandLimb, FConstraintInstance*>& FingerTypeToConstrs)
{
	// Set the fingers to control
	FingerTypeToConstraintsMMap = FingerTypeToConstrs;

	// Set default state
	SetState(ERCGGraspState::Free);

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

	// If the curr and prev step have the same sign, 
	// the grasping state has not changed (e.g. still closing, still opening)
	if (Step * PrevStep >= 0)
	{
		// Ignore movements if state is blocked or attached
		if ((GraspState != ERCGGraspState::Blocked) || (GraspState != ERCGGraspState::Attached))
		{
			// Update target
			GraspUpdateLambda(Step);

			// If all the fingers are blocked set state as blocked
			if (BlockedFingers.Num() == FingerToTargetMap.Num())
			{
				SetState(ERCGGraspState::Blocked);
			}
		}
	}
	else
	{
		// The orientation of the movement changed, free blocked fingers
		FRCGGrasp::FreeFingers();
		// Set state to free
		SetState(ERCGGraspState::Free);
		// Update target
		GraspUpdateLambda(Step);
	}

	// Update the previous step (used for checking open/close variation)
	PrevStep = Step;
}

// Set the grasp state
void FRCGGrasp::SetState(const ERCGGraspState State)
{
	GraspState = State;
}

// Get the grasp state
ERCGGraspState FRCGGrasp::GetState()
{
	return GraspState;
}

// Block the given finger at its current target
void FRCGGrasp::BlockFinger(ERCGHandLimb Finger)
{
	BlockedFingers.AddUnique(Finger);
}

// Free finger
void FRCGGrasp::FreeFinger(ERCGHandLimb Finger)
{	
	BlockedFingers.Remove(Finger);
}

// Free all fingers
void FRCGGrasp::FreeFingers()
{
	BlockedFingers.Empty();
}

// Check if finger is free
bool FRCGGrasp::IsFingerBlocked(const ERCGHandLimb Finger)
{
	return BlockedFingers.Contains(Finger);
}