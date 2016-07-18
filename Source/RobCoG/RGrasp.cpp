// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RGrasp.h"

// Set default values
FRGrasp::FRGrasp()
{
	// Set default state
	SetState(ERGraspState::Free);	
}

// Set default values 
FRGrasp::FRGrasp(TMultiMap<ERHandLimb, FConstraintInstance*>& FingerTypeToConstrs)
{
	// Set the fingers to control
	FingerTypeToConstraintsMMap = FingerTypeToConstrs;

	// Set default state
	SetState(ERGraspState::Free);

	// Set every finger default target
	// This iterates every finger multiple types, since it is a multimap
	for (const auto FingerToConstr : FingerTypeToConstraintsMMap)
	{
		// Get the finger type
		const ERHandLimb FingerType = FingerToConstr.Key;
		// Set finger target
		FingerToTargetMap.Emplace(FingerType, 0.0f);
	}

	// Set prev step as a positive value (opening hand)
	PrevStep = 0.1;
}

// Destructor
FRGrasp::~FRGrasp()
{
}

// Update grasping
void FRGrasp::Update(const float Step)
{
	auto GraspUpdateLambda = [&](const float LambStep)
	{
		// Iterate through all the fingers
		for (auto FingerToConstrs : FingerToTargetMap)
		{
			// Finger type
			const ERHandLimb Finger = FingerToConstrs.Key;

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
					FRGrasp::BlockFinger(Finger);
					// Set it's taget to the contsraint limit,
					// otherwise the value might still be over the limit in the next tick,
					// thus blocking the fingers again
					FingerToTargetMap[Finger] = ConstrLimit;
				}
				else if (Target < - /*ConstrLimit*/ 5)
				{
					// The finger reached it's limit, mark it blocked
					FRGrasp::BlockFinger(Finger);
					// Set it's taget to the contsraint limit,
					// otherwise the value might still be over the limit in the next tick,
					// thus blocking the fingers again
					FingerToTargetMap[Finger] = - /*ConstrLimit*/ 5;
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
		// Update movements if state differs of blocked and attached
		//if ((GraspState != ERGraspState::Blocked) && (GraspState != ERGraspState::Attached))
		if (GraspState == ERGraspState::Free)
		{
			// Update target
			GraspUpdateLambda(Step);

			// If all the fingers are blocked set state as blocked
			if (BlockedFingers.Num() == FingerToTargetMap.Num())
			{
				SetState(ERGraspState::Blocked);				
			}
		}
	}
	else
	{
		// The orientation of the movement changed, free blocked fingers
		FRGrasp::FreeFingers();
		// Set state to free
		SetState(ERGraspState::Free);
		// Update target
		GraspUpdateLambda(Step);
	}

	// Update the previous step (used for checking open/close variation)
	PrevStep = Step;
}

// Set the grasp state
void FRGrasp::SetState(const ERGraspState State)
{
	GraspState = State;
}

// Get the grasp state
ERGraspState FRGrasp::GetState()
{
	return GraspState;
}

// Block the given finger at its current target
void FRGrasp::BlockFinger(ERHandLimb Finger)
{
	BlockedFingers.AddUnique(Finger);
}

// Free finger
void FRGrasp::FreeFinger(ERHandLimb Finger)
{	
	BlockedFingers.Remove(Finger);
}

// Free all fingers
void FRGrasp::FreeFingers()
{
	BlockedFingers.Empty();
}

// Check if finger is free
bool FRGrasp::IsFingerBlocked(const ERHandLimb Finger)
{
	return BlockedFingers.Contains(Finger);
}

// Check if finger is free
FString FRGrasp::GetStateAsString()
{
	const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("ERGraspState"), true);
	if (!EnumPtr)
	{
		return FString("Invalid");
	}
	return EnumPtr->GetEnumName((int32)GraspState);
}