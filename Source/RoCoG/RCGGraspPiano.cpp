// Fill out your copyright notice in the Description page of Project Settings.

#include "RoCoG.h"
#include "RCGGraspPiano.h"

// Set default values
FRCGGraspPiano::FRCGGraspPiano()
{
	// Set default state
	State = EGraspState::Free;
}

// Set default values
FRCGGraspPiano::FRCGGraspPiano(TMultiMap<ERCGHandLimb, FConstraintInstance*>& FingerTypeToConstrs)
{
	// Set default state
	State = EGraspState::Free;

	// Set the fingers to control
	FingerTypeToConstraintsMMap = FingerTypeToConstrs;
	
	// Set the finger states
	for (const auto TypeToConstr : FingerTypeToConstrs)
	{
		// Get current finger type
		const ERCGHandLimb Type = TypeToConstr.Key;
		// Add type to the array
		FingerTypesArr.Add(Type);
		// Set finger state
		FingerToStateMap.Add(Type, EGraspState::Free);
		// Set finger target 
		FingerToTargetMap.Add(Type, 0.0f);
	}

	// Set the active finger
	ActiveFingerIdx = 0;
	// Set the active finger
	SetActiveFinger(ActiveFingerIdx);
}

// Destructor
FRCGGraspPiano::~FRCGGraspPiano()
{
}

// Update grasping
void FRCGGraspPiano::Update(const float Step)
{
	// Lambda function for updating grasping
	auto FingerUpdateLambda = [&](TMultiMap<ERCGHandLimb, FConstraintInstance*>& FingerTypeToConstrsLambda,
		ERCGHandLimb Type)
	{
		// Get finger constraints array
		TArray<FConstraintInstance*> FingerConstrArr;
		FingerTypeToConstrsLambda.MultiFind(Type, FingerConstrArr);

		// Current finger target
		const float CurrFingerTarget = FingerToTargetMap[Type];

		// Apply position to the current finger types constraints
		for (FConstraintInstance* Constr : FingerConstrArr)
		{
			// Get the limit of the constraint
			const float ConstrLimit = Constr->TwistLimitAngle;			
			// Apply orientation target if constraints are not violated
			if (CurrFingerTarget > ConstrLimit)
			{
				FingerToStateMap[Type] = EGraspState::Closed;
			}
			else if (CurrFingerTarget < - ConstrLimit)
			{
				FingerToStateMap[Type] = EGraspState::Opened;
			}
			else
			{
				Constr->SetAngularOrientationTarget(FQuat(FRotator(CurrFingerTarget, 0.0f, 0.0f)));
			}
		}
	};
	
	// Lambda for choosing the right finger
	auto ChooseFingerLambda = [&]()
	{
		// Update finger target
		FingerToTargetMap[ActiveFinger] += Step;

		// Check if the active finger is in state free
		if (FingerToStateMap[ActiveFinger] == EGraspState::Free)
		{
			// Apply movement to the active finger
			FingerUpdateLambda(FingerTypeToConstraintsMMap, ActiveFinger);
		}
		else if (FingerTypesArr.IsValidIndex(++ActiveFingerIdx))
		{
			// Set active finger
			SetActiveFinger(ActiveFingerIdx);
			// Apply movement to the active finger
			FingerUpdateLambda(FingerTypeToConstraintsMMap, ActiveFinger);
		}
		else
		{
			// If the last finger was set, set its state (closed/opened) to the grasp
			State = FingerToStateMap[ActiveFinger];
		}
	};

	// Check grasping states in order to apply/or not the relevant movement
	if (State == EGraspState::Free)
	{
		ChooseFingerLambda();
	}
	else if (State == EGraspState::Closed && Step < 0)
	{
		// Set state to free and apply opening movement
		State = EGraspState::Free;

		ActiveFingerIdx = 0;

		ChooseFingerLambda();
	}
	else if (State == EGraspState::Opened && Step > 0)
	{
		// Set state to free and apply closing movement
		State = EGraspState::Free;

		ActiveFingerIdx = 0;

		ChooseFingerLambda();
	}
}

// Set the active finger in the grasp
bool FRCGGraspPiano::SetActiveFinger(uint8 ActiveFingerIdx)
{
	// Set active finger
	if (FingerTypesArr.IsValidIndex(ActiveFingerIdx))
	{
		ActiveFinger = FingerTypesArr[ActiveFingerIdx];
		return true;
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Invalid finger index: %i"), ActiveFingerIdx);
		return false;
	}
}
