// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RGraspPiano.h"

// Set default values
FRGraspPiano::FRGraspPiano()
{
	// Set default state
	SetState(ERGraspState::Free);
}

// Set default values
FRGraspPiano::FRGraspPiano(TMultiMap<ERHandLimb, FConstraintInstance*>& FingerTypeToConstrs)
{
	// Set default state
	SetState(ERGraspState::Free);

	// Set the fingers to control
	FingerTypeToConstraintsMMap = FingerTypeToConstrs;
	
	// Set the finger states
	for (const auto TypeToConstr : FingerTypeToConstrs)
	{
		// Get current finger type
		const ERHandLimb Type = TypeToConstr.Key;
		// Add type to the array
		FingerTypesArr.Add(Type);
		// Set finger state
		FingerToStateMap.Add(Type, ERGraspState::Free);
		// Set finger target 
		FingerToTargetMap.Add(Type, 0.0f);
	}

	// Set the active finger
	ActiveFingerIdx = 0;
	// Set the active finger
	SetActiveFinger(ActiveFingerIdx);
}

// Destructor
FRGraspPiano::~FRGraspPiano()
{
}

// Update grasping
void FRGraspPiano::Update(const float Step)
{
	// Lambda function for updating grasping
	auto FingerUpdateLambda = [&](TMultiMap<ERHandLimb, FConstraintInstance*>& FingerTypeToConstrsLambda,
		ERHandLimb Type)
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
				FingerToStateMap[Type] = ERGraspState::Closed;
			}
			else if (CurrFingerTarget < - ConstrLimit)
			{
				FingerToStateMap[Type] = ERGraspState::Opened;
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
		if (FingerToStateMap[ActiveFinger] == ERGraspState::Free)
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
	if (State == ERGraspState::Free)
	{
		ChooseFingerLambda();
	}
	else if (State == ERGraspState::Closed && Step < 0)
	{
		// Set state to free and apply opening movement
		State = ERGraspState::Free;

		ActiveFingerIdx = 0;

		ChooseFingerLambda();
	}
	else if (State == ERGraspState::Opened && Step > 0)
	{
		// Set state to free and apply closing movement
		State = ERGraspState::Free;

		ActiveFingerIdx = 0;

		ChooseFingerLambda();
	}
}

// Set the active finger in the grasp
bool FRGraspPiano::SetActiveFinger(uint8 ActiveFingerIdx)
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
