// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "Grasp.h"
#include "Hand.h"
#include "Engine/Engine.h"
#include "Paths.h"

Grasp::Grasp()
{
	HandInformationParserPtr = MakeShareable(new HandInformationParser());
	HandLoggerPtr = NewObject<HandLogger>();

	GraspStatus = EGraspStatus::Velocity;
	CurrentAngularDriveMode = EAngularDriveMode::SLERP;
	CurrentGraspType = EGraspType::FullGrasp;

	const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EGraspType"), true);
	if (!EnumPtr) return;

	FString ConfigDir = FPaths::GameConfigDir();

	//For creating new HandInformation .Ini
	//FString TestConfig = ConfigDir + "TestGrasp.ini";
	//HandOrientationParserPtr->SetHandInformationForGraspType(InitialHandOrientation, ClosedHandOrientation, HandVelocity, TestConfig);

	FString GraspTypeString = EnumPtr->GetDisplayNameTextByIndex(static_cast<int64>(CurrentGraspType)).ToString();
	FString ConfigName = ConfigDir + GraspTypeString + ".ini";
	HandInformationParserPtr->GetHandInformationForGraspType(InitialHandOrientation, ClosedHandOrientation, HandVelocity, ConfigName);
}

Grasp::~Grasp()
{
}

void Grasp::DriveToInitialOrientation(const AHand * const Hand)
{
	DriveToHandOrientationTarget(InitialHandOrientation, Hand);
}

void Grasp::DriveToHandOrientationTarget(const FHandOrientation & HandOrientation, const AHand * const Hand)
{
	DriveToFingerOrientationTarget(HandOrientation.ThumbOrientation, Hand->Thumb);
	DriveToFingerOrientationTarget(HandOrientation.IndexOrientation, Hand->Index);
	DriveToFingerOrientationTarget(HandOrientation.MiddleOrientation, Hand->Middle);
	DriveToFingerOrientationTarget(HandOrientation.RingOrientation, Hand->Ring);
	DriveToFingerOrientationTarget(HandOrientation.PinkyOrientation, Hand->Pinky);
}

void Grasp::DriveToFingerOrientationTarget(const FFingerOrientation & FingerOrientation, const FFinger & Finger)
{
	FConstraintInstance* Constraint = nullptr;

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Distal];
	if (Constraint)
		Constraint->SetAngularOrientationTarget(FingerOrientation.DistalOrientation.Orientation.Quaternion());

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Intermediate];
	if (Constraint)
		Constraint->SetAngularOrientationTarget(FingerOrientation.IntermediateOrientation.Orientation.Quaternion());

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Proximal];
	if (Constraint)
		Constraint->SetAngularOrientationTarget(FingerOrientation.ProximalOrientation.Orientation.Quaternion());

	/* Not Implemented yet
	Constraint = Finger.FingerPartToConstraint[EFingerPart::Metacarpal];
	if (Constraint)
	Constraint->SetAngularOrientationTarget(FingerOrientation.MetacarpalOrientation.Orientation.Quaternion());
	*/
}

void Grasp::DriveToHandVelocityTarget(const FHandVelocity & HandVelocity, const AHand * const Hand)
{
	DriveToFingerVelocityTarget(HandVelocity.ThumbVelocity, Hand->Thumb);
	DriveToFingerVelocityTarget(HandVelocity.IndexVelocity, Hand->Index);
	DriveToFingerVelocityTarget(HandVelocity.MiddleVelocity, Hand->Middle);
	DriveToFingerVelocityTarget(HandVelocity.RingVelocity, Hand->Ring);
	DriveToFingerVelocityTarget(HandVelocity.PinkyVelocity, Hand->Pinky);
}

void Grasp::DriveToFingerVelocityTarget(const FFingerVelocity & FingerVelocity, const FFinger & Finger)
{
	FConstraintInstance* Constraint = nullptr;

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Distal];
	if (Constraint && FingerVelocity.DistalVelocity.Velocity.Size() != 0.0)
		Constraint->SetAngularVelocityTarget(FingerVelocity.DistalVelocity.Velocity);

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Intermediate];
	if (Constraint && FingerVelocity.IntermediateVelocity.Velocity.Size() != 0.0)
		Constraint->SetAngularVelocityTarget(FingerVelocity.IntermediateVelocity.Velocity);

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Proximal];
	if (Constraint && FingerVelocity.ProximalVelocity.Velocity.Size() != 0.0)
		Constraint->SetAngularVelocityTarget(FingerVelocity.ProximalVelocity.Velocity);

	/* Not Implemented yet
	Constraint = Finger.FingerPartToConstraint[EFingerPart::Metacarpal];
	if (Constraint && FingerVelocity.MetacarpalOrientation.Velocity.Size() != 0.0)
	Constraint->SetAngularVelocityTarget(FingerOrientation.MetacarpalOrientation.Orientation.Vector());
	*/
}

void Grasp::UpdateGrasp(const float Alpha, const float VelocityThreshold, AHand * const Hand)
{
	if (Alpha != 0)
	{
		if (GraspStatus == EGraspStatus::Stopped)
		{
			// Manipulate Orientation Drives
			FHandOrientation TargetOrientation;
			LerpHandOrientation(TargetOrientation, InitialHandOrientation, ClosedHandOrientation, Alpha);
			DriveToHandOrientationTarget(TargetOrientation, Hand);

			if (CheckDistalVelocity(Hand, VelocityThreshold, EComparison::Bigger))
			{
				GraspStatus = EGraspStatus::Orientation;
				if (GEngine) GEngine->AddOnScreenDebugMessage(1, 5, FColor::Green, "GraspStatus: Orientation");
			}
		}
		else if (GraspStatus == EGraspStatus::Orientation)
		{
			// Manipulate Orientation Drives
			FHandOrientation TargetOrientation;
			LerpHandOrientation(TargetOrientation, InitialHandOrientation, ClosedHandOrientation, Alpha);
			DriveToHandOrientationTarget(TargetOrientation, Hand);

			if (Alpha == 1.0 && CheckDistalVelocity(Hand, VelocityThreshold, EComparison::Smaller))
			{
				GraspStatus = EGraspStatus::Velocity;
				if (GEngine) GEngine->AddOnScreenDebugMessage(1, 5, FColor::Green, "GraspStatus: Velocity");				
			}

		}
		else if (GraspStatus == EGraspStatus::Velocity)
		{
			// Initialize Velocity Drives
			Hand->ResetAngularDriveValues(CurrentAngularDriveMode, EAngularDriveType::Velocity);
			// Lock Proximal Thumb
			//LockConstraint(Hand->Thumb.FingerPartToConstraint[EFingerPart::Proximal]);

			// Manipulate Velocity Drives - Orientations of SLERP and T&S are different
			if (CurrentAngularDriveMode == EAngularDriveMode::SLERP)
				DriveToHandVelocityTarget(HandVelocity, Hand);
			else if(CurrentAngularDriveMode == EAngularDriveMode::TwistAndSwing)
				DriveToHandVelocityTarget(HandVelocity*-1, Hand);
		}
	}
	else
	{
		// Stop Grasp
		if (GraspStatus != EGraspStatus::Stopped)
		{
			//UnlockConstraint(Hand->Thumb.FingerPartToConstraint[EFingerPart::Proximal]);
			Hand->ResetAngularDriveValues(CurrentAngularDriveMode, EAngularDriveType::Orientation);
			DriveToInitialOrientation(Hand);
			GraspStatus = EGraspStatus::Stopped;
			if (GEngine) GEngine->AddOnScreenDebugMessage(1, 5, FColor::Green, "GraspStatus: Stopped");
		}
	}
}

bool Grasp::CheckDistalVelocity(const AHand* const Hand, const float VelocityThreshold, const EComparison Comparison)
{
	bool bVelocitySmaler = false;
	if (Comparison == EComparison::Smaller)
	{
		bVelocitySmaler = true;

		bVelocitySmaler = bVelocitySmaler && (Hand->Index.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() < VelocityThreshold);
		bVelocitySmaler = bVelocitySmaler && (Hand->Middle.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() < VelocityThreshold);
		bVelocitySmaler = bVelocitySmaler && (Hand->Ring.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() < VelocityThreshold);
		bVelocitySmaler = bVelocitySmaler && (Hand->Pinky.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() < VelocityThreshold);
		bVelocitySmaler = bVelocitySmaler && (Hand->Thumb.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() < VelocityThreshold);
	}
	else if (Comparison == EComparison::Bigger)
	{
		bVelocitySmaler = false;

		bVelocitySmaler = bVelocitySmaler || (Hand->Index.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() > VelocityThreshold);
		bVelocitySmaler = bVelocitySmaler || (Hand->Middle.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() > VelocityThreshold);
		bVelocitySmaler = bVelocitySmaler || (Hand->Ring.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() > VelocityThreshold);
		bVelocitySmaler = bVelocitySmaler || (Hand->Pinky.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() > VelocityThreshold);
		bVelocitySmaler = bVelocitySmaler || (Hand->Thumb.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() > VelocityThreshold);
	}
	return bVelocitySmaler;
}

void Grasp::SwitchGraspStyle(const AHand * const Hand, EGraspType GraspType)
{
	if (HandInformationParserPtr.IsValid())
	{
		FString ConfigDir = FPaths::GameConfigDir();

		const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EGraspType"), true);
		if (!EnumPtr) return;

		CurrentGraspType = GraspType;
		
		FString GraspTypeString = EnumPtr->GetDisplayNameTextByIndex(static_cast<int64>(CurrentGraspType)).ToString();
		FString ConfigName = ConfigDir + GraspTypeString + ".ini";

		if (GEngine)
			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, FString::Printf(TEXT("CurrentGraspProcess: %s"), *GraspTypeString));

		HandInformationParserPtr->GetHandInformationForGraspType(InitialHandOrientation, ClosedHandOrientation, HandVelocity, ConfigName);

		DriveToInitialOrientation(Hand);
	}
}

void Grasp::SwitchGraspProcess(AHand * const Hand, const float InSpring, const float InDamping, const float ForceLimit)
{
	// TODO: Just if else

	const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EAngularDriveMode"), true);
	if (!EnumPtr) return;

	int64 CurrentAngularDriveModeValue = static_cast<int64>(CurrentAngularDriveMode) + 1;

	if (CurrentAngularDriveModeValue >= EnumPtr->GetMaxEnumValue())
		CurrentAngularDriveModeValue = 0;

	CurrentAngularDriveMode = static_cast<EAngularDriveMode::Type>(CurrentAngularDriveModeValue);
	FString CurrentGraspProcessString = EnumPtr->GetDisplayNameTextByIndex(static_cast<int64>(CurrentAngularDriveMode)).ToString();

	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, FString::Printf(TEXT("CurrentGraspProcess: %s"), *CurrentGraspProcessString));

	switch (CurrentAngularDriveMode)
	{
	case EAngularDriveMode::TwistAndSwing:
		CurrentAngularDriveMode = EAngularDriveMode::TwistAndSwing;
		break;

	case EAngularDriveMode::SLERP:
		CurrentAngularDriveMode = EAngularDriveMode::SLERP;
		break;

	default:
		break;
	}

	Hand->ResetAngularDriveValues(CurrentAngularDriveMode, EAngularDriveType::Orientation);
}

void Grasp::LerpHandOrientation(FHandOrientation & TargetHandOrientation, const FHandOrientation & InitialHandOrientation, const FHandOrientation & ClosedHandOrientation, const float Alpha)
{
	LerpFingerOrientation(TargetHandOrientation.ThumbOrientation, InitialHandOrientation.ThumbOrientation, ClosedHandOrientation.ThumbOrientation, Alpha);
	LerpFingerOrientation(TargetHandOrientation.IndexOrientation,InitialHandOrientation.IndexOrientation, ClosedHandOrientation.IndexOrientation, Alpha);
	LerpFingerOrientation(TargetHandOrientation.MiddleOrientation, InitialHandOrientation.MiddleOrientation, ClosedHandOrientation.MiddleOrientation, Alpha);
	LerpFingerOrientation(TargetHandOrientation.RingOrientation, InitialHandOrientation.RingOrientation, ClosedHandOrientation.RingOrientation, Alpha);
	LerpFingerOrientation(TargetHandOrientation.PinkyOrientation, InitialHandOrientation.PinkyOrientation, ClosedHandOrientation.PinkyOrientation, Alpha);

}

void Grasp::LerpFingerOrientation(FFingerOrientation & TargetFingerOrientation, const FFingerOrientation & InitialFingerOrientation, const FFingerOrientation & ClosedFingerOrientation, const float Alpha)
{
	TargetFingerOrientation.DistalOrientation.Orientation = FMath::LerpRange(
		InitialFingerOrientation.DistalOrientation.Orientation,
		ClosedFingerOrientation.DistalOrientation.Orientation, Alpha);
	TargetFingerOrientation.MetacarpalOrientation.Orientation = FMath::LerpRange(
		InitialFingerOrientation.MetacarpalOrientation.Orientation,
		ClosedFingerOrientation.MetacarpalOrientation.Orientation, Alpha);
	TargetFingerOrientation.ProximalOrientation.Orientation = FMath::LerpRange(
		InitialFingerOrientation.ProximalOrientation.Orientation,
		ClosedFingerOrientation.ProximalOrientation.Orientation, Alpha);
	TargetFingerOrientation.IntermediateOrientation.Orientation = FMath::LerpRange(
		InitialFingerOrientation.IntermediateOrientation.Orientation,
		ClosedFingerOrientation.IntermediateOrientation.Orientation, Alpha);
}

void Grasp::PrintHandInfo(const AHand * const Hand) const
{
	FVector OutLinearForce;
	FVector OutAngularForce;

	Hand->Thumb.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(2, 1, FColor::Blue, FString::Printf(TEXT("Thumb - Distal: %f"), OutAngularForce.Size()));
	Hand->Thumb.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(3, 1, FColor::Blue, FString::Printf(TEXT("Thumb - Intermediate: %f"), OutAngularForce.Size()));
	Hand->Thumb.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(4, 1, FColor::Blue, FString::Printf(TEXT("Thumb - Proximal: %f"), OutAngularForce.Size()));

	Hand->Index.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(5, 1, FColor::Blue, FString::Printf(TEXT("Index - Distal: %f"), OutAngularForce.Size()));
	Hand->Index.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(6, 1, FColor::Blue, FString::Printf(TEXT("Index - Intermediate: %f"), OutAngularForce.Size()));
	Hand->Index.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(7, 1, FColor::Blue, FString::Printf(TEXT("Index - Proximal: %f"), OutAngularForce.Size()));

	Hand->Middle.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(8, 1, FColor::Blue, FString::Printf(TEXT("Middle - Distal: %f"), OutAngularForce.Size()));
	Hand->Middle.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(9, 1, FColor::Blue, FString::Printf(TEXT("Middle - Intermediate: %f"), OutAngularForce.Size()));
	Hand->Middle.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(10, 1, FColor::Blue, FString::Printf(TEXT("Middle - Proximal: %f"), OutAngularForce.Size()));

	Hand->Ring.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(11, 1, FColor::Blue, FString::Printf(TEXT("Ring - Distal: %f"), OutAngularForce.Size()));
	Hand->Ring.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(12, 1, FColor::Blue, FString::Printf(TEXT("Ring - Intermediate: %f"), OutAngularForce.Size()));
	Hand->Ring.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(13, 1, FColor::Blue, FString::Printf(TEXT("Ring - Proximal: %f"), OutAngularForce.Size()));

	Hand->Pinky.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(14, 1, FColor::Blue, FString::Printf(TEXT("Pinky - Distal: %f"), OutAngularForce.Size()));
	Hand->Pinky.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(15, 1, FColor::Blue, FString::Printf(TEXT("Pinky - Intermediate: %f"), OutAngularForce.Size()));
	Hand->Pinky.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(16, 1, FColor::Blue, FString::Printf(TEXT("Pinky - Proximal: %f"), OutAngularForce.Size()));

}

void Grasp::LockConstraint( FConstraintInstance* Constraint)
{
	Constraint->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked,500.0);
	Constraint->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 500.0);
	Constraint->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked,500.0);
}

void Grasp::UnlockConstraint(FConstraintInstance* Constraint)
{
	Constraint->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Limited, 500.0);
	Constraint->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, 500.0);
	Constraint->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, 500.0);
}