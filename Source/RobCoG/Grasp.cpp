// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "Grasp.h"
#include "Hand.h"
#include "Engine/Engine.h"
#include "Paths.h"

Grasp::Grasp()
{
	HandInformationParserPtr = MakeShareable(new HandInformationParser());
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
	FConstraintInstance* Constraint;

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Distal];
	if (Constraint)
		Constraint->SetAngularVelocityTarget(FingerVelocity.DistalVelocity.Velocity);

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Intermediate];
	if (Constraint)
		Constraint->SetAngularVelocityTarget(FingerVelocity.IntermediateVelocity.Velocity);

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Proximal];
	if (Constraint)
		Constraint->SetAngularVelocityTarget(FingerVelocity.ProximalVelocity.Velocity);

	/* Not Implemented yet
	Constraint = Finger.FingerPartToConstraint[EFingerPart::Metacarpal];
	if (Constraint)
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
			// Manipulate Velocity Drives
			DriveToHandVelocityTarget(HandVelocity, Hand);
		}
	}
	else
	{
		// Stop Grasp
		if (GraspStatus != EGraspStatus::Stopped)
		{
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

	//UE_LOG(LogTemp, Warning, TEXT("Index - Distal - Velocity: %f"),
	//	Hand->Index.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size());
	//UE_LOG(LogTemp, Warning, TEXT("Middle - Distal - Velocity: %f"),
	//	Hand->Middle.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size());
	//UE_LOG(LogTemp, Warning, TEXT("Ring - Distal - Velocity: %f"),
	//	Hand->Ring.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size());
	//UE_LOG(LogTemp, Warning, TEXT("Pinky - Distal - Velocity: %f"),
	//	Hand->Pinky.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size());
	//UE_LOG(LogTemp, Warning, TEXT("Thumb - Distal - Velocity: %f"),
	//	Hand->Thumb.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size());

	//UE_LOG(LogTemp, Warning, TEXT("Index - Distal - AngularVelocity: %f"),
	//	Hand->Index.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldAngularVelocity().Size());

	FVector OutLinearForce;
	FVector OutAngularForce;

	Hand->Thumb.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(2, 1, FColor::Blue, FString::Printf(TEXT("Thumb - Part: DistalConstraint - Force: %f"), OutAngularForce.Size()));

	Hand->Index.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	if (GEngine) GEngine->AddOnScreenDebugMessage(3, 1, FColor::Blue, FString::Printf(TEXT("Index - Part: DistalConstraint - Force: %f"), OutAngularForce.Size()));

	/*
		Hand->Thumb.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Thumb - Part: DistalConstraint - Force: %f"), OutAngularForce.Size());
		Hand->Thumb.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Thumb - Part: Intermetiate - Force: %f"), OutAngularForce.Size());
		Hand->Thumb.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Thumb - Part: ProximalConstraint - Force: %f"), OutAngularForce.Size());

		Hand->Index.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Index - Part: DistalConstraint - Force: %f"), OutAngularForce.Size());
		Hand->Index.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Index - Part: Intermetiate - Force: %f"), OutAngularForce.Size());
		Hand->Index.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Index - Part: ProximalConstraint - Force: %f"), OutAngularForce.Size());

		Hand->Middle.FingerPartToConstraint[EFingerPart::DistalConstraint]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Middle - Part: DistalConstraint - Force: %s"), *OutAngularForce.ToString());
		Hand->Middle.FingerPartToConstraint[EFingerPart::IntermediateConstraint]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Middle - Part: Intermetiate - Force: %s"), *OutAngularForce.ToString());
		Hand->Middle.FingerPartToConstraint[EFingerPart::ProximalConstraint]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Middle - Part: ProximalConstraint - Force: %s"), *OutAngularForce.ToString());

		Hand->Ring.FingerPartToConstraint[EFingerPart::DistalConstraint]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Ring - Part: DistalConstraint - Force: %s"), *OutAngularForce.ToString());
		Hand->Ring.FingerPartToConstraint[EFingerPart::IntermediateConstraint]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Ring - Part: Intermetiate - Force: %s"), *OutAngularForce.ToString());
		Hand->Ring.FingerPartToConstraint[EFingerPart::ProximalConstraint]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Ring - Part: ProximalConstraint - Force: %s"), *OutAngularForce.ToString());

		Hand->Pinky.FingerPartToConstraint[EFingerPart::DistalConstraint]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Pinky - Part: DistalConstraint - Force: %s"), *OutAngularForce.ToString());
		Hand->Pinky.FingerPartToConstraint[EFingerPart::IntermediateConstraint]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Pinky - Part: Intermetiate - Force: %s"), *OutAngularForce.ToString());
		Hand->Pinky.FingerPartToConstraint[EFingerPart::ProximalConstraint]->GetConstraintForce(OutLinearForce, OutAngularForce);
		UE_LOG(LogTemp, Warning, TEXT("Pinky - Part: ProximalConstraint - Force: %s"), *OutAngularForce.ToString());
	*/
}
