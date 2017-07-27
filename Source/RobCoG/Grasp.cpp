// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "Grasp.h"
#include "Hand.h"
#include "Engine/Engine.h"
#include "Paths.h"

Grasp::Grasp()
{
	HandOrientationParserPtr = MakeShareable(new HandOrientationParser());
	CurrentGraspProcess = EGraspProcess::TwistAndSwing_Orientation;
	CurrentGraspType = EGraspType::FullGrasp;

	const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EGraspType"), true);
	if (!EnumPtr) return;

	FString ConfigDir = FPaths::GameConfigDir();

	FString GraspTypeString = EnumPtr->GetDisplayNameTextByIndex(static_cast<int64>(CurrentGraspType)).ToString();
	FString ConfigName = ConfigDir + GraspTypeString + ".ini";
	HandOrientationParserPtr->GetHandOrientationsForGraspType(InitialHandOrientation, ClosedHandOrientation, ConfigName);
}

Grasp::~Grasp()
{
}

void Grasp::SetInitialHandOrientation(const FHandOrientation & InitialHandOrientation)
{
	this->InitialHandOrientation = InitialHandOrientation;
}

void Grasp::SetClosedHandOrientation(const FHandOrientation & ClosedHandOrientation)
{
	this->ClosedHandOrientation = ClosedHandOrientation;
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
	FConstraintInstance* Constraint;

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

void Grasp::DriveToHandVelocityTarget(const FHandOrientation & HandOrientation, const AHand * const Hand)
{
	DriveToFingerVelocityTarget(HandOrientation.ThumbOrientation, Hand->Thumb);
	DriveToFingerVelocityTarget(HandOrientation.IndexOrientation, Hand->Index);
	DriveToFingerVelocityTarget(HandOrientation.MiddleOrientation, Hand->Middle);
	DriveToFingerVelocityTarget(HandOrientation.RingOrientation, Hand->Ring);
	DriveToFingerVelocityTarget(HandOrientation.PinkyOrientation, Hand->Pinky);
}

void Grasp::DriveToFingerVelocityTarget(const FFingerOrientation & FingerOrientation, const FFinger & Finger)
{
	FConstraintInstance* Constraint;

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Distal];
	if (Constraint)
		Constraint->SetAngularVelocityTarget(FingerOrientation.DistalOrientation.Orientation.Vector());

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Intermediate];
	if (Constraint)
		Constraint->SetAngularVelocityTarget(FingerOrientation.IntermediateOrientation.Orientation.Vector());

	Constraint = Finger.FingerPartToConstraint[EFingerPart::Proximal];
	if (Constraint)
		Constraint->SetAngularVelocityTarget(FingerOrientation.ProximalOrientation.Orientation.Vector());

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
			GraspStatus = EGraspStatus::Orientation;
			if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Green, "Orientation");
		}
		else if (GraspStatus == EGraspStatus::Orientation)
		{
			// Initialize Orientation Drives
			Hand->ResetAngularDriveValues(EAngularDriveMode::TwistAndSwing, EAngularDriveType::Orientation);
			// Manipulate Orientation Drives
			DriveToHandOrientationTarget(LerpHandOrientation(InitialHandOrientation, ClosedHandOrientation, Alpha), Hand);

			if (Alpha == 1.0 && CheckDistalVelocity(Hand, VelocityThreshold))
			{
				GraspStatus = EGraspStatus::Velocity;
				if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Green, "Velocity");
			}

		}
		else if (GraspStatus == EGraspStatus::Velocity)
		{
			// Initialize Velocity Drives
			Hand->ResetAngularDriveValues(EAngularDriveMode::TwistAndSwing, EAngularDriveType::Velocity);
			// Manipulate Velocity Drives
			// TODO: VELOCITY
			DriveToHandVelocityTarget(LerpHandOrientation(InitialHandOrientation, ClosedHandOrientation, Alpha), Hand);
		}
	}
	else
	{
		if (GraspStatus != EGraspStatus::Stopped)
		{
			DriveToInitialOrientation(Hand);
			GraspStatus = EGraspStatus::Stopped;
			if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Green, "Stopped");
		}
	}
}

bool Grasp::CheckDistalVelocity(const AHand* const Hand, const float VelocityThreshold)
{
	bool bVelocitySmaler = true;

	bVelocitySmaler = bVelocitySmaler && (Hand->Index.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() < VelocityThreshold);
	bVelocitySmaler = bVelocitySmaler && (Hand->Middle.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() < VelocityThreshold);
	bVelocitySmaler = bVelocitySmaler && (Hand->Ring.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() < VelocityThreshold);
	bVelocitySmaler = bVelocitySmaler && (Hand->Pinky.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() < VelocityThreshold);
	bVelocitySmaler = bVelocitySmaler && (Hand->Thumb.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size() < VelocityThreshold);

	return bVelocitySmaler;
}

bool Grasp::ForceOfAllConstraintsSmaler(const AHand* const Hand, const float ForceThreshold)
{
	bool bAllForcesSmaler = true;

	FVector OutAngularForce;
	FVector OutLinearForce;

	bAllForcesSmaler = bAllForcesSmaler && ForceOfAllFingerConstraintsSmaler(Hand->Index, ForceThreshold);
	bAllForcesSmaler = bAllForcesSmaler &&ForceOfAllFingerConstraintsSmaler(Hand->Middle, ForceThreshold);
	bAllForcesSmaler = bAllForcesSmaler &&ForceOfAllFingerConstraintsSmaler(Hand->Ring, ForceThreshold);
	bAllForcesSmaler = bAllForcesSmaler &&ForceOfAllFingerConstraintsSmaler(Hand->Pinky, ForceThreshold);
	bAllForcesSmaler = bAllForcesSmaler &&ForceOfAllFingerConstraintsSmaler(Hand->Thumb, ForceThreshold);

	return bAllForcesSmaler;
}

bool Grasp::ForceOfAllFingerConstraintsSmaler(const FFinger & Finger, const float ForceThreshold)
{
	bool bAllForcesSmaler = true;

	FVector OutAngularForce;
	FVector OutLinearForce;

	Finger.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	bAllForcesSmaler = bAllForcesSmaler && (OutAngularForce.Size() < ForceThreshold);
	Finger.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
	bAllForcesSmaler = bAllForcesSmaler && (OutAngularForce.Size() < ForceThreshold);
	Finger.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	bAllForcesSmaler = bAllForcesSmaler && (OutAngularForce.Size() < ForceThreshold);
	//Hand->Index.FingerPartToConstraint[EFingerPart::Metacarpal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	//bAllForcesSmaler = bAllForcesSmaler && (OutAngularForce.Size() < ForceThreshold);

	return bAllForcesSmaler;
}
bool Grasp::ForceOfAllConstraintsBigger(const AHand* const Hand, const float ForceThreshold)
{
	bool bAllForcesSmaler = true;

	FVector OutAngularForce;
	FVector OutLinearForce;

	bAllForcesSmaler = bAllForcesSmaler && ForceOfAllFingerConstraintsBigger(Hand->Index, ForceThreshold);
	bAllForcesSmaler = bAllForcesSmaler && ForceOfAllFingerConstraintsBigger(Hand->Middle, ForceThreshold);
	bAllForcesSmaler = bAllForcesSmaler && ForceOfAllFingerConstraintsBigger(Hand->Ring, ForceThreshold);
	bAllForcesSmaler = bAllForcesSmaler && ForceOfAllFingerConstraintsBigger(Hand->Pinky, ForceThreshold);
	bAllForcesSmaler = bAllForcesSmaler && ForceOfAllFingerConstraintsBigger(Hand->Thumb, ForceThreshold);

	return bAllForcesSmaler;
}

bool Grasp::ForceOfAllFingerConstraintsBigger(const FFinger & Finger, const float ForceThreshold)
{
	bool bAllForcesSmaler = true;

	FVector OutAngularForce;
	FVector OutLinearForce;

	Finger.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	bAllForcesSmaler = bAllForcesSmaler && (OutAngularForce.Size() < ForceThreshold);
	Finger.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
	bAllForcesSmaler = bAllForcesSmaler && (OutAngularForce.Size() < ForceThreshold);
	Finger.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	bAllForcesSmaler = bAllForcesSmaler && (OutAngularForce.Size() < ForceThreshold);
	//Hand->Index.FingerPartToConstraint[EFingerPart::Metacarpal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	//bAllForcesSmaler = bAllForcesSmaler && (OutAngularForce.Size() < ForceThreshold);

	return bAllForcesSmaler;
}

void Grasp::SwitchGraspStyle(const AHand * const Hand)
{
	if (HandOrientationParserPtr.IsValid())
	{
		FHandOrientation InitialHandOrientation;
		FHandOrientation ClosedHandOrientation;

		FString ConfigDir = FPaths::GameConfigDir();

		const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EGraspType"), true);
		if (!EnumPtr) return;

		int64 CurrentGraspTypeValue = static_cast<int64>(CurrentGraspType) + 1;

		if (CurrentGraspTypeValue >= EnumPtr->GetMaxEnumValue())
			CurrentGraspTypeValue = 0;

		CurrentGraspType = static_cast<EGraspType>(CurrentGraspTypeValue);

		FString GraspTypeString = EnumPtr->GetDisplayNameTextByIndex(static_cast<int64>(CurrentGraspType)).ToString();
		FString ConfigName = ConfigDir + GraspTypeString + ".ini";

		if (GEngine)
			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, FString::Printf(TEXT("CurrentGraspProcess: %s"), *GraspTypeString));

		HandOrientationParserPtr->GetHandOrientationsForGraspType(InitialHandOrientation, ClosedHandOrientation, ConfigName);

		SetInitialHandOrientation(InitialHandOrientation);
		SetClosedHandOrientation(ClosedHandOrientation);
		DriveToInitialOrientation(Hand);
	}
}

void Grasp::SwitchGraspProcess(AHand * const Hand, const float InSpring, const float InDamping, const float ForceLimit)
{
	const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EGraspProcess"), true);
	if (!EnumPtr) return;

	int64 CurrentGraspProcessValue = static_cast<int64>(CurrentGraspProcess) + 1;

	if (CurrentGraspProcessValue >= EnumPtr->GetMaxEnumValue())
		CurrentGraspProcessValue = 0;

	CurrentGraspProcess = static_cast<EGraspProcess>(CurrentGraspProcessValue);
	FString CurrentGraspProcessString = EnumPtr->GetDisplayNameTextByIndex(static_cast<int64>(CurrentGraspProcess)).ToString();

	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, FString::Printf(TEXT("CurrentGraspProcess: %s"), *CurrentGraspProcessString));

	//TODO: Initialize different grasp processes
	switch (CurrentGraspProcess)
	{
	case EGraspProcess::TwistAndSwing_Orientation:
		Hand->ResetAngularDriveValues(EAngularDriveMode::TwistAndSwing, EAngularDriveType::Orientation);
		break;

	case EGraspProcess::TwistAndSwing_Velocity:
		Hand->ResetAngularDriveValues(EAngularDriveMode::TwistAndSwing, EAngularDriveType::Velocity);
		break;

	case EGraspProcess::SLERP_Orientation:
		Hand->ResetAngularDriveValues(EAngularDriveMode::SLERP, EAngularDriveType::Orientation);
		break;

	case EGraspProcess::SLERP_Velocity:
		Hand->ResetAngularDriveValues(EAngularDriveMode::SLERP, EAngularDriveType::Velocity);
		break;
	}
}

void Grasp::PrintHandInfo(const AHand * const Hand)
{

	UE_LOG(LogTemp, Warning, TEXT("Index - Distal - Velocity: %f"),
		Hand->Index.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size());
	UE_LOG(LogTemp, Warning, TEXT("Middle - Distal - Velocity: %f"),
		Hand->Middle.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size());
	UE_LOG(LogTemp, Warning, TEXT("Ring - Distal - Velocity: %f"),
		Hand->Ring.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size());
	UE_LOG(LogTemp, Warning, TEXT("Pinky - Distal - Velocity: %f"),
		Hand->Pinky.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size());
	UE_LOG(LogTemp, Warning, TEXT("Thumb - Distal - Velocity: %f"),
		Hand->Thumb.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldVelocity().Size());

	//UE_LOG(LogTemp, Warning, TEXT("Index - Distal - AngularVelocity: %f"),
		//Hand->Index.FingerPartToBone[EFingerPart::Distal]->GetUnrealWorldAngularVelocity().Size());


	
	/*
	FVector OutLinearForce;
	FVector OutAngularForce;

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


FHandOrientation Grasp::LerpHandOrientation(const FHandOrientation & InitialHandOrientation, const FHandOrientation & ClosedHandOrientation, const float Alpha)
{
	FHandOrientation LerpedHandOrientation;

	LerpedHandOrientation.ThumbOrientation = LerpFingerOrientation(InitialHandOrientation.ThumbOrientation, ClosedHandOrientation.ThumbOrientation, Alpha);
	LerpedHandOrientation.IndexOrientation = LerpFingerOrientation(InitialHandOrientation.IndexOrientation, ClosedHandOrientation.IndexOrientation, Alpha);
	LerpedHandOrientation.MiddleOrientation = LerpFingerOrientation(InitialHandOrientation.MiddleOrientation, ClosedHandOrientation.MiddleOrientation, Alpha);
	LerpedHandOrientation.RingOrientation = LerpFingerOrientation(InitialHandOrientation.RingOrientation, ClosedHandOrientation.RingOrientation, Alpha);
	LerpedHandOrientation.PinkyOrientation = LerpFingerOrientation(InitialHandOrientation.PinkyOrientation, ClosedHandOrientation.PinkyOrientation, Alpha);

	return LerpedHandOrientation;
}

FFingerOrientation Grasp::LerpFingerOrientation(const FFingerOrientation & InitialFingerOrientation, const FFingerOrientation & ClosedFingerOrientation, const float Alpha)
{
	FFingerOrientation LerpedFingerOrientation;

	LerpedFingerOrientation.DistalOrientation.Orientation = FMath::LerpRange(
		InitialFingerOrientation.DistalOrientation.Orientation,
		ClosedFingerOrientation.DistalOrientation.Orientation, Alpha);
	LerpedFingerOrientation.MetacarpalOrientation.Orientation = FMath::LerpRange(
		InitialFingerOrientation.MetacarpalOrientation.Orientation,
		ClosedFingerOrientation.MetacarpalOrientation.Orientation, Alpha);
	LerpedFingerOrientation.ProximalOrientation.Orientation = FMath::LerpRange(
		InitialFingerOrientation.ProximalOrientation.Orientation,
		ClosedFingerOrientation.ProximalOrientation.Orientation, Alpha);
	LerpedFingerOrientation.IntermediateOrientation.Orientation = FMath::LerpRange(
		InitialFingerOrientation.IntermediateOrientation.Orientation,
		ClosedFingerOrientation.IntermediateOrientation.Orientation, Alpha);

	return LerpedFingerOrientation;
}
