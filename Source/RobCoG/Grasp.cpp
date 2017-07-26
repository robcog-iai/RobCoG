// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "Grasp.h"
#include "Hand.h"
#include "Engine/Engine.h"
#include "Paths.h"

Grasp::Grasp()
{
	HandOrientationParserPtr = MakeShareable(new HandOrientationParser());
	CurrentGraspProcess = EGraspProcess::TwistAndSwing;
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

void Grasp::SetInitialHandOrientation(FHandOrientation InitialHandOrientation)
{
	this->InitialHandOrientation = InitialHandOrientation;
}

void Grasp::SetClosedHandOrientation(FHandOrientation ClosedHandOrientation)
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

void Grasp::UpdateGrasp(const float Alpha, const float ForceThreshold, AHand * const Hand)
{
	if (Alpha != 0)
	{
		if (GraspStatus == EGraspStatus::Stopped)
		{
			GraspStatus = EGraspStatus::OrientationStarting;
		}
		else if (GraspStatus == EGraspStatus::OrientationStarting)
		{
			// TODO: Initialize Orientation Drives
			// TODO: Manipulate Orientation Drives

			if (ForceOfAllConstraintsBigger(Hand, 1))
			{
				GraspStatus = EGraspStatus::OrientationRunning;
			}

		}
		else if (GraspStatus == EGraspStatus::OrientationRunning)
		{
			if (ForceOfAllConstraintsSmaler(Hand, ForceThreshold))
			{
				GraspStatus = EGraspStatus::Velocity;
			}
		}
		else if (GraspStatus == EGraspStatus::Velocity)
		{
			// TODO: Initialize Velocity Drives
			// TODO: Manipulate Velocity Drives
		}
	}
	else
	{
		if (GraspStatus != EGraspStatus::Stopped)
		{
			// TODO: Drive to Initial
			GraspStatus = EGraspStatus::Stopped;
		}
	}
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
	case EGraspProcess::TwistAndSwing:
		Hand->Thumb.SetFingerDriveMode(EAngularDriveMode::TwistAndSwing, InSpring, InDamping, ForceLimit);
		Hand->Index.SetFingerDriveMode(EAngularDriveMode::TwistAndSwing, InSpring, InDamping, ForceLimit);
		Hand->Middle.SetFingerDriveMode(EAngularDriveMode::TwistAndSwing, InSpring, InDamping, ForceLimit);
		Hand->Ring.SetFingerDriveMode(EAngularDriveMode::TwistAndSwing, InSpring, InDamping, ForceLimit);
		Hand->Pinky.SetFingerDriveMode(EAngularDriveMode::TwistAndSwing, InSpring, InDamping, ForceLimit);
		break;

	case EGraspProcess::SLERP:
		Hand->Thumb.SetFingerDriveMode(EAngularDriveMode::SLERP, InSpring, InDamping, ForceLimit);
		Hand->Index.SetFingerDriveMode(EAngularDriveMode::SLERP, InSpring, InDamping, ForceLimit);
		Hand->Middle.SetFingerDriveMode(EAngularDriveMode::SLERP, InSpring, InDamping, ForceLimit);
		Hand->Ring.SetFingerDriveMode(EAngularDriveMode::SLERP, InSpring, InDamping, ForceLimit);
		Hand->Pinky.SetFingerDriveMode(EAngularDriveMode::SLERP, InSpring, InDamping, ForceLimit);
		break;
	}
}

void Grasp::PrintConstraintForce(const AHand * const Hand)
{
	FVector OutLinearForce;
	FVector OutAngularForce;

	Hand->Thumb.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	UE_LOG(LogTemp, Warning, TEXT("Thumb - Part: DistalConstraint - Force: %s"), *OutAngularForce.ToString());
	Hand->Thumb.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
	UE_LOG(LogTemp, Warning, TEXT("Thumb - Part: Intermetiate - Force: %s"), *OutAngularForce.ToString());
	Hand->Thumb.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	UE_LOG(LogTemp, Warning, TEXT("Thumb - Part: ProximalConstraint - Force: %s"), *OutAngularForce.ToString());

	Hand->Index.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	UE_LOG(LogTemp, Warning, TEXT("Index - Part: DistalConstraint - Force: %s"), *OutAngularForce.ToString());
	Hand->Index.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
	UE_LOG(LogTemp, Warning, TEXT("Index - Part: Intermetiate - Force: %s"), *OutAngularForce.ToString());
	Hand->Index.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
	UE_LOG(LogTemp, Warning, TEXT("Index - Part: ProximalConstraint - Force: %s"), *OutAngularForce.ToString());
	/*
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

void Grasp::ChangeGraspToVelocity()
{

}

FHandOrientation Grasp::LerpHandOrientation(FHandOrientation InitialHandOrientation, FHandOrientation ClosedHandOrientation, float Alpha)
{
	FHandOrientation LerpedHandOrientation;

	LerpedHandOrientation.ThumbOrientation = LerpFingerOrientation(InitialHandOrientation.ThumbOrientation, ClosedHandOrientation.ThumbOrientation, Alpha);
	LerpedHandOrientation.IndexOrientation = LerpFingerOrientation(InitialHandOrientation.IndexOrientation, ClosedHandOrientation.IndexOrientation, Alpha);
	LerpedHandOrientation.MiddleOrientation = LerpFingerOrientation(InitialHandOrientation.MiddleOrientation, ClosedHandOrientation.MiddleOrientation, Alpha);
	LerpedHandOrientation.RingOrientation = LerpFingerOrientation(InitialHandOrientation.RingOrientation, ClosedHandOrientation.RingOrientation, Alpha);
	LerpedHandOrientation.PinkyOrientation = LerpFingerOrientation(InitialHandOrientation.PinkyOrientation, ClosedHandOrientation.PinkyOrientation, Alpha);

	return LerpedHandOrientation;
}

FFingerOrientation Grasp::LerpFingerOrientation(FFingerOrientation InitialFingerOrientation, FFingerOrientation ClosedFingerOrientation, float Alpha)
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
