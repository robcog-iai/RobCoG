// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "Grasp.h"
#include "Hand.h"
#include "Engine/Engine.h"

Grasp::Grasp()
{
	HandOrientationParserPtr = MakeShareable(new HandOrientationParser());
	CurrentGraspType = EGraspType::FullGrasp;

	HandOrientationParserPtr->WriteIni();
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

void Grasp::DriveToHandOrientation(const FHandOrientation & HandOrientation,const AHand * const Hand)
{
	DriveToFingerOrientation(HandOrientation.ThumbOrientation, Hand->Thumb);
	DriveToFingerOrientation(HandOrientation.IndexOrientation, Hand->Index);
	DriveToFingerOrientation(HandOrientation.MiddleOrientation, Hand->Middle);
	DriveToFingerOrientation(HandOrientation.RingOrientation, Hand->Ring);
	DriveToFingerOrientation(HandOrientation.PinkyOrientation, Hand->Pinky);
}

void Grasp::DriveToFingerOrientation(const FFingerOrientation & FingerOrientation, const FFinger & Finger)
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

	/* Not Implemented
	Constraint = Finger.FingerPartToConstraint[EFingerPart::Metacarpal];
	if (Constraint)
		Constraint->SetAngularOrientationTarget(FingerOrientation.MetacarpalOrientation.Orientation.Quaternion());
	*/
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

void Grasp::DriveToInitialOrientation(const AHand * const Hand)
{
	DriveToHandOrientation(InitialHandOrientation, Hand);
}

void Grasp::UpdateGrasp(const float Alpha,const AHand * const Hand)
{
	UE_LOG(LogTemp, Warning, TEXT("Alpha: %f"), Alpha);

	DriveToHandOrientation(LerpHandOrientation(InitialHandOrientation, ClosedHandOrientation, Alpha), Hand);

}

void Grasp::SwitchGrasp(const AHand * const Hand)
{
	if (HandOrientationParserPtr.IsValid())
	{
		if (CurrentGraspType == EGraspType::FullGrasp)
		{
			CurrentGraspType = EGraspType::PinchGrasp;
			if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 1.5, FColor::White, "GraspType changed to: PinchGrasp");

			SetInitialHandOrientation(HandOrientationParserPtr->GetInitialHandOrientationForGraspType(EGraspType::PinchGrasp));
			SetClosedHandOrientation(HandOrientationParserPtr->GetClosedHandOrientationForGraspType(EGraspType::PinchGrasp));

			DriveToInitialOrientation(Hand);
		}
		else if (CurrentGraspType == EGraspType::PinchGrasp)
		{
			CurrentGraspType = EGraspType::PinchThreeGrasp;
			if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 1.5, FColor::White, "GraspType changed to: PinchThreeGrasp");

			SetInitialHandOrientation(HandOrientationParserPtr->GetInitialHandOrientationForGraspType(EGraspType::PinchThreeGrasp));
			SetClosedHandOrientation(HandOrientationParserPtr->GetClosedHandOrientationForGraspType(EGraspType::PinchThreeGrasp));

			DriveToInitialOrientation(Hand);
		}
		else if (CurrentGraspType == EGraspType::PinchThreeGrasp)
		{
			CurrentGraspType = EGraspType::FullGrasp;
			if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 1.5, FColor::White, "GraspType changed to: FullGrasp");

			
			SetInitialHandOrientation(HandOrientationParserPtr->GetInitialHandOrientationForGraspType(EGraspType::FullGrasp));
			SetClosedHandOrientation(HandOrientationParserPtr->GetClosedHandOrientationForGraspType(EGraspType::FullGrasp));

			DriveToInitialOrientation(Hand);
		}
	}
}