// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "GraspLogger.h"
#include "TimerManager.h"
#include "Paths.h"


// Sets default values
AGraspLogger::AGraspLogger() :
	GraspingGame(nullptr),
	Hand(nullptr),
	LastGraspStatus(EGraspStatus::Stopped)
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	ForceFileWriterPtr = MakeShareable(new ForceFileWriter());
}

// Called when the game starts or when spawned
void AGraspLogger::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AGraspLogger::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Hand || !GraspingGame)
		return;

	if (LastGraspStatus == Hand->GraspPtr->GraspStatus || GraspingGame->CurrentItemName != "")
	{
		return;
	}
	else if (LastGraspStatus == EGraspStatus::Stopped && Hand->GraspPtr->GraspStatus == EGraspStatus::Orientation)
	{
		UE_LOG(LogTemp, Warning, TEXT("StartLogging"));
		ClearCurrentGraspInfo();

		CurrentItemName = GraspingGame->CurrentItemName;

		UE_LOG(LogTemp, Warning, TEXT("CurrentItemName: %s"), *CurrentItemName);

		GetWorldTimerManager().SetTimer(TimerHandle, this, &AGraspLogger::UpdateTimer, 0.05f, true);
	}
	else if ((LastGraspStatus == EGraspStatus::Velocity || LastGraspStatus == EGraspStatus::Orientation)
		&& Hand->GraspPtr->GraspStatus == EGraspStatus::Stopped)
	{
		GetWorldTimerManager().ClearTimer(TimerHandle);

		// TODO: Use the Map...
		SaveValues();
		if (ForceFileWriterPtr.IsValid())
		{
			UE_LOG(LogTemp, Warning, TEXT("Logged to File"));
			ForceFileWriterPtr->WriteGraspInfoMapToFile(ItemToGraspInfoMap, FPaths::GameSavedDir() + "Force.csv", GraspingGame->bRoundSuccessfulFinished);
		}
		ItemToGraspInfoMap.Empty();
		ClearCurrentGraspInfo();

		UE_LOG(LogTemp, Warning, TEXT("StopLogging"));
	}
	else
	{
		GetWorldTimerManager().ClearTimer(TimerHandle);
		UE_LOG(LogTemp, Warning, TEXT("AbortLogging"));
	}

	LastGraspStatus = Hand->GraspPtr->GraspStatus;
}

void AGraspLogger::UpdateTimer()
{
	FVector OutLinearForce;
	FVector OutAngularForce;

	if (Hand->GraspPtr->GraspStatus == EGraspStatus::Orientation)
	{
		Hand->Thumb.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.ThumbDistal.Add(OutAngularForce.Size());

		Hand->Thumb.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.ThumbIntermediate.Add(OutAngularForce.Size());

		Hand->Thumb.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.ThumbProximal.Add(OutAngularForce.Size());

		Hand->Index.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.IndexDistal.Add(OutAngularForce.Size());

		Hand->Index.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.IndexIntermediate.Add(OutAngularForce.Size());

		Hand->Index.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.IndexProximal.Add(OutAngularForce.Size());

		Hand->Middle.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.MiddleDistal.Add(OutAngularForce.Size());

		Hand->Middle.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.MiddleIntermediate.Add(OutAngularForce.Size());

		Hand->Middle.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.MiddleProximal.Add(OutAngularForce.Size());

		Hand->Ring.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.RingDistal.Add(OutAngularForce.Size());

		Hand->Ring.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.RingIntermediate.Add(OutAngularForce.Size());

		Hand->Ring.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.RingProximal.Add(OutAngularForce.Size());

		Hand->Pinky.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.PinkyDistal.Add(OutAngularForce.Size());

		Hand->Pinky.FingerPartToConstraint[EFingerPart::Intermediate]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.PinkyIntermediate.Add(OutAngularForce.Size());

		Hand->Pinky.FingerPartToConstraint[EFingerPart::Proximal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationHandForces.PinkyProximal.Add(OutAngularForce.Size());


	}
	else if (Hand->GraspPtr->GraspStatus == EGraspStatus::Velocity)
	{
		//TODO: Log Velocity
	}
}

void AGraspLogger::ClearCurrentGraspInfo()
{
	CurrentItemName = "";
	CurrentLogInfo.Clear();
}

void AGraspLogger::SaveValues()
{
	ItemToGraspInfoMap.Add(CurrentItemName, CurrentLogInfo);
}