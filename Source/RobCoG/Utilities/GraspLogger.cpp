// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "GraspLogger.h"
#include "TimerManager.h"
#include "Paths.h"


// Sets default values
AGraspLogger::AGraspLogger() :
	GraspingGame(nullptr),
	Hand(nullptr),
	LastGraspStatus(EGraspStatus::Stopped),
	bUpdateTimer(false),
	ForceTableFilename("Force.csv")
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	ForceFileWriterPtr = MakeShareable(new ForceFileWriter());
}

// Called when the game starts or when spawned
void AGraspLogger::BeginPlay()
{
	Super::BeginPlay();

	if(ForceFileWriterPtr.IsValid())
	{
		ForceFileWriterPtr->CreateNewForceTableFileAndSaveOld(FPaths::GameSavedDir(), ForceTableFilename);
	}
}

// Called every frame
void AGraspLogger::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Hand || !GraspingGame)
		return;

	if (LastGraspStatus != Hand->GraspPtr->GraspStatus && GraspingGame->CurrentItemName != "")
	{
		if (LastGraspStatus == EGraspStatus::Stopped && Hand->GraspPtr->GraspStatus == EGraspStatus::Orientation &&  GraspingGame->CurrentItemName != "")
		{
			UE_LOG(LogTemp, Warning, TEXT("StartLogging"));
			ClearCurrentGraspInfo();

			CurrentItemName = GraspingGame->CurrentItemName;

			UE_LOG(LogTemp, Warning, TEXT("CurrentItemName: %s"), *CurrentItemName);

			bUpdateTimer = true;
		}
		else if (LastGraspStatus == EGraspStatus::Orientation && Hand->GraspPtr->GraspStatus == EGraspStatus::Stopped)
		{
			bUpdateTimer = false;

			// TODO: Use Map
			SaveValues();
			if (ForceFileWriterPtr.IsValid())
			{
				if (ForceFileWriterPtr->WriteGraspInfoMapToFile(ItemToGraspInfoMap, FPaths::GameSavedDir() + ForceTableFilename, GraspingGame->bRoundSuccessfulFinished))
				{
					UE_LOG(LogTemp, Warning, TEXT("Logged to File"));
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("Logging to File Failed!"));
				}
			}
			ItemToGraspInfoMap.Empty();
			ClearCurrentGraspInfo();

			UE_LOG(LogTemp, Warning, TEXT("StopLogging"));
		}
		else
		{
			bUpdateTimer = false;
			UE_LOG(LogTemp, Warning, TEXT("AbortLogging"));
		}
	
		LastGraspStatus = Hand->GraspPtr->GraspStatus;
	}

	if (bUpdateTimer)
	{
		UpdateTimer();
	}
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