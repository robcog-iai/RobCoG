// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "GraspLogger.h"
#include "TimerManager.h"
#include "Paths.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"

// Sets default values
AGraspLogger::AGraspLogger() :
	Hand(nullptr),
	LastGraspStatus(EGraspStatus::Stopped),
	ForceTableFilename("Force.csv"),
	LoggingCounter(1),
	bLoggingEnabled(false),
	bUpdateTimer(false)
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	ForceFileWriterPtr = MakeShareable(new ForceFileWriter());
}

// Called when the game starts or when spawned
void AGraspLogger::BeginPlay()
{
	Super::BeginPlay();

	UE_LOG(LogTemp, Warning, TEXT("Try to Bind Logging Toggler"));

	APlayerController* PlayerController = UGameplayStatics::GetPlayerController(GetWorld(), 0);
	if (PlayerController)
	{
		if (PlayerController->InputComponent)
		{

			UE_LOG(LogTemp, Warning, TEXT("Bind Logging Toggler"));
			PlayerController->InputComponent->BindAction("ToggleHandLogging", IE_Pressed, this, &AGraspLogger::ToggleHandLogging);
		}

	}

	if (ForceFileWriterPtr.IsValid())
	{
		ForceFileWriterPtr->CreateNewForceTableFileAndSaveOld(FPaths::ProjectSavedDir(), ForceTableFilename);
	}
}

// Called every frame
void AGraspLogger::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Hand || !bLoggingEnabled)
		return;

	if (LoggingCounter != 5) {
		LoggingCounter++;
		return;
	}
	else
	{
		LoggingCounter = 0;
	}

	if (LastGraspStatus != Hand->GraspPtr->GraspStatus)
	{
		if (Hand->GraspPtr->GraspStatus == EGraspStatus::Stopped) {
			UE_LOG(LogTemp, Warning, TEXT("GraspStatus: Stopped"));
		}
		else if (Hand->GraspPtr->GraspStatus == EGraspStatus::Orientation) {
			UE_LOG(LogTemp, Warning, TEXT("GraspStatus: Orientation"));
		}


		if (LastGraspStatus == EGraspStatus::Stopped && Hand->GraspPtr->GraspStatus == EGraspStatus::Orientation)
		{
			UE_LOG(LogTemp, Warning, TEXT("StartLogging"));
			ClearCurrentGraspInfo();

			bUpdateTimer = true;
		}
		else if (LastGraspStatus == EGraspStatus::Orientation && Hand->GraspPtr->GraspStatus == EGraspStatus::Stopped)
		{
			bUpdateTimer = false;

			if (ForceFileWriterPtr.IsValid())
			{
				if (ForceFileWriterPtr->WriteGraspInfoMapToFile(CurrentLogInfo, FPaths::ProjectSavedDir() + ForceTableFilename))
				{
					UE_LOG(LogTemp, Warning, TEXT("Logged to File"));
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("Logging to File Failed!"));
				}
			}
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
	if (!Hand)
		return;

	FVector OutLinearForce;
	FVector OutAngularForce;

	CurrentLogInfo.GraspType = Hand->GraspPtr->CurrentGraspType;

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
	CurrentLogInfo.Clear();
}

void AGraspLogger::ToggleHandLogging()
{
	if (bLoggingEnabled == true)
	{
		bLoggingEnabled = false;
	}
	else
	{
		bLoggingEnabled = true;
	}
	UE_LOG(LogTemp, Warning, TEXT("ToggleHandLogging: %s"), (bLoggingEnabled ? "true" : "false"));
}