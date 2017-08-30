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

	if (LastGraspStatus == Hand->GraspPtr->GraspStatus)
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
	else if (LastGraspStatus == EGraspStatus::Orientation && Hand->GraspPtr->GraspStatus == EGraspStatus::Velocity)
	{
	}
	else if (LastGraspStatus == EGraspStatus::Velocity && Hand->GraspPtr->GraspStatus == EGraspStatus::Stopped)
	{
		
		GetWorldTimerManager().ClearTimer(TimerHandle);
		if (GraspingGame->bRoundSuccessfulFinished)
		{
			// TODO: Use the Map...
			SaveValues();
			if (ForceFileWriterPtr.IsValid()) 
			{
				UE_LOG(LogTemp, Warning, TEXT("Logged to File"));
				ForceFileWriterPtr->WriteGraspInfoMapToFile(ItemToGraspInfoMap, FPaths::GameSavedDir() + "Force.csv");
			}
			ItemToGraspInfoMap.Empty();
			ClearCurrentGraspInfo();
		}
		UE_LOG(LogTemp, Warning, TEXT("StopLogging"));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("AbortLogging"));
		GetWorldTimerManager().ClearTimer(TimerHandle);
	}

	LastGraspStatus = Hand->GraspPtr->GraspStatus;
}

void AGraspLogger::UpdateTimer()
{
	FVector OutLinearForce;
	FVector OutAngularForce;

	//Logs only Thumb - Distal
	if (Hand->GraspPtr->GraspStatus == EGraspStatus::Orientation)
	{
		Hand->Thumb.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.OrientationGrasp.Add(OutAngularForce.Size());
		UE_LOG(LogTemp, Warning, TEXT("Orientation - Force: %f"),OutAngularForce.Size());
	}
	else if (Hand->GraspPtr->GraspStatus == EGraspStatus::Velocity)
	{

		Hand->Thumb.FingerPartToConstraint[EFingerPart::Distal]->GetConstraintForce(OutLinearForce, OutAngularForce);
		CurrentLogInfo.VelocityGrasp.Add(OutAngularForce.Size());

		UE_LOG(LogTemp, Warning, TEXT("Velocity - Force: %f"), OutAngularForce.Size());
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