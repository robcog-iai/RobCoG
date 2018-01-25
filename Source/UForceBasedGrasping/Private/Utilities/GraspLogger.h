// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GraspingGame.h"
#include "Hand.h"
#include "Hand/Grasp.h"
#include "ForceFileWriter.h"

#include "GraspLogger.generated.h"

UCLASS()
class UFORCEBASEDGRASPING_API AGraspLogger : public AActor
{
GENERATED_BODY()

public:
	// The filename of the file to be written
	const FString ForceTableFilename;

	// The Hand to be logged
	UPROPERTY(EditAnywhere)
	AHand* Hand;

	// Sets default values for this actor's properties
	AGraspLogger();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	// Should the Timer Update
	bool bUpdateTimer;

	// Is Logging Enabled
	bool bLoggingEnabled;

	// A counter to log just ever n-th tick
	int LoggingCounter;

	// The pointer to the file writer
	TSharedPtr<ForceFileWriter> ForceFileWriterPtr;

	// The current log info
	FLogInfo CurrentLogInfo;

	// The start countdown timer
	FTimerHandle TimerHandle;

	//The Last GraspType of the Hand
	EGraspStatus LastGraspStatus;

	// Updates the Log info
	void UpdateTimer();

	// Clears the current Grasp Info Map
	void ClearCurrentGraspInfo();

	// Toggles the logging
	void ToggleHandLogging();

};
