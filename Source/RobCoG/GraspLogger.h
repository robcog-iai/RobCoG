// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GraspingGame.h"
#include "Hand.h"

#include "GraspLogger.generated.h"


struct FLogInfo
{
	// Default constructor
	FLogInfo() : GraspType(EGraspType::FullGrasp) {}

	EGraspType GraspType;
	TArray<float> OrientationGrasp;
	TArray<float> VelocityGrasp;

	FORCEINLINE void Clear()
	{
		OrientationGrasp.Empty();
		VelocityGrasp.Empty();
	}
};


UCLASS()
class ROBCOG_API AGraspLogger : public AActor
{
	GENERATED_BODY()

public:

	// The grasping game
	UPROPERTY(EditAnywhere)
		AGraspingGame* GraspingGame;

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
	FString CurrentItemName;
	FLogInfo CurrentLogInfo;
	// Saves the Itemname and the chosen 
	TMap<FString, FLogInfo> ItemToGraspInfoMap;

	// The start countdown timer
	FTimerHandle TimerHandle;

	//The Last GraspType of the Hand
	EGraspStatus LastGraspStatus;

	void UpdateTimer();

	void ClearGraspInfo();
	void SaveValues();

};
