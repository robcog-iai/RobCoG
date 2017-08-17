// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/TriggerBox.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"

#include "GraspingGame.generated.h"


UCLASS()
class ROBCOG_API AGraspingGame : public AActor
{
	GENERATED_BODY()

public:

	// The Paths to be scanned for Items
	UPROPERTY(EditAnywhere)
		TArray<FString> Paths;

	// The Box in which the new Items are spawned
	UPROPERTY(EditAnywhere)
		ATriggerBox* SpawningBox;

	// The Box in which the Items have to be placed
	UPROPERTY(EditAnywhere)
		ATriggerBox* TargetBox;

	// Sets default values for this actor's properties
	AGraspingGame();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	bool bGameRunning;

	FTransform CharacterStartTransform;

	// All found Items to be found
	TArray<FString> Items;

	UStaticMeshComponent* SpawnedMesh;

	//How long, in seconds, the countdown will run
	UPROPERTY(EditAnywhere, Category = "Timer")
		int32 StartTime;
	//How long, in seconds, the countdown will run
	UPROPERTY(EditAnywhere, Category = "Timer")
		int32 GameTime;

	UPROPERTY(EditAnywhere, Category = "Timer")
		UTextRenderComponent* TimerText;

	FTimerHandle StartTimerHandle;
	FTimerHandle GameTimerHandle;

	void UpdateStartTimer();

	void StartTimerHasFinished();

	void UpdateGameTimer();

	void GameTimerHasFinished();

	void GetAllAssetsInFolder(const FString & Directory, TArray<FString> & Assets) const;

	void AbsoluteToGamePath(TArray<FString> & Assets) const;

	void SpawnRandomItem(TArray<FString> & Assets);

	void ResetCharacterTransform() const;

	void ControlGame();

	void StartGame();

	void StopGame();

	void ResetGame();

};
