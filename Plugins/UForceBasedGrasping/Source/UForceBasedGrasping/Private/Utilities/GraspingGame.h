// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/TriggerBox.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"

#include "GraspingGame.generated.h"



UCLASS()
class UFORCEBASEDGRASPING_API AGraspingGame : public AActor
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

	// The Name of the currently spawned item
	FString CurrentItemName;

	// Is the round successful finished
	bool bRoundSuccessfulFinished;

	// Sets default values for this actor's properties
	AGraspingGame();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	// Is a Game Running
	bool bGameRunning;

	
	// The Start transform of the Character
	FTransform CharacterStartTransform;

	// All found Items to be found
	TArray<FString> Items;

	// The Spawned Mesh to be carried
	UStaticMeshComponent* SpawnedMesh;

	// How long, in seconds, the countdown will run
	UPROPERTY(EditAnywhere, Category = "Timer")
		int32 StartTime;

	// The Text which is updated by the start timer
	UPROPERTY(EditAnywhere, Category = "Timer")
		UTextRenderComponent* TimerText;
	
	// The start countdown timer
	FTimerHandle StartTimerHandle;

	// The game timer to log the 
	FTimerHandle GameTimerHandle;

	// Updates the start timer
	void UpdateStartTimer();

	// Called when the start timer has finished
	void StartTimerHasFinished();

	// If an Actor Overlaps
	UFUNCTION()
		void ActorOverlaped(AActor* OverlappedActor, AActor* OtherActor);

	// Called when the game tmer has finished
	void RoundFinished();

	// Writes all assetnames into a list
	void GetAllAssetsInFolder(const FString & Directory, TArray<FString> & Assets);

	// Absolute paths to ue4 valid loading paths
	void AbsoluteToGamePath(TArray<FString> & Assets);

	// Spawns a random item
	void SpawnRandomItem(TArray<FString> & Assets);

	// Reset the character to the base position
	void ResetCharacterTransform();

	// This method is called on the input
	void ControlGame();

	// Starts the game functionality
	void StartGame();

	// Stops the game functionality
	void StopGame();

	// Resets the game
	void ResetGame();

};
