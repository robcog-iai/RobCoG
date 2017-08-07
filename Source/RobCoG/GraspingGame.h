// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/TriggerBox.h"
#include "Components/StaticMeshComponent.h"

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
	UStaticMeshComponent* SpawnedMesh;
	// All found Items to be found
	TArray<FString> Items;

	void GetAssetsInFolder(const FString & Directory, TArray<FString> & Assets);

	void SpawnItem(TArray<FString> & Assets);
};
