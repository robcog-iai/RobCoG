// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "RTriggerBoxRaster.generated.h"

/**
*  Raster of trigger boxes sending contact information to the semantic events exporter
*/
UCLASS()
class ROBCOG_API ARTriggerBoxRaster : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARTriggerBoxRaster();

	// Destructor
	~ARTriggerBoxRaster();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
private:	
	// Check the number of flex particle collisions
	UFUNCTION()
	void CheckParticleCount();

	// Static mesh actor parent for the trigger
	UPROPERTY(EditAnywhere, Category = "TriggerBox Raster")
	AStaticMeshActor* Parent;

	// Number of rows in the raster
	UPROPERTY(EditAnywhere, Category = "TriggerBox Raster")
	uint32 NrRows;

	// Number of columns in the raster
	UPROPERTY(EditAnywhere, Category = "TriggerBox Raster")
	uint32 NrColumns;

	// Raster visibility
	UPROPERTY(EditAnywhere, Category = "TriggerBox Raster")
	bool bRasterHiddenInGame;

	// Particle collision update frequency
	UPROPERTY(EditAnywhere, Category = "TriggerBox Raster")
	float UpdateFrequency;

	// Timer handle
	FTimerHandle TimerHandle;

	// Array of trigger boxes
	TArray<UBoxComponent*> TriggerBoxes;
};
