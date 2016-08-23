// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Engine/TriggerBox.h"
#include "RSemanticContactTriggerBox.generated.h"

/**
 *  Trigger box sending contact information to the semantic events exporter
 */
UCLASS()
class ROBCOG_API ARSemanticContactTriggerBox : public ATriggerBox
{
	GENERATED_BODY()

	// Constructor
	ARSemanticContactTriggerBox();
	
	// Destructor
	~ARSemanticContactTriggerBox();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	// Callback on begin overlap
	UFUNCTION()
	void BeginSemanticContact(AActor* Self, AActor* OtherActor);

	// Callback on end overlap
	UFUNCTION()
	void EndSemanticContact(AActor* Self, AActor* OtherActor);

	// Create raster
	void CreateRaster();

	// Check particle cound on raster
	void CheckParticleCount();

	// Static mesh actor parent for the trigger
	UPROPERTY(EditAnywhere, Category = "Semantic TriggerBox")
	AStaticMeshActor* Parent;

	// Create raster for particle collisions
	UPROPERTY(EditAnywhere, Category = "Semantic TriggerBox")
	bool bCreateRaster;

	// Number of rows in the raster
	UPROPERTY(EditAnywhere, Category = "Semantic TriggerBox")
	uint32 RasterNrRows;

	// Number of columns in the raster
	UPROPERTY(EditAnywhere, Category = "Semantic TriggerBox")
	uint32 RasterNrColumns;

	// Particle collision update rate
	UPROPERTY(EditAnywhere, Category = "Semantic TriggerBox")
	float RasterUpdateRate;

	// Raster visibility
	UPROPERTY(EditAnywhere, Category = "Semantic TriggerBox")
	bool bRasterHiddenInGame;

	// Array of trigger boxes from the raster
	TArray<UBoxComponent*> RasterTriggerBoxes;

	// Timer handle
	FTimerHandle TimerHandle;
};
