// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "RRawDataExporter.h"
#include <string>
#include <algorithm>
#include "RSemLogManager.generated.h"

UCLASS()
class ROBCOG_API ARSemLogManager : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARSemLogManager();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

private:
	// Set unique names of items
	void SetUniqueNames();

	// Create directory path for logging
	void CreateDirectoryPath(FString Path);

	// Create level meta data
	void CreateLevelMetadata(FString Path);

	// Calculates the episode number by counting the existing episode folders
	uint32 GetEpisodeNumber(FString Path);

	// Generate random string
	FString GenerateRandomString(const int32 Length);

	// Directory to save the logs
	UPROPERTY(EditAnywhere, Category = "Semantic logger")
	FString LogRootDirectoryName;

	// Raw data log flag
	UPROPERTY(EditAnywhere, Category = "Raw Data")
	bool bLogRawData;

	// Distance threshold (squared) for raw data logging
	UPROPERTY(EditAnywhere, Category = "Raw Data")
	float DistanceThresholdSquared;
	
	// Map static mesh actor to unique name
	TMap<AStaticMeshActor*, FString> SMActorToUniqueName;

	// Raw data exporter
	FRRawDataExporter* RawDataExporter;

};
