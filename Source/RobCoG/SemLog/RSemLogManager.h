// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "Animation/SkeletalMeshActor.h"
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
	// Create directory path for logging
	void CreateDirectoryPath(FString Path);

	// Set items to be loggeed (from tags)
	void SetLogItems();

	// Set unique names of items
	void GenerateUniqueNames();

	// Read unique names from file
	bool ReadUniqueNames(const FString Path);

	// Write generated unique names to file
	void WriteUniqueNames(const FString Path);

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

	// Map of skeletal component (to be logged) names to actor map 
	TMap<FString, ASkeletalMeshActor*> SkelActNameToCompPtrMap;

	// Map of dynamic actors (to be logged) names to actor map 
	TMap<FString, AStaticMeshActor*> DynamicActNameToActPtrMap;

	// Map of static actors (to be logged) names to actor map 
	TMap<FString, AStaticMeshActor*> StaticActNameToActPtrMap;

	// Map of skeletal component (to be logged) to unique name
	TMap<ASkeletalMeshActor*, FString> SkelActPtrToUniqNameMap;

	// Map of dynamic actors (to be logged) to unique name
	TMap<AStaticMeshActor*, FString> DynamicActPtrToUniqNameMap;

	// Map of static map actors (to be logged) to unique name
	TMap<AStaticMeshActor*, FString> StaticActPtrToUniqNameMap;	
	
	// Raw data exporter
	FRRawDataExporter* RawDataExporter;

};
