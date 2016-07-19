// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

/**
 * Base class exporting raw data during gameplay
 */
class ROBCOG_API FRRawDataExporter
{
public:
	// Constructor
	FRRawDataExporter(const float DistThreshSqr, UWorld* World, TSharedPtr<IFileHandle> FileHandle);

	// Destructor
	~FRRawDataExporter();

	// Log next step
	void Update();
	
private:
	// Init items to log from the level
	void InitItemsToLog(UWorld* World);

	// Distance threshold (squared) for raw data logging
	float DistanceThresholdSquared;

	// File handle to append raw data
	TSharedPtr<IFileHandle> RawFileHandle;

	// Array of Skeletal Meshes tuple with prev position and orientation 
	TArray<TTuple<USkeletalMeshComponent*, FVector, FRotator>> SkelMeshComponentsWithPose;

	// Array of Static Mesh Actors tuple with prev position and orientation 
	TArray<TTuple<UStaticMeshComponent*, FVector, FRotator>> StaticMeshComponentsWithPose;
};

