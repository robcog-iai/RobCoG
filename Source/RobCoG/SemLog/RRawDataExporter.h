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

	// Log step
	void Update(const float Timestamp);

	// Structure of skeletal mesh with its previous pose
	struct FRSkelMeshWPrevPose
	{
		FRSkelMeshWPrevPose(USkeletalMeshComponent* SkMComp)
			: SkelMeshComp(SkMComp), PrevLoc(FVector(0.0f)), PrevRot(FRotator(0.0f)) {};
		USkeletalMeshComponent* SkelMeshComp;
		FVector PrevLoc;
		FRotator PrevRot;
	};

	// Structure of static mesh with its previous pose
	struct FRStaticMeshActWPrevPose
	{
		FRStaticMeshActWPrevPose(AStaticMeshActor* StMAct)
			: StaticMeshAct(StMAct), PrevLoc(FVector(0.0f)), PrevRot(FRotator(0.0f)) {};
		AStaticMeshActor* StaticMeshAct;
		FVector PrevLoc;
		FRotator PrevRot;
	};
	
private:
	// Create Json object with a 3d location
	TSharedPtr<FJsonObject> CreateLocationJsonObject(const FVector Location);

	// Create Json object with a 3d rotation as quaternion 
	TSharedPtr<FJsonObject> CreateRotationJsonObject(const FQuat Rotation);

	// Create Json object with name location and rotation
	TSharedPtr<FJsonObject> CreateNameLocRotJsonObject(const FString Name, const FVector Location, const FQuat Rotation);

	// Init items to log from the level
	void InitItemsToLog(UWorld* World);

	// Distance threshold (squared) for raw data logging
	float DistanceThresholdSquared;

	// File handle to append raw data
	TSharedPtr<IFileHandle> RawFileHandle;

	// Array of skeletal meshes with prev position and orientation
	TArray<FRSkelMeshWPrevPose> SkelMeshComponentsWithPrevPose;

	// Array of static meshes with prev position and orientation
	TArray<FRStaticMeshActWPrevPose> StaticMeshActorsWithPrevPose;
};

