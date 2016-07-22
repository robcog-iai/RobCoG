// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Animation/SkeletalMeshActor.h"

/**
 * Base class exporting raw data during gameplay
 */
class ROBCOG_API FRRawDataExporter
{
public:
	// Constructor
	FRRawDataExporter(const float DistThreshSqr,
		TSharedPtr<IFileHandle> FileHandle,
		TMap<ASkeletalMeshActor*, FString> SkelActPtrToUniqNameMap,
		TMap<AStaticMeshActor*, FString> DynamicActPtrToUniqNameMap,
		TMap<AStaticMeshActor*, FString> StaticActPtrToUniqNameMap);

	// Destructor
	~FRRawDataExporter();

	// Log step
	void Update(const float Timestamp);

	// Structure of skeletal mesh comp with its previous pose
	struct FRSkelLogRawStruct
	{
		FRSkelLogRawStruct(ASkeletalMeshActor* SkMComp, const FString UniqName) :
			SkelMeshComp(SkMComp),
			UniqueName(UniqName),
			PrevLoc(FVector(0.0f)),
			PrevRot(FRotator(0.0f)) {};
		ASkeletalMeshActor* SkelMeshComp;
		FString UniqueName;
		FVector PrevLoc;
		FRotator PrevRot;
	};

	// Structure of dyamic actors with prev pose and unique name
	struct FRDynActLogRawStruct
	{
		FRDynActLogRawStruct(AStaticMeshActor* StMAct, const FString UniqName) : 
			StaticMeshAct(StMAct),
			UniqueName(UniqName),
			PrevLoc(FVector(0.0f)),
			PrevRot(FRotator(0.0f)) {};
		AStaticMeshActor* StaticMeshAct;
		FString UniqueName;
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
	void InitItemsToLog(TMap<ASkeletalMeshActor*, FString>& SkelActPtrToUniqNameMap,
		TMap<AStaticMeshActor*, FString>& DynamicActPtrToUniqNameMap,
		TMap<AStaticMeshActor*, FString>& StaticActPtrToUniqNameMap);

	// Distance threshold (squared) for raw data logging
	float DistanceThresholdSquared;

	// File handle to append raw data
	TSharedPtr<IFileHandle> RawFileHandle;

	// Array of skeletal meshes with prev position and orientation
	TArray<FRSkelLogRawStruct> SkelActStructArr;

	// Array of static meshes with prev position and orientation
	TArray<FRDynActLogRawStruct> DynamicActStructArr;

	// Map of static map actors  to unique name
	TMap<AStaticMeshActor*, FString> StaticActToUniqName;
};

