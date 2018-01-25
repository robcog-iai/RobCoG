// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "EngineMinimal.h"
#include "FileHelper.h"
#include "Enums/GraspType.h"
#include "PlatformFilemanager.h"

// A struct wich saves hand forces or torques to be written into a file
struct FHandForces
{
	FHandForces(){}

	TArray<float> ThumbDistal;
	TArray<float> ThumbIntermediate;
	TArray<float> ThumbProximal;

	TArray<float> IndexDistal;
	TArray<float> IndexIntermediate;
	TArray<float> IndexProximal;

	TArray<float> MiddleDistal;
	TArray<float> MiddleIntermediate;
	TArray<float> MiddleProximal;

	TArray<float> RingDistal;
	TArray<float> RingIntermediate;
	TArray<float> RingProximal;

	TArray<float> PinkyDistal;
	TArray<float> PinkyIntermediate;
	TArray<float> PinkyProximal;
	
	FORCEINLINE void Clear()
	{
		ThumbDistal.Empty();
		ThumbIntermediate.Empty();
		ThumbProximal.Empty();

		IndexDistal.Empty();
		IndexIntermediate.Empty();
		IndexProximal.Empty();

		MiddleDistal.Empty();
		MiddleIntermediate.Empty();
		MiddleProximal.Empty();

		RingDistal.Empty();
		RingIntermediate.Empty();
		RingProximal.Empty();

		PinkyDistal.Empty();
		PinkyIntermediate.Empty();
		PinkyProximal.Empty();
	}
};

struct FLogInfo
{
	// Default constructor
	FLogInfo() : GraspType(EGraspType::LargeDiameter) {}

	EGraspType GraspType;
	FHandForces OrientationHandForces;

	FORCEINLINE void Clear()
	{
		OrientationHandForces.Clear();
	}
};

/**
 * This class Writes the force to a file
 */
class UFORCEBASEDGRASPING_API ForceFileWriter
{
public:
	// Constructor
	ForceFileWriter();

	//Destructor
	~ForceFileWriter();

	// To write an FLog info struct into a file 
	bool WriteGraspInfoMapToFile(
		const FLogInfo & ItemToGraspInfoMap,
		const FString & Filename,
		FFileHelper::EEncodingOptions EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());

	// To backup old file with the same name with timestamp
	void CreateNewForceTableFileAndSaveOld(
		const FString & Path, 
		const FString & Filename,
		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile());

private:

	// Appends a single float to a file
	bool AppendFloatToFile(
		const float Value,
		const FString & Filename,
		FFileHelper::EEncodingOptions EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());

	//Appends a string to a file
	bool AppendStringToFile(
		const FString & Value,
		const FString & Filename,
		FFileHelper::EEncodingOptions EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());
};
