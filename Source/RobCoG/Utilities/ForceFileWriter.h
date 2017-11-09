// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "FileHelper.h"
#include "Enums/GraspType.h"
#include "PlatformFilemanager.h"

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
	FLogInfo() : GraspType(EGraspType::FullGrasp) {}

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
class ROBCOG_API ForceFileWriter
{
public:

	ForceFileWriter();
	~ForceFileWriter();


	bool WriteGraspInfoMapToFile(
		const TMap<FString, FLogInfo> & ItemToGraspInfoMap,
		const FString & Filename,
		bool GraspSucceeded,
		FFileHelper::EEncodingOptions EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());

	void CreateNewForceTableFileAndSaveOld(
		const FString & Path, 
		const FString & Filename,
		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile());

private:

	bool AppendFloatToFile(
		const float Value,
		const FString & Filename,
		FFileHelper::EEncodingOptions EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());

	bool AppendStringToFile(
		const FString & Value,
		const FString & Filename,
		FFileHelper::EEncodingOptions EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());
};