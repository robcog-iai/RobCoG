// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "FileHelper.h"
#include "Enums/GraspType.h"

struct FLogInfo
{
	// Default constructor
	FLogInfo() : GraspType(EGraspType::FullGrasp) {}

	EGraspType GraspType;
	TArray<float> OrientationGrasp;
	TArray<float> VelocityGrasp;

	FORCEINLINE void Clear()
	{
		OrientationGrasp.Empty();
		VelocityGrasp.Empty();
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

	bool AppendFloatToFile(
		const float Value,
		const FString & Filename,
		FFileHelper::EEncodingOptions::Type EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());

	bool AppendStringToFile(
		const FString & Value,
		const FString & Filename,
		FFileHelper::EEncodingOptions::Type EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());

	bool WriteGraspInfoMapToFile(const TMap<FString, FLogInfo> & ItemToGraspInfoMap,
		const FString & Filename,
		FFileHelper::EEncodingOptions::Type EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());

};
