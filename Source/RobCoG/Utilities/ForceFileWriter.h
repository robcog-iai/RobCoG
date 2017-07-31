// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "FileHelper.h"

/**
 * This class Writes the force to a file
 */
class ROBCOG_API ForceFileWriter
{
private:
	int CurrentValueNumber;
	int NumberOfValues;
	FString AbsoluteFilePath;

public:

	ForceFileWriter();
	~ForceFileWriter();

	void InitializeFile(const FString & AbsoluteFilePath, const int32 NumberOfValuesToBeWritten,
		FFileHelper::EEncodingOptions::Type EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());

	bool AppendFloatToFile(
		const float Value,
		const FString & Filename,
		FFileHelper::EEncodingOptions::Type EncodingOptions = FFileHelper::EEncodingOptions::AutoDetect,
		IFileManager* FileManager = &IFileManager::Get());



};
