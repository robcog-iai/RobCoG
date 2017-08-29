// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "ForceFileWriter.h"
#include "UObjectGlobals.h"

ForceFileWriter::ForceFileWriter()
{
}

ForceFileWriter::~ForceFileWriter()
{
	
}


bool ForceFileWriter::AppendFloatToFile(
	const float Value,
	const FString & Filename,
	FFileHelper::EEncodingOptions::Type EncodingOptions,
	IFileManager* FileManager)
{
	return FFileHelper::SaveStringToFile(";" + FString::SanitizeFloat(Value), *Filename, EncodingOptions, FileManager, FILEWRITE_Append);
}

bool ForceFileWriter::AppendStringToFile(
	const FString & Value,
	const FString & Filename,
	FFileHelper::EEncodingOptions::Type EncodingOptions,
	IFileManager* FileManager)
{
	return FFileHelper::SaveStringToFile(Value, *Filename, EncodingOptions, FileManager, FILEWRITE_Append);
}

bool ForceFileWriter::WriteGraspInfoMapToFile(const TMap<FString, FLogInfo> & ItemToGraspInfoMap,
	const FString & Filename,
	FFileHelper::EEncodingOptions::Type EncodingOptions,
	IFileManager* FileManager)
{
	bool Success = true;
	for(auto Entry : ItemToGraspInfoMap)
	{
		FString ItemName = Entry.Key;
		FLogInfo LogInfo = Entry.Value;

		FString GraspType;
		UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EGraspType"), true);
		
		if (EnumPtr)
			GraspType = EnumPtr->GetDisplayNameTextByIndex(static_cast<int64>(LogInfo.GraspType)).ToString();
		else
			GraspType = "Error";

		Success = Success && AppendStringToFile("\nItemName:;" + ItemName, Filename);
		Success = Success && AppendStringToFile("\nGraspType:;" + GraspType, Filename);
		Success = Success && AppendStringToFile("\nOrientation:", Filename);
		for(auto OrientationValue : LogInfo.OrientationGrasp)
		{
			Success = Success && AppendFloatToFile(OrientationValue, Filename);
		}
		Success = Success && AppendStringToFile("\nVelocity:", Filename);
		for (auto VelocityValue : LogInfo.VelocityGrasp)
		{
			Success = Success && AppendFloatToFile(VelocityValue, Filename);
		}
		Success = Success && AppendStringToFile("\n\n", Filename);

	}
	return Success;
}