// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "ForceFileWriter.h"

ForceFileWriter::ForceFileWriter()
{
	CurrentValueNumber = 0;
	NumberOfValues = 300;
	AbsoluteFilePath = "";
}

ForceFileWriter::~ForceFileWriter()
{
	
}

void ForceFileWriter::InitializeFile(
	const FString & AbsoluteFilePath, 
	const int32 NumberOfValuesToBeWritten, 
	FFileHelper::EEncodingOptions::Type EncodingOptions,
	IFileManager* FileManager)
{
	this->NumberOfValues = NumberOfValuesToBeWritten;
	this->AbsoluteFilePath = AbsoluteFilePath;

	UE_LOG(LogTemp, Warning, TEXT("AbsoluteFilePath: %s"), *this->AbsoluteFilePath);
	FileManager->Delete(*this->AbsoluteFilePath);

	for (int i = 0; i < NumberOfValues; i++)
		FFileHelper::SaveStringToFile(";Tick " + FString::FromInt(i), *this->AbsoluteFilePath, EncodingOptions, FileManager, FILEWRITE_Append);

}

bool ForceFileWriter::AppendFloatToFile(
	const float Value, 
	const FString & Filename, 
	FFileHelper::EEncodingOptions::Type EncodingOptions,
	IFileManager* FileManager)
{
	if (CurrentValueNumber >= NumberOfValues)
		return false;
	 
	if(CurrentValueNumber == 0 )
		FFileHelper::SaveStringToFile("\nForce:", *AbsoluteFilePath, EncodingOptions, FileManager, FILEWRITE_Append);

	FFileHelper::SaveStringToFile(";" + FString::SanitizeFloat(Value), *Filename, EncodingOptions, FileManager, FILEWRITE_Append);

	//UE_LOG(LogTemp, Warning, TEXT("Float: %f | Sanitized: %s"), Value, *FString::SanitizeFloat(Value));
	CurrentValueNumber += 1;
	return false; 
}
