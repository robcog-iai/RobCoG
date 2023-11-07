// Copyright 2017-2021, Institute for Artificial Intelligence - University of Bremen


#include "CSVFileManager.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformFilemanager.h"

bool UCSVFileManager::SaveArrayText(FString SaveDirectory, FString FileName, TArray<FString> SaveText, bool AllowOverWriting = false) {
	
	// Set complete file path.
	SaveDirectory += "\\";
	SaveDirectory += FileName;

	// if overwriting is allowed then overwrite the file each time the game starts.
	FString FinalString = "";
	for (FString& Each : SaveText) {
		FinalString += Each;
		FinalString += LINE_TERMINATOR;
	}

	return FFileHelper::SaveStringToFile(FinalString, *SaveDirectory);
}