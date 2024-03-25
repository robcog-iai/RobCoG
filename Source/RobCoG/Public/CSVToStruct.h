// Copyright 2017-2021, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Engine/DataTable.h"
#include "CSVToStruct.generated.h"

/**
 * 
 */
UCLASS()
class ROBCOG_API UCSVToStruct : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = ImportCSVToTrialStruct)
	static void LoadCSV(FString FilePath, bool& bOutSuccess, FString& OutInfoMsg, TArray<FString>& OutFileContents);
};
