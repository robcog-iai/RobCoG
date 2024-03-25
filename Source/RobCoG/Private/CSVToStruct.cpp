// Copyright 2017-2021, Institute for Artificial Intelligence - University of Bremen


#include "CSVToStruct.h"

/*
      This function reads in the csv file provided from Content/HordeMode/filename and parses it into
      a datatable.  The first column is the rowkey.  Tables aren't indexed by row index in Unreal, but by a key, so they may be in any order.
    */
void UCSVToStruct::LoadCSV(FString FilePath, bool& bOutSuccess, FString& OutInfoMsg, TArray<FString>& OutFileContents)
{
    bOutSuccess = FFileHelper::FFileHelper::LoadFileToStringArray(OutFileContents, *FilePath);
    OutInfoMsg = (!bOutSuccess) ? FString::Printf(TEXT("Error in loading the file - Check reading permissions at '%s'"), *FilePath) : FString::Printf(TEXT("File content '%s' was succesfully loaded to array"), *FilePath);
}