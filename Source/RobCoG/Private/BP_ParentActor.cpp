// Copyright 2017-2021, Institute for Artificial Intelligence - University of Bremen


#include "BP_ParentActor.h"

#include "Misc/FileHelper.h"
#include "HAL/PlatformFilemanager.h"

// Sets default values
ABP_ParentActor::ABP_ParentActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ABP_ParentActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ABP_ParentActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void ABP_ParentActor::EndPlay(const EEndPlayReason::Type reason) {
	Super::EndPlay(reason);

	// Set complete file path.
	FString SaveDirectory = FPaths::ProjectDir();
	SaveDirectory += "\\";
	SaveDirectory += TEXT("SavedData.csv");

	// if overwriting is allowed then overwrite the file each time the game starts.
	FString FinalString = TEXT("SourceContainer,DestinationContainer,PouringTime,RotationSpeed,MinimumAngle,MaximumAngle,PosDiffX,PosDiffY,PosDiffZ,IsPouringFailed");
	FinalString += LINE_TERMINATOR;

	FString FinalOutPutStr = "";

	// Get only first and last raw from DataRaws variable, since we are only concern with first and last partcile colliding
	int TotalDataRaws = DataRaws.Num();
	FString PourinFailedStr = isPouringFailed ? TEXT("True") : TEXT("False");
	if (TotalDataRaws > 0) {
		// Keep in mind that the DataRaws do not have Pouring Faied Status in them so add it here to do correct calculation of that array
		FString FirstDataRaw = DataRaws[0] + TEXT(",") + PourinFailedStr;
		FString LastDataRaw = DataRaws[TotalDataRaws - 1] + TEXT(",") + PourinFailedStr;
		
		// TODO: Remove it in final version add all 3 lines first raw and last raw and modified raw into csv file
		/*FinalString += FirstDataRaw ;
		FinalString += LINE_TERMINATOR;

		FinalString += LastDataRaw;
		FinalString += LINE_TERMINATOR;*/

		TArray<FString> FirstRawArrayData, LastRawArrayData;
		FString MajorString1, MajorString2;
		
		// separate all comma separated values from first raw, 
		// keep in mind that the last element will be ignored(i.e. IsPouringFailed is not in an array)
		while (FirstDataRaw.Split(TEXT(","), &MajorString1, &FirstDataRaw)) {
			FirstRawArrayData.Add(MajorString1);
		}

		// separate all comma separated values from last raw, 
		// keep in mind that the last element will be ignored(i.e. IsPouringFailed is not in an array)
		while (LastDataRaw.Split(TEXT(","), &MajorString2, &LastDataRaw)) {
			LastRawArrayData.Add(MajorString2);
		}

		// this is basically time stamps from first and last raws and getting how long it took to pour.
		double PouringTime = 0.0;
		FString MinimumPouringAngle, MaximumPouringAngle;
		// check if all data are correctly logged from the blueprint
		if(LastRawArrayData.Num() > 5 && FirstRawArrayData.Num() > 5) {
			PouringTime = FCString::Atod(*LastRawArrayData[2]) - FCString::Atod(*FirstRawArrayData[2]);
			MinimumPouringAngle = FirstRawArrayData[4];
			MaximumPouringAngle = LastRawArrayData[4];
		}
		
		for (int i = 0; i < FirstRawArrayData.Num(); i++) {
			// if the value of array is time (i == 2) or angle value (i == 4) then use calculated values
			if(i == 2) {
				FinalOutPutStr += FString::SanitizeFloat(PouringTime) + TEXT(",");
			}
			else if (i == 4) {
				FinalOutPutStr += MinimumPouringAngle + TEXT(",");
				FinalOutPutStr += MaximumPouringAngle + TEXT(",");
			}
			else {
				FinalOutPutStr += FirstRawArrayData[i] + TEXT(",");
			}
		}
		// DO NOT forget to add pouring failed status to finaloutput str
		FinalOutPutStr += PourinFailedStr;
		FinalOutPutStr += LINE_TERMINATOR;
		FinalString += FinalOutPutStr;
	}

	//Checking file exists
	if (FPaths::FileExists(*SaveDirectory))
	{
		UE_LOG(LogTemp, Warning, TEXT("it exists"));
		TArray<FString> FileData, FinalData;
		if (FFileHelper::LoadFileToStringArray(FileData, *SaveDirectory))
		{
			
			for (FString& FileDataRaw : FileData) {
				if (!FileDataRaw.IsEmpty()) {
					FinalData.Add(FileDataRaw);
				}
			}
			FinalData.Add(FinalOutPutStr);
			FFileHelper::SaveStringArrayToFile(FinalData, *SaveDirectory);
		}
		else//File could not be found?
		{
			UE_LOG(LogTemp, Warning, TEXT("No file"));
		}
	}
	else {
		FFileHelper::SaveStringToFile(FinalString, *SaveDirectory);
	}
	
}

