// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RSemLogManager.h"

// Sets default values
ARSemLogManager::ARSemLogManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Actor hidden in game
	SetActorHiddenInGame(true);

	// Log directory name
	LogRootDirectoryName = "MyLogs";	

	// Default flag values
	bLogRawData = true;

	// Distance squared threshold for logging the raw data
	DistanceThresholdSquared = 0.01;
}

// Called when the game starts or when spawned
void ARSemLogManager::BeginPlay()
{
	Super::BeginPlay();

	// Get platform file
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

	// Level directory path
	const FString LevelPath = LogRootDirectoryName + "/" +	GetWorld()->GetName();

	// Episode directory path
	const FString EpisodePath = LevelPath + "/" + "rcg_" + FDateTime::Now().ToString();

	// Create the directory path
	ARSemLogManager::CreateDirectoryPath(EpisodePath);

	// Set items unique names
	ARSemLogManager::SetUniqueNames();

	// Init raw data logger
	if (bLogRawData)
	{
		// Path to the json file
		const FString RawFilePath = EpisodePath + "/RawData.json";
		// Init raw data exporter
		RawDataExporter = new FRRawDataExporter(DistanceThresholdSquared, GetWorld(),
			MakeShareable(PlatformFile.OpenWrite(*RawFilePath, true, true)));
	}
}

// Called every frame
void ARSemLogManager::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

	// Current time
	const float Timestamp = GetWorld()->GetTimeSeconds();
	
	// Log raw data
	if (RawDataExporter)
	{
		RawDataExporter->Update(Timestamp);
	}

	//UE_LOG(LogTemp, Warning, TEXT("tIME:%f"), GetWorld()->GetTimeSeconds());

}

// Set items unique names
void ARSemLogManager::SetUniqueNames()
{
	UE_LOG(LogTemp, Warning, TEXT("StaticMeshActors unique names:"));
	for (TActorIterator<AStaticMeshActor> StaticMeshActItr(GetWorld()); StaticMeshActItr; ++StaticMeshActItr)
	{
		// Get SkeletalMeshComponent
		UStaticMeshComponent* StaticMeshComp = StaticMeshActItr->GetStaticMeshComponent();
		// Get object tags
		const TArray<FName> Tags = StaticMeshComp->ComponentTags;
		// Skip if object has no tags
		if (Tags.Num() > 0)
		{
			// Get the first tag 
			const FString Tag0 = Tags[0].ToString();

			// check tag type and remove it if correct
			if ((Tag0.Contains("Log")) || (Tag0.Contains("StaticMap")))
			{
				// Generate unique name and make sure there is an underscore before the unique hash
				FString UniqueName = StaticMeshActItr->GetName();
				UniqueName += (UniqueName.Contains("_"))
					? ARSemLogManager::GenerateRandomString(4) 
					: "_" + ARSemLogManager::GenerateRandomString(4);

				// Add actor and its unique name to the map
				SMActorToUniqueName.Add(*StaticMeshActItr, UniqueName);
				UE_LOG(LogTemp, Warning, TEXT("\t %s -> %s"),
					*StaticMeshActItr->GetName(), **SMActorToUniqueName.Find(*StaticMeshActItr));
			}
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("\t  !!! %s has no tags, NO unique names.. "),
				*StaticMeshActItr->GetName());
		}
	}
}

// Create directory path for logging
void ARSemLogManager::CreateDirectoryPath(FString Path)
{
	// Create array of the directory names
	TArray<FString> DirNames;
	Path.ParseIntoArray(DirNames, TEXT("/"), true);

	// Get platform file
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

	// Current path
	FString CurrPath;

	// Create directory path
	for (const auto DirName : DirNames)
	{
		// Append current directory name to the path
		CurrPath.Append(DirName + "/");
		// Create directory if not existent
		if (!PlatformFile.DirectoryExists(*CurrPath))
		{
			PlatformFile.CreateDirectory(*CurrPath);
		}
	}
}

// Create directory path for logging
void ARSemLogManager::CreateLevelMetadata(FString Path)
{
	// Path as string
	FString CurrPath;
	// Append current directory name to the path
	for (const auto DirName : Path)
	{
		CurrPath.Append(DirName + "/");
	}
	CurrPath.Append("meta");

	// Get platform file
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

	// Check if meta file exists
	if (!PlatformFile.FileExists(*CurrPath))
	{
		// Create file, open for write / append / read
		PlatformFile.OpenWrite(*CurrPath, true, true);
	}

	TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);

	JsonObject->SetStringField("SemanticMap", "MyMap");
	JsonObject->SetStringField("EpisodeNr", "0");

}

// Get the episode number by counting the exising folders
uint32 ARSemLogManager::GetEpisodeNumber(FString Path)
{
	// Get platform file
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

	// Path as string
	FString CurrPath;
	// Append current directory name to the path
	for (const auto DirName : Path)
	{
		CurrPath.Append(DirName + "/");
	}
	
	// Episode number by counting the folders
	FLocalTimestampDirectoryVisitor Visitor(PlatformFile, TArray<FString>(), TArray<FString>(), false);
	PlatformFile.IterateDirectory(*CurrPath, Visitor);

	return Visitor.FileTimes.Num();
}

// Generate a random string
FString ARSemLogManager::GenerateRandomString(const int32 Length)
{
	auto RandChar = []() -> char
	{
		const char CharSet[] =
			"0123456789"
			"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
			"abcdefghijklmnopqrstuvwxyz";
		const size_t MaxIndex = (sizeof(CharSet) - 1);
		return CharSet[rand() % MaxIndex];
	};
	std::string RandString(Length, 0);
	std::generate_n(RandString.begin(), Length, RandChar);
	// Return as FString
	return FString(RandString.c_str());
}
