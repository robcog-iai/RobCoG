// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RSemLogManager.h"
#include "Json.h"


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

	// Init raw data logger
	if (bLogRawData)
	{
		// Path to the json file
		const FString RawFilePath = EpisodePath + "/RawData.json";
		// Init raw data exporter
		RawDataExp = new FRRawDataExporter(DistanceThresholdSquared, GetWorld(),
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
	if (RawDataExp)
	{
		RawDataExp->Update(Timestamp);
	}

	//UE_LOG(LogTemp, Warning, TEXT("tIME:%f"), GetWorld()->GetTimeSeconds());

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