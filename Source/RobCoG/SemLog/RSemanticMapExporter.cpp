// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RSemanticMapExporter.h"


// Sets default values
ARSemanticMapExporter::ARSemanticMapExporter()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	// Map folder name
	MapDirName = "SemanticMaps";

	// Map file name
	MapName = "MySemanticMap.owl";

}

// Called when the game starts or when spawned
void ARSemanticMapExporter::BeginPlay()
{
	Super::BeginPlay();

	// Create directory
	ARSemanticMapExporter::CreateMapDir();
	
}

// Create directories for saving the log files
void ARSemanticMapExporter::CreateMapDir()
{
	// Get platform file
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

	// Check if the Map folder exist, if not create it
	if (!PlatformFile.DirectoryExists(*MapDirName))
	{
		// Create directory
		PlatformFile.CreateDirectory(*MapDirName);
	}

	// Raw / event data file path
	FString MapPath = MapDirName + "/" + MapName;

	// Create map file handle for writing
	MapFileHandle = MakeShareable(PlatformFile.OpenWrite(*MapPath, true, true));
}