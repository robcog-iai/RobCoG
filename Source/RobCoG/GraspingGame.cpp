// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "GraspingGame.h"
#include "Paths.h"
#include "FileManagerGeneric.h"


// Sets default values
AGraspingGame::AGraspingGame()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	Paths.Add("Items/Scanned/Meshes");
}

// Called when the game starts or when spawned
void AGraspingGame::BeginPlay()
{
	Super::BeginPlay();
	
	FString AbsolutePath = "";
	auto ContentFolder = FPaths::GameContentDir();

	for (auto && Path : Paths)
	{
		AbsolutePath = ContentFolder + Path;
		UE_LOG(LogTemp, Warning, TEXT("AbsolutePath: %s"), *AbsolutePath);
		GetAssetsInFolder(AbsolutePath, Items);
	}	
}

// Called every frame
void AGraspingGame::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AGraspingGame::GetAssetsInFolder(const FString & Directory, TArray<FString> & Assets)
{
	Assets.Empty();
	if (FPaths::DirectoryExists(Directory))
	{
		auto Path = Directory + "/*.uasset";
		UE_LOG(LogTemp, Warning, TEXT("Path: %s"), *Path);
		FFileManagerGeneric::Get().FindFiles(Assets, *Path, true, false);
		for (auto & Asset : Assets)
		{
			Asset = Directory + Asset;
		}
	}
}

void AGraspingGame::SpawnItem(TArray<FString> & Assets)
{

	//SpawnedMesh->SetStaticMesh();
	//SpawnedMesh->
}