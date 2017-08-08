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

	SpawnedMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SpawnedMesh"), this);
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
		GetAssetsInFolder(AbsolutePath, Items);
	}
	NormalizePaths(Items);

	SpawnItem(Items);
}

// Called every frame
void AGraspingGame::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AGraspingGame::GetAssetsInFolder(const FString & Directory, TArray<FString> & Assets)
{
	UE_LOG(LogTemp, Warning, TEXT("Directory: %s"), *Directory);
	if (FPaths::DirectoryExists(Directory))
	{
		auto Path = Directory + "/*.uasset";
		FFileManagerGeneric::Get().FindFiles(Assets, *Path, true, false);
		for (auto & Asset : Assets)
		{
			Asset = Directory + "/" + Asset;
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Found Assets: %i"), Assets.Num());
}

void AGraspingGame::NormalizePaths(TArray<FString> & Assets)
{
	FString Left = "";
	for (auto & Asset : Assets)
	{
		Left = Asset.Left(Asset.Find("Content") + 7);
		Asset = Asset.Replace(*Left, *FString("/Game"));
	}
}

void AGraspingGame::SpawnItem(TArray<FString> & Assets)
{
	if (Assets.Num() <= 0) return;

	int32 LastSlashIndex = 0;
	int32 LastPointIndex = 0;
	int32 RandomIndex = FMath::RandRange(0, Assets.Num() - 1);
	FString AbsoluteMeshPath = Assets[RandomIndex];

	AbsoluteMeshPath.FindLastChar('/', LastSlashIndex);
	AbsoluteMeshPath.FindLastChar('.', LastPointIndex);


	UE_LOG(LogTemp, Warning, TEXT("AbsoluteMeshPath: %s"), *AbsoluteMeshPath);

	FString MeshName = AbsoluteMeshPath.Mid(LastSlashIndex + 1, LastPointIndex - (LastSlashIndex + 1));


	UE_LOG(LogTemp, Warning, TEXT("MeshName: %s"), *MeshName);

	FString PathName = "StaticMesh'" + AbsoluteMeshPath.Left(LastSlashIndex) + "/" + MeshName + "." + MeshName + "'";

	UE_LOG(LogTemp, Warning, TEXT("PathName: %s"), *PathName);

	UStaticMesh* Mesh = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), nullptr, *PathName));

	if (Mesh) {
		SpawnedMesh->SetStaticMesh(Mesh);
		SpawnedMesh->SetEnableGravity(true);
		SpawnedMesh->SetSimulatePhysics(true);
	}
	if (SpawningBox) SpawnedMesh->SetWorldLocation(SpawningBox->GetActorLocation());

}