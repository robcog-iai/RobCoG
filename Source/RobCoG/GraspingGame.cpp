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

	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"),this);

	SpawnedMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SpawnedMesh"), this);
	SpawnedMesh->AttachToComponent(RootComponent,FAttachmentTransformRules::KeepRelativeTransform);

	CountdownText = CreateDefaultSubobject<UTextRenderComponent>(TEXT("CountdownNumber"));
	CountdownText->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	CountdownText->SetHorizontalAlignment(EHTA_Center);
	CountdownText->SetWorldSize(150.0f);

	CountdownTime = 3;
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
		GetAllAssetsInFolder(AbsolutePath, Items);
	}

}

// Called every frame
void AGraspingGame::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AGraspingGame::GetAllAssetsInFolder(const FString & Directory, TArray<FString> & Assets) const
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
	AbsoluteToGamePath(Assets);
}

void AGraspingGame::AbsoluteToGamePath(TArray<FString> & Assets) const
{
	if (Assets.Num() <= 0) return;

	FString MeshName = "";
	FString Left = "";

	auto LastSlashIndex = 0;
	auto LastPointIndex = 0;

	for (auto & Asset : Assets)
	{
		// Path to "/Game"
		Left = Asset.Left(Asset.Find("Content") + 7);
		Asset = Asset.Replace(*Left, *FString("/Game"));

		// Change "*.uasset" filename to "name.name"
		Asset.FindLastChar('/', LastSlashIndex);
		Asset.FindLastChar('.', LastPointIndex);

		MeshName = Asset.Mid(LastSlashIndex + 1, LastPointIndex - (LastSlashIndex + 1));

		Asset = "StaticMesh'" + Asset.Left(LastSlashIndex) + "/" + MeshName + "." + MeshName + "'";
		UE_LOG(LogTemp, Warning, TEXT("PathName: %s"), *Asset);
	}
}

void AGraspingGame::SpawnRandomItem(TArray<FString> & Assets)
{
	if (Assets.Num() <= 0) return;

	int32 RandomIndex = FMath::RandRange(0, Assets.Num() - 1);
	FString MeshPath = Assets[RandomIndex];


	UStaticMesh* Mesh = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), nullptr, *MeshPath));

	if (Mesh) {
		SpawnedMesh->SetStaticMesh(Mesh);
		SpawnedMesh->SetEnableGravity(true);
		SpawnedMesh->SetSimulatePhysics(true);
	}
	if (SpawningBox) SpawnedMesh->SetWorldLocation(SpawningBox->GetActorLocation());
}

void AGraspingGame::ResetGame()
{
	UpdateTimerDisplay();
	GetWorldTimerManager().SetTimer(CountdownTimerHandle, this, &AGraspingGame::AdvanceTimer, 1.0f, true);
}

void AGraspingGame::UpdateTimerDisplay()
{
	CountdownText->SetText(FText::AsNumber(FMath::Max(CountdownTime, 0)));
}

void AGraspingGame::AdvanceTimer()
{
	--CountdownTime;
	UpdateTimerDisplay();
	if (CountdownTime < 1)
	{
		// We're done counting down, so stop running the timer.
		GetWorldTimerManager().ClearTimer(CountdownTimerHandle);
		//Perform any special actions we want to do when the timer ends.
		CountdownHasFinished();
	}
}

void AGraspingGame::CountdownHasFinished()
{
	//Change to a special readout
	CountdownText->SetText(FText::FromName("GO!"));
}