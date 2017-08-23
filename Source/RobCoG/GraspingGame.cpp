// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "GraspingGame.h"
#include "Paths.h"
#include "FileManagerGeneric.h"
#include "TimerManager.h"
#include "Engine/World.h"
#include "Components/InputComponent.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Character.h"
#include "Engine/StaticMesh.h"


// Sets default values
AGraspingGame::AGraspingGame()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	bGameRunning = false;
	CharacterStartTransform = FTransform();

	Paths.Add("Items/Scanned/Meshes");

	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"), this);

	SpawnedMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SpawnedMesh"), this);
	SpawnedMesh->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);

	TimerText = CreateDefaultSubobject<UTextRenderComponent>(TEXT("CountdownNumber"));
	TimerText->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	TimerText->SetText(FText::FromName("Countdown!"));
	TimerText->SetHorizontalAlignment(EHTA_Center);
	TimerText->SetWorldSize(50.0f);

	StartTime = 3;
	GameTime = 30;
}

// Called when the game starts or when spawned
void AGraspingGame::BeginPlay()
{
	Super::BeginPlay();

	APlayerController* PlayerController = GetWorld()->GetFirstPlayerController();
	if (PlayerController)
	{
		EnableInput(PlayerController);
		InputComponent->BindAction("GameControl", IE_Pressed, this, &AGraspingGame::ControlGame);
		CharacterStartTransform = PlayerController->GetActorTransform();
	}

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

void AGraspingGame::GetAllAssetsInFolder(const FString & Directory, TArray<FString> & Assets)
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

void AGraspingGame::AbsoluteToGamePath(TArray<FString> & Assets)
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
		SpawnedMesh->bGenerateOverlapEvents = true;
	}
	if (SpawningBox)
	{
		SpawnedMesh->SetWorldRotation(FRotator(0, 0, 0), false, nullptr, ETeleportType::TeleportPhysics);
		SpawnedMesh->SetWorldLocation(SpawningBox->GetActorLocation(), false, nullptr, ETeleportType::TeleportPhysics);

	}
}

void AGraspingGame::ControlGame()
{
	UE_LOG(LogTemp, Warning, TEXT("ControlGame"));
	if (bGameRunning)
	{
		ResetGame();
	}
	else
	{
		StartGame();
		bGameRunning = true;
	}
}

void AGraspingGame::StartGame()
{
	UE_LOG(LogTemp, Warning, TEXT("StartGame"));
	TimerText->SetText(FText::AsNumber(FMath::Max(StartTime, 0)));
	GetWorldTimerManager().SetTimer(StartTimerHandle, this, &AGraspingGame::UpdateStartTimer, 1.0f, true);
}

void AGraspingGame::StopGame()
{
}

void AGraspingGame::ResetGame()
{
	UE_LOG(LogTemp, Warning, TEXT("ResetGame"));
	ResetCharacterTransform();
	StartGame();
}

void AGraspingGame::ResetCharacterTransform()
{
	UE_LOG(LogTemp, Warning, TEXT("ResetCharacterTransform"));
	APlayerController* PlayerController = GetWorld()->GetFirstPlayerController();
	if (PlayerController)
	{
		UE_LOG(LogTemp, Warning, TEXT("StartTrans: %s | CurrentTrans: %s"), *CharacterStartTransform.ToString(), *PlayerController->GetActorTransform().ToString());
		PlayerController->GetCharacter()->SetActorTransform(CharacterStartTransform, false, nullptr, ETeleportType::TeleportPhysics);
	}
}

void AGraspingGame::UpdateStartTimer()
{
	UE_LOG(LogTemp, Warning, TEXT("UpdateStartTimer"));

	--StartTime;
	TimerText->SetText(FText::AsNumber(FMath::Max(StartTime, 0)));
	if (StartTime < 1)
	{
		// We're done counting down, so stop running the timer.
		GetWorldTimerManager().ClearTimer(StartTimerHandle);
		//Perform any special actions we want to do when the timer ends.
		StartTimerHasFinished();
	}
}

void AGraspingGame::StartTimerHasFinished()
{
	UE_LOG(LogTemp, Warning, TEXT("StartTimerHasFinished"));
	//Change to a special readout
	SpawnRandomItem(Items);
	TimerText->SetText(FText::FromName("GO!"));
	TimerText->SetText(FText::AsNumber(FMath::Max(StartTime, 0)));
	GetWorldTimerManager().SetTimer(GameTimerHandle, this, &AGraspingGame::UpdateGameTimer, 1.0f, true);
	StartTime = 3;
}

void AGraspingGame::UpdateGameTimer()
{
	UE_LOG(LogTemp, Warning, TEXT("UpdateGameTimer"));
	TArray<UPrimitiveComponent*> Components;
	TargetBox->GetOverlappingComponents(Components);
	for (auto Component : Components)
	{
		UE_LOG(LogTemp, Warning, TEXT("Component: %s"), *Component->GetName());

	}
	bool bRoundFinished = Components.Contains(SpawnedMesh);

	--GameTime;
	TimerText->SetText(FText::AsNumber(FMath::Max(GameTime, 0)));

	if (GameTime < 1 || bRoundFinished)
	{
		// We're done counting down, so stop running the timer.
		GetWorldTimerManager().ClearTimer(GameTimerHandle);
		//Perform any special actions we want to do when the timer ends.
		GameTimerHasFinished();
	}
}

void AGraspingGame::GameTimerHasFinished()
{
	UE_LOG(LogTemp, Warning, TEXT("GameTimerHasFinished"));
	//Change to a special readout
	TimerText->SetText(FText::FromName("Round Finished. Press SPACE to reload"));
	GameTime = 30;
}