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
	bRoundSuccessfulFinished = false;

	CharacterStartTransform = FTransform();

	CurrentItemName = "";
	Paths.Add("Items/Scanned/Meshes");

	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));

	SpawnedMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SpawnedMesh"));
	SpawnedMesh->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);

	TimerText = CreateDefaultSubobject<UTextRenderComponent>(TEXT("CountdownNumber"));
	TimerText->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	TimerText->SetText(FText::FromName("Countdown!"));
	TimerText->SetHorizontalAlignment(EHTA_Center);
	TimerText->SetWorldSize(50.0f);

	StartTime = 3;
}

// Called when the game starts or when spawned
void AGraspingGame::BeginPlay()
{
	UE_LOG(LogTemp, Warning, TEXT("BEGIN"));
	Super::BeginPlay();

	if (TargetBox) {
		TargetBox->OnActorBeginOverlap.AddDynamic(this, &AGraspingGame::ActorOverlaped);
	}

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

		auto SlashPosition = 0;
		auto PointPosition = 0;
		MeshPath.FindLastChar('/', SlashPosition);
		MeshPath.FindLastChar('.', PointPosition);

		CurrentItemName = MeshPath.Mid(SlashPosition + 1, PointPosition - (SlashPosition + 1));
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
	//UE_LOG(LogTemp, Warning, TEXT("ControlGame"));
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
	//UE_LOG(LogTemp, Warning, TEXT("StartGame"));
	TimerText->SetText(FText::AsNumber(FMath::Max(StartTime, 0)));
	bRoundSuccessfulFinished = false;
	GetWorldTimerManager().SetTimer(StartTimerHandle, this, &AGraspingGame::UpdateStartTimer, 1.0f, true);
}

void AGraspingGame::StopGame()
{
}

void AGraspingGame::ResetGame()
{
	//UE_LOG(LogTemp, Warning, TEXT("ResetGame"));
	//ResetCharacterTransform();
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
	//UE_LOG(LogTemp, Warning, TEXT("UpdateStartTimer"));

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
	//UE_LOG(LogTemp, Warning, TEXT("StartTimerHasFinished"));
	//Change to a special readout
	SpawnRandomItem(Items);
	TimerText->SetText(FText::FromString("Round Running"));
	TimerText->SetText(FText::AsNumber(FMath::Max(StartTime, 0)));
	StartTime = 3;
	//GetWorldTimerManager().SetTimer(GameTimerHandle, this, &AGraspingGame::UpdateGameTimer, 0.1f, true);
	bGameRunning = true;
}

void AGraspingGame::ActorOverlaped(AActor* OverlappedActor, AActor* OtherActor)
{
	UE_LOG(LogTemp, Warning, TEXT("ActorOverlapped: %s - %s - %s"), *OtherActor->GetName(), *OverlappedActor->GetName(), *SpawnedMesh->GetName());

	AGraspingGame* GraspingGame = Cast<AGraspingGame>(OtherActor);
	if (GraspingGame) {
		UE_LOG(LogTemp, Warning, TEXT("Cast Okay: CurrentItemNameCast: %s - CurrentItemName: %s"), *GraspingGame->CurrentItemName, *CurrentItemName);

		if(GraspingGame->CurrentItemName.Equals(CurrentItemName))
		{
			bRoundSuccessfulFinished = true;
			//Perform any special actions we want to do when the timer ends.
			RoundFinished();
		}
	}
}

void AGraspingGame::RoundFinished()
{
	UE_LOG(LogTemp, Warning, TEXT("RoundFinished"));
	bGameRunning = false;
	TimerText->SetText(FText::FromName("Round Finished. <br> Press SPACE to reload"));
}