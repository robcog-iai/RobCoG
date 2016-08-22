// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "SemLog/RSemEventsExporterSingl.h"
#include "RTriggerBoxRaster.h"


// Sets default values
ARTriggerBoxRaster::ARTriggerBoxRaster()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	// Add scene component as root 
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RasterRootSceneComp"));

	// Number of rows in the raster
	NrRows = 1;

	// Number of columns in the raster
	NrColumns = 1;

	// Raster visibility
	bRasterHiddenInGame = true;

	// Particle collision update frequency
	UpdateFrequency = 1.0f;
}

// Destructor
ARTriggerBoxRaster::~ARTriggerBoxRaster()
{
}

// Called when the game starts or when spawned
void ARTriggerBoxRaster::BeginPlay()
{
	Super::BeginPlay();

	// Check if parent is set
	if (Parent)
	{
		// Get the bounding box of the parent
		const FBox ParentBB = Parent->GetComponentsBoundingBox();
		const FVector ParentExtent = ParentBB.GetExtent();
		const FVector ParentCenter = ParentBB.GetCenter();
		const FVector RasterOrigin = ParentCenter - ParentExtent;

		// Trigger box extent size (radii size)
		const float TriggerBoxExtentX = ParentExtent.X / NrRows;
		const float TriggerBoxExtentY = ParentExtent.Y / NrColumns;

		// Set root component location at the top of parent
		RootComponent->SetWorldLocation(ParentCenter + FVector(0.f, 0.f, ParentExtent.Z));

		// Trigger box initial offset
		FVector TriggerBoxOffset = FVector(TriggerBoxExtentX, TriggerBoxExtentY, 2 * ParentExtent.Z);

		UE_LOG(LogTemp, Warning, 
			TEXT(" ******* \n TriggerBoxExtentX: %f \n, TriggerBoxExtentY %f\n, FIRSTTriggerBoxOffset %s"),
			TriggerBoxExtentX, TriggerBoxExtentY, *TriggerBoxOffset.ToString());

		// Create the trigger box rasters
		for (uint32 i = 1; i <= NrRows; ++i)
		{
			for (uint32 j = 1; j <= NrColumns; ++j)
			{
				// Create trigger box component
				UBoxComponent* CurrTB = NewObject<UBoxComponent>(this, UBoxComponent::StaticClass());

				// Attach to the root component
				CurrTB->SetupAttachment(RootComponent);

				// Set location
				CurrTB->SetWorldLocation(RasterOrigin + TriggerBoxOffset);

				// Set box size (radii size)
				CurrTB->InitBoxExtent(FVector(TriggerBoxExtentX, TriggerBoxExtentY, 0.5));

				// Set visibility
				CurrTB->SetHiddenInGame(bRasterHiddenInGame);

				// Generate overlap events
				CurrTB->bGenerateOverlapEvents = true;

				// Count particle collisions
				CurrTB->bFlexEnableParticleCounter = true;
				//CurrTB->bFlexParticleDrain = true;
				//CurrTB->SetCollisionProfileName("OverlapAll");

				// Register component
				CurrTB->RegisterComponent();

				// Add box to array
				TriggerBoxes.Add(CurrTB);

				// Update offset on the columns (Y axis)
				TriggerBoxOffset.Y += TriggerBoxExtentY * 2;
			}
			// Reset the Y offset
			TriggerBoxOffset.Y = TriggerBoxExtentY;
			// Update offset on the rows (X axis)
			TriggerBoxOffset.X += TriggerBoxExtentX * 2;
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT(" %s's parent is not set!"), *GetName());
	}

	// Set visibility
	SetActorHiddenInGame(bRasterHiddenInGame);

	// Update collision checking with the given frequency
	GetWorldTimerManager().SetTimer(
		TimerHandle, this, &ARTriggerBoxRaster::CheckParticleCount, UpdateFrequency, true);
	
}

// Check particle collisions
void ARTriggerBoxRaster::CheckParticleCount()
{
	for (const auto TriggerBoxItr : TriggerBoxes)
	{
		if (TriggerBoxItr->FlexParticleCount > 0)
		{
			UE_LOG(LogTemp, Warning, TEXT(" **** TriggerBox: %s, FlexParticleCount: %i"),
				*TriggerBoxItr->GetName(), TriggerBoxItr->FlexParticleCount);
		}
	}

	UE_LOG(LogTemp, Warning, TEXT(" **** "));
}
