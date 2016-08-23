// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "SemLog/RSemEventsExporterSingl.h"
#include "RSemanticContactTriggerBox.h"

// Set default values
ARSemanticContactTriggerBox::ARSemanticContactTriggerBox()
{
	// Create raster
	bCreateRaster = false;

	// Number of rows in the raster
	RasterNrRows = 1;

	// Number of columns in the raster
	RasterNrColumns = 1;

	// Raster visibility
	bRasterHiddenInGame = true;

	// Particle collision update frequency
	RasterUpdateRate = 1.0f;
}

// Destructor
ARSemanticContactTriggerBox::~ARSemanticContactTriggerBox()
{
}

// Called when the games starts or when spawned
void ARSemanticContactTriggerBox::BeginPlay()
{
	Super::BeginPlay();

	// Check if parent is set
	if (Parent)
	{
		// Bind overlap events
		OnActorBeginOverlap.AddDynamic(this, &ARSemanticContactTriggerBox::BeginSemanticContact);
		OnActorEndOverlap.AddDynamic(this, &ARSemanticContactTriggerBox::EndSemanticContact);

		// Create raster if required
		if (bCreateRaster)
		{
			ARSemanticContactTriggerBox::CreateRaster();
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT(" %s's parent is not set!"), *GetName());
	}
}

// Callback on start overlap, end the semantic contact
void ARSemanticContactTriggerBox::BeginSemanticContact(
	AActor* Self, AActor* OtherActor)
{
	if (FRSemEventsExporterSingl::Get().IsInit())
	{
		FRSemEventsExporterSingl::Get().BeginTouchingEvent(
			Parent, OtherActor, GetWorld()->GetTimeSeconds());
	}
}

// Callback on end overlap, end the semantic contact
void ARSemanticContactTriggerBox::EndSemanticContact(
	AActor* Self, AActor* OtherActor)
{
	if (FRSemEventsExporterSingl::Get().IsInit())
	{
		FRSemEventsExporterSingl::Get().EndTouchingEvent(
			Parent, OtherActor, GetWorld()->GetTimeSeconds());
	}
}

// Create raster
void ARSemanticContactTriggerBox::CreateRaster()
{
	// Get the bounding box of the parent
	const FBox ParentBB = Parent->GetComponentsBoundingBox();
	const FVector ParentExtent = ParentBB.GetExtent();
	const FVector ParentCenter = ParentBB.GetCenter();
	const FVector RasterOrigin = ParentCenter - ParentExtent;

	// Trigger box extent size (radii size)
	const float TriggerBoxExtentX = ParentExtent.X / RasterNrRows;
	const float TriggerBoxExtentY = ParentExtent.Y / RasterNrColumns;

	// Set root component location at the top of parent
	RootComponent->SetWorldLocation(ParentCenter + FVector(0.f, 0.f, ParentExtent.Z));

	// Trigger box initial offset
	FVector TriggerBoxOffset = FVector(TriggerBoxExtentX, TriggerBoxExtentY, 2 * ParentExtent.Z);

	// Create the trigger box rasters
	for (uint32 i = 1; i <= RasterNrRows; ++i)
	{
		for (uint32 j = 1; j <= RasterNrColumns; ++j)
		{

			const FString TBName = GetName() + "_M_" + FString::FromInt(i) + "_" + FString::FromInt(j);

			// Create trigger box component
			UBoxComponent* CurrTB = NewObject<UBoxComponent>(this, *TBName);
			//UBoxComponent::StaticClass());

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
			CurrTB->SetCollisionProfileName("Trigger");

			// Register component
			CurrTB->RegisterComponent();

			// Add box to array
			RasterTriggerBoxes.Add(CurrTB);

			// Update offset on the columns (Y axis)
			TriggerBoxOffset.Y += TriggerBoxExtentY * 2;
		}
		// Reset the Y offset
		TriggerBoxOffset.Y = TriggerBoxExtentY;
		// Update offset on the rows (X axis)
		TriggerBoxOffset.X += TriggerBoxExtentX * 2;
	}
	// Set visibility
	SetActorHiddenInGame(bRasterHiddenInGame);

	// Update collision checking with the given frequency
	GetWorldTimerManager().SetTimer(
		TimerHandle, this, &ARSemanticContactTriggerBox::CheckParticleCount, RasterUpdateRate, true);
}

// Check particle collisions
void ARSemanticContactTriggerBox::CheckParticleCount()
{
	//UE_LOG(LogTemp, Warning, TEXT(" **** < ---  "));
	//for (const auto TriggerBoxItr : RasterTriggerBoxes)
	//{
	//	if (TriggerBoxItr->FlexParticleCount > 0)
	//	{
	//		UE_LOG(LogTemp, Warning, TEXT(" **** TriggerBox: %s, FlexParticleCount: %i"),
	//			*TriggerBoxItr->GetName(), TriggerBoxItr->FlexParticleCount);
	//	}

	//}
	//UE_LOG(LogTemp, Warning, TEXT(" --- > **** "));
}