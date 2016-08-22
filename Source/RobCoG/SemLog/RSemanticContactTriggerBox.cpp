// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "SemLog/RSemEventsExporterSingl.h"
#include "RSemanticContactTriggerBox.h"

// Set default values
ARSemanticContactTriggerBox::ARSemanticContactTriggerBox()
{
	GetCollisionComponent()->bFlexEnableParticleCounter = true;


	BC = CreateDefaultSubobject<UBoxComponent>(TEXT("BoxComponent"));
	BC->SetupAttachment(RootComponent);
	BC->InitBoxExtent(FVector(100, 100, 1));

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
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT(" %s's parent is not set!"), *GetName());
	}

	GetWorldTimerManager().SetTimer(TimerHandle, this, &ARSemanticContactTriggerBox::CheckParticleCount, 1.0f, true);

	//GetWorldTimerManager().SetTimer(FTimerDelegate::CreateUObject(this, &ARSemanticContactTriggerBox::CheckParticleCount), 5.f, false);

	BC->bFlexEnableParticleCounter = true;
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

// Check the number of flex particle collisions
void ARSemanticContactTriggerBox::CheckParticleCount()
{
	UE_LOG(LogTemp, Warning, TEXT(" **** FlexParticleCount: %i BC: %i"), GetCollisionComponent()->FlexParticleCount, BC->FlexParticleCount);
}