// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "SemLog/RSemEventsExporterSingl.h"
#include "RSemDrawerStateLog.h"


// Sets default values
ARSemDrawerStateLog::ARSemDrawerStateLog()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	// Set default update rate
	UpdateRate = 1.0f;
}

// Called when the game starts or when spawned
void ARSemDrawerStateLog::BeginPlay()
{
	Super::BeginPlay();

	// Get the drawer/door constraints
	for (TActorIterator<APhysicsConstraintActor> ActorItr(GetWorld()); ActorItr; ++ActorItr)
	{
		Constraints.Add(ActorItr->GetConstraintComp());
	}

	// Close all drawers
	ARSemDrawerStateLog::CloseDrawers();

	// Check drawer states
	GetWorldTimerManager().SetTimer(
		TimerHandle, this, &ARSemDrawerStateLog::CheckDrawerStates, UpdateRate, true);
}


// Close drawers
void ARSemDrawerStateLog::CloseDrawers()
{
	for (const auto ConstrItr : Constraints)
	{
		AStaticMeshActor* SMAct = Cast<AStaticMeshActor>(ConstrItr->ConstraintActor2);
		
		if (SMAct)
		{
			UE_LOG(LogTemp, Warning, TEXT(" Closing: %s %s"),
				*SMAct->GetName(), 
				*SMAct->GetActorLocation().ToString());
			UStaticMeshComponent* SMComp = SMAct->GetStaticMeshComponent();
			
			// Add impule to static mesh in order to close the drawer/door
			SMAct->GetStaticMeshComponent()->AddImpulse(FVector(-6000) * SMComp->GetForwardVector());
		}
	}
}

// Check drawer states
void ARSemDrawerStateLog::CheckDrawerStates()
{
	UE_LOG(LogTemp, Warning, TEXT(" !! ------- !! --------"));
	for (const auto ConstrItr : Constraints)
	{
		if (ConstrItr->ConstraintInstance.LinearXMotion == ELinearConstraintMotion::LCM_Free)
		{
			UE_LOG(LogTemp, Warning, TEXT(" Update: %s %s %s"),
				*ConstrItr->ConstraintActor2->GetName(),
				*ConstrItr->ConstraintActor2->GetActorLocation().ToString(),
				*ConstrItr->ConstraintActor2->GetActorForwardVector().ToString());
		}
		else
		{

		}

		//UE_LOG(LogTemp, Warning, TEXT(" !! ------- !! -------- Actor name : %s, LinPosTarget: %s"),
		//	*ConstrItr->ConstraintActor2->GetName(),
		//	*ConstrItr->ConstraintActor2->GetActorLocation().ToString());
	}
}