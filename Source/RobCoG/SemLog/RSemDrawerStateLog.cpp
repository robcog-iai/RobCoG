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
		// Cast to static mesh actor
		AStaticMeshActor* SMAct = Cast<AStaticMeshActor>(ConstrItr->ConstraintActor2);
		
		if (SMAct)
		{
			// Copy of the constraint instance
			const FConstraintInstance& CurrConstr = ConstrItr->ConstraintInstance;

			if (CurrConstr.LinearXMotion == ELinearConstraintMotion::LCM_Limited)
			{
				// Add the drawer and it's initial position to the map
				DrawerToInitLocMap.Add(SMAct, SMAct->GetActorLocation());
			}
			else if (CurrConstr.AngularSwing1Motion == EAngularConstraintMotion::ACM_Limited)
			{
				DoorToInitLocMap.Add(SMAct, CurrConstr.GetCurrentSwing1());
			}
			else if (CurrConstr.AngularSwing2Motion == EAngularConstraintMotion::ACM_Limited)
			{
				DoorToInitLocMap.Add(SMAct, CurrConstr.GetCurrentSwing2());
			}

			// Get the static mesh component of the drawer
			UStaticMeshComponent* SMComp = SMAct->GetStaticMeshComponent();
			
			// Add impule to static mesh in order to close the drawer/door
			SMAct->GetStaticMeshComponent()->AddImpulse(FVector(-6000) * SMAct->GetActorForwardVector());
		}
	}
}

// Check drawer states
void ARSemDrawerStateLog::CheckDrawerStates()
{
	UE_LOG(LogTemp, Warning, TEXT(" !! ------- !! --------"));
	for (const auto ConstrItr : Constraints)
	{
		AActor* Drawer = ConstrItr->ConstraintActor2;

		if (ConstrItr->ConstraintInstance.LinearXMotion == ELinearConstraintMotion::LCM_Limited)
		{
			const FVector CurrPos = Drawer->GetActorLocation();
			const FVector InitPos = *DrawerToInitLocMap.Find(Drawer);

			const float Dist = (CurrPos.X - InitPos.X) * Drawer->GetActorForwardVector().X;

			if (Dist < -20)
			{
				//UE_LOG(LogTemp, Warning, TEXT(" Update: %s : Closed"), *Drawer->GetName());
			}
			else if (Dist < 0 && Dist > -20)
			{
				//UE_LOG(LogTemp, Warning, TEXT(" Update: %s : HalfClosed"), *Drawer->GetName());
			}
			else if (Dist > 20)
			{
				//UE_LOG(LogTemp, Warning, TEXT(" Update: %s : Opened"), *Drawer->GetName());
			}
			else if (Dist > 0 && Dist < 20)
			{
				//UE_LOG(LogTemp, Warning, TEXT(" Update: %s : HalfOpened"), *Drawer->GetName());
			}
		}
		else if(ConstrItr->ConstraintInstance.AngularSwing1Motion == EAngularConstraintMotion::ACM_Limited)
		{
			const float InitPos = *DoorToInitLocMap.Find(Drawer);

			UE_LOG(LogTemp, Warning, TEXT(" AngularSwing1Motion: %s %f"), 
				*Drawer->GetName(), ConstrItr->ConstraintInstance.GetCurrentSwing1());
			
		}
		else if (ConstrItr->ConstraintInstance.AngularSwing2Motion == EAngularConstraintMotion::ACM_Limited)
		{
			const float InitPos = *DoorToInitLocMap.Find(Drawer);

			UE_LOG(LogTemp, Warning, TEXT(" AngularSwing2Motion: %s %f"),
				*Drawer->GetName(), ConstrItr->ConstraintInstance.GetCurrentSwing2());
		}
	}
}