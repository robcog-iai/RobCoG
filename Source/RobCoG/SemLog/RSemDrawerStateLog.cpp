// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "SemLog/RSemEventsExporterSingl.h"
#include "RSemDrawerStateLog.h"


// Sets default values
ARSemDrawerStateLog::ARSemDrawerStateLog()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	// Set default update rate (sec)
	UpdateRate = 0.25f;
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

	// Check drawer states with the given update rate
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
				// Add the drawer and its initial position to the map
				DrawerToInitLocMap.Add(SMAct, SMAct->GetActorLocation());
			}
			else if (CurrConstr.AngularSwing1Motion == EAngularConstraintMotion::ACM_Limited)
			{
				// Compute the min and max value of the joint
				TPair<float, float> MinMax;
				MinMax.Key = CurrConstr.GetCurrentSwing1();
				MinMax.Value = MinMax.Key + FMath::DegreesToRadians(CurrConstr.Swing1LimitAngle + CurrConstr.AngularRotationOffset.Yaw);
				// Add the doors minmax pos
				DoorToMinMaxMap.Add(SMAct, MinMax);
			}
			else if (CurrConstr.AngularSwing2Motion == EAngularConstraintMotion::ACM_Limited)
			{
				// Compute the min and max value of the joint
				TPair<float, float> MinMax;
				MinMax.Key = CurrConstr.GetCurrentSwing2();
				MinMax.Value = MinMax.Key - FMath::DegreesToRadians(CurrConstr.Swing2LimitAngle + CurrConstr.AngularRotationOffset.Pitch);
				// Add the doors minmax pos
				DoorToMinMaxMap.Add(SMAct, MinMax);
			}

			// Get the static mesh component of the actor
			UStaticMeshComponent* SMComp = SMAct->GetStaticMeshComponent();
			
			// Add impule to static mesh in order to close the drawer/door
			SMAct->GetStaticMeshComponent()->AddImpulse(FVector(-6000) * SMAct->GetActorForwardVector());
		}
	}
}

// Check drawer states
void ARSemDrawerStateLog::CheckDrawerStates()
{
	// Iterate all constraints
	for (const auto ConstrItr : Constraints)
	{
		// Get actor2
		AActor* FurnitureAct = ConstrItr->ConstraintActor2;

		// Check if it's type drawer or door (all drawers have linear X motion)
		if (ConstrItr->ConstraintInstance.LinearXMotion == ELinearConstraintMotion::LCM_Limited)
		{
			const FVector CurrPos = FurnitureAct->GetActorLocation();
			const FVector InitPos = *DrawerToInitLocMap.Find(FurnitureAct);
			const float Dist = (CurrPos.X - InitPos.X) * FurnitureAct->GetActorForwardVector().X;

			if (Dist < -20.0f)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "Closed");
			}
			else if (Dist < 0.0f && Dist > -20.0f)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "HalfClosed");
			}
			else if (Dist > 20.0f)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "Opened");
			}
			else if (Dist > 0.0f && Dist < 20.0f)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "HalfOpened");
			}
		}
		else if(ConstrItr->ConstraintInstance.AngularSwing1Motion == EAngularConstraintMotion::ACM_Limited)
		{
			const float CurrPos = ConstrItr->ConstraintInstance.GetCurrentSwing1();
			TPair<float, float> MinMax = *DoorToMinMaxMap.Find(FurnitureAct);

			const float ClosedVal = MinMax.Key + 0.15f;
			const float HalfVal = (MinMax.Value + MinMax.Key) - 0.5f;
			const float OpenedVal = MinMax.Value * 0.5;

			if (CurrPos < ClosedVal)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "Closed");
			}
			else if (CurrPos < HalfVal && CurrPos > ClosedVal)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "HalfClosed");
			}
			else if (CurrPos > OpenedVal)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "Opened");
			}
			else if (CurrPos < OpenedVal && CurrPos > HalfVal)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "HalfOpened");
			}			
		}
		else if (ConstrItr->ConstraintInstance.AngularSwing2Motion == EAngularConstraintMotion::ACM_Limited)
		{
			const float CurrPos = ConstrItr->ConstraintInstance.GetCurrentSwing2();
			TPair<float, float> MinMax = *DoorToMinMaxMap.Find(FurnitureAct);

			const float ClosedVal = MinMax.Key - 0.1f;
			const float HalfVal = ClosedVal * 0.5f;
			const float OpenedVal = MinMax.Value + 0.1f;

			if (CurrPos > ClosedVal)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "Closed");
			}
			else if (CurrPos > HalfVal && CurrPos < ClosedVal)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "HalfClosed");
			}
			else if (CurrPos < OpenedVal)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "Opened");
			}
			else if (CurrPos > OpenedVal && CurrPos < HalfVal)
			{
				ARSemDrawerStateLog::LogState(FurnitureAct, "HalfOpened");
			}
		}
	}
}

// Log state
void ARSemDrawerStateLog::LogState(AActor* Furniture, const FString State)
{
	// Get the previous state of the furniture
	FString PrevState = FurnitureToStateMap.FindRef(Furniture);

	// Create state if it the first
	if(PrevState.IsEmpty())
	{
		// Add to map
		FurnitureToStateMap.Add(Furniture, State);
		// Log first state, init the semantic event
		FRSemEventsExporterSingl::Get().FurnitureStateEvent
			(Furniture, State, GetWorld()->GetTimeSeconds());
	}
	else
	{
		if (PrevState.Equals(State))
		{
			// Skip if the current state is the same with the previous one
			return;
		}
		else
		{
			// Update map state
			FurnitureToStateMap.Add(Furniture, State);
			// Log state
			FRSemEventsExporterSingl::Get().FurnitureStateEvent
				(Furniture, State, GetWorld()->GetTimeSeconds());
		}
	}
}