// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RCloseDrawers.h"


// Sets default values
ARCloseDrawers::ARCloseDrawers()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

}

// Called when the game starts or when spawned
void ARCloseDrawers::BeginPlay()
{
	Super::BeginPlay();

	//Array to store all actors in the world; used to find which object is selected
	TArray<AActor*> AllActors;

	//Gets all actors in the world, used for identifying our drawers and setting their initial state to closed
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), AllActors);

	//Loop that checks if current object matches our drawer
	for (const auto ActorIt : AllActors)
	{
		//Finds the actors for the Handles, used to set the initial state of our drawers to closed 
		if (ActorIt->GetName().Contains("Handle"))
		{
			for (const auto Comp : ActorIt->GetComponents())
			{
				if (Comp->GetName().Contains("StaticMeshComponent"))
				{
					UStaticMeshComponent* CurrObjectMesh = Cast<UStaticMeshComponent>(Comp);

					if (CurrObjectMesh)
					{
						if (!ActorIt->GetName().Contains("Door"))
						{
							CurrObjectMesh->AddImpulse(FVector(-6000) * ActorIt->GetActorForwardVector());
						}
					}
					break;
				}
			}
		}
	}	
}


