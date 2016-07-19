// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "Animation/SkeletalMeshActor.h"
#include "RRawDataExporter.h"


// Set default values
FRRawDataExporter::FRRawDataExporter(
	const float DistThreshSqr, 
	UWorld* World,
	TSharedPtr<IFileHandle> FileHandle)	:
	DistanceThresholdSquared(DistThreshSqr),
	RawFileHandle(FileHandle)
{
	// Init items we want to log
	FRRawDataExporter::InitItemsToLog(World);
}

// Destructor
FRRawDataExporter::~FRRawDataExporter()
{
	RawFileHandle.Reset();
}

// Update grasping
void FRRawDataExporter::Update()
{
	//UE_LOG(LogTemp, Warning, TEXT("Log raw %s"));
}

// Update grasping
void FRRawDataExporter::InitItemsToLog(UWorld* World)
{
	////////////////////////////////////////////////////////
	// Iterate all characters to get the skeleton components
	UE_LOG(LogTemp, Warning, TEXT("SkeletonComponents to be logged:"));
	for (TActorIterator<ACharacter> CharItr(World); CharItr; ++CharItr)
	{
		// Get the skeletal components from the character
		TArray<UActorComponent*> SkelComponents = (*CharItr)->GetComponentsByClass(USkeletalMeshComponent::StaticClass());
		// Itrate through the skeletal components
		for (const auto SkelCompItr : SkelComponents)
		{
			// Cast UActorComponent to USkeletalMeshComponent
			USkeletalMeshComponent* SkelComp = Cast<USkeletalMeshComponent>(SkelCompItr);
			// Check that the skel mesh is not empty
			if (SkelComp->bHasValidBodies)
			{
				// Get component tags
				const TArray<FName> Tags = SkelCompItr->ComponentTags;
				// Skip if object has no tags
				if (Tags.Num() > 0)
				{
					// Get the first tag 
					FString Tag0 = Tags[0].ToString();
					// Check tag type and remove it if correct
					if (Tag0.RemoveFromStart("Log"))
					{
						// Add component to the list of objects to be logged, initialize prev position and orientation
						SkelMeshComponentsWithPose.Add(
							TTuple<USkeletalMeshComponent*, FVector, FRotator>
							(SkelComp, FVector(0.0f), FRotator(0.0f)));
						UE_LOG(LogTemp, Warning, TEXT("\t %s"), *SkelCompItr->GetName());
					}
				}
			}
		}
	}

	////////////////////////////////////////////////////////
	// Iterate world skeletal mesh actors
	// In the current setup the skeletal meshes are loaded from the character
	for (TActorIterator<ASkeletalMeshActor> SkelMeshItr(World); SkelMeshItr; ++SkelMeshItr)
	{
		// Get SkeletalMeshComponent
		USkeletalMeshComponent* SkelComp = SkelMeshItr->GetSkeletalMeshComponent();
		// Check that the skel mesh is not empty
		if (SkelComp->bHasValidBodies)
		{
			// Get component tags
			const TArray<FName> Tags = SkelComp->ComponentTags;
			// Skip if object has no tags
			if (Tags.Num() > 0)
			{
				// Get the first tag 
				FString Tag0 = Tags[0].ToString();
				// check tag type and remove it if correct
				if (Tag0.RemoveFromStart("Log"))
				{
					// Add component to the list of objects to be logged, initialize prev position and orientation
					SkelMeshComponentsWithPose.Add(
						TTuple<USkeletalMeshComponent*, FVector, FRotator>
						(SkelComp, FVector(0.0f), FRotator(0.0f)));
					UE_LOG(LogTemp, Warning, TEXT("\t %s"), *SkelMeshItr->GetName());
				}
			}
		}
	}

	////////////////////////////////////////////////////////
	// Iterate for object individuals and actors to log
	UE_LOG(LogTemp, Warning, TEXT("StaticMeshActors to be logged:"));
	for (TActorIterator<AStaticMeshActor> StaticMeshActItr(World); StaticMeshActItr; ++StaticMeshActItr)
	{
		// Get SkeletalMeshComponent
		UStaticMeshComponent* StaticMeshComp = StaticMeshActItr->GetStaticMeshComponent();
		// Get object tags
		const TArray<FName> Tags = StaticMeshComp->ComponentTags;
		// Skip if object has no tags
		if (Tags.Num() > 0)
		{
			// Get the first tag 
			FString Tag0 = Tags[0].ToString();

			// check tag type and remove it if correct
			if (Tag0.RemoveFromStart("Log"))
			{
				// Add actor to the list of objects to be logged, initialize prev position and orientation
				StaticMeshComponentsWithPose.Add(
					TTuple<UStaticMeshComponent*, FVector, FRotator>
					(StaticMeshComp, FVector(0.0f), FRotator(0.0f)));
				UE_LOG(LogTemp, Warning, TEXT("\t %s"), *(*StaticMeshActItr)->GetName());
			}	
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("\t  !!! %s has no tags, NO raw data logging.. "),
				*StaticMeshActItr->GetName());
		}
	}
}
