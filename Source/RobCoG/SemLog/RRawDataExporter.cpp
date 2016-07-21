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
void FRRawDataExporter::Update(const float Timestamp)
{
	// Json root object
	TSharedPtr<FJsonObject> JsonRootObj = MakeShareable(new FJsonObject);

	// Set timestamp
	JsonRootObj->SetNumberField("timestamp", Timestamp);

	// Json array of actors
	TArray< TSharedPtr<FJsonValue> > JsonActorArr;

	// Iterate through the skeletal mesh components
	for (auto& SkMWPoseItr : SkelMeshComponentsWithPrevPose)
	{
		// Get component current location
		const FVector CurrCompLocation = SkMWPoseItr.SkelMeshComp->GetComponentLocation();

		// Squared distance between the current and the previous pose
		const float DistSqr = FVector::DistSquared(CurrCompLocation, SkMWPoseItr.PrevLoc);

		// Save data if distance larger than threshold
		if (DistSqr > DistanceThresholdSquared)
		{
			// Get a local pointer of the skeletal mesh
			USkeletalMeshComponent* CurrSkelMesh = SkMWPoseItr.SkelMeshComp;
			// Update previous location
			SkMWPoseItr.PrevLoc = CurrCompLocation;
			// Json actor object with name location and rotation
			TSharedPtr<FJsonObject> JsonActorObj = FRRawDataExporter::CreateNameLocRotJsonObject(
				CurrSkelMesh->SkeletalMesh->GetName(), CurrCompLocation * 0.01, CurrSkelMesh->GetComponentQuat());

			// Json array of bones
			TArray< TSharedPtr<FJsonValue> > JsonBoneArr;
			// Get bone names
			TArray<FName> BoneNames;
			CurrSkelMesh->GetBoneNames(BoneNames);

			// Iterate through the bones of the skeletal mesh
			for (const auto BoneName : BoneNames)
			{
				// TODO black voodo magic crashes, bug report, crashes if this is not called before
				CurrSkelMesh->GetBoneQuaternion(BoneName);

				// Json bone object with name location and rotation
				TSharedPtr<FJsonObject> JsonBoneObj = FRRawDataExporter::CreateNameLocRotJsonObject(
					BoneName.ToString(), CurrSkelMesh->GetBoneLocation(BoneName) * 0.01,
					CurrSkelMesh->GetBoneQuaternion(BoneName));

				// Add bone to Json array
				JsonBoneArr.Add(MakeShareable(new FJsonValueObject(JsonBoneObj)));
			}
			// Add bones to Json actor
			JsonActorObj->SetArrayField("bones", JsonBoneArr);

			// Add actor to Json array
			JsonActorArr.Add(MakeShareable(new FJsonValueObject(JsonActorObj)));
		}
	}

	// Iterate through the static mesh components
	for (auto& StMActWPoseItr : StaticMeshActorsWithPrevPose)
	{
		// Get component current location
		const FVector CurrActLocation = StMActWPoseItr.StaticMeshAct->GetActorLocation();

		// Squared distance between the current and the previous pose
		const float DistSqr = FVector::DistSquared(CurrActLocation, StMActWPoseItr.PrevLoc);

		// Save data if distance larger than threshold
		if (DistSqr > DistanceThresholdSquared)
		{
			// Get a local pointer of the skeletal mesh actor
			AStaticMeshActor* CurrStaticMeshAct = StMActWPoseItr.StaticMeshAct;
			// Update previous location
			StMActWPoseItr.PrevLoc = CurrActLocation;

			// Json actor object with name location and rotation
			TSharedPtr<FJsonObject> JsonActorObj = FRRawDataExporter::CreateNameLocRotJsonObject(
				CurrStaticMeshAct->GetName(), CurrActLocation * 0.01, CurrStaticMeshAct->GetActorQuat());

			// Add actor to Json array
			JsonActorArr.Add(MakeShareable(new FJsonValueObject(JsonActorObj)));
		}
	}

	// Check if static map actors need to be logged
	if (StaticMapActors.Num() > 0)
	{
		// Iterate the static map actors (done only once)
		for (auto& StaticMapActItr : StaticMapActors)
		{
			// Json actor object with name location and rotation
			TSharedPtr<FJsonObject> JsonActorObj = FRRawDataExporter::CreateNameLocRotJsonObject(
				StaticMapActItr->GetName(), StaticMapActItr->GetActorLocation() * 0.01, StaticMapActItr->GetActorQuat());

			// Add actor to Json array
			JsonActorArr.Add(MakeShareable(new FJsonValueObject(JsonActorObj)));
		}
		// Empty array, only needs to be logged once;
		StaticMapActors.Empty();
	}

	
	// Add actors to Json root
	JsonRootObj->SetArrayField("actors", JsonActorArr);

	// Transform to string
	FString JsonOutputString;
	TSharedRef< TJsonWriter<> > Writer = TJsonWriterFactory<>::Create(&JsonOutputString);
	FJsonSerializer::Serialize(JsonRootObj.ToSharedRef(), Writer);

	// Write string to file
	RawFileHandle->Write((const uint8*)TCHAR_TO_ANSI(*JsonOutputString), JsonOutputString.Len());
}

// Create Json object with a 3d location
inline TSharedPtr<FJsonObject> FRRawDataExporter::CreateLocationJsonObject(const FVector Location)
{
	// Json location object
	TSharedPtr<FJsonObject> JsonObj = MakeShareable(new FJsonObject);
	// Add fields
	JsonObj->SetNumberField("x", Location.X);
	JsonObj->SetNumberField("y", -Location.Y); // left to right handed
	JsonObj->SetNumberField("z", Location.Z);
	
	return JsonObj;
}

// Create Json object with a 3d rotation as quaternion 
inline TSharedPtr<FJsonObject> FRRawDataExporter::CreateRotationJsonObject(const FQuat Rotation)
{
	// Json rotation object
	TSharedPtr<FJsonObject> JsonObj = MakeShareable(new FJsonObject);
	// Add fields
	JsonObj->SetNumberField("w", Rotation.W);
	JsonObj->SetNumberField("x", -Rotation.X); // left to right handed
	JsonObj->SetNumberField("y", Rotation.Y);
	JsonObj->SetNumberField("z", -Rotation.Z); // left to right handed

	return JsonObj;
}

// Create Json object with name location and rotation
inline TSharedPtr<FJsonObject> FRRawDataExporter::CreateNameLocRotJsonObject(const FString Name, const FVector Location, const FQuat Rotation)
{
	// Json  actor object
	TSharedPtr<FJsonObject> JsonObj = MakeShareable(new FJsonObject);
	// Add fields
	JsonObj->SetStringField("name", Name);
	JsonObj->SetObjectField("pos", FRRawDataExporter::CreateLocationJsonObject(Location));
	JsonObj->SetObjectField("rot", FRRawDataExporter::CreateRotationJsonObject(Rotation));

	return JsonObj;
}

// Update grasping
void FRRawDataExporter::InitItemsToLog(UWorld* World)
{
	// Iterate all characters to get their skeleton components (if available)
	UE_LOG(LogTemp, Warning, TEXT("Skeletal components to be logged:"));
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
					// Check first tag type
					if (Tags[0].ToString().Contains("Log"))
					{
						// Add component to the list of objects to be logged, initialize prev position and orientation
						SkelMeshComponentsWithPrevPose.Add(FRSkelMeshWPrevPose(SkelComp));
						UE_LOG(LogTemp, Warning, TEXT("\t %s"), *SkelCompItr->GetName());
					}
				}
			}
		}
	}

	// Iterate world skeletal mesh actors
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
				// Check first tag type
				if (Tags[0].ToString().Contains("Log"))
				{
					// Add component to the list of objects to be logged, initialize prev position and orientation
					SkelMeshComponentsWithPrevPose.Add(SkelComp);
					UE_LOG(LogTemp, Warning, TEXT("\t %s"), *SkelMeshItr->GetName());
				}
			}
		}
	}

	// Iterate for object individuals and actors to log
	UE_LOG(LogTemp, Warning, TEXT("Items to be logged:"));
	for (TActorIterator<AStaticMeshActor> StaticMeshActItr(World); StaticMeshActItr; ++StaticMeshActItr)
	{
		// Get SkeletalMeshComponent
		UStaticMeshComponent* StaticMeshComp = StaticMeshActItr->GetStaticMeshComponent();
		// Get object tags
		const TArray<FName> Tags = StaticMeshComp->ComponentTags;
		// Skip if object has no tags
		if (Tags.Num() > 0)
		{
			// Check first tag type
			if (Tags[0].ToString().Contains("Log"))
			{
				// Add actor to the list of objects to be logged, initialize prev position and orientation
				StaticMeshActorsWithPrevPose.Add(*StaticMeshActItr);
				UE_LOG(LogTemp, Warning, TEXT("\t %s"), *StaticMeshActItr->GetName());
			}	
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("\t  !!! %s has no tags, NO raw data logging.. "),
				*StaticMeshActItr->GetName());
		}
	}

	// Iterate for object individuals and actors to log
	UE_LOG(LogTemp, Warning, TEXT("Static map to be logged (only once):"));
	for (TActorIterator<AStaticMeshActor> StaticMeshActItr(World); StaticMeshActItr; ++StaticMeshActItr)
	{
		// Get SkeletalMeshComponent
		UStaticMeshComponent* StaticMeshComp = StaticMeshActItr->GetStaticMeshComponent();
		// Get object tags
		const TArray<FName> Tags = StaticMeshComp->ComponentTags;
		// Skip if object has no tags
		if (Tags.Num() > 0)
		{
			// Check first tag type
			if (Tags[0].ToString().Contains("StaticMap"))
			{
				// Add actor to the list of objects to be logged, initialize prev position and orientation
				StaticMapActors.Add(*StaticMeshActItr);
				UE_LOG(LogTemp, Warning, TEXT("\t %s"), *StaticMeshActItr->GetName());
			}
		}
	}
}
