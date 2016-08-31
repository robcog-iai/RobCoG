// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RUtils.h"
#include "RSemEventsExporterSingl.h"
#include "RSemLogManager.h"

// Sets default values
ARSemLogManager::ARSemLogManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Actor hidden in game
	SetActorHiddenInGame(true);

	// Log directory name
	LogRootDirectoryName = "MyLogs";

	// Episode unique tag
	EpisodeUniqueTag = FRUtils::GenerateRandomFString(4);

	// Default flag values
	bLogRawData = true;
	bLogSemanticMap = true;
	bLogSemanticEvents = true;

	// Distance squared threshold for logging the raw data
	DistanceThresholdSquared = 0.01;
}

// Called when the game starts or when spawned
void ARSemLogManager::BeginPlay()
{
	Super::BeginPlay();

	// Level directory path
	LevelPath = LogRootDirectoryName + "/" +	GetWorld()->GetName();
	// Episode directory path
	EpisodePath = LevelPath + "/Episodes/" + "rcg_" + FDateTime::Now().ToString();
	// Raw data directory path
	RawDataPath = LevelPath + "/RawData/";
	// Create the directory paths
	ARSemLogManager::CreateDirectoryPath(EpisodePath);
	ARSemLogManager::CreateDirectoryPath(RawDataPath);

	// Check items tags to see which should be logged
	ARSemLogManager::SetLogItems();
	// Check if unique names already generated (past episodes)
	if (!ARSemLogManager::ReadUniqueNames(LevelPath + "/MetaData.json"))
	{
		// Generate new unique names if not generated or out of sync
		ARSemLogManager::GenerateUniqueNames();
		// Save unique names to file (for future use)
		ARSemLogManager::WriteUniqueNames(LevelPath + "/MetaData.json");
	}

	// Log Semantic map
	if (bLogSemanticMap)
	{
		// Semantic map path
		const FString SemMapPath = LevelPath + "/SemanticMap.owl";
		// Chek if semantic map is not already created		

		// Check if map is not already created for this level
		if (!IFileManager::Get().FileExists(*SemMapPath))
		{
			// Create sem map exporter
			SemMapExporter = new FRSemMapExporter();
			// Generate and write level semantic map
			SemMapExporter->WriteSemanticMap(
				DynamicActPtrToUniqNameMap,
				StaticActPtrToUniqNameMap,
				ActorToClassTypeMap,
				CameraToUniqueName,
				SemMapPath);
		}
	}

	// Init raw data logger
	if (bLogRawData)
	{
		// Path to the json file
		const FString RawFilePath = RawDataPath + "/RawData_" + EpisodeUniqueTag + ".json";
		// Init raw data exporter
		RawDataExporter = new FRRawDataExporter(
			DistanceThresholdSquared,
			RawFilePath,
			SkelActPtrToUniqNameMap,
			DynamicActPtrToUniqNameMap,
			StaticActPtrToUniqNameMap,
			CameraToUniqueName);
	}

	// Init semantic events logger
	if (bLogSemanticEvents)
	{
		FRSemEventsExporterSingl::Get().Init(
			EpisodeUniqueTag,
			ActorToUniqueNameMap, 
			ActorToClassTypeMap, 
			GetWorld()->GetTimeSeconds());
	}
}

// Called when the game is terminated
void ARSemLogManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Write events and terminate the singleton
	if (FRSemEventsExporterSingl::Get().IsInit())
	{
		// Save logged events
		FRSemEventsExporterSingl::Get().WriteEvents(
			EpisodePath,
			GetWorld()->GetTimeSeconds());

		// Reset the singleton (if we run it in editor it does not get deleted)
		FRSemEventsExporterSingl::Get().Reset();
	}	
}

// Called every frame
void ARSemLogManager::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	
	// Log raw data
	if (RawDataExporter)
	{
		RawDataExporter->Update(GetWorld()->GetTimeSeconds());
	}
}

// Create directory path for logging
void ARSemLogManager::CreateDirectoryPath(FString Path)
{
	// Create array of the directory names
	TArray<FString> DirNames;
	Path.ParseIntoArray(DirNames, TEXT("/"), true);

	// Get platform file
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

	// Current path
	FString CurrPath;

	// Create directory path
	for (const auto DirName : DirNames)
	{
		// Append current directory name to the path
		CurrPath.Append(DirName + "/");
		// Create directory if not existent
		if (!PlatformFile.DirectoryExists(*CurrPath))
		{
			PlatformFile.CreateDirectory(*CurrPath);
		}
	}
}

// Set items to be logged (from tags)
void ARSemLogManager::SetLogItems()
{
	// Iterate through the static mesh actors and check tags to see which objects should be logged
	for (TActorIterator<AStaticMeshActor> StaticMeshActItr(GetWorld()); StaticMeshActItr; ++StaticMeshActItr)
	{
		// Get static mesh comp tags
		const TArray<FName> Tags = StaticMeshActItr->GetStaticMeshComponent()->ComponentTags;
		// Skip if object has less than 2 tags (type, class)
		if (Tags.Num() > 1)
		{
			// Get the first tag 
			const FString Tag0 = Tags[0].ToString();
			// Check tag type
			if (Tag0.Contains("DynamicItem"))
			{
				// Add dynamic actor to be loggend
				DynamicActNameToActPtrMap.Add(*StaticMeshActItr->GetName(), *StaticMeshActItr);
			}
			else if (Tag0.Contains("StaticMap"))
			{
				// Add static actor to be loggend
				StaticActNameToActPtrMap.Add(*StaticMeshActItr->GetName(), *StaticMeshActItr);
			}

			// Get the second tag 
			const FString Tag1 = Tags[1].ToString();
			// Add class type to the map (unique name will be changed)
			ActorToClassTypeMap.Add(*StaticMeshActItr, Tag1);
		}
	}

	//// Iterate through characters to check for skeletal components to be logged
	//for (TActorIterator<ACharacter> CharItr(GetWorld()); CharItr; ++CharItr)
	//{
	//	// Get the skeletal components from the character
	//	TArray<UActorComponent*> SkelComponents = (*CharItr)->GetComponentsByClass(USkeletalMeshComponent::StaticClass());
	//	// Itrate through the skeletal components
	//	for (const auto SkelCompItr : SkelComponents)
	//	{
	//		// Cast UActorComponent to USkeletalMeshComponent
	//		USkeletalMeshComponent* SkelComp = Cast<USkeletalMeshComponent>(SkelCompItr);
	//		// Check that the skel mesh is not empty
	//		if (SkelComp->bHasValidBodies)
	//		{
	//			// Get component tags
	//			const TArray<FName> Tags = SkelCompItr->ComponentTags;
	//			// Skip if object has no tags
	//			if (Tags.Num() > 0)
	//			{
	//				// Get the first tag 
	//				const FString Tag0 = Tags[0].ToString();
	//				// Check first tag type
	//				if (Tag0.Contains("DynamicItem"))
	//				{
	//					// Add skel component to be loggend
	//					SkelCompNameToCompPtrMap.Add(SkelComp->GetName(), SkelComp);
	//				}
	//			}
	//		}
	//	}
	//}

	// Iterate the world directly for skeletal mesh actors to be logged
	for (TActorIterator<ASkeletalMeshActor> SkelMeshActItr(GetWorld()); SkelMeshActItr; ++SkelMeshActItr)
	{
		// Get component tags
		const TArray<FName> Tags = SkelMeshActItr->Tags;
		// Skip if object has no tags
		if (Tags.Num() > 1)
		{
			// Get the first tag 
			const FString Tag0 = Tags[0].ToString();
			// Check first tag type
			if (Tag0.Contains("DynamicItem"))
			{
				// Add skel component to be loggend
				SkelActNameToActPtrMap.Add(SkelMeshActItr->GetName(), *SkelMeshActItr);
			}

			// Get the second tag 
			const FString Tag1 = Tags[1].ToString();
			// Add class type to the map (unique name will be changed)
			ActorToClassTypeMap.Add(*SkelMeshActItr, Tag1);
		}
	}

	// Iterate characters to get the the camera component
	for (TActorIterator<ACharacter> CharactItr(GetWorld()); CharactItr; ++CharactItr)
	{
		// Get children components and check for UCameraComponent
		TArray<USceneComponent*> ChildrenArr;
		CharactItr->GetCapsuleComponent()->GetChildrenComponents(true, ChildrenArr);
		for (const auto ChildItr : ChildrenArr)
		{
			if (ChildItr->IsA(UCameraComponent::StaticClass()))
			{
				CameraToUniqueName.Key = ChildItr;
				CameraToUniqueName.Value = ""; // mockup unique name
				UE_LOG(LogTemp, Error, TEXT("Class: %s"), *CameraToUniqueName.Key->GetName());
			}
		}
	}
}

// Generate items unique names
void ARSemLogManager::GenerateUniqueNames()
{
	// Lambda function to iterate through actor types and generate unique names
	auto GenerateUniqueNamesLambda = [this](const TMap<FString,AStaticMeshActor*>& ActNameToActPtrMap)
	{
		// Iterate all dynamic items to be logged and generate unique names
		for (const auto ActNameToActPtrItr : ActNameToActPtrMap)
		{
			// Actor name
			const FString NameKey = ActNameToActPtrItr.Key;

			// Generate unique name and make sure there is an underscore before the unique hash
			FString UniqueName = NameKey;
			UniqueName += (UniqueName.Contains("_"))
				? FRUtils::GenerateRandomFString(4)
				: "_" + FRUtils::GenerateRandomFString(4);

			// Add generated unqique name to the dynamic actors map
			DynamicActPtrToUniqNameMap.Add(ActNameToActPtrItr.Value, UniqueName);

			// Add actor to unique name
			ActorToUniqueNameMap.Add(ActNameToActPtrItr.Value, UniqueName);
		}
	};

	// Call lambda function to interate through actors and add them unique names
	GenerateUniqueNamesLambda(DynamicActNameToActPtrMap);
	GenerateUniqueNamesLambda(StaticActNameToActPtrMap);

	// Iterate all skeletal actors to be logged and generate unique names
	for (const auto SkelActNameToCompPtrItr : SkelActNameToActPtrMap)
	{
		// Generate unique name and make sure there is an underscore before the unique hash
		FString UniqueName = SkelActNameToCompPtrItr.Key;
		UniqueName += (UniqueName.Contains("_"))
			? FRUtils::GenerateRandomFString(4)
			: "_" + FRUtils::GenerateRandomFString(4);

		// Add generated unqique name to map
		SkelActPtrToUniqNameMap.Add(SkelActNameToCompPtrItr.Value, UniqueName);

		// Add actor to unique name
		ActorToUniqueNameMap.Add(SkelActNameToCompPtrItr.Value, UniqueName);
	}

	// Iterate characters to get the the camera component
	for (TActorIterator<ACharacter> CharactItr(GetWorld()); CharactItr; ++CharactItr)
	{
		// Get children components and check for UCameraComponent
		TArray<USceneComponent*> ChildrenArr;
		CharactItr->GetCapsuleComponent()->GetChildrenComponents(true, ChildrenArr);
		for (const auto ChildItr : ChildrenArr)
		{
			if (ChildItr->IsA(UCameraComponent::StaticClass()))
			{
				CameraToUniqueName.Key = ChildItr;
				CameraToUniqueName.Value = ChildItr->GetName() + "_" + FRUtils::GenerateRandomFString(4);
				UE_LOG(LogTemp, Error, TEXT("Generated Unique Name: %s"), *CameraToUniqueName.Value);
			}
		}
	}
}

// Read unique names from file
bool ARSemLogManager::ReadUniqueNames(const FString Path)
{
	// Check if file exists, and see if it is in sync with the level
	if (IFileManager::Get().FileExists(*Path))
	{
		// Read string from file		
		FString JsonString;
		FFileHelper::LoadFileToString(JsonString, *Path);
		
		// Create json from string
		TSharedRef< TJsonReader<> > Reader = TJsonReaderFactory<>::Create(JsonString);
		TSharedPtr<FJsonObject> JsonObject;		
		if (FJsonSerializer::Deserialize(Reader, JsonObject))
		{
			// Get the actor to unique name array
			TArray< TSharedPtr<FJsonValue> > JsonArr = JsonObject->GetArrayField("level_unique_names");
			
			// Check if the items to be logged are in the json array
			for (const auto JsonItr : JsonArr)
			{
				// Check if actor name is to be logged, and if the unique name is the same
				const FString NameField = *JsonItr->AsObject()->GetStringField("name");
				if (DynamicActNameToActPtrMap.Contains(NameField))
				{
					// Get unique name and add it to the map
					const FString UniqueName = *JsonItr->AsObject()->GetStringField("unique_name");
					// Add generated unique name to map
					DynamicActPtrToUniqNameMap.Add(DynamicActNameToActPtrMap[NameField], UniqueName);

					// Add actor to unique name
					ActorToUniqueNameMap.Add(DynamicActNameToActPtrMap[NameField], UniqueName);

				}
				else if(StaticActNameToActPtrMap.Contains(NameField))
				{
					// Get unique name and add it to the map
					const FString UniqueName = *JsonItr->AsObject()->GetStringField("unique_name");
					// Add generated unique name to map
					StaticActPtrToUniqNameMap.Add(StaticActNameToActPtrMap[NameField], UniqueName);

					// Add actor to unique name
					ActorToUniqueNameMap.Add(StaticActNameToActPtrMap[NameField], UniqueName);
				}
				else if (SkelActNameToActPtrMap.Contains(NameField))
				{
					// Get unique name and add it to the map
					const FString UniqueName = *JsonItr->AsObject()->GetStringField("unique_name");
					// Add generated unique name to map
					SkelActPtrToUniqNameMap.Add(SkelActNameToActPtrMap[NameField], UniqueName);

					// Add actor to unique name
					ActorToUniqueNameMap.Add(SkelActNameToActPtrMap[NameField], UniqueName);
				}
				else if (CameraToUniqueName.Key->GetName().Equals(NameField))
				{
					// Get unique name and add it to the map
					const FString UniqueName = *JsonItr->AsObject()->GetStringField("unique_name");
					// Add generated unique name to map
					CameraToUniqueName.Value = UniqueName;
					UE_LOG(LogTemp, Error, TEXT("Stored Json Unique Name: %s"), *CameraToUniqueName.Value);
				}
				else
				{
					UE_LOG(LogTemp, Error, TEXT("Unique names not in sync with older episode! %s not found!"), *NameField);
					return false;
				}
			}
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Unique names cannot be read! Json string: %s"), *JsonString);
			return false;
		}

		// Succesfully read all the unique values
		return true;
	}
	else
	{
		UE_LOG(LogTemp, Warning, 
			TEXT("No previous level unique names found at: %s, generating new ones!"), *Path);
		return false;
	}
}

// Write generated unique names to file
void ARSemLogManager::WriteUniqueNames(const FString Path)
{
	// Json root object
	TSharedPtr<FJsonObject> JsonRootObj = MakeShareable(new FJsonObject);

	// Json array of actors
	TArray< TSharedPtr<FJsonValue> > JsonUniqueNamesArr;

	// Iterate through the dynamic items
	for (const auto DynActPtrToUniqNameItr : DynamicActPtrToUniqNameMap)
	{
		// Json location object
		TSharedPtr<FJsonObject> JsonObj = MakeShareable(new FJsonObject);
		// Add fields
		JsonObj->SetStringField("name", DynActPtrToUniqNameItr.Key->GetName());
		JsonObj->SetStringField("unique_name", DynActPtrToUniqNameItr.Value);
		// Add actor to Json array
		JsonUniqueNamesArr.Add(MakeShareable(new FJsonValueObject(JsonObj)));
	}

	// Iterate through the static map items
	for (const auto StaActPtrToUniqNameItr : StaticActPtrToUniqNameMap)
	{
		// Json location object
		TSharedPtr<FJsonObject> JsonObj = MakeShareable(new FJsonObject);
		// Add fields
		JsonObj->SetStringField("name", StaActPtrToUniqNameItr.Key->GetName());
		JsonObj->SetStringField("unique_name", StaActPtrToUniqNameItr.Value);
		// Add actor to Json array
		JsonUniqueNamesArr.Add(MakeShareable(new FJsonValueObject(JsonObj)));
	}

	// Iterate through the static map items
	for (const auto SkelActPtrToUniqNameItr : SkelActPtrToUniqNameMap)
	{
		// Json location object
		TSharedPtr<FJsonObject> JsonObj = MakeShareable(new FJsonObject);
		// Add fields
		JsonObj->SetStringField("name", SkelActPtrToUniqNameItr.Key->GetName());
		JsonObj->SetStringField("unique_name", SkelActPtrToUniqNameItr.Value);
		// Add actor to Json array
		JsonUniqueNamesArr.Add(MakeShareable(new FJsonValueObject(JsonObj)));
	}

	// Write character camera unique name
	{
		// Json location object
		TSharedPtr<FJsonObject> JsonObj = MakeShareable(new FJsonObject);
		// Add fields
		JsonObj->SetStringField("name", CameraToUniqueName.Key->GetName());
		JsonObj->SetStringField("unique_name", CameraToUniqueName.Value);
		// Add actor to Json array
		JsonUniqueNamesArr.Add(MakeShareable(new FJsonValueObject(JsonObj)));
	}

	// Add actors to Json root
	JsonRootObj->SetArrayField("level_unique_names", JsonUniqueNamesArr);

	// Transform to string
	FString JsonOutputString;
	TSharedRef< TJsonWriter<> > Writer = TJsonWriterFactory<>::Create(&JsonOutputString);
	FJsonSerializer::Serialize(JsonRootObj.ToSharedRef(), Writer);

	// Write string to file
	FFileHelper::SaveStringToFile(JsonOutputString, *Path);
}
