// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RSemLogManager.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidxml/rapidxml_print.hpp"


// Sets default values
ARSemLogManager::ARSemLogManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	//  Set flags
	LogSemanticMap = true;
	LogSemanticData = true;
	LogEpisodeRating = true;
	LogRawData = true;
	LogTimelines = true;


	// Init elapsed time
	ElapsedTime = 0.0f;

	// Init static map logged flag
	StaticMapLogged = false;

	// Log directory name
	LogDirName = "MyLogs";

	// Raw data file name
	RawDataFileName = "RawData.json";

	// Event data file name
	EventsFileName = "EventData.owl";

	// Semantic map file name
	SemanticMapFileName = "SemanticMap.owl";

	// Episode rating file name
	EpRatingFileName = "EpisodeRating.owl";

	// Timeline filename
	TimelineFileName = "Timeline.html";

	// Distance squared threshold for logging
	DistSquaredThresh = 0.01;

	// TODO move from here
	// Add the property definitions
	PropertyDefinitions.Add("&knowrob;taskContext");
	PropertyDefinitions.Add("&knowrob;taskSuccess");
	PropertyDefinitions.Add("&knowrob;startTime");
	PropertyDefinitions.Add("&knowrob;endTime");
	PropertyDefinitions.Add("&knowrob;objectActedOn");
	PropertyDefinitions.Add("&knowrob;experiment");
	PropertyDefinitions.Add("&knowrob;experimentName");
	PropertyDefinitions.Add("&knowrob_u;inContact");
	PropertyDefinitions.Add("&knowrob_u;semanticMap");
	PropertyDefinitions.Add("&knowrob_u;rating");

	// Add the class definitions //TODO rename to declarations?
	ClassDefinitions.Add("&knowrob;GraspingSomething");
	ClassDefinitions.Add("&knowrob_u;UnrealExperiment");
	ClassDefinitions.Add("&knowrob_u;TouchingSituation");
	ClassDefinitions.Add("&knowrob_u;KitchenEpisode");
	ClassDefinitions.Add("&knowrob_u;ParticleTranslation");


}

// Called when the game starts or when spawned
void ARSemLogManager::BeginPlay()
{
	Super::BeginPlay();

	// Init characters/actors/events to log
	ARSemLogManager::InitThingsToLog();

	// Create directories for logging
	ARSemLogManager::CreateLogDir();

	// Create metadata owl event
	MetaDataIndividual = new FOwlEvent("&log;", "UnrealExperiment", ARSemLogManager::RandStringUnderscore(4), ElapsedTime);

	// Add OWL StartTime to metadata
	FString StartTime = "&log;timepoint_" + FString::SanitizeFloat(ElapsedTime);
	MetaDataIndividual->AddProperty("rdf:type", "rdf:resource", "&knowrob;UnrealExperiment");
	MetaDataIndividual->AddProperty("knowrob:startTime", "rdf:resource", StartTime);
	TimepointIndividuals.AddUnique(StartTime);

	// Write semantic map
	if (LogSemanticMap)
	{
		ARSemLogManager::WriteSemanticMap();
	}
	
}

// Called when the game stops
void ARSemLogManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	// Finish remaining opened touch events
	for (auto& EventIter : UnfinishedTouchEvents)
	{
		// Get and remove the touch even from the unfinished events map
		FOwlEvent UnfinishedTouchEvent = EventIter.Value;

		// Add EndTime
		UnfinishedTouchEvent.EndTime = ElapsedTime;
		const FString EndTime = "&log;timepoint_" + FString::SanitizeFloat(ElapsedTime);
		UnfinishedTouchEvent.AddProperty("knowrob:endTime", "rdf:resource", EndTime);
		TimepointIndividuals.AddUnique(EndTime);

		// Mark event as finished
		UnfinishedTouchEvent.SetFinshed(true);

		// Add finished touch event to the event individuals array
		EventIndividuals.Add(UnfinishedTouchEvent);

		UE_LOG(LogTemp, Warning, TEXT("Touch event %s, uniqe name: %s, finished with episode ending.."), *EventIter.Value.GetName(), *EventIter.Key);
	}

	// Write the episode rating
	if (LogEpisodeRating)
	{
		ARSemLogManager::WriteEpisodeRating();
	}

	// Write the metadata
	ARSemLogManager::AddMetaData();

	// Write the owl doc
	ARSemLogManager::WriteEvents();

	// Write the timelines
	if (LogTimelines)
	{
		ARSemLogManager::WriteTimelines();
	}
}

// Called every frame
void ARSemLogManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	// Increase elapsed time (timestamp)
	ElapsedTime += DeltaTime;

	// Tick raw data
	ARSemLogManager::TickRawData();
}

// Init things to log
void ARSemLogManager::InitThingsToLog()
{
	////////////////////////////////////////////////////////
	// Iterate all characters to get the skeleton components
	UE_LOG(LogTemp, Warning, TEXT("SkeletonComponents to be logged:"));
	for (TActorIterator<ACharacter> CharItr(GetWorld()); CharItr; ++CharItr)
	{
		// Get the skeletal components from the character
		TArray<UActorComponent*> SkelComponents = (*CharItr)->GetComponentsByClass(USkeletalMeshComponent::StaticClass());

		// Iterate through the skeletal components
		for (auto SkelCompIter : SkelComponents)
		{
			// Cast UActorComponent to USkeletalMeshComponent
			USkeletalMeshComponent* SkelComp = Cast<USkeletalMeshComponent>(SkelCompIter);
			// Check that the skel mesh is not empty
			if (SkelComp->bHasValidBodies)
			{
				// Get component tags
				const TArray<FName> Tags = SkelCompIter->ComponentTags;
				// Skip if object has no tags
				if (Tags.Num() > 0)
				{
					// Get the first tag 
					FString Tag0 = Tags[0].ToString();

					// check tag type and remove it if correct
					if (Tag0.RemoveFromStart("Log."))
					{
						// Add component to the list of objects to be logged, initialize prev position and orientation
						TTuple<USkeletalMeshComponent*, FString, FVector*, FRotator*> LogCompAndClassType(SkelComp, Tag0, new FVector(0, 0, 0), new FRotator(0, 0, 0));
						LogSkelComp.Add(LogCompAndClassType);
						UE_LOG(LogTemp, Warning, TEXT("\t %s, \t Tag0: %s"), *SkelCompIter->GetName(), *Tag0);
					}
				}
			}
		}
	}

	////////////////////////////////////////////////////////
	// Iterate world skeletal mesh actors
	// In the current setup the skeletal meshes are loaded from the character
	for (TActorIterator<ASkeletalMeshActor> SkelMeshItr(GetWorld()); SkelMeshItr; ++SkelMeshItr)
	{
		// Cast UActorComponent to USkeletalMeshComponent
		USkeletalMeshComponent* SkelComp = Cast<USkeletalMeshComponent>(*SkelMeshItr);
		// Check that the skel mesh is not empty
		if (SkelComp->bHasValidBodies)
		{
			// Get component tags
			const TArray<FName> Tags = SkelMeshItr->Tags;
			// Skip if object has no tags
			if (Tags.Num() > 0)
			{
				// Get the first tag 
				FString Tag0 = Tags[0].ToString();

				// check tag type and remove it if correct
				if (Tag0.RemoveFromStart("Log."))
				{
					// Add component to the list of objects to be logged, initialize prev position and orientation
					TTuple<USkeletalMeshComponent*, FString, FVector*, FRotator*> LogCompAndClassType(SkelComp, Tag0, new FVector(0, 0, 0), new FRotator(0, 0, 0));
					LogSkelComp.Add(LogCompAndClassType);
					UE_LOG(LogTemp, Warning, TEXT("\t %s, \t Tag0: %s"), *SkelMeshItr->GetName(), *Tag0);
				}
			}
		}
	}

	////////////////////////////////////////////////////////
	// Iterate for object individuals and actors to log
	UE_LOG(LogTemp, Warning, TEXT("StaticMeshActors to be logged:"));
	for (TActorIterator<AStaticMeshActor> StaticMeshItr(GetWorld()); StaticMeshItr; ++StaticMeshItr)
	{
		// Get object tags
		const TArray<FName> Tags = StaticMeshItr->Tags;

		// Skip if object has no tags
		if (Tags.Num() > 0)
		{
			// Get the first tag 
			FString Tag0 = Tags[0].ToString();

			// check tag type and remove it if correct
			if (Tag0.RemoveFromStart("Log."))
			{
				// Add the namespace to the object name
				FString ObjUniqueName = StaticMeshItr->GetName();
				// Make sure there is an underscore before adding the unique hash
				ObjUniqueName += (ObjUniqueName.Contains("_"))
					? ARSemLogManager::RandString(4) : ARSemLogManager::RandStringUnderscore(4);

				// Map actor to its unique name
				ActorToUniqueName.Add(*StaticMeshItr, ObjUniqueName);

				// Add the namspace to the object class type (First Tag)
				const FString ObjCLassType = FString("&knowrob;") + Tag0;

				// Map actor to its class // no namespace
				ActorToClassType.Add(*StaticMeshItr, Tag0);

				// Add to class definitions, make sure it doesn't repeat
				ClassDefinitions.AddUnique(ObjCLassType);

				// Add to OWL object individuals
				TTuple<FString, FString> ObjAndClassTuple(ObjUniqueName, ObjCLassType);
				ObjectIndividuals.Add(ObjAndClassTuple);

				// Add actor to the list of objects to be logged, initialize prev position and orientation
				TTuple<AActor*, FString, FVector*, FRotator*> LogActorAndClassType(*StaticMeshItr, Tag0, new FVector(0, 0, 0), new FRotator(0, 0, 0));
				LogActors.Add(LogActorAndClassType);
				UE_LOG(LogTemp, Warning, TEXT("\t %s, \t Tag0: %s, \t Unique name: %s"), *(*StaticMeshItr)->GetName(), *Tag0, *ObjUniqueName);
			}
			// if the object is marked as static (map) query pose only once
			else if (Tag0.RemoveFromStart("Map."))
			{
				// Add the namespace to the object name
				FString ObjUniqueName = StaticMeshItr->GetName();
				// Make sure there is an underscore before adding the unique hash
				if (ObjUniqueName.Contains("_"))
				{
					ObjUniqueName += ARSemLogManager::RandString(4);
				}
				else
				{
					ObjUniqueName += FString("_") + ARSemLogManager::RandString(4);
				}

				// Map actor to its unique name
				ActorToUniqueName.Add(*StaticMeshItr, ObjUniqueName);

				// Add the namspace to the object class type (First Tag)
				const FString ObjCLassType = FString("&knowrob;") + Tag0;

				// Map actor to its class // no namespace
				ActorToClassType.Add(*StaticMeshItr, Tag0);

				// Add to class definitions, make sure it doesn't repeat
				ClassDefinitions.AddUnique(ObjCLassType);

				// Add to OWL object individuals
				TTuple<FString, FString> ObjAndClassTuple(ObjUniqueName, ObjCLassType);
				ObjectIndividuals.Add(ObjAndClassTuple);

				// Add actor to the list of objects to be logged only once
				TTuple<AActor*, FString> MapActorAndClassType(*StaticMeshItr, Tag0);
				StaticMapActors.Add(MapActorAndClassType);
				UE_LOG(LogTemp, Warning, TEXT("\t %s, \t Tag0: %s, \t Unique name: %s  [Part of the map. (Logged once)]"), *(*StaticMeshItr)->GetName(), *Tag0, *ObjUniqueName);
			}
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("\t  !!! %s has no tags, ignored as Obj to be logged.. "), *StaticMeshItr->GetName());
		}
	}

	////////////////////////////////////////////////////////
	// Iterate and bind trigger events
	UE_LOG(LogTemp, Warning, TEXT("Bounded triggers:"));
	for (TActorIterator<ATriggerBase> TriggerItr(GetWorld()); TriggerItr; ++TriggerItr)
	{
		// Bind the trigger to the its function
		if (TriggerItr->GetName().Contains(FString("IslandTopTrigger")))
		{
			IslandTriggerArea = *TriggerItr;
			TriggerItr->OnActorBeginOverlap.AddDynamic(this, &ARSemLogManager::OnIslandTriggerOverlapBegin);
			TriggerItr->OnActorEndOverlap.AddDynamic(this, &ARSemLogManager::OnIslandTriggerOverlapEnd);
			UE_LOG(LogTemp, Warning, TEXT("\t %s is bounded.."), *TriggerItr->GetName());
		}
		else if (TriggerItr->GetName().Contains(FString("SinkTopTrigger")))
		{
			SinkTriggerArea = *TriggerItr;
			TriggerItr->OnActorBeginOverlap.AddDynamic(this, &ARSemLogManager::OnSinkTriggerOverlapBegin);
			TriggerItr->OnActorEndOverlap.AddDynamic(this, &ARSemLogManager::OnSinkTriggerOverlapEnd);
			UE_LOG(LogTemp, Warning, TEXT("\t %s is bounded.."), *TriggerItr->GetName());
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("\t %s is NOT bounded.."), *TriggerItr->GetName());
		}
	}
}

// Save raw data to a JsonFile
void ARSemLogManager::TickRawData()
{
	// Json Document
	rapidjson::Document JsonDoc;
	JsonDoc.SetObject();
	// JsonDoc allocator
	rapidjson::Document::AllocatorType& JsonAllocator = JsonDoc.GetAllocator();

	// Add timestamp
	JsonDoc.AddMember("timestamp", ElapsedTime, JsonAllocator);

	// Create json actor array
	rapidjson::Value ActorArrayJson(rapidjson::kArrayType);

	// Check if data static map data has been logged already 
	if (!StaticMapLogged)
	{
		////////////////////////////////////////////////////////
		// Iterate static map actors and add them to the logs (once)
		for (auto MapActorIter : StaticMapActors)
		{
			// Add actor position / orientation to the json array
			ActorArrayJson.PushBack(ARSemLogManager::AddJsonActor(MapActorIter.Get<0>(), MapActorIter.Get<1>(), JsonAllocator), JsonAllocator);
		}
		// Set flag to true
		StaticMapLogged = true;
	}


	////////////////////////////////////////////////////////
	// Get all the skeletal mesh components from the character
	// Iterate through the skeletal components
	for (auto SkelCompIter : LogSkelComp)
	{
		// check if the prev pose differs of the current one by more than the threshold
		if (FVector::DistSquared(SkelCompIter.Get<0>()->GetComponentLocation(), *SkelCompIter.Get<2>()) > DistSquaredThresh)
		{
			// Update the previous location
			*SkelCompIter.Get<2>() = SkelCompIter.Get<0>()->GetComponentLocation();

			// Current character actor
			rapidjson::Value SkelCharJsonObj;
			SkelCharJsonObj.SetObject();

			// Add character position / orientation to the json array
			SkelCharJsonObj = ARSemLogManager::AddJsonComponent(SkelCompIter.Get<0>(), SkelCompIter.Get<1>(), JsonAllocator);

			// Array of the actor bone names
			TArray<FName> SkelBoneNames;

			// Get the bone names
			SkelCompIter.Get<0>()->GetBoneNames(SkelBoneNames);

			// Create json bone array
			rapidjson::Value JsonBoneArray(rapidjson::kArrayType);

			// Iterate through the skeletal mesh bones
			for (auto BoneNameIter : SkelBoneNames)
			{
				// Add bone position / orientation to the json array
				JsonBoneArray.PushBack(ARSemLogManager::AddJsonBone(SkelCompIter.Get<0>(), BoneNameIter, JsonAllocator), JsonAllocator);
			}

			// Add the bone members
			SkelCharJsonObj.AddMember("bones", JsonBoneArray, JsonAllocator);

			// Add json obj to the skel array
			ActorArrayJson.PushBack(SkelCharJsonObj, JsonAllocator);
		}
	}

	////////////////////////////////////////////////////////
	// Iterate static mesh actors and add them to the logs
	for (auto LogActorIter : LogActors)
	{
		// check if the prev pose differs of the current one by more than the threshold
		if (FVector::DistSquared(LogActorIter.Get<0>()->GetActorLocation(), *LogActorIter.Get<2>()) > DistSquaredThresh)
		{
			// Update the previous location
			*LogActorIter.Get<2>() = LogActorIter.Get<0>()->GetActorLocation();

			// Add actor position / orientation to the json array
			ActorArrayJson.PushBack(ARSemLogManager::AddJsonActor(LogActorIter.Get<0>(), LogActorIter.Get<1>(), JsonAllocator), JsonAllocator);
		}
	}

	// Add to actors array
	JsonDoc.AddMember("actors", ActorArrayJson, JsonAllocator);

	// Create string from the json document
	rapidjson::StringBuffer JsonStrBuf;
	rapidjson::Writer<rapidjson::StringBuffer> RapidJsonWriter(JsonStrBuf);
	JsonDoc.Accept(RapidJsonWriter);

	const FString JsonOutputString = JsonStrBuf.GetString() + FString(LINE_TERMINATOR);
	//UE_LOG(LogTemp, Warning, TEXT("%s"), *JsonOutputString);

	// Write to file
	RawFileHandle->Write((const uint8*)TCHAR_TO_ANSI(*JsonOutputString), JsonOutputString.Len());

	UE_LOG(LogTemp, Warning, TEXT("tEST1:"));
	if (IslandTriggerArea != nullptr)
	{
		//UCollisionComponent ColComp = TriggerItr->CollisionComponent;
		UShapeComponent* coll = IslandTriggerArea->GetCollisionComponent();
		uint32 particleCount = coll->FlexParticleCount;

		UE_LOG(LogTemp, Warning, TEXT("PARTICLE COUNT: %i"), particleCount);
	}
}

// Create directories for saving the log files
void ARSemLogManager::CreateLogDir()
{
	// Get platform file
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

	// Episode number by counting the folders
	FLocalTimestampDirectoryVisitor Visitor(PlatformFile, TArray<FString>(), TArray<FString>());
	PlatformFile.IterateDirectory(*LogDirName, Visitor);
	EpisodeNr = Visitor.FileTimes.Num() / 2; // TODO for some reason it returns the double of the number of folders

											 // Episode folder name
	EpisodeDirName = FString("u_") + FString::FromInt(EpisodeNr) + FString("_") + FDateTime::Now().ToString();

	// Check if LOG folder exist, if not create it
	if (!PlatformFile.DirectoryExists(*LogDirName))
	{
		// Create directory
		PlatformFile.CreateDirectory(*LogDirName);
	}
	// Create episode LOG folder
	if (!PlatformFile.DirectoryExists(*(LogDirName + "/" + EpisodeDirName)))
	{
		// Create episode directory
		PlatformFile.CreateDirectory(*(LogDirName + "/" + EpisodeDirName));
	}

	// Raw / event data / sem map / timelines file path
	FString RawFilePath = LogDirName + "/" + EpisodeDirName + "/" + FString("u_")
		+ FString::FromInt(EpisodeNr) + FString("_") + RawDataFileName;
	FString OwlFilePath = LogDirName + "/" + EpisodeDirName + "/" + FString("u_")
		+ FString::FromInt(EpisodeNr) + FString("_") + EventsFileName;
	FString SemanticMapFilePath = LogDirName + "/" + EpisodeDirName + "/" + FString("u_")
		+ FString::FromInt(EpisodeNr) + FString("_") + SemanticMapFileName;
	FString EpRatingFilePath = LogDirName + "/" + EpisodeDirName + "/" + FString("u_")
		+ FString::FromInt(EpisodeNr) + FString("_") + EpRatingFileName;
	FString TimelineFilePath = LogDirName + "/" + EpisodeDirName + "/" + FString("u_")
		+ FString::FromInt(EpisodeNr) + FString("_") + TimelineFileName;

	// Create raw/owl/semantic map file handle for appending
	RawFileHandle = MakeShareable(PlatformFile.OpenWrite(*RawFilePath, true, true));
	EventsFileHandle = MakeShareable(PlatformFile.OpenWrite(*OwlFilePath, true, true));
	SemanticMapFileHandle = MakeShareable(PlatformFile.OpenWrite(*SemanticMapFilePath, true, true));
	EpRatingFileHandle = MakeShareable(PlatformFile.OpenWrite(*EpRatingFilePath, true, true));
	TimelineFileHandle = MakeShareable(PlatformFile.OpenWrite(*TimelineFilePath, true, true));
}

// Write semantic map
void ARSemLogManager::WriteSemanticMap()
{
	// generate uniqe name of the sem map
	const FString SemMapUniqueName = "USemMap_" + ARSemLogManager::RandString(4);

	// Semantic map owl document
	rapidxml::xml_document<>* SemMapOwlDoc = new rapidxml::xml_document<>();;

	// Semantic map RDF node
	rapidxml::xml_node<>* SemMapRDFNode;

	// Add semantic map to the metadata
	MetaDataIndividual->AddProperty("knowrob_u:semanticMap", "rdf:resource", "&u-map;" + SemMapUniqueName);

	///////// DOC
	// Create declaration node <?xml version="1.0" encoding="utf-8"?>
	rapidxml::xml_node<> *DeclarationNode = SemMapOwlDoc->allocate_node(rapidxml::node_declaration);
	// Create attibutes
	DeclarationNode->append_attribute(SemMapOwlDoc->allocate_attribute("version", "1.0"));
	DeclarationNode->append_attribute(SemMapOwlDoc->allocate_attribute("encoding", "utf-8"));
	// Add node to document
	SemMapOwlDoc->append_node(DeclarationNode);

	///////// DOCTYPE
	// Doctype text
	const char* doctype = "rdf:RDF[ \n"
		"\t<!ENTITY rdf \"http://www.w3.org/1999/02/22-rdf-syntax-ns\">\n"
		"\t<!ENTITY rdfs \"http://www.w3.org/2000/01/rdf-schema\">\n"
		"\t<!ENTITY owl \"http://www.w3.org/2002/07/owl\">\n"
		"\t<!ENTITY xsd \"http://www.w3.org/2001/XMLSchema#\">\n"
		"\t<!ENTITY knowrob \"http://knowrob.org/kb/knowrob.owl#\">\n"
		"\t<!ENTITY knowrob_u \"http://knowrob.org/kb/knowrob_u.owl#\">\n"
		"\t<!ENTITY log \"http://knowrob.org/kb/unreal_log.owl#\">\n"
		"\t<!ENTITY u-map \"http://knowrob.org/kb/u_map.owl#\">\n"
		"]";
	// Create doctype node
	rapidxml::xml_node<> *DoctypeNode = SemMapOwlDoc->allocate_node(rapidxml::node_doctype, "", doctype);
	// Add node to document
	SemMapOwlDoc->append_node(DoctypeNode);

	///////// RDF NODE
	// Add attributes
	SemMapRDFNode = SemMapOwlDoc->allocate_node(rapidxml::node_element, "rdf:RDF", "");
	SemMapRDFNode->append_attribute(SemMapOwlDoc->allocate_attribute("xmlns:computable", "http://knowrob.org/kb/computable.owl#"));
	SemMapRDFNode->append_attribute(SemMapOwlDoc->allocate_attribute("xmlns:swrl", "http://www.w3.org/2003/11/swrl#"));
	SemMapRDFNode->append_attribute(SemMapOwlDoc->allocate_attribute("xmlns:rdf", "http://www.w3.org/1999/02/22-rdf-syntax-ns#"));
	SemMapRDFNode->append_attribute(SemMapOwlDoc->allocate_attribute("xmlns:rdfs", "http://www.w3.org/2000/01/rdf-schema#"));
	SemMapRDFNode->append_attribute(SemMapOwlDoc->allocate_attribute("xmlns:owl", "http://www.w3.org/2002/07/owl#"));
	SemMapRDFNode->append_attribute(SemMapOwlDoc->allocate_attribute("xmlns:knowrob", "http://knowrob.org/kb/knowrob.owl#"));
	SemMapRDFNode->append_attribute(SemMapOwlDoc->allocate_attribute("xmlns:knowrob_u", "http://knowrob.org/kb/knowrob_u.owl#"));
	SemMapRDFNode->append_attribute(SemMapOwlDoc->allocate_attribute("xmlns:u-map", "http://knowrob.org/kb/u_map.owl#"));
	SemMapRDFNode->append_attribute(SemMapOwlDoc->allocate_attribute("xml:base", "http://knowrob.org/kb/u_map.owl#"));

	///////// ONTOLOGY IMPORT		
	// Add parent-child node to RDF
	SemMapRDFNode->append_node(ARSemLogManager::AddParentChildTriple(SemMapOwlDoc,
		"owl:Ontology", "rdf:about", "http://knowrob.org/kb/u_map.owl",
		"owl:imports", "rdf:resource", "package://knowrob_common/owl/knowrob.owl"));

	///////// PROPERTY DEFINITIONS
	// Add Comment
	ARSemLogManager::AddOwlComment(SemMapOwlDoc, SemMapRDFNode, "General Definitions");
	// Obj Property 
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:ObjectProperty", "rdf:about", "&knowrob;describedInMap"));
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:ObjectProperty", "rdf:about", "&knowrob_u;episodeInstance"));
	// Datatype Property
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:DatatypeProperty", "rdf:about", "&knowrob;depthOfObject"));
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:DatatypeProperty", "rdf:about", "&knowrob;heightOfObject"));
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:DatatypeProperty", "rdf:about", "&knowrob;widthOfObject"));
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:DatatypeProperty", "rdf:about", "&knowrob;vectorX"));
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:DatatypeProperty", "rdf:about", "&knowrob;vectorY"));
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:DatatypeProperty", "rdf:about", "&knowrob;vectorZ"));
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:DatatypeProperty", "rdf:about", "&knowrob;pathToCadModel"));

	///////// CLASS DEFINITIONS
	// Add Comment
	ARSemLogManager::AddOwlComment(SemMapOwlDoc, SemMapRDFNode, "Class Definitions");
	// Add class definitions to RDF
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:Class", "rdf:about", "&knowrob;SemanticEnvironmentMap"));
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:Class", "rdf:about", "&knowrob;SemanticMapPerception"));
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:Class", "rdf:about", "&knowrob;TimePoint"));
	SemMapRDFNode->append_node(ARSemLogManager::AddTriple(SemMapOwlDoc, "owl:Class", "rdf:about", "&knowrob;Vector"));


	///////// NAMED INDIVIDUALS

	// Add Comment
	ARSemLogManager::AddOwlComment(SemMapOwlDoc, SemMapRDFNode, "Semantic environment map");
	// Array of properties <Subject, Predicate, Object, Value>
	TArray<OwlTriple> SemMapProperties;
	// Add properties
	SemMapProperties.Add(OwlTriple("rdf:type", "rdf:resource", "&knowrob;SemanticEnvironmentMap", ""));
	SemMapProperties.Add(OwlTriple("knowrob_u:episodeInstance", "rdf:resource", MetaDataIndividual->GetFullName(), ""));
	// Add the semantic map instance
	SemMapRDFNode->append_node(ARSemLogManager::AddEvent(SemMapOwlDoc,
		"owl:NamedIndividual", "rdf:about", "&u-map;" + SemMapUniqueName, SemMapProperties));


	// Add Comment
	ARSemLogManager::AddOwlComment(SemMapOwlDoc, SemMapRDFNode, "Timepoint");
	// Add the timepoint instance
	SemMapRDFNode->append_node(ARSemLogManager::AddParentChildTriple(SemMapOwlDoc,
		"owl:NamedIndividual", "rdf:about", "&u-map;timepoint_0",
		"rdf:type", "rdf:resource", "&knowrob;Timepoint"));



	// Iterate through the actors with unique names and add them to the semantic map
	for (auto CurrActorToUniqueName : ActorToUniqueName)
	{
		// generate uniqe transformation name
		const FString TransfUniqueName = "Transformation_" + ARSemLogManager::RandString(4);

		// loc and quat of the object as strings, change from left hand to right hand coord
		const FVector loc = CurrActorToUniqueName.Key->GetActorLocation() / 100;
		const FString loc_str = FString::SanitizeFloat(loc.X) + " "
			+ FString::SanitizeFloat(-loc.Y) + " "
			+ FString::SanitizeFloat(loc.Z);

		const FQuat quat = CurrActorToUniqueName.Key->GetActorQuat();
		const FString quat_str = FString::SanitizeFloat(quat.W) + " "
			+ FString::SanitizeFloat(-quat.X) + " "
			+ FString::SanitizeFloat(quat.Y) + " "
			+ FString::SanitizeFloat(-quat.Z);

		// Add Comment
		ARSemLogManager::AddOwlComment(SemMapOwlDoc, SemMapRDFNode, "Object: " + CurrActorToUniqueName.Value);

		// Array of properties <Subject, Predicate, Object, Value>
		TArray<OwlTriple> ObjProperties;
		// Add object event properties
		ObjProperties.Add(OwlTriple("rdf:type", "rdf:resource", "&knowrob;" + ActorToClassType[CurrActorToUniqueName.Key], ""));
		ObjProperties.Add(OwlTriple("knowrob:pathToCadModel", "rdf:datatype", "&xsd; string",
			"package://sim/unreal/" + ActorToClassType[CurrActorToUniqueName.Key] + ".dae"));
		ObjProperties.Add(OwlTriple("knowrob:describedInMap", "rdf:resource", "&u-map;" + SemMapUniqueName, ""));
		// Add named individual
		SemMapRDFNode->append_node(ARSemLogManager::AddEvent(SemMapOwlDoc,
			"owl:NamedIndividual", "rdf:about", "&log;" + CurrActorToUniqueName.Value, ObjProperties));


		// Array of properties <Subject, Predicate, Object, Value>
		TArray<OwlTriple> MapPerceptionProperties;
		// Add object event properties
		MapPerceptionProperties.Add(OwlTriple("rdf:type", "rdf:resource", "&knowrob;SemanticMapPerception", ""));
		MapPerceptionProperties.Add(OwlTriple("knowrob:eventOccursAt", "rdf:resource", "&u-map;" + TransfUniqueName, ""));
		MapPerceptionProperties.Add(OwlTriple("knowrob:startTime", "rdf:resource", "&u-map;timepoint_0", ""));
		MapPerceptionProperties.Add(OwlTriple("knowrob:objectActedOn", "rdf:resource", "&log;" + CurrActorToUniqueName.Value, ""));
		// Add named individual
		SemMapRDFNode->append_node(ARSemLogManager::AddEvent(SemMapOwlDoc,
			"owl:NamedIndividual", "rdf:about", "&u-map;SemanticMapPerception_" + ARSemLogManager::RandString(4), MapPerceptionProperties));


		// Array of properties <Subject, Predicate, Object, Value>
		TArray<OwlTriple> TransfProperties;
		// Add object event properties
		TransfProperties.Add(OwlTriple("rdf:type", "rdf:resource", "&knowrob;Transformation", ""));
		TransfProperties.Add(OwlTriple("knowrob:quaternion", "rdf:datatype", "&xsd;string", quat_str));
		TransfProperties.Add(OwlTriple("knowrob:translation", "rdf:datatype", "&xsd;string", loc_str));
		// Add named individual
		SemMapRDFNode->append_node(ARSemLogManager::AddEvent(SemMapOwlDoc,
			"owl:NamedIndividual", "rdf:about", "&u-map;" + TransfUniqueName, TransfProperties));
	}


	///////// ADD RDF TO OWL DOC
	SemMapOwlDoc->append_node(SemMapRDFNode);


	///////// WRITE TO FILE
	// Create string
	std::string RapidXmlString;
	rapidxml::print(std::back_inserter(RapidXmlString), *SemMapOwlDoc, 0 /*rapidxml::print_no_indenting*/);
	// Unreal string
	FString OwlString = UTF8_TO_TCHAR(RapidXmlString.c_str());
	// Terminal debug
	//UE_LOG(LogTemp, Warning, TEXT("%s"), *OwlString);
	// Write to file
	SemanticMapFileHandle->Write((const uint8*)TCHAR_TO_ANSI(*OwlString), OwlString.Len());
}

// Write episode rating
void ARSemLogManager::WriteEpisodeRating()
{
	// generate uniqe name of the episode rating
	const FString EpRatingUniqueName = "Rating_" + ARSemLogManager::RandString(4);

	// Episode rating owl document
	rapidxml::xml_document<>* EpRatingOwlDoc = new rapidxml::xml_document<>();;

	// Episode rating RDF node
	rapidxml::xml_node<>* EpRatingRDFNode;

	// Add episode rating property to the metadata
	MetaDataIndividual->AddProperty("knowrob_u:rating", "rdf:resource", "&log;" + EpRatingUniqueName);

	///////// DOC
	// Create declaration node <?xml version="1.0" encoding="utf-8"?>
	rapidxml::xml_node<> *DeclarationNode = EpRatingOwlDoc->allocate_node(rapidxml::node_declaration);
	// Create attibutes
	DeclarationNode->append_attribute(EpRatingOwlDoc->allocate_attribute("version", "1.0"));
	DeclarationNode->append_attribute(EpRatingOwlDoc->allocate_attribute("encoding", "utf-8"));
	// Add node to document
	EpRatingOwlDoc->append_node(DeclarationNode);

	///////// DOCTYPE
	// Doctype text
	const char* doctype = "rdf:RDF[ \n"
		"\t<!ENTITY rdf \"http://www.w3.org/1999/02/22-rdf-syntax-ns\">\n"
		"\t<!ENTITY rdfs \"http://www.w3.org/2000/01/rdf-schema\">\n"
		"\t<!ENTITY owl \"http://www.w3.org/2002/07/owl\">\n"
		"\t<!ENTITY xsd \"http://www.w3.org/2001/XMLSchema#\">\n"
		"\t<!ENTITY knowrob \"http://knowrob.org/kb/knowrob.owl#\">\n"
		"\t<!ENTITY knowrob_u \"http://knowrob.org/kb/knowrob_u.owl#\">\n"
		"\t<!ENTITY log \"http://knowrob.org/kb/unreal_log.owl#\">\n"
		"]";
	// Create doctype node
	rapidxml::xml_node<> *DoctypeNode = EpRatingOwlDoc->allocate_node(rapidxml::node_doctype, "", doctype);
	// Add node to document
	EpRatingOwlDoc->append_node(DoctypeNode);

	//TODO use lambda
	///////// RDF NODE
	// Add attributes
	EpRatingRDFNode = EpRatingOwlDoc->allocate_node(rapidxml::node_element, "rdf:RDF", "");

	EpRatingRDFNode->append_attribute(EpRatingOwlDoc->allocate_attribute("xmlns:computable", "http://knowrob.org/kb/computable.owl#"));
	EpRatingRDFNode->append_attribute(EpRatingOwlDoc->allocate_attribute("xmlns:swrl", "http://www.w3.org/2003/11/swrl#"));
	EpRatingRDFNode->append_attribute(EpRatingOwlDoc->allocate_attribute("xmlns:rdf", "http://www.w3.org/1999/02/22-rdf-syntax-ns#"));
	EpRatingRDFNode->append_attribute(EpRatingOwlDoc->allocate_attribute("xmlns:rdfs", "http://www.w3.org/2000/01/rdf-schema#"));
	EpRatingRDFNode->append_attribute(EpRatingOwlDoc->allocate_attribute("xmlns:owl", "http://www.w3.org/2002/07/owl#"));
	EpRatingRDFNode->append_attribute(EpRatingOwlDoc->allocate_attribute("xmlns:knowrob", "http://knowrob.org/kb/knowrob.owl#"));
	EpRatingRDFNode->append_attribute(EpRatingOwlDoc->allocate_attribute("xmlns:knowrob_u", "http://knowrob.org/kb/knowrob_u.owl#"));
	EpRatingRDFNode->append_attribute(EpRatingOwlDoc->allocate_attribute("xml:base", "http://knowrob.org/kb/unreal_log.owl#"));

	///////// ONTOLOGY IMPORT		
	// Add parent-child node to RDF
	EpRatingRDFNode->append_node(ARSemLogManager::AddParentChildTriple(EpRatingOwlDoc,
		"owl:Ontology", "rdf:about", "http://knowrob.org/kb/u_map.owl",
		"owl:imports", "rdf:resource", "package://knowrob_common/owl/knowrob.owl"));

	///////// PROPERTY DEFINITIONS
	// Add Comment
	ARSemLogManager::AddOwlComment(EpRatingOwlDoc, EpRatingRDFNode, "General Definitions");
	// Obj Property 
	EpRatingRDFNode->append_node(ARSemLogManager::AddTriple(EpRatingOwlDoc, "owl:ObjectProperty", "rdf:about", "&knowrob_u;episodeInstance"));
	EpRatingRDFNode->append_node(ARSemLogManager::AddTriple(EpRatingOwlDoc, "owl:ObjectProperty", "rdf:about", "&knowrob_u;ratingOf"));
	EpRatingRDFNode->append_node(ARSemLogManager::AddTriple(EpRatingOwlDoc, "owl:ObjectProperty", "rdf:about", "&knowrob_u;ratingScore"));

	///////// CLASS DEFINITIONS
	// Add Comment
	ARSemLogManager::AddOwlComment(EpRatingOwlDoc, EpRatingRDFNode, "Class Definitions");
	// Add class definitions to RDF
	EpRatingRDFNode->append_node(ARSemLogManager::AddTriple(EpRatingOwlDoc, "owl:Class", "rdf:about", "&knowrob_u;Rating"));
	EpRatingRDFNode->append_node(ARSemLogManager::AddTriple(EpRatingOwlDoc, "owl:Class", "rdf:about", "&knowrob_u;UserRating"));
	EpRatingRDFNode->append_node(ARSemLogManager::AddTriple(EpRatingOwlDoc, "owl:Class", "rdf:about", "&knowrob_u;RobotRating"));


	///////// NAMED INDIVIDUALS
	// Add Comment
	ARSemLogManager::AddOwlComment(EpRatingOwlDoc, EpRatingRDFNode, "Rating individual");
	// Array of properties <Subject, Predicate, Object, Value>
	TArray<OwlTriple> EpRatingProperties;
	// Add properties
	EpRatingProperties.Add(OwlTriple("rdf:type", "rdf:resource", "&knowrob_u;Rating", ""));
	EpRatingProperties.Add(OwlTriple("knowrob_u:episodeInstance", "rdf:resource", MetaDataIndividual->GetFullName(), ""));
	// Add the episode rating instance
	EpRatingRDFNode->append_node(ARSemLogManager::AddEvent(EpRatingOwlDoc,
		"owl:NamedIndividual", "rdf:about", "&log;" + EpRatingUniqueName, EpRatingProperties));


	///////// USER RATING INDIVIDUAL
	// generate uniqe name of user rating
	const FString UserRatingUniqueName = "UserRating_" + ARSemLogManager::RandString(4);

	// Add Comment
	ARSemLogManager::AddOwlComment(EpRatingOwlDoc, EpRatingRDFNode, "Object: " + UserRatingUniqueName);

	// Array of properties <Subject, Predicate, Object, Value>
	TArray<OwlTriple> UserRatingProperties;
	// Add object event properties
	UserRatingProperties.Add(OwlTriple("rdf:type", "rdf:resource", "&knowrob_u;UserRating", ""));
	UserRatingProperties.Add(OwlTriple("knowrob_u:ratingScore", "rdf:datatype", "&xsd; float",
		"10"));
	UserRatingProperties.Add(OwlTriple("knowrob_u:ratingOf", "rdf:resource", "&log;" + EpRatingUniqueName, ""));
	// Add named individual
	EpRatingRDFNode->append_node(ARSemLogManager::AddEvent(EpRatingOwlDoc,
		"owl:NamedIndividual", "rdf:about", "&log;" + UserRatingUniqueName, UserRatingProperties));


	///////// ADD RDF TO OWL DOC
	EpRatingOwlDoc->append_node(EpRatingRDFNode);


	///////// WRITE TO FILE
	// Create string
	std::string RapidXmlString;
	rapidxml::print(std::back_inserter(RapidXmlString), *EpRatingOwlDoc, 0 /*rapidxml::print_no_indenting*/);
	// Unreal string
	FString OwlString = UTF8_TO_TCHAR(RapidXmlString.c_str());
	// Terminal debug
	//UE_LOG(LogTemp, Warning, TEXT("%s"), *OwlString);
	// Write to file
	EpRatingFileHandle->Write((const uint8*)TCHAR_TO_ANSI(*OwlString), OwlString.Len());
}

// Write the owl events to file
void ARSemLogManager::WriteEvents()
{
	// Events owl document
	rapidxml::xml_document<>* EventsOwlDoc = new rapidxml::xml_document<>();

	// Events RDF node
	rapidxml::xml_node<>* EvRDFNode;

	///////// DOC
	// Create declaration node <?xml version="1.0" encoding="utf-8"?>
	rapidxml::xml_node<> *DeclarationNode = EventsOwlDoc->allocate_node(rapidxml::node_declaration);
	// Create attibutes
	DeclarationNode->append_attribute(EventsOwlDoc->allocate_attribute("version", "1.0"));
	DeclarationNode->append_attribute(EventsOwlDoc->allocate_attribute("encoding", "utf-8"));
	// Add node to document
	EventsOwlDoc->append_node(DeclarationNode);

	///////// DOCTYPE
	// Doctype text
	const char* doctype = "rdf:RDF[ \n"
		"\t<!ENTITY owl \"http://www.w3.org/2002/07/owl#\">\n"
		"\t<!ENTITY xsd \"http://www.w3.org/2001/XMLSchema#\">\n"
		"\t<!ENTITY knowrob \"http://knowrob.org/kb/knowrob.owl#\">\n"
		"\t<!ENTITY rdfs \"http://www.w3.org/2000/01/rdf-schema#\">\n"
		"\t<!ENTITY rdf \"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">\n"
		"\t<!ENTITY u-map \"http://knowrob.org/kb/u_map.owl#\">\n"
		"\t<!ENTITY log \"http://knowrob.org/kb/unreal_log.owl#\">\n"
		"\t<!ENTITY knowrob_u \"http://knowrob.org/kb/knowrob_u.owl#\">\n"
		"]";
	// Create doctype node
	rapidxml::xml_node<> *DoctypeNode = EventsOwlDoc->allocate_node(rapidxml::node_doctype, "", doctype);
	// Add node to document
	EventsOwlDoc->append_node(DoctypeNode);

	///////// RDF NODE
	// Add attributes
	EvRDFNode = EventsOwlDoc->allocate_node(rapidxml::node_element, "rdf:RDF", "");
	EvRDFNode->append_attribute(EventsOwlDoc->allocate_attribute("xmlns", "http://knowrob.org/kb/unreal_log.owl#"));
	EvRDFNode->append_attribute(EventsOwlDoc->allocate_attribute("xml:base", "http://knowrob.org/kb/unreal_log.owl#"));
	EvRDFNode->append_attribute(EventsOwlDoc->allocate_attribute("xmlns:owl", "http://www.w3.org/2002/07/owl#"));
	EvRDFNode->append_attribute(EventsOwlDoc->allocate_attribute("xmlns:xsd", "http://www.w3.org/2001/XMLSchema"));
	EvRDFNode->append_attribute(EventsOwlDoc->allocate_attribute("xmlns:knowrob", "http://knowrob.org/kb/knowrob.owl#"));
	EvRDFNode->append_attribute(EventsOwlDoc->allocate_attribute("xmlns:rdfs", "http://www.w3.org/2000/01/rdf-schema#"));
	EvRDFNode->append_attribute(EventsOwlDoc->allocate_attribute("xmlns:rdf", "http://www.w3.org/1999/02/22-rdf-syntax-ns#"));
	EvRDFNode->append_attribute(EventsOwlDoc->allocate_attribute("xmlns:log", "http://knowrob.org/kb/unreal_log.owl#"));
	EvRDFNode->append_attribute(EventsOwlDoc->allocate_attribute("xmlns:u-map", "http://knowrob.org/kb/u_map.owl#"));
	EvRDFNode->append_attribute(EventsOwlDoc->allocate_attribute("xmlns:knowrob_u", "http://knowrob.org/kb/knowrob_u.owl#"));

	///////// ONTOLOGY IMPORT	
	ARSemLogManager::AddOwlComment(EventsOwlDoc, EvRDFNode, "Ontoligies");
	// Add parent-child node to RDF
	EvRDFNode->append_node(ARSemLogManager::AddParentChildTriple(EventsOwlDoc,
		"owl:Ontology", "rdf:about", "http://knowrob.org/kb/unreal_log.owl",
		"owl:imports", "rdf:resource", "package://knowrob_common/owl/knowrob.owl"));

	///////// PROPERTY DEFINITIONS
	// Add Comment
	ARSemLogManager::AddOwlComment(EventsOwlDoc, EvRDFNode, "Property Definitions");
	// Add property definitions to RDF
	for (auto PorpertyIter : PropertyDefinitions)
	{
		EvRDFNode->append_node(ARSemLogManager::AddTriple(EventsOwlDoc, "owl:ObjectProperty", "rdf:about", PorpertyIter));
	}

	///////// CLASS DEFINITIONS
	// Add Comment
	ARSemLogManager::AddOwlComment(EventsOwlDoc, EvRDFNode, "Class Definitions");
	// Add class definitions to RDF
	for (auto ClassIter : ClassDefinitions)
	{
		EvRDFNode->append_node(ARSemLogManager::AddTriple(EventsOwlDoc, "owl:Class", "rdf:about", ClassIter));
	}

	///////// EVENT INDIVIDUALS
	// Add Comment
	ARSemLogManager::AddOwlComment(EventsOwlDoc, EvRDFNode, "Event Individuals");
	// Add event individuals to RDF
	for (auto EventIter : EventIndividuals)
	{
		EvRDFNode->append_node(ARSemLogManager::AddEvent(EventsOwlDoc,
			"owl:NamedIndividual", "rdf:about", EventIter.GetFullName(), EventIter.GetProperties()));
	}

	///////// OBJECT INDIVIDUALS
	// Add Comment
	ARSemLogManager::AddOwlComment(EventsOwlDoc, EvRDFNode, "Object Individuals");
	// Add object individuals to RDF
	for (auto ObjectIter : ObjectIndividuals)
	{
		// ObjectUnique(Get<0>) name appended with namespace "&log", Class(Get<1>) is already appended
		EvRDFNode->append_node(ARSemLogManager::AddParentChildTriple(EventsOwlDoc,
			"owl:NamedIndividual", "rdf:about", FString("&log;") + ObjectIter.Get<0>(),
			"rdf:type", "rdf:resource", ObjectIter.Get<1>()));
	};

	///////// TIMEPOINT INDIVIDUALS
	// Add Comment
	ARSemLogManager::AddOwlComment(EventsOwlDoc, EvRDFNode, "Timepoint Individuals");
	// Add time individuals to RDF
	for (auto TimepointIter : TimepointIndividuals)
	{
		EvRDFNode->append_node(ARSemLogManager::AddParentChildTriple(EventsOwlDoc,
			"owl:NamedIndividual", "rdf:about", TimepointIter,
			"rdf:type", "rdf:resource", "&knowrob;TimePoint"));
	};


	///////// METADATA INDIVIDUAL
	// Add Comment
	ARSemLogManager::AddOwlComment(EventsOwlDoc, EvRDFNode, "Meta Data Individuals");
	// Add metadata to RDF
	EvRDFNode->append_node(ARSemLogManager::AddEvent(EventsOwlDoc,
		"owl:NamedIndividual", "rdf:about", MetaDataIndividual->GetFullName(), MetaDataIndividual->GetProperties()));

	///////// ADD RDF TO OWL DOC
	EventsOwlDoc->append_node(EvRDFNode);


	///////// WRITE TO FILE
	// Create string
	std::string RapidXmlString;
	rapidxml::print(std::back_inserter(RapidXmlString), *EventsOwlDoc, 0 /*rapidxml::print_no_indenting*/);
	// Unreal string
	FString OwlString = UTF8_TO_TCHAR(RapidXmlString.c_str());
	// Terminal debug
	//UE_LOG(LogTemp, Warning, TEXT("%s"), *OwlString);
	// Write to file
	EventsFileHandle->Write((const uint8*)TCHAR_TO_ANSI(*OwlString), OwlString.Len());
}

// Write metadata owl
void ARSemLogManager::AddMetaData()
{
	// Add subactions
	for (auto EventItr : EventIndividuals)
	{
		MetaDataIndividual->AddProperty("knowrob:subAction", "rdf:resource", EventItr.GetFullName());
	}

	// Set EndTime
	FString EndTime = "&log;timepoint_" + FString::SanitizeFloat(ElapsedTime);
	MetaDataIndividual->AddProperty("knowrob:endTime", "rdf:resource", EndTime);
	TimepointIndividuals.AddUnique(EndTime);

	// Set end time and as finished
	MetaDataIndividual->EndTime = ElapsedTime;
	MetaDataIndividual->SetFinshed(true);

	// Add episode info
	MetaDataIndividual->AddProperty("knowrob:experiment", "rdf:datatype", "&xsd;string", FString("u_") + FString::FromInt(EpisodeNr));
	MetaDataIndividual->AddProperty("knowrob:experimentName", "rdf:datatype", "&xsd;string", EpisodeDirName);
}

// Write timelines
void ARSemLogManager::WriteTimelines()
{
	UE_LOG(LogTemp, Warning, TEXT("Writing timelines.."));

	FString TimelineStr = "<html>\n"
		"<script type=\"text/javascript\" src=\"https://www.google.com/jsapi?autoload={'modules':[{'name':'visualization',\n"
		"       'version':'1','packages':['timeline']}]}\"></script>\n"
		"<script type=\"text/javascript\">\n"
		"google.setOnLoadCallback(drawChart);\n"
		"\n"
		"function drawChart() {\n"
		"  var container = document.getElementById('sim_timeline_ex');\n"
		"\n"
		"  var chart = new google.visualization.Timeline(container);\n"
		"\n"
		"  var dataTable = new google.visualization.DataTable();\n"
		"\n"
		"  dataTable.addColumn({ type: 'string', id: 'Event' });\n"
		"  dataTable.addColumn({ type: 'number', id: 'Start' });\n"
		"  dataTable.addColumn({ type: 'number', id: 'End' });\n"
		"\n"
		"  dataTable.addRows([\n";

	// Add events to the timelines
	for (auto EventIter : EventIndividuals)
	{
		if (!EventIter.GetContext().IsEmpty())
		{
			TimelineStr.Append("    [ '" + EventIter.GetContext() + "', "
				+ FString::SanitizeFloat(EventIter.StartTime * 1000) + ", "
				+ FString::SanitizeFloat(EventIter.EndTime * 1000) + "],\n");
		}
	}

	TimelineStr.Append("  ]); \n"
		"\n"
		"  chart.draw(dataTable);\n"
		"}\n"
		"</script>\n"
		"<div id=\"sim_timeline_ex\" style=\"width: 1300px; height: 900px;\"></div>\n"
		"\n"
		"</html>"
	);

	TimelineFileHandle->Write((const uint8*)TCHAR_TO_ANSI(*TimelineStr), TimelineStr.Len());
}

// Start touching event
FOwlEvent ARSemLogManager::StartTouchingSituation(AActor* Trigger, AActor* OtherActor)
{
	// Create touch situation at the current time
	FOwlEvent TouchEvent("&log;", "TouchingSituation", ARSemLogManager::RandStringUnderscore(4), ElapsedTime);

	// Add event type property
	TouchEvent.AddProperty("rdf:type", "rdf:resource", "&knowrob_u;TouchingSituation");

	// Set task context
	const FString TaskContext("Contact-" + ActorToUniqueName[Trigger->GetAttachParentActor()] + "-" + ActorToUniqueName[OtherActor]);
	// Add context to the event, and as property
	TouchEvent.SetContext(TaskContext);
	TouchEvent.AddProperty("knowrob:taskContext", "rdf:datatype", "&xsd;string", TaskContext);

	// Set event StartTime and add it as property
	const FString StartTime = "&log;timepoint_" + FString::SanitizeFloat(TouchEvent.StartTime);
	TouchEvent.AddProperty("knowrob:startTime", "rdf:resource", StartTime);

	// Make sure the time individual is not repeating
	TimepointIndividuals.AddUnique(StartTime);

	// Add objects in contact
	TouchEvent.AddProperty("knowrob_u:inContact", "rdf:resource", FString("&log;") + ActorToUniqueName[Trigger->GetAttachParentActor()]);
	TouchEvent.AddProperty("knowrob_u:inContact", "rdf:resource", FString("&log;") + ActorToUniqueName[OtherActor]);

	return TouchEvent;
}

// Convert FString to char* (TODO report issue)
char* ARSemLogManager::FStrToChar(const FString FStr)
{
	const std::string str = TCHAR_TO_UTF8(*FStr);
	char *cstr = new char[str.length() + 1];
	strcpy(cstr, str.c_str());
	//delete[] cstr; // TODO check if memory leaks happen, or rapidxml eventually clears memory
	return cstr;
}

// Generate a random string
FString ARSemLogManager::RandString(const int32 Length)
{
	auto RandChar = []() -> char
	{
		const char CharSet[] =
			"0123456789"
			"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
			"abcdefghijklmnopqrstuvwxyz";
		const size_t MaxIndex = (sizeof(CharSet) - 1);
		return CharSet[rand() % MaxIndex];
	};
	std::string RandString(Length, 0);
	std::generate_n(RandString.begin(), Length, RandChar);

	// Return as FString
	return FString(RandString.c_str());
}

// Generate a random string with an underscore
FString ARSemLogManager::RandStringUnderscore(const int32 Length)
{
	// Return rand string with underscore
	return "_" + ARSemLogManager::RandString(Length);
}

// Create an OWL triple
rapidxml::xml_node<>* ARSemLogManager::AddTriple(
	rapidxml::xml_document<>* OwlDoc,
	const FString Subject,
	const FString Predicate,
	const FString Object,
	const FString Value)
{
	rapidxml::xml_node<> *Triple = OwlDoc->allocate_node(rapidxml::node_element, ARSemLogManager::FStrToChar(Subject), ARSemLogManager::FStrToChar(Value));
	Triple->append_attribute(OwlDoc->allocate_attribute(ARSemLogManager::FStrToChar(Predicate), ARSemLogManager::FStrToChar(Object)));
	return Triple;
}

// Create an OWL triple with a child
rapidxml::xml_node<>* ARSemLogManager::AddParentChildTriple(
	rapidxml::xml_document<>* OwlDoc,
	const FString ParentSubject,
	const FString ParentPredicate,
	const FString ParentObject,
	const FString ChildSubject,
	const FString ChildPredicate,
	const FString ChildObject,
	const FString ChildValue)
{
	rapidxml::xml_node<> *Parent = ARSemLogManager::AddTriple(OwlDoc, ParentSubject, ParentPredicate, ParentObject);
	rapidxml::xml_node<> *Child = ARSemLogManager::AddTriple(OwlDoc, ChildSubject, ChildPredicate, ChildObject, ChildValue);
	// Append child node
	Parent->append_node(Child);
	return Parent;
}

// Create an OWL event with a property
rapidxml::xml_node<>* ARSemLogManager::AddEvent(
	rapidxml::xml_document<>* OwlDoc,
	const FString Subject,
	const FString Predicate,
	const FString Object,
	const OwlTriple Property)
{
	// Parent node
	rapidxml::xml_node<> *Parent = ARSemLogManager::AddTriple(OwlDoc, Subject, Predicate, Object);

	// Add Subject, Predicate, Object, Value (if available)
	Parent->append_node(ARSemLogManager::AddTriple(OwlDoc, Property.Get<0>(), Property.Get<1>(), Property.Get<2>(), Property.Get<3>()));

	return Parent;
}

// Create an OWL event with properties
rapidxml::xml_node<>* ARSemLogManager::AddEvent(
	rapidxml::xml_document<>* OwlDoc,
	const FString Subject,
	const FString Predicate,
	const FString Object,
	const TArray<OwlTriple>& Properties)
{
	// Parent node
	rapidxml::xml_node<> *Parent = ARSemLogManager::AddTriple(OwlDoc, Subject, Predicate, Object);

	// Iterate the properties
	for (auto PropertyIter : Properties)
	{
		// Add Subject, Predicate, Object, Value (if available)
		Parent->append_node(ARSemLogManager::AddTriple(OwlDoc, PropertyIter.Get<0>(), PropertyIter.Get<1>(), PropertyIter.Get<2>(), PropertyIter.Get<3>()));
	};
	return Parent;
}

// Insert comment into the OWL doc
void ARSemLogManager::AddOwlComment(
	rapidxml::xml_document<>* OwlDoc,
	rapidxml::xml_node<>* RDFNode,
	const FString CommentStr)
{
	// Comment
	rapidxml::xml_node<> *CommentNode = OwlDoc->allocate_node(rapidxml::node_comment, "", ARSemLogManager::FStrToChar(CommentStr));
	// Add comment to document
	RDFNode->append_node(CommentNode);
}

// Add Json Actor Member
rapidjson::Value ARSemLogManager::AddJsonActor(
	AActor* Actor,
	const FString ClassType,
	rapidjson::Document::AllocatorType& JsonAllocator)
{
	// Json object
	//TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);

	// Value array
	//TArray< TSharedPtr<FJsonValue> > ValueArray;

	//// Create two values and add them to the array
	//TSharedPtr<FJsonValue> Value1 = MakeShareable(new FJsonValueString("value1"));
	//ValueArray.Add(Value1);
	//TSharedPtr<FJsonValue> Value2 = MakeShareable(new FJsonValueString("value2"));
	//ValueArray.Add(Value2);

	//// Add the array to the Json object
	//JsonObject->SetArrayField("array", ValueArray);

	//FString OutputString;
	//TSharedRef< TJsonWriter<> > Writer = TJsonWriterFactory<>::Create(&OutputString);
	//FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

	//UE_LOG(LogTemp, Warning, TEXT("json test: %s"), *OutputString);


	//// create Json values array
	//TArray< TSharedPtr<FJsonValue> > ObjArray;

	//// create a Json object and add a string field
	//TSharedPtr<FJsonObject> JsonObj = MakeShareable(new FJsonObject);
	//JsonObj->SetStringField("Abc", "AbcValue");
	//// create a Json value object to hold your Json object
	//TSharedRef< FJsonValueObject > JsonValue = MakeShareable(new FJsonValueObject(JsonObj));


	//// create a Json object and add a string field
	//TSharedPtr<FJsonObject> JsonObj2 = MakeShareable(new FJsonObject);
	//JsonObj2->SetStringField("Abc2", "AbcValue2");
	//// create a Json value object to hold your Json object
	//TSharedRef< FJsonValueObject > JsonValue2 = MakeShareable(new FJsonValueObject(JsonObj2));

	//// add the object to the array
	//ObjArray.Add(JsonValue);
	//ObjArray.Add(JsonValue2);

	//// THIS should work
	//JsonObject->SetArrayField("array", ObjArray);

	//FString OutputString;
	//TSharedRef< TJsonWriter<> > Writer = TJsonWriterFactory<>::Create(&OutputString);
	//FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

	//UE_LOG(LogTemp, Warning, TEXT("json test: %s"), *OutputString);

	// Current json actor obj
	rapidjson::Value ActorJsonObj;
	ActorJsonObj.SetObject();

	// Actor name
	rapidjson::Value NameJsonVal;
	NameJsonVal.SetString(TCHAR_TO_ANSI(*ActorToUniqueName[Actor]), JsonAllocator);
	// Add actor name
	ActorJsonObj.AddMember("name", NameJsonVal, JsonAllocator);

	// Actor class type
	rapidjson::Value ClassTypeJsonVal;
	ClassTypeJsonVal.SetString(TCHAR_TO_ANSI(*ClassType), JsonAllocator);
	// Add actor class type
	ActorJsonObj.AddMember("class_type", ClassTypeJsonVal, JsonAllocator);

	// Get actor position
	FVector Loc = Actor->GetActorLocation() * 0.01;

	// Create json position
	rapidjson::Value PosJsonVal;
	PosJsonVal.SetObject();
	PosJsonVal.AddMember("x", Loc.X, JsonAllocator);
	PosJsonVal.AddMember("y", -Loc.Y, JsonAllocator); // switch from left to right handed
	PosJsonVal.AddMember("z", Loc.Z, JsonAllocator);

	// Add json position 
	ActorJsonObj.AddMember("pos", PosJsonVal, JsonAllocator);

	// Get actor rotation
	FQuat Quat = Actor->GetActorQuat();

	// Create json rotation
	rapidjson::Value RotJsonVal;
	RotJsonVal.SetObject();
	RotJsonVal.AddMember("w", Quat.W, JsonAllocator);
	RotJsonVal.AddMember("x", -Quat.X, JsonAllocator); // switch from left to right handed
	RotJsonVal.AddMember("y", Quat.Y, JsonAllocator);
	RotJsonVal.AddMember("z", -Quat.Z, JsonAllocator); // switch from left to right handed

													   // Add json rot
	ActorJsonObj.AddMember("rot", RotJsonVal, JsonAllocator);

	// Return the json actor
	return ActorJsonObj;
}

// Add Json ActorComponent Member
rapidjson::Value ARSemLogManager::AddJsonComponent(
	UPrimitiveComponent* Component,
	const FString ClassType,
	rapidjson::Document::AllocatorType& JsonAllocator)
{
	// Current json actor obj
	rapidjson::Value ActorJsonObj;
	ActorJsonObj.SetObject();

	// Actor name
	rapidjson::Value NameJsonVal;
	NameJsonVal.SetString(TCHAR_TO_ANSI(*Component->GetName()), JsonAllocator);
	// Add actor name
	ActorJsonObj.AddMember("name", NameJsonVal, JsonAllocator);

	// Actor class type
	rapidjson::Value ClassTypeJsonVal;
	ClassTypeJsonVal.SetString(TCHAR_TO_ANSI(*ClassType), JsonAllocator);
	// Add actor class type
	ActorJsonObj.AddMember("class_type", ClassTypeJsonVal, JsonAllocator);

	// Get actor position
	FVector Loc = Component->GetComponentLocation() * 0.01;

	// Create json position
	rapidjson::Value PosJsonVal;
	PosJsonVal.SetObject();
	PosJsonVal.AddMember("x", Loc.X, JsonAllocator);
	PosJsonVal.AddMember("y", -Loc.Y, JsonAllocator); // switch from left to right handed
	PosJsonVal.AddMember("z", Loc.Z, JsonAllocator);

	// Add json position 
	ActorJsonObj.AddMember("pos", PosJsonVal, JsonAllocator);

	// Get actor rotation
	FQuat Quat = Component->GetComponentQuat();

	// Create json rotation
	rapidjson::Value RotJsonVal;
	RotJsonVal.SetObject();
	RotJsonVal.AddMember("w", Quat.W, JsonAllocator);
	RotJsonVal.AddMember("x", -Quat.X, JsonAllocator); // switch from left to right handed
	RotJsonVal.AddMember("y", Quat.Y, JsonAllocator);
	RotJsonVal.AddMember("z", -Quat.Z, JsonAllocator); // switch from left to right handed

													   // Add json rot
	ActorJsonObj.AddMember("rot", RotJsonVal, JsonAllocator);

	// Return the json actor
	return ActorJsonObj;
}

// Add Json Bone Member
rapidjson::Value ARSemLogManager::AddJsonBone(
	USkeletalMeshComponent* SkeletalComponent,
	const FName BoneName,
	rapidjson::Document::AllocatorType& JsonAllocator)
{
	// Current bone json obj
	rapidjson::Value BoneJsonObj;
	BoneJsonObj.SetObject();

	// Bone name
	rapidjson::Value NameJsonVal;
	NameJsonVal.SetString(TCHAR_TO_ANSI(*BoneName.ToString()), JsonAllocator);
	// Add Bone name
	BoneJsonObj.AddMember("name", NameJsonVal, JsonAllocator);

	// Get bone position
	FVector BoneLoc = SkeletalComponent->GetBoneLocation(BoneName) * 0.01;

	// Create json position
	rapidjson::Value PosJsonVal;
	PosJsonVal.SetObject();
	PosJsonVal.AddMember("x", BoneLoc.X, JsonAllocator);
	PosJsonVal.AddMember("y", -BoneLoc.Y, JsonAllocator); // switch from left to right handed
	PosJsonVal.AddMember("z", BoneLoc.Z, JsonAllocator);

	// Add json position 
	BoneJsonObj.AddMember("pos", PosJsonVal, JsonAllocator);

	// Get bone rotation
	FQuat BoneQuat = SkeletalComponent->GetBoneQuaternion(BoneName);

	// Create json rotation
	rapidjson::Value RotJsonVal;
	RotJsonVal.SetObject();
	RotJsonVal.AddMember("w", BoneQuat.W, JsonAllocator);
	RotJsonVal.AddMember("x", -BoneQuat.X, JsonAllocator); // switch from left to right handed
	RotJsonVal.AddMember("y", BoneQuat.Y, JsonAllocator);
	RotJsonVal.AddMember("z", -BoneQuat.Z, JsonAllocator); // switch from left to right handed

														   // Add json rot
	BoneJsonObj.AddMember("rot", RotJsonVal, JsonAllocator);

	// Return the json bone
	return BoneJsonObj;
}

// Right hand grasp blueprint callable function
void ARSemLogManager::RightGraspLog(AActor* OtherActor)
{
	// Return if the grasped actor has no unique name set
	if (!ActorToUniqueName.Contains(OtherActor))
	{
		UE_LOG(LogTemp, Error, TEXT("Actor %s has no unique name set up at during **RIGHT GRASP**.. skipping"), *OtherActor->GetName());
		return;
	}

	// Create grasp event
	RightGraspEvent = new FOwlEvent("&log;", "GraspingSomething", ARSemLogManager::RandStringUnderscore(4), ElapsedTime);

	// Add event type property
	RightGraspEvent->AddProperty("rdf:type", "rdf:resource", "&knowrob;GraspingSomething");

	// Set task context
	const FString TaskContext("Grasp_" + ActorToUniqueName[OtherActor]);
	// Add context to the event, and as property
	RightGraspEvent->SetContext(TaskContext);
	RightGraspEvent->AddProperty("knowrob:taskContext", "rdf:datatype", "&xsd;string", "Grasp_" + ActorToUniqueName[OtherActor]);

	// Add StartTime
	const FString StartTime = "&log;timepoint_" + FString::SanitizeFloat(ElapsedTime);
	RightGraspEvent->AddProperty("knowrob:startTime", "rdf:resource", StartTime);
	TimepointIndividuals.AddUnique(StartTime);

	// Add object acted on
	RightGraspEvent->AddProperty("knowrob:objectActedOn", "rdf:resource", FString("&log;") + ActorToUniqueName[OtherActor]);

	UE_LOG(LogTemp, Warning, TEXT("\tRight hand grasp: %s"), *OtherActor->GetName());
}

// Right hand release blueprint callable function
void ARSemLogManager::RightReleaseLog()
{
	// Add EndTime
	RightGraspEvent->EndTime = ElapsedTime;
	const FString EndTime = "&log;timepoint_" + FString::SanitizeFloat(ElapsedTime);
	RightGraspEvent->AddProperty("knowrob:endTime", "rdf:resource", EndTime);
	TimepointIndividuals.AddUnique(EndTime);

	// Mark event as finished
	RightGraspEvent->SetFinshed(true);

	// Add finished grasp event to the event individuals array
	EventIndividuals.Add(*RightGraspEvent);

	// Delete current grasp pointer
	delete RightGraspEvent;
	RightGraspEvent = nullptr;
}

// Left hand grasp blueprint callable function
void ARSemLogManager::LeftGraspLog(AActor* OtherActor)
{
	// Return if the grasped actor has no unique name set
	if (!ActorToUniqueName.Contains(OtherActor))
	{
		UE_LOG(LogTemp, Error, TEXT("Actor %s has no unique name set up at during **LEFT GRASP**.. skipping"), *OtherActor->GetName());
		return;
	}

	// Create grasp event
	LeftGraspEvent = new FOwlEvent("&log;", "GraspingSomething", ARSemLogManager::RandStringUnderscore(4), ElapsedTime);

	// Add event type property
	LeftGraspEvent->AddProperty("rdf:type", "rdf:resource", "&knowrob;GraspingSomething");

	// Set task context
	const FString TaskContext("Grasp_" + ActorToUniqueName[OtherActor]);
	// Add context to the event, and as property
	LeftGraspEvent->SetContext(TaskContext);
	LeftGraspEvent->AddProperty("knowrob:taskContext", "rdf:datatype", "&xsd;string", "Grasp_" + ActorToUniqueName[OtherActor]);

	// Add StartTime
	const FString StartTime = "&log;timepoint_" + FString::SanitizeFloat(ElapsedTime);
	LeftGraspEvent->AddProperty("knowrob:startTime", "rdf:resource", StartTime);
	TimepointIndividuals.AddUnique(StartTime);

	// Add object acted on
	LeftGraspEvent->AddProperty("knowrob:objectActedOn", "rdf:resource", FString("&log;") + ActorToUniqueName[OtherActor]);

	UE_LOG(LogTemp, Warning, TEXT("\tLeft hand grasp: %s"), *OtherActor->GetName());
}

// Left hand release blueprint callable function
void ARSemLogManager::LeftReleaseLog()
{
	// Add EndTime
	LeftGraspEvent->EndTime = ElapsedTime;
	const FString EndTime = "&log;timepoint_" + FString::SanitizeFloat(ElapsedTime);
	LeftGraspEvent->AddProperty("knowrob:endTime", "rdf:resource", EndTime);
	TimepointIndividuals.AddUnique(EndTime);

	// Mark event as finished
	LeftGraspEvent->SetFinshed(true);

	// Add finished grasp event to the event individuals array
	EventIndividuals.Add(*LeftGraspEvent);

	// Delete current grasp pointer
	delete LeftGraspEvent;
	LeftGraspEvent = nullptr;
}

// Bound function with the overlap begin event
void ARSemLogManager::OnIslandTriggerOverlapBegin(AActor* OverlappedActor, AActor* OtherActor)
{
	// Return if the actor in contact has no unique name set
	if (!ActorToUniqueName.Contains(OtherActor))
	{
		UE_LOG(LogTemp, Error, TEXT("%s: Actor %s has no unique name set up at contact \t**BEGIN**.. skipping"), *IslandTriggerArea->GetName(), *OtherActor->GetName());
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("\t Contact-%s-%s \t**BEGIN**"), *IslandTriggerArea->GetName(), *OtherActor->GetName());
	// Island touch event
	FOwlEvent IslandTouchEvent = ARSemLogManager::StartTouchingSituation(IslandTriggerArea, OtherActor);

	// Add event to the unfinished touch events (key is the unique name of the trigger + uniqe name of the contact actor
	UnfinishedTouchEvents.Add(ActorToUniqueName[IslandTriggerArea->GetAttachParentActor()] + ActorToUniqueName[OtherActor], IslandTouchEvent);
}

// Bound function with the overlap end event
void ARSemLogManager::OnIslandTriggerOverlapEnd(AActor* OverlappedActor, AActor* OtherActor)
{
	// Return if the actor out of contact contact has no unique name set
	if (!ActorToUniqueName.Contains(OtherActor))
	{
		UE_LOG(LogTemp, Error, TEXT("%s: Actor %s has no unique name set up at contact \t**END**.. skipping"), *IslandTriggerArea->GetName(), *OtherActor->GetName());
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("\t Contact-%s-%s \t**END**"), *IslandTriggerArea->GetName(), *OtherActor->GetName());
	FString UniqueEventName = ActorToUniqueName[IslandTriggerArea->GetAttachParentActor()] + ActorToUniqueName[OtherActor];

	// Return if the event is not in the unfinished touch events map
	if (!UnfinishedTouchEvents.Contains(UniqueEventName))
	{
		UE_LOG(LogTemp, Error, TEXT("UnfinishedTouchEvents does not contain the unique name: %s"), *UniqueEventName);
		return;
	}

	// Get and remove the touch even from the unfinished events map
	FOwlEvent IslandTouchEvent = UnfinishedTouchEvents.FindAndRemoveChecked(UniqueEventName);

	// Add EndTime
	IslandTouchEvent.EndTime = ElapsedTime;
	const FString EndTime = "&log;timepoint_" + FString::SanitizeFloat(ElapsedTime);
	IslandTouchEvent.AddProperty("knowrob:endTime", "rdf:resource", EndTime);
	TimepointIndividuals.AddUnique(EndTime);

	// Mark event as finished
	IslandTouchEvent.SetFinshed(true);

	// Add finished touch event to the event individuals array
	EventIndividuals.Add(IslandTouchEvent);
}

// Bound function with the overlap begin event
void ARSemLogManager::OnSinkTriggerOverlapBegin(AActor* OverlappedActor, AActor* OtherActor)
{
	// Return if the actor in contact has no unique name set
	if (!ActorToUniqueName.Contains(OtherActor))
	{
		UE_LOG(LogTemp, Error, TEXT("%s: Actor %s has no unique name set up at contact \t**BEGIN**.. skipping"), *SinkTriggerArea->GetName(), *OtherActor->GetName());
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("\t Contact-%s-%s \t**BEGIN**"), *SinkTriggerArea->GetName(), *OtherActor->GetName());

	// Sink touch event
	FOwlEvent SinkTouchEvent = ARSemLogManager::StartTouchingSituation(SinkTriggerArea, OtherActor);

	// Add event to the unfinished touch events (key is the unique name of the trigger + uniqe name of the contact actor
	UnfinishedTouchEvents.Add(ActorToUniqueName[SinkTriggerArea->GetAttachParentActor()] + ActorToUniqueName[OtherActor], SinkTouchEvent);
}

// Bound function with the overlap end event
void ARSemLogManager::OnSinkTriggerOverlapEnd(AActor* OverlappedActor, AActor* OtherActor)
{
	// Return if the actor out of contact contact has no unique name set
	if (!ActorToUniqueName.Contains(OtherActor))
	{
		UE_LOG(LogTemp, Error, TEXT("%s: Actor %s has no unique name set up at contact \t**END**.. skipping"), *SinkTriggerArea->GetName(), *OtherActor->GetName());
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("\t Contact-%s-%s \t**END**"), *SinkTriggerArea->GetName(), *OtherActor->GetName());
	FString UniqueEventName = ActorToUniqueName[SinkTriggerArea->GetAttachParentActor()] + ActorToUniqueName[OtherActor];

	// Return if the event is not in the unfinished touch events map
	if (!UnfinishedTouchEvents.Contains(UniqueEventName))
	{
		UE_LOG(LogTemp, Error, TEXT("UnfinishedTouchEvents does not contain the unique name: %s"), *UniqueEventName);
		return;
	}

	// Get and remove the touch even from the unfinished events map
	FOwlEvent SinkTouchEvent = UnfinishedTouchEvents.FindAndRemoveChecked(UniqueEventName);

	// Add EndTime
	SinkTouchEvent.EndTime = ElapsedTime;
	const FString EndTime = "&log;timepoint_" + FString::SanitizeFloat(ElapsedTime);
	SinkTouchEvent.AddProperty("knowrob:endTime", "rdf:resource", EndTime);
	TimepointIndividuals.AddUnique(EndTime);

	// Mark event as finished
	SinkTouchEvent.SetFinshed(true);

	// Add finished touch event to the event individuals array
	EventIndividuals.Add(SinkTouchEvent);
}
