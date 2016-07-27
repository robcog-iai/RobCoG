// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RUtils.h"
#include "ROwlUtils.h"
#include "RSemEventsExporterSingl.h"

// Constructor
FRSemEventsExporterSingl::FRSemEventsExporterSingl()
{
	// Set init flag to false
	bInit = false;
}

// Destructor
FRSemEventsExporterSingl::~FRSemEventsExporterSingl()
{
}

// Init exporter
void FRSemEventsExporterSingl::Init(
	const FString UniqueTag,
	const TMap<AActor*, FString>& ActorToUniqueName,
	const TMap<AActor*, FString>& ActorToClassType,
	const float Timestamp)
{
	// Set episode unique tag
	EpisodeUniqueTag = UniqueTag;

	// Set the map references to the member maps
	EvActorToUniqueName = ActorToUniqueName;
	EvActorToClassType = ActorToClassType;

	// Init metadata
	Metadata = new RSemEvent("&log;",
		"UnrealExperiment_" + EpisodeUniqueTag, Timestamp);
	// Add class property
	Metadata->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob:experiment", "rdf:datatype", "&xsd;string", 
		FRUtils::FStringToChar(EpisodeUniqueTag)));
	// Add experiment unique name tag
	Metadata->Properties.Add(FROwlUtils::ROwlTriple(
		"rdf:type", "rdf:resource", "&knowrob;UnrealExperiment"));
	// Add startTime property
	Metadata->Properties.Add(
		FROwlUtils::ROwlTriple("knowrob:startTime", "rdf:resource",
			FRUtils::FStringToChar("&log;" + 
				FRSemEventsExporterSingl::AddTimestamp(Timestamp))));

	// Set init flag to true
	bInit = true;
}

// Check if the singleton is initialized
bool FRSemEventsExporterSingl::IsInit()
{
	return bInit;
}

// Reset singleton
void FRSemEventsExporterSingl::Reset()
{
	// Set init to false
	bInit = false;
	// Empty containers
	EvActorToUniqueName.Empty();
	EvActorToClassType.Empty();
	NameToEventsMap.Empty();
	ObjectIndividuals.Empty();
	TimepointIndividuals.Empty();
	EpisodeUniqueTag.Empty();
	// Empty metadata
	delete Metadata;
}

// Write events to file
void FRSemEventsExporterSingl::WriteEvents(const FString Path, const float Timestamp, bool bWriteTimelines)
{
	// End all opened events
	FRSemEventsExporterSingl::TerminateEvents(Timestamp);
	// Set metadata as finished
	Metadata->End = Timestamp;
	// Add endTime property
	Metadata->Properties.Add(
		FROwlUtils::ROwlTriple("knowrob:endTime", "rdf:resource",
			FRUtils::FStringToChar("&log;" + 
				FRSemEventsExporterSingl::AddTimestamp(Timestamp))));

	///////// DOC
	rapidxml::xml_document<>* EventsDoc = new rapidxml::xml_document<>();

	///////// TYPE DECLARATION
	rapidxml::xml_node<> *DeclarationNode = EventsDoc->allocate_node(rapidxml::node_declaration);
	// Create attibutes
	FROwlUtils::AddNodeAttribute(EventsDoc, DeclarationNode, "version", "1.0");
	FROwlUtils::AddNodeAttribute(EventsDoc, DeclarationNode, "encoding", "utf-8");
	// Add node to document
	EventsDoc->append_node(DeclarationNode);

	///////// DOCTYPE
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
	rapidxml::xml_node<> *DoctypeNode = EventsDoc->allocate_node(rapidxml::node_doctype, "", doctype);
	// Add node to document
	EventsDoc->append_node(DoctypeNode);

	///////// RDF NODE
	rapidxml::xml_node<>* RDFNode = EventsDoc->allocate_node(rapidxml::node_element, "rdf:RDF", "");
	// Add attributes
	FROwlUtils::AddNodeAttribute(EventsDoc, RDFNode,
		"xmlns:computable", "http://knowrob.org/kb/computable.owl#");
	FROwlUtils::AddNodeAttribute(EventsDoc, RDFNode,
		"xmlns:swrl", "http://www.w3.org/2003/11/swrl#");
	FROwlUtils::AddNodeAttribute(EventsDoc, RDFNode,
		"xmlns:rdf", "http://www.w3.org/1999/02/22-rdf-syntax-ns#");
	FROwlUtils::AddNodeAttribute(EventsDoc, RDFNode,
		"xmlns:rdfs", "http://www.w3.org/2000/01/rdf-schema#");
	FROwlUtils::AddNodeAttribute(EventsDoc, RDFNode,
		"xmlns:owl", "http://www.w3.org/2002/07/owl#");
	FROwlUtils::AddNodeAttribute(EventsDoc, RDFNode,
		"xmlns:knowrob", "http://knowrob.org/kb/knowrob.owl#");
	FROwlUtils::AddNodeAttribute(EventsDoc, RDFNode,
		"xmlns:knowrob_u", "http://knowrob.org/kb/knowrob_u.owl#");
	FROwlUtils::AddNodeAttribute(EventsDoc, RDFNode,
		"xmlns:u-map", "http://knowrob.org/kb/u_map.owl#");
	FROwlUtils::AddNodeAttribute(EventsDoc, RDFNode,
		"xml:base", "http://knowrob.org/kb/u_map.owl#");

	///////// ONTOLOGY IMPORT
	FROwlUtils::AddNodeComment(EventsDoc, RDFNode, "Ontologies");
	// Create entity node with property
	FROwlUtils::AddNodeEntityWithProperty(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Ontology", "rdf:about", "http://knowrob.org/kb/unreal_log.owl"),
		FROwlUtils::ROwlTriple("owl:imports", "rdf:resource", "package://knowrob_common/owl/knowrob.owl"));

	///////// GENERAL DEFINITIONS
	FROwlUtils::AddNodeComment(EventsDoc, RDFNode, "Property Definitions");
	// Object property definitions
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:ObjectProperty", "rdf:about", "&knowrob;taskContext"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:ObjectProperty", "rdf:about", "&knowrob;taskSuccess"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:ObjectProperty", "rdf:about", "&knowrob;startTime"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:ObjectProperty", "rdf:about", "&knowrob;endTime"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:ObjectProperty", "rdf:about", "&knowrob;experiment"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:ObjectProperty", "rdf:about", "&knowrob_u;inContact"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:ObjectProperty", "rdf:about", "&knowrob_u;semanticMap"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:ObjectProperty", "rdf:about", "&knowrob_u;rating"));
	// Class definitions
	FROwlUtils::AddNodeComment(EventsDoc, RDFNode, "Class Definitions");
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Class", "rdf:about", "&knowrob;GraspingSomething"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Class", "rdf:about", "&knowrob_u;UnrealExperiment"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Class", "rdf:about", "&knowrob_u;TouchingSituation"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Class", "rdf:about", "&knowrob_u;KitchenEpisode"));
	FROwlUtils::AddNodeTriple(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Class", "rdf:about", "&knowrob_u;ParticleTranslation"));

	///////// EVENT INDIVIDUALS
	FROwlUtils::AddNodeComment(EventsDoc, RDFNode, "Event Individuals");
	// Add event individuals to RDF node
	for (const auto EventMapItr : NameToEventsMap)
	{
		FROwlUtils::AddNodeEntityWithProperties(EventsDoc, RDFNode,
			FROwlUtils::ROwlTriple("owl:NamedIndividual", "rdf:about",
				FRUtils::FStringToChar(EventMapItr.Value->Name)),
			EventMapItr.Value->Properties);
	}

	///////// OBJECT INDIVIDUALS
	FROwlUtils::AddNodeComment(EventsDoc, RDFNode, "Object Individuals");


	// Add event individuals to RDF node
	for (const auto ObjIndividualItr : ObjectIndividuals)
	{		
		// Check that both unique name and class is available
		if ((EvActorToUniqueName.Contains(ObjIndividualItr)) &&
			(EvActorToClassType.Contains(ObjIndividualItr)))
		{
			FROwlUtils::AddNodeEntityWithProperty(EventsDoc, RDFNode,
				FROwlUtils::ROwlTriple("owl:NamedIndividual", "rdf:about",
					FRUtils::FStringToChar("&log;" + EvActorToUniqueName[ObjIndividualItr])),
				FROwlUtils::ROwlTriple("rdf:type", "rdf:resource",
					FRUtils::FStringToChar("&knowrob;" + EvActorToClassType[ObjIndividualItr])));
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("%s 's unique name is not set! Writing object individual skipped!"), *ObjIndividualItr->GetName());
		}
	}

	///////// TIMEPOINT INDIVIDUALS
	FROwlUtils::AddNodeComment(EventsDoc, RDFNode, "Timepoint Individuals");
	// Add time individuals to RDF node
	for (const auto TimepointIter : TimepointIndividuals)
	{
		FROwlUtils::AddNodeEntityWithProperty(EventsDoc, RDFNode,
			FROwlUtils::ROwlTriple("owl:NamedIndividual", "rdf:about",
				FRUtils::FStringToChar("&log;" + TimepointIter)),
			FROwlUtils::ROwlTriple("rdf:type", "rdf:resource", "&knowrob;TimePoint"));
	};

	///////// METADATA INDIVIDUAL
	FROwlUtils::AddNodeComment(EventsDoc, RDFNode, "Metadata Individual");
	// Add metadata to RDF node
	FROwlUtils::AddNodeEntityWithProperties(EventsDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:NamedIndividual", "rdf:about",
			FRUtils::FStringToChar(Metadata->Ns + Metadata->Name)),
		Metadata->Properties);
	

	///////// ADD RDF TO OWL DOC
	EventsDoc->append_node(RDFNode);

	// Create string
	std::string RapidXmlString;
	rapidxml::print(std::back_inserter(RapidXmlString), *EventsDoc, 0 /*rapidxml::print_no_indenting*/);
	FString OwlString = UTF8_TO_TCHAR(RapidXmlString.c_str());

	// Write string to file
	const FString FilePath = Path + "/EventData_" + EpisodeUniqueTag + ".owl";
	FFileHelper::SaveStringToFile(OwlString, *FilePath);

	if (bWriteTimelines)
	{
		FRSemEventsExporterSingl::WriteTimelines();
	}
}

// Add beginning of grasping event
void FRSemEventsExporterSingl::BeginGraspingEvent(
	AActor* Self, AActor* Other, const float Timestamp)
{
	const FString HandName = Self->GetName();
	const FString GraspedActorName = Other->GetName();

	// Skip saving the event if one of the actor is not registered with unique name
	if (!(EvActorToUniqueName.Contains(Self) && EvActorToUniqueName.Contains(Other)))
	{
		UE_LOG(LogTemp, Error, TEXT(" %s or %s's unique name is not set! Begin grasp event skipped!"), *HandName, *GraspedActorName);
		return;
	}
	// Get unique name of the hand and object
	const FString HandUniqueName = EvActorToUniqueName[Self];
	const FString GraspedActorUniqueName = EvActorToUniqueName[Other];
	
	// Add hand and object to the object individuals array
	ObjectIndividuals.AddUnique(Self);
	ObjectIndividuals.AddUnique(Other);

	// TODO rm
	UE_LOG(LogTemp, Warning, TEXT("Grasp Start: %s %s %f"), *HandUniqueName, *GraspedActorUniqueName,
		Timestamp);
	
	// Init grasp event
	RSemEvent* GraspEvent = new RSemEvent("&log;",
		"GraspingSomething_" + FRUtils::GenerateRandomFString(4), Timestamp);
	// Add class property
	GraspEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"rdf:type", "rdf:resource", "&knowrob;GraspingSomething"));
	// Add taskContext
	GraspEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob:taskContext", "rdf:datatype", "&xsd;string", 
		FRUtils::FStringToChar("Grasp-" + HandUniqueName + "-" + GraspedActorUniqueName)));
	// Add startTime
	GraspEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob:startTime", "rdf:resource", 
		FRUtils::FStringToChar("&log;" + 
			FRSemEventsExporterSingl::AddTimestamp(Timestamp))));
	// Add objectActedOn
	GraspEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob:objectActedOn", "rdf:resource", 
		FRUtils::FStringToChar("&log;" + GraspedActorUniqueName)));

	// Add events to the map
	NameToEventsMap.Add("Grasp" + HandName + GraspedActorName, GraspEvent);
}

// Add ending of grasping event
void FRSemEventsExporterSingl::EndGraspingEvent(
	AActor* Self, AActor* Other, const float Timestamp)
{
	const FString HandName = Self->GetName();
	const FString GraspedActorName = Other->GetName();

	// TODO rm
	UE_LOG(LogTemp, Warning, TEXT("Grasp End: %s %s %f"), *HandName, *GraspedActorName,
		Timestamp);

	// Check if grasp is started
	if (NameToEventsMap.Contains("Grasp" + HandName + GraspedActorName))
	{
		// Get the grasp event and finish it
		RSemEvent* CurrGraspEv = NameToEventsMap["Grasp" + HandName + GraspedActorName];

		// Add finishing time
		CurrGraspEv->End = Timestamp;

		// Add endTime property
		CurrGraspEv->Properties.Add(
			FROwlUtils::ROwlTriple("knowrob:endTime", "rdf:resource",
				FRUtils::FStringToChar("&log;" + 
					FRSemEventsExporterSingl::AddTimestamp(Timestamp))));

		// Add as subAction property to Metadata
		Metadata->Properties.Add(
			FROwlUtils::ROwlTriple("knowrob:subAction", "rdf:resource",
				FRUtils::FStringToChar(CurrGraspEv->Ns + CurrGraspEv->Name)));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT(" Trying to end grasp which did not start: %s !"), *FString("Grasp" + HandName + GraspedActorName));
	}
}

// Add beginning of touching event
void FRSemEventsExporterSingl::BeginTouchingEvent(
	AActor* TriggerParent, AActor* OtherActor, const float Timestamp)
{
	const FString TriggerParentName = TriggerParent->GetName();
	const FString OtherActorName = OtherActor->GetName();

	// TODO rm
	UE_LOG(LogTemp, Warning, TEXT("%s %s begin contact."),
		*TriggerParentName, *OtherActorName);

	// Skip saving the event if one of the actor is not registered with unique name
	if (!(EvActorToUniqueName.Contains(TriggerParent) && EvActorToUniqueName.Contains(OtherActor)))
	{
		UE_LOG(LogTemp, Error, TEXT(" %s or %s's unique name is not set! Begin touch event skipped!"), *TriggerParent->GetName(), *OtherActor->GetName());
		return;
	}
	// Get unique names of the objects in contact
	const FString TriggerUniqueName = EvActorToUniqueName[TriggerParent];
	const FString OtherActorUniqueName = EvActorToUniqueName[OtherActor];

	// Add objects to the object individuals array
	ObjectIndividuals.AddUnique(TriggerParent);	
	ObjectIndividuals.AddUnique(OtherActor);

	// Init grasp event
	RSemEvent* ContactEvent = new RSemEvent("&log;",
		"TouchingSituation_" + FRUtils::GenerateRandomFString(4), Timestamp);
	// Add class property
	ContactEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"rdf:type", "rdf:resource", "&knowrob_u;TouchingSituation"));
	// Add taskContext
	ContactEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob:taskContext", "rdf:datatype", "&xsd;string",
		FRUtils::FStringToChar("Contact-" + TriggerUniqueName + "-" + OtherActorUniqueName)));
	// Add startTime
	ContactEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob:startTime", "rdf:resource",
		FRUtils::FStringToChar("&log;" +
			FRSemEventsExporterSingl::AddTimestamp(Timestamp))));
	// Add in contact 1
	ContactEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob_u:inContact", "rdf:resource",
		FRUtils::FStringToChar("&log;" + TriggerUniqueName)));
	// Add in contact 2
	ContactEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob_u:inContact", "rdf:resource",
		FRUtils::FStringToChar("&log;" + OtherActorUniqueName)));

	// Add events to the map
	NameToEventsMap.Add("Contact" + TriggerParentName + OtherActorName, ContactEvent);
}

// Add end of touching event
void FRSemEventsExporterSingl::EndTouchingEvent(
	AActor* TriggerParent, AActor* OtherActor, const float Timestamp)
{
	const FString TriggerParentName = TriggerParent->GetName();
	const FString OtherActorName = OtherActor->GetName();

	// TODO rm
	UE_LOG(LogTemp, Warning,
		TEXT("%s %s end contact"), *TriggerParentName, *OtherActorName);

	// Check if grasp is started
	if (NameToEventsMap.Contains("Contact" + TriggerParent->GetName() + OtherActor->GetName()))
	{
		// Get the grasp event and finish it
		RSemEvent* CurrContactEv = NameToEventsMap[
			"Contact" + TriggerParentName + OtherActorName];

		// Add finishing time
		CurrContactEv->End = Timestamp;

		// Add endTime property
		CurrContactEv->Properties.Add(
			FROwlUtils::ROwlTriple("knowrob:endTime", "rdf:resource",
				FRUtils::FStringToChar("&log;" +
					FRSemEventsExporterSingl::AddTimestamp(Timestamp))));

		// Add as subAction property to Metadata
		Metadata->Properties.Add(
			FROwlUtils::ROwlTriple("knowrob:subAction", "rdf:resource",
				FRUtils::FStringToChar(CurrContactEv->Ns + CurrContactEv->Name)));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT(" Trying to end a contact event which did not start: %s !"),
			*FString("Contact" + TriggerParent->GetName() + OtherActor->GetName()));
	}
}

// Add finish time to all events
void FRSemEventsExporterSingl::TerminateEvents(const float Timestamp)
{
	// Iterate all events
	for (const auto NameToEvItr : NameToEventsMap)
	{
		// Check if event is still open
		if (NameToEvItr.Value->End < 0)
		{
			// Add finishing time
			NameToEvItr.Value->End = Timestamp;

			// Add endTime property
			NameToEvItr.Value->Properties.Add(
				FROwlUtils::ROwlTriple("knowrob:endTime", "rdf:resource",
					FRUtils::FStringToChar("&log;" + 
						FRSemEventsExporterSingl::AddTimestamp(Timestamp))));

			// Add as subAction property to Metadata
			Metadata->Properties.Add(
				FROwlUtils::ROwlTriple("knowrob:subAction", "rdf:resource",
					FRUtils::FStringToChar(NameToEvItr.Value->Ns + NameToEvItr.Value->Name)));
		}
	}
}

// Write events as timelines
void FRSemEventsExporterSingl::WriteTimelines()
{

}

// Add timepoint to array, and return Knowrob specific timestamp
inline const FString FRSemEventsExporterSingl::AddTimestamp(const float Timestamp)
{
	// KnowRob Timepoint
	const FString TimepointStr = "timepoint_" + FString::SanitizeFloat(Timestamp);
	// Add to timepoints array
	TimepointIndividuals.AddUnique(TimepointStr);
	// Return string ts
	return TimepointStr;
}
