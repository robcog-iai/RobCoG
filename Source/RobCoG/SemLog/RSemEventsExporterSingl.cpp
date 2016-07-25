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
void FRSemEventsExporterSingl::Init(const float Timestamp)
{
	// Init metadata
	Metadata = new RSemEvent("&log;",
		"UnrealExperiment_" + FRUtils::GenerateRandomFString(4), Timestamp);
	// Add class property
	Metadata->Properties.Add(FROwlUtils::ROwlTriple(
		"rdf:type", "rdf:resource", "&knowrob;UnrealExperiment"));
	// Add startTime property
	Metadata->Properties.Add(
		FROwlUtils::ROwlTriple("knowrob:startTime", "rdf:resource",
			FRUtils::FStringToChar("&log;timepoint_" + FString::SanitizeFloat(Timestamp))));


	// Set init flag to true
	bInit = true;
}

// Check if the singleton is initialized
bool FRSemEventsExporterSingl::IsInit()
{
	return bInit;
}

// Write events to file
void FRSemEventsExporterSingl::WriteEvents(const FString Path,
	const TMap<FString, FString>& ActUniqNameToClassTypeMap,
	const float Timestamp)
{
	// End all opened events
	FRSemEventsExporterSingl::TerminateEvents(Timestamp);

	// Set metadata as finished
	Metadata->End = Timestamp;
	// Add endTime property
	Metadata->Properties.Add(
		FROwlUtils::ROwlTriple("knowrob:endTime", "rdf:resource",
			FRUtils::FStringToChar("&log;timepoint_" + FString::SanitizeFloat(Timestamp))));

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
	for (const auto ActUniqNameToClassTypeItr : ActUniqNameToClassTypeMap)
	{
		FROwlUtils::AddNodeEntityWithProperty(EventsDoc, RDFNode,
			FROwlUtils::ROwlTriple("owl:NamedIndividual", "rdf:about",
				FRUtils::FStringToChar("&log;" + ActUniqNameToClassTypeItr.Key)),
			FROwlUtils::ROwlTriple("rdf:type", "rdf:resource",
				FRUtils::FStringToChar("&knowrob;" + ActUniqNameToClassTypeItr.Value)));
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
	FROwlUtils::AddNodeComment(EventsDoc, RDFNode, "Metadata Individuals");
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
	FFileHelper::SaveStringToFile(OwlString, *Path);
}

// Add beginning of grasping event
void FRSemEventsExporterSingl::BeginGraspingEvent(
	AActor* Self, AActor* Other, const float Timestamp)
{
	const FString HandName = Self->GetName();
	const FString GraspedActName = Other->GetName();

	UE_LOG(LogTemp, Warning, TEXT("Grasp Start: %s %s %f"), *HandName, *GraspedActName,
		Timestamp);
	
	// Init grasp event
	RSemEvent* GraspEvent = new RSemEvent("&log;",
		"GraspingSomething_" + FRUtils::GenerateRandomFString(4), Timestamp);
	// Add class property
	GraspEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"rdf:type", "rdf:resource", "&knowrob; GraspingSomething"));
	// Add taskContext
	GraspEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob:taskContext", "rdf:datatype", "&xsd;string", "TODO obj unique name"));
	// KnowRob Timepoint
	const FString TimepointStr = "timepoint_" + FString::SanitizeFloat(Timestamp);
	// Add to timepoints array
	TimepointIndividuals.Add(TimepointStr);
	// Add startTime
	GraspEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob:startTime", "rdf:resource", 
		FRUtils::FStringToChar("&log;" + TimepointStr)));
	// Add objectActedOn
	GraspEvent->Properties.Add(FROwlUtils::ROwlTriple(
		"knowrob:objectActedOn", "rdf:resource", 
		FRUtils::FStringToChar("&log;" + GraspedActName + "TODO")));

	// Add events to the map
	NameToEventsMap.Add("Grasp" + HandName + GraspedActName, GraspEvent);
}

// Add ending of grasping event
void FRSemEventsExporterSingl::EndGraspingEvent(
	AActor* Self, AActor* Other, const float Timestamp)
{
	const FString HandName = Self->GetName();
	const FString GraspedActName = Other->GetName();

	UE_LOG(LogTemp, Warning, TEXT("Grasp End: %s %s %f"), *HandName, *GraspedActName,
		Timestamp);

	// Check if grasp is started
	if (NameToEventsMap.Contains("Grasp" + HandName + GraspedActName))
	{
		// Get the grasp event and finish it
		RSemEvent* CurrGraspEv = NameToEventsMap["Grasp" + HandName + GraspedActName];

		// Add finishing time
		CurrGraspEv->End = Timestamp;
		// KnowRob Timepoint
		const FString TimepointStr = "timepoint_" + FString::SanitizeFloat(Timestamp);
		// Add to timepoints array
		TimepointIndividuals.Add(TimepointStr);

		// Add endTime property
		CurrGraspEv->Properties.Add(
			FROwlUtils::ROwlTriple("knowrob:endTime", "rdf:resource",
				FRUtils::FStringToChar("&log;" + TimepointStr)));

		// Add as subAction property to Metadata
		Metadata->Properties.Add(
			FROwlUtils::ROwlTriple("knowrob:subAction", "rdf:resource",
				FRUtils::FStringToChar(CurrGraspEv->Ns + CurrGraspEv->Name)));
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
			// KnowRob Timepoint
			const FString TimepointStr = "timepoint_" + FString::SanitizeFloat(Timestamp);
			// Add to timepoints array
			TimepointIndividuals.Add(TimepointStr);

			// Add endTime property
			NameToEvItr.Value->Properties.Add(
				FROwlUtils::ROwlTriple("knowrob:endTime", "rdf:resource",
					FRUtils::FStringToChar("&log;" + TimepointStr)));

			// Add as subAction property to Metadata
			Metadata->Properties.Add(
				FROwlUtils::ROwlTriple("knowrob:subAction", "rdf:resource",
					FRUtils::FStringToChar(NameToEvItr.Value->Ns + NameToEvItr.Value->Name)));
		}
	}
}

// Add timepoint to array, and return Knowrob specific timestamp
FString FRSemEventsExporterSingl::AddTimestamp(const float Timestamp) const
{
	// KnowRob Timepoint
	const FString TimepointStr = "timepoint_" + FString::SanitizeFloat(Timestamp);
	//// Add to timepoints array
	//TimepointIndividuals.Add(TimepointStr);
	// Return string
	return TimepointStr;
}

//// Start touching event
//FOwlEvent ARSemLogManager::StartTouchingSituation(AActor* Trigger, AActor* OtherActor)
//{
//	// Create touch situation at the current time
//	FOwlEvent TouchEvent("&log;", "TouchingSituation", ARSemLogManager::RandStringUnderscore(4), ElapsedTime);
//
//	// Add event type property
//	TouchEvent.AddProperty("rdf:type", "rdf:resource", "&knowrob_u;TouchingSituation");
//
//	// Set task context
//	const FString TaskContext("Contact-" + ActorToUniqueName[Trigger->GetAttachParentActor()] + "-" + ActorToUniqueName[OtherActor]);
//	// Add context to the event, and as property
//	TouchEvent.SetContext(TaskContext);
//	TouchEvent.AddProperty("knowrob:taskContext", "rdf:datatype", "&xsd;string", TaskContext);
//
//	// Set event StartTime and add it as property
//	const FString StartTime = "&log;timepoint_" + FString::SanitizeFloat(TouchEvent.StartTime);
//	TouchEvent.AddProperty("knowrob:startTime", "rdf:resource", StartTime);
//
//	// Make sure the time individual is not repeating
//	TimepointIndividuals.AddUnique(StartTime);
//
//	// Add objects in contact
//	TouchEvent.AddProperty("knowrob_u:inContact", "rdf:resource", FString("&log;") + ActorToUniqueName[Trigger->GetAttachParentActor()]);
//	TouchEvent.AddProperty("knowrob_u:inContact", "rdf:resource", FString("&log;") + ActorToUniqueName[OtherActor]);
//
//	return TouchEvent;
//}