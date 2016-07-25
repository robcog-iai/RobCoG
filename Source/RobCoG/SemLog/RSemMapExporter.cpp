// Fill out your copyright notice in the Description page of Project Settings.

#include "RobCoG.h"
#include "RUtils.h"
#include "ROwlUtils.h"
#include "RSemMapExporter.h"

// Set default values
FRSemMapExporter::FRSemMapExporter()
{
	// Generate unique name
	UniqueName = "USemMap_" + FRUtils::GenerateRandomFString(4);
}

// Destructor
FRSemMapExporter::~FRSemMapExporter()
{
}

// Write semantic map
void FRSemMapExporter::WriteSemanticMap(
	const TMap<AStaticMeshActor*, FString>& DynamicActPtrToUniqNameMap,
	const TMap<AStaticMeshActor*, FString>& StaticActPtrToUniqNameMap,
	const TMap<FString, FString>& ActUniqNameToClassTypeMap,
	const FString Path)
{
	///////// DOC
	// Semantic map document
	rapidxml::xml_document<>* SemMapDoc = new rapidxml::xml_document<>();

	///////// TYPE DECLARATION
	// Create declaration node <?xml version="1.0" encoding="utf-8"?>
	rapidxml::xml_node<> *DeclarationNode = SemMapDoc->allocate_node(rapidxml::node_declaration);
	// Create attibutes
	FROwlUtils::AddNodeAttribute(SemMapDoc, DeclarationNode, "version", "1.0");
	FROwlUtils::AddNodeAttribute(SemMapDoc, DeclarationNode, "encoding", "utf-8");
	// Add node to document
	SemMapDoc->append_node(DeclarationNode);

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
	rapidxml::xml_node<> *DoctypeNode = SemMapDoc->allocate_node(rapidxml::node_doctype, "", doctype);
	// Add node to document
	SemMapDoc->append_node(DoctypeNode);

	///////// RDF NODE
	// Semantic map RDF node
	rapidxml::xml_node<>* RDFNode = SemMapDoc->allocate_node(rapidxml::node_element, "rdf:RDF", "");

	// Add attributes
	FROwlUtils::AddNodeAttribute(SemMapDoc, RDFNode,
		"xmlns:computable", "http://knowrob.org/kb/computable.owl#");
	FROwlUtils::AddNodeAttribute(SemMapDoc, RDFNode,
		"xmlns:swrl", "http://www.w3.org/2003/11/swrl#");
	FROwlUtils::AddNodeAttribute(SemMapDoc, RDFNode,
		"xmlns:rdf", "http://www.w3.org/1999/02/22-rdf-syntax-ns#");
	FROwlUtils::AddNodeAttribute(SemMapDoc, RDFNode,
		"xmlns:rdfs", "http://www.w3.org/2000/01/rdf-schema#");
	FROwlUtils::AddNodeAttribute(SemMapDoc, RDFNode,
		"xmlns:owl", "http://www.w3.org/2002/07/owl#");
	FROwlUtils::AddNodeAttribute(SemMapDoc, RDFNode,
		"xmlns:knowrob", "http://knowrob.org/kb/knowrob.owl#");
	FROwlUtils::AddNodeAttribute(SemMapDoc, RDFNode,
		"xmlns:knowrob_u", "http://knowrob.org/kb/knowrob_u.owl#");
	FROwlUtils::AddNodeAttribute(SemMapDoc, RDFNode,
		"xmlns:u-map", "http://knowrob.org/kb/u_map.owl#");
	FROwlUtils::AddNodeAttribute(SemMapDoc, RDFNode,
		"xml:base", "http://knowrob.org/kb/u_map.owl#");

	///////// ONTOLOGY IMPORT
	// Create entity node with property
	FROwlUtils::AddNodeComment(SemMapDoc, RDFNode, "Ontologies");
	FROwlUtils::AddNodeEntityWithProperty(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Ontology", "rdf:about", "http://knowrob.org/kb/u_map.owl"),
		FROwlUtils::ROwlTriple("owl:imports", "rdf:resource", "package://knowrob_common/owl/knowrob.owl"));

	///////// GENERAL DEFINITIONS
	// Object property definitions
	FROwlUtils::AddNodeComment(SemMapDoc, RDFNode, "Property Definitions");
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:ObjectProperty", "rdf:about", "&knowrob;describedInMap"));

	// Datatype property definitions
	FROwlUtils::AddNodeComment(SemMapDoc, RDFNode, "Datatype Definitions");
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:DatatypeProperty", "rdf:about", "&knowrob;depthOfObject"));
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:DatatypeProperty", "rdf:about", "&knowrob;heightOfObject"));
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:DatatypeProperty", "rdf:about", "&knowrob;widthOfObject"));
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:DatatypeProperty", "rdf:about", "&knowrob;vectorX"));
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:DatatypeProperty", "rdf:about", "&knowrob;vectorY"));
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:DatatypeProperty", "rdf:about", "&knowrob;vectorZ"));
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:DatatypeProperty", "rdf:about", "&knowrob;pathToCadModel"));

	// Class definitions
	FROwlUtils::AddNodeComment(SemMapDoc, RDFNode, "Class Definitions");
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Class", "rdf:about", "&knowrob;SemanticEnvironmentMap"));
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Class", "rdf:about", "&knowrob;SemanticMapPerception"));
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Class", "rdf:about", "&knowrob;TimePoint"));
	FROwlUtils::AddNodeTriple(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:Class", "rdf:about", "&knowrob;Vector"));


	///////// EVENT INDIVIDUALS
	// Semantic map individual
	FROwlUtils::AddNodeComment(SemMapDoc, RDFNode, "Semantic Environment Map");
	// Add semantic map instance
	FROwlUtils::AddNodeEntityWithProperty(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("rdf:type", "rdf:resource", "&knowrob;SemanticEnvironmentMap"),
		FROwlUtils::ROwlTriple("owl:NamedIndividual", "rdf:about",
			FRUtils::FStringToChar("&u-map;" + UniqueName)));

	// Timepoint individual
	FROwlUtils::AddNodeComment(SemMapDoc, RDFNode, "Timepoint");
	// Add timepoint instance
	FROwlUtils::AddNodeEntityWithProperty(SemMapDoc, RDFNode,
		FROwlUtils::ROwlTriple("owl:NamedIndividual", "rdf:about", "&u-map;timepoint_0"),
		FROwlUtils::ROwlTriple("rdf:type", "rdf:resource", "&knowrob;Timepoint"));
	
	UE_LOG(LogTemp, Warning, TEXT(" *** "));
	UE_LOG(LogTemp, Warning, TEXT("Semantic map components:"));

	// Lambda to add actors to the semantic map as perception events
	auto AddActorsToSemMapLambda = [SemMapDoc, RDFNode, this](const TMap<AStaticMeshActor*, FString>& ActPtrToUniqNameMap, const TMap<FString, FString>& ActUniqNameToClassTypeMap)
	{
		for (const auto ActPtrToUniqNameItr : ActPtrToUniqNameMap)
		{
			// Local copies of name and unique name
			const FString ActName = ActPtrToUniqNameItr.Key->GetName();
			const FString ActUniqueName = ActPtrToUniqNameItr.Value;
			const FString ActClass = ActUniqNameToClassTypeMap[ActUniqueName];
			UE_LOG(LogTemp, Warning, TEXT("\t%s -> %s"), *ActName, *ActUniqueName);

			// Transf unique name
			const FString TransfUniqueName = "Transformation_" + FRUtils::GenerateRandomFString(4);

			// Loc and rotation as quat of the objects as strings, change from left hand to right hand coord
			const FVector Loc = ActPtrToUniqNameItr.Key->GetActorLocation() / 100;
			const FString LocStr = FString::SanitizeFloat(Loc.X) + " "
				+ FString::SanitizeFloat(-Loc.Y) + " "
				+ FString::SanitizeFloat(Loc.Z);

			const FQuat Quat = ActPtrToUniqNameItr.Key->GetActorQuat();
			const FString QuatStr = FString::SanitizeFloat(Quat.W) + " "
				+ FString::SanitizeFloat(-Quat.X) + " "
				+ FString::SanitizeFloat(Quat.Y) + " "
				+ FString::SanitizeFloat(-Quat.Z);

			// Object instance
			FROwlUtils::AddNodeComment(SemMapDoc, RDFNode, FRUtils::FStringToChar("Object " + ActUniqueName));

			// Array of object properties
			TArray<FROwlUtils::ROwlTriple> ObjProperties;
			// Add obj event properties
			ObjProperties.Add(FROwlUtils::ROwlTriple(
				"rdf:type", "rdf:resource", FRUtils::FStringToChar("&knowrob;" + ActClass)));
			ObjProperties.Add(FROwlUtils::ROwlTriple(
				"knowrob:pathToCadModel", "rdf:datatype", "&xsd; string",
				FRUtils::FStringToChar("package://sim/unreal/" + ActClass + ".dae")));
			ObjProperties.Add(FROwlUtils::ROwlTriple(
				"knowrob:describedInMap", "rdf:resource",
				FRUtils::FStringToChar(FRUtils::FStringToChar("&u-map;" + UniqueName))));
			// Add instance with properties
			FROwlUtils::AddNodeEntityWithProperties(SemMapDoc, RDFNode,
				FROwlUtils::ROwlTriple("owl:NamedIndividual", "rdf:about",
					FRUtils::FStringToChar("&log;" + ActUniqueName)), ObjProperties);

			// Map perception unique name
			const FString MapPerceptionUniqueName = "SemanticMapPerception_" + FRUtils::GenerateRandomFString(4);

			// Map perception properties
			TArray<FROwlUtils::ROwlTriple> MapPerceptionProperties;
			// Add obj event properties
			MapPerceptionProperties.Add(FROwlUtils::ROwlTriple(
				"rdf:type", "rdf:resource", "&knowrob;SemanticMapPerception"));
			MapPerceptionProperties.Add(FROwlUtils::ROwlTriple(
				"knowrob:eventOccursAt", "rdf:resource",
				FRUtils::FStringToChar("&u-map;" + TransfUniqueName)));
			MapPerceptionProperties.Add(FROwlUtils::ROwlTriple(
				"knowrob:startTime", "rdf:resource", "&u-map;timepoint_0"));
			MapPerceptionProperties.Add(FROwlUtils::ROwlTriple(
				"knowrob:objectActedOn", "rdf:resource",
				FRUtils::FStringToChar("&log;" + ActUniqueName)));
			// Add instance with properties
			FROwlUtils::AddNodeEntityWithProperties(SemMapDoc, RDFNode,
				FROwlUtils::ROwlTriple("owl:NamedIndividual", "rdf:about",
					FRUtils::FStringToChar(MapPerceptionUniqueName)),
				MapPerceptionProperties);

			// Transformation properties
			TArray<FROwlUtils::ROwlTriple> TransfProperties;
			// Add obj event properties
			TransfProperties.Add(FROwlUtils::ROwlTriple(
				"rdf:type", "rdf:resource", "&knowrob;Transformation"));
			TransfProperties.Add(FROwlUtils::ROwlTriple(
				"knowrob:quaternion", "rdf:datatype", "&xsd;string", FRUtils::FStringToChar(QuatStr)));
			TransfProperties.Add(FROwlUtils::ROwlTriple(
				"knowrob:translation", "rdf:datatype", "&xsd;string", FRUtils::FStringToChar(LocStr)));
			// Add instance with properties
			FROwlUtils::AddNodeEntityWithProperties(SemMapDoc, RDFNode,
				FROwlUtils::ROwlTriple("owl:NamedIndividual", "rdf:about",
					FRUtils::FStringToChar("&u-map;" + TransfUniqueName)),
				TransfProperties);
		}
	};

	// Add dynamic and static actors initial position to the map
	AddActorsToSemMapLambda(DynamicActPtrToUniqNameMap, ActUniqNameToClassTypeMap);
	AddActorsToSemMapLambda(StaticActPtrToUniqNameMap, ActUniqNameToClassTypeMap);

	///////// ADD RDF TO OWL DOC
	SemMapDoc->append_node(RDFNode);

	// Create string
	std::string RapidXmlString;
	rapidxml::print(std::back_inserter(RapidXmlString), *SemMapDoc, 0 /*rapidxml::print_no_indenting*/);
	FString OwlString = UTF8_TO_TCHAR(RapidXmlString.c_str());

	// Write string to file
	FFileHelper::SaveStringToFile(OwlString, *Path);
}

// Get unique name
FString FRSemMapExporter::GetUniqueName()
{
	return UniqueName;
}
