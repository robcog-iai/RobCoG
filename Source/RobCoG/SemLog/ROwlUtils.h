// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "rapidxml/rapidxml_print.hpp"

/**
 * Utils for OWL generation
 */
struct ROBCOG_API FROwlUtils
{
public:
	// Constructor
	FROwlUtils()
	{
	};

	// Destructor
	~FROwlUtils()
	{
	};

	// Node attribute as struct
	struct RNodeAttribute
	{
		RNodeAttribute(const char* Name, const char* Value)
			: Name(Name), Value(Value)
		{}
		const char* Name;
		const char* Value;
	};

	// Owl triple as struct
	struct ROwlTriple
	{
		ROwlTriple(const char* Subj, const char* Pred, const char* Obj, const char* Val = "")
			: Subject(Subj), Predicate(Pred), Object(Obj), Value(Val)
		{}
		const char* Subject;
		const char* Predicate;
		const char* Object;
		const char* Value;
	};

	// Add attribute to node
	static FORCEINLINE void AddNodeAttribute(
		rapidxml::xml_document<>* Doc,
		rapidxml::xml_node<>* ParentNode,
		const char* Name,
		const char* Value)
	{
		ParentNode->append_attribute(Doc->allocate_attribute(
			Name, Value));
	}

	// Add attributes to node
	static FORCEINLINE void AddNodeAttributes(
		rapidxml::xml_document<>* Doc,
		rapidxml::xml_node<>* ParentNode,
		TArray<RNodeAttribute> Attributes)
	{
		for(const auto AttributeItr : Attributes)
		{
			ParentNode->append_attribute(Doc->allocate_attribute(
				AttributeItr.Name, AttributeItr.Value));
		}
	}

	// Add comment to node
	static FORCEINLINE void AddNodeComment(
		rapidxml::xml_document<>* Doc,
		rapidxml::xml_node<>* ParentNode,
		const char* Comment)
	{
		// Create comment node
		rapidxml::xml_node<> *CommentNode = Doc->allocate_node(rapidxml::node_comment, "", Comment);
		// Append comment node to parent
		ParentNode->append_node(CommentNode);
	}

	// Add OWL triple to node
	static FORCEINLINE void AddNodeTriple(
		rapidxml::xml_document<>* Doc,
		rapidxml::xml_node<>* ParentNode,
		const ROwlTriple Triple)
	{
		// Create Triple node
		rapidxml::xml_node<> *TripleNode = Doc->allocate_node(rapidxml::node_element, Triple.Subject, Triple.Value);
		// Add predicate and object to Triple
		TripleNode->append_attribute(Doc->allocate_attribute(Triple.Predicate, Triple.Object));
		// Append triple to parent
		ParentNode->append_node(TripleNode);
	}

	// Add OWL triples to node
	static FORCEINLINE void AddNodeTriples(
		rapidxml::xml_document<>* Doc,
		rapidxml::xml_node<>* ParentNode,
		const TArray<ROwlTriple>& Triples)
	{
		for (const auto TripleItr : Triples)
		{
			FROwlUtils::AddNodeTriple(Doc, ParentNode, TripleItr);
		}
	}

	// Add OWL entity with one property to node
	static FORCEINLINE void AddNodeEntityWithProperty(
		rapidxml::xml_document<>* Doc,
		rapidxml::xml_node<>* ParentNode,
		const ROwlTriple EntityTriple,
		const ROwlTriple PropertyTriple)
	{
		// Create the entity node
		rapidxml::xml_node<> *EntityNode = Doc->allocate_node(rapidxml::node_element, EntityTriple.Subject, EntityTriple.Value);
		// Add predicate and object to entity node
		EntityNode->append_attribute(Doc->allocate_attribute(EntityTriple.Predicate, EntityTriple.Object));
		// Add property triple to entity node
		FROwlUtils::AddNodeTriple(Doc, EntityNode, PropertyTriple);
		// Append entity to parent
		ParentNode->append_node(EntityNode);
	}
	
	// Add OWL entity with multiple properties to node
	static FORCEINLINE void AddNodeEntityWithProperties(
		rapidxml::xml_document<>* Doc,
		rapidxml::xml_node<>* ParentNode,
		const ROwlTriple EntityTriple,
		const TArray<ROwlTriple>& PropertyTriples)
	{
		// Create the entity node
		rapidxml::xml_node<> *EntityNode = Doc->allocate_node(rapidxml::node_element, EntityTriple.Subject, EntityTriple.Value);
		// Add predicate and object to entity node
		EntityNode->append_attribute(Doc->allocate_attribute(EntityTriple.Predicate, EntityTriple.Object));
		// Add property triples to entity node
		FROwlUtils::AddNodeTriples(Doc, EntityNode, PropertyTriples);
		// Append entity to parent
		ParentNode->append_node(EntityNode);
	}
};
