// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "ROwlUtils.h"

/**
 * Semantic map exporter
 */
class ROBCOG_API FRSemEventsExporterSingl
{
public:
	// Get singleton instance
	static FRSemEventsExporterSingl& Get()
	{
		static FRSemEventsExporterSingl Instance;
		return Instance;	
	}
	// Remove copy constructor and operator
	FRSemEventsExporterSingl(FRSemEventsExporterSingl const&) = delete;
	void operator=(FRSemEventsExporterSingl const&) = delete;

	// Initialize
	void Init(const float Timestamp);

	// Check if init
	bool IsInit();

	// Add beginning of touching event
	void BeginTouchingEvent();

	// Add end of touching event
	void EndTouchingEvent();

	// Add beginning of grasping event
	void BeginGraspingEvent(AActor* Self, AActor* Other, const float Timestamp);

	// Add end of grasping event
	void EndGraspingEvent(AActor* Self, AActor* Other, const float Timestamp);
	
	// Write events to file
	void WriteEvents(const FString Path,
		const TMap<FString, FString>& ActUniqNameToClassTypeMap,
		const float Timestamp);

	// Event struct
	struct RSemEvent
	{
		RSemEvent(FString Namespace, FString UniqueName, float Start = -1.0f, float End = -1.0f)
			: Ns(Namespace), Name(UniqueName), Start(Start), End(End)
		{}
		FString Ns;
		FString Name;
		float Start;
		float End;
		TArray<FROwlUtils::ROwlTriple> Properties;
	};
	
private:
	// Constructor
	FRSemEventsExporterSingl();

	// Destructor
	~FRSemEventsExporterSingl();

	// Add finish time to all events
	void TerminateEvents(const float Timestamp);

	// Add timepoint to array, and return Knowrob specific timestamp
	FString AddTimestamp(const float Timestamp) const;

	// Flag showing if the exportert is init
	bool bInit;

	// Event name to event individuals map
	TMap<FString, RSemEvent*> NameToEventsMap;

	// Objects unique name to class map
	TMap<FString, FString> ObjUNameToClassMap;

	// Timepoint individuals
	TArray<FString> TimepointIndividuals;

	// Metadata individual semantic event
	RSemEvent* Metadata;
};

