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
	void Init(
		const FString EpisodeUniqueTag,
		const TMap<AActor*, FString>& ActorToUniqueName, 
		const TMap<AActor*, FString>& ActorToClassType,
		const float Timestamp);

	// Check if init
	bool IsInit();

	// Reset singleton
	void Reset();

	// Add beginning of touching event
	void BeginTouchingEvent(AActor* TriggerParent, AActor* OtherActor, const float Timestamp);

	// Add end of touching event
	void EndTouchingEvent(AActor* TriggerParent, AActor* OtherActor, const float Timestamp);

	// Add beginning of grasping event
	void BeginGraspingEvent(AActor* Self, AActor* Other, const float Timestamp);

	// Add end of grasping event
	void EndGraspingEvent(AActor* Self, AActor* Other, const float Timestamp);
	
	// Write events to file
	void WriteEvents(const FString Path, const float Timestamp, bool bWriteTimelines = true);

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

	// Write events as timelines
	void WriteTimelines(const FString FilePath);

	// Add timepoint to array, and return Knowrob specific timestamp
	const FString AddTimestamp(const float Timestamp);

	// Flag showing if the exportert is init
	bool bInit;

	// Episode unique tag
	FString EpisodeUniqueTag;

	// Reference map of actors to their unique name
	TMap<AActor*, FString> EvActorToUniqueName;

	// Reference map of actors to their class type
	TMap<AActor*, FString> EvActorToClassType;

	// Event name to event individuals map
	TMap<FString, RSemEvent*> NameToOpenedEventsMap;

	// Array of all the finished events
	TArray<RSemEvent*> FinishedEvents;

	// Array of object individuals
	TArray<AActor*> ObjectIndividuals;

	// Timepoint individuals
	TArray<FString> TimepointIndividuals;

	// Metadata individual semantic event
	RSemEvent* Metadata;
};

