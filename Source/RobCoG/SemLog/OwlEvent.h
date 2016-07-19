#pragma once
#include "OwlEvent.generated.h"

// Subject, Predicate, Object, Value
//typedef TTuple<FString, FString, FString, FString> UOwlTriple;
typedef TTuple<FString, FString, FString, FString> OwlTriple;

// Owl events
USTRUCT()
struct FOwlEvent
{
	GENERATED_USTRUCT_BODY()

	// Constructor
	FOwlEvent(
		FString Namespace = "",
		FString Name = "",
		FString UniqueHash = "",
		float Start = -1.0f,
		float End = -1.0f);

	// Event name
	UPROPERTY()
		FString EventName;

	// Event namespace
	UPROPERTY()
		FString EventNamespace;

	// Event unique hash
	UPROPERTY()
		FString EventUniqueHash;

	// Start timepoint
	UPROPERTY()
		float StartTime;

	// End timepoint
	UPROPERTY()
		float EndTime;

	// Event finished flag
	UPROPERTY()
		bool Finished;

	// Array of OWLTriple(TTuple) + Value
	TArray<OwlTriple> EventProperties;

	// Context of the event
	FString EventContext;

	// Set event name
	void SetName(FString Name);

	// Get event name
	FString GetName();

	// Set event namespace
	void SetNamespace(FString Namespace);

	// Get event namespace
	FString GetNamespace();

	// Set event name unique hash
	void SetUniqueHash(FString UniqueHash);

	// Get event name unique hash
	FString GetUniqueHash();

	// Get the full name of the event (ns + name + hash)
	FString GetFullName();

	// Set finished
	void SetFinshed(bool Fini);

	// Get finished
	bool IsFinished();

	// Get properties
	TArray<OwlTriple>& GetProperties();

	// Add property to the event
	void AddProperty(
		const FString Subject,
		const FString Predicate,
		const FString Object,
		const FString Value = "");

	// Get duration
	float GetDuration();

	// Set context
	void SetContext(const FString Context);

	// Get event context
	FString GetContext();
};