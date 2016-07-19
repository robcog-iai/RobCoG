#include "RobCoG.h"
#include "OwlEvent.h"

// Constructor
FOwlEvent::FOwlEvent(
	FString Namespace,
	FString Name,
	FString UniqueHash,
	float Start,
	float End) :
	EventNamespace(Namespace),
	EventName(Name),
	EventUniqueHash(UniqueHash),
	StartTime(Start),
	EndTime(End)
{
	Finished = false;
}

// Set event name
void FOwlEvent::SetName(FString Name)
{
	EventName = Name;
}

// Get event name
FString FOwlEvent::GetName()
{
	return EventName;
}

// Set event namespace
void FOwlEvent::SetNamespace(FString Namespace)
{
	EventNamespace = Namespace;
}

// Get event namespace
FString FOwlEvent::GetNamespace()
{
	return EventNamespace;
}

// Set event name unique hash
void FOwlEvent::SetUniqueHash(FString UniqueHash)
{
	EventUniqueHash = UniqueHash;
}

// Get event name unique hash
FString FOwlEvent::GetUniqueHash()
{
	return EventUniqueHash;
}

// Get the full name of the event (ns + name + hash)
FString FOwlEvent::GetFullName()
{
	return EventNamespace + EventName + EventUniqueHash;
}

// Set finished
void FOwlEvent::SetFinshed(bool Fini)
{
	Finished = Fini;
}

// Get finished
bool FOwlEvent::IsFinished()
{
	return Finished;
}

// Get properties
TArray<OwlTriple>& FOwlEvent::GetProperties()
{
	return EventProperties;
}

// Add property to the event
void FOwlEvent::AddProperty(
	const FString Subject, 
	const FString Predicate, 
	const FString Object, 
	const FString Value)
{
	// Create and add the OWL triple to the property array
	OwlTriple PropertyTriple(Subject, Predicate, Object, Value);
	EventProperties.Add(PropertyTriple);
};

// Get duration
float FOwlEvent::GetDuration()
{
	if (Finished)
	{
		return EndTime - StartTime;
	}
	else
	{
		return -1.0f;
	};
}

// Set context
void FOwlEvent::SetContext(const FString Context)
{
	EventContext = Context;
}

// Get event context
FString FOwlEvent::GetContext()
{
	return EventContext;
}