// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

/**
 * Semantic map exporter
 */
class ROBCOG_API FRSemMapExporter
{
public:
	// Constructor
	FRSemMapExporter();

	// Destructor
	~FRSemMapExporter();

	// Generate and write semantic map
	void WriteSemanticMap(
		const TMap<AStaticMeshActor*, FString>& DynamicActPtrToUniqNameMap,
		const TMap<AStaticMeshActor*, FString>& StaticActPtrToUniqNameMap,
		const TMap<AActor*, FString>& ActorToClassTypeMap,
		const FString Path);

	// Get semantic map unique name
	FString GetUniqueName();

private:
	// Unique name of the map
	FString UniqueName;
};

