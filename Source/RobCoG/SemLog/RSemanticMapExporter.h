// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "RSemanticMapExporter.generated.h"

UCLASS()
class ROBCOG_API ARSemanticMapExporter : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARSemanticMapExporter();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(EditAnywhere, Category = "SemanticMap")
	// Map directory name
	FString MapDirName;

	UPROPERTY(EditAnywhere, Category = "SemanticMap")
	// Map name
	FString MapName;

private:
	// Create folders for logging
	void CreateMapDir();

	// File handle to write the semantic map
	TSharedPtr<IFileHandle> MapFileHandle;
};
