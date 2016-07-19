// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "Animation/SkeletalMeshActor.h"
#include "Runtime/Engine/Classes/Engine/TriggerBase.h"
#include "Runtime/Core/Public/Misc/DateTime.h"
#include "rapidjson/document.h"
#include "rapidxml/rapidxml.hpp"
#include <algorithm>	// used for generating unique hash
#include "OwlEvent.h"
#include "RSemLogManager.generated.h"

UCLASS()
class ROBCOG_API ARSemLogManager : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARSemLogManager();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	
	// Called when the game is terminated
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Right hand grasp blueprint callable function
	UFUNCTION(BlueprintCallable, Category = "LoggerActor")
	void RightGraspLog(AActor* OtherActor);

	// Right hand release blueprint callable function
	UFUNCTION(BlueprintCallable, Category = "LoggerActor")
	void RightReleaseLog();

	// Left hand grasp blueprint callable function
	UFUNCTION(BlueprintCallable, Category = "LoggerActor")
	void LeftGraspLog(AActor* OtherActor);

	// Left hand release blueprint callable function
	UFUNCTION(BlueprintCallable, Category = "LoggerActor")
	void LeftReleaseLog();

	/** Log the semantic map */
	UPROPERTY(EditAnywhere, Category = "Logging")
	bool LogSemanticMap;

	/** Log the semantic events */
	UPROPERTY(EditAnywhere, Category = "Logging")
	bool LogSemanticData;

	/** Log the semantic raw data */
	UPROPERTY(EditAnywhere, Category = "Logging")
	bool LogRawData;

	/** Log the episode rating */
	UPROPERTY(EditAnywhere, Category = "Logging")
	bool LogEpisodeRating;

	/** Log the timeline visualisation */
	UPROPERTY(EditAnywhere, Category = "Logging")
	bool LogTimelines;

	/** The squared distance variation for which to record the raw data 
	(the minimum distance the object moved in order to log it) */
	UPROPERTY(EditAnywhere, Category = "Logging")
	float DistSquaredThresh;

private:
	// On island trigger overlap delegate
	UFUNCTION()
	void OnIslandTriggerOverlapBegin(AActor* OverlappedActor, AActor* OtherActor);

	// On island trigger overlap end delegate
	UFUNCTION()
	void OnIslandTriggerOverlapEnd(AActor* OverlappedActor, AActor* OtherActor);

	// On sink trigger overlap delegate
	UFUNCTION()
	void OnSinkTriggerOverlapBegin(AActor* OverlappedActor, AActor* OtherActor);

	// On sink trigger overlap end delegate
	UFUNCTION()
	void OnSinkTriggerOverlapEnd(AActor* OverlappedActor, AActor* OtherActor);

	// Init the objects/charaters/events to be logged
	void InitThingsToLog();

	// Tick raw data
	void TickRawData();

	// Create folders for logging
	void CreateLogDir();

	// Write the semantic map to owl file
	void WriteSemanticMap();

	// Write the episode rating to owl file
	void WriteEpisodeRating();

	// Write the episode events to owl file
	void WriteEvents();

	// Write metadata OWL 
	void AddMetaData();

	// Write the timelines
	void WriteTimelines();

	// Start touching situation
	FOwlEvent StartTouchingSituation(AActor* Trigger, AActor* OtherActor);

	// Change FString to std::string to char* (TODO report issue)
	inline char* FStrToChar(const FString FStr);

	// Random string generator 
	// http://stackoverflow.com/questions/440133/how-do-i-create-a-random-alpha-numeric-string-in-c
	FString RandString(const int32 Length);

	// Generate a random string with an underscore
	FString RandStringUnderscore(const int32 Length);

	// Add Owl triple
	rapidxml::xml_node<>* AddTriple(
		rapidxml::xml_document<>* OwlDoc,
		const FString Subject,
		const FString Predicate,
		const FString Object,
		const FString Value = "");

	// Add Parent-Child nested triple
	rapidxml::xml_node<>* AddParentChildTriple(
		rapidxml::xml_document<>* OwlDoc,
		const FString ParentSubject,
		const FString ParentPredicate,
		const FString ParentObject,
		const FString ChildSubject,
		const FString ChildPredicate,
		const FString ChildObject,
		const FString ChildValue = "");

	// Add Owl event with a property
	rapidxml::xml_node<>* AddEvent(
		rapidxml::xml_document<>* OwlDoc,
		const FString Subject,
		const FString Predicate,
		const FString Object,
		const OwlTriple Property);

	// Add Owl event with multiple properties
	rapidxml::xml_node<>* AddEvent(
		rapidxml::xml_document<>* OwlDoc,
		const FString Subject,
		const FString Predicate,
		const FString Object,
		const TArray<OwlTriple>& Properties);

	// Add Owl comment
	void AddOwlComment(
		rapidxml::xml_document<>* OwlDoc,
		rapidxml::xml_node<>* RDFNode,
		const FString CommentStr);

	// Add Json Actor Member
	rapidjson::Value AddJsonActor(
		AActor* Actor,
		const FString ClassType,
		rapidjson::Document::AllocatorType& JsonAllocator);

	// Add Json ActorComponent Member
	rapidjson::Value AddJsonComponent(
		UPrimitiveComponent* Component,
		const FString ClassType,
		rapidjson::Document::AllocatorType& JsonAllocator);

	// Add Json Bone Member
	rapidjson::Value AddJsonBone(
		USkeletalMeshComponent* SkeletalComponent,
		const FName BoneName,
		rapidjson::Document::AllocatorType& JsonAllocator);

	// Skeleton components and class type (from tags), with prev loc and orient. to log
	TArray<TTuple<USkeletalMeshComponent*, FString, FVector*, FRotator*>> LogSkelComp;

	// Actors and their class type name (from tags), with prev loc and orient. to log
	TArray<TTuple<AActor*, FString, FVector*, FRotator*>> LogActors;

	// Actors and their class type name (from tags) (will only be logged once)
	TArray<TTuple<AActor*, FString>> StaticMapActors;

	// Elapsed time
	float ElapsedTime;

	// flag for checking if the static map has been logged
	bool StaticMapLogged;

	// Log directory name
	FString LogDirName;
	
	// Episode folder name
	FString EpisodeDirName;

	// Raw data file name
	FString RawDataFileName;

	// File handle to append raw data
	TSharedPtr<IFileHandle> RawFileHandle;	

	// Owl event data file name
	FString EventsFileName;

	// Owl file handle to append event data
	TSharedPtr<IFileHandle> EventsFileHandle;

	// Semantic map file name
	FString SemanticMapFileName;

	// Semantic map file handle to write the semantic map
	TSharedPtr<IFileHandle> SemanticMapFileHandle;

	// Episode rating file name
	FString EpRatingFileName;

	// Episode rating file handle
	TSharedPtr<IFileHandle> EpRatingFileHandle;

	// Semantic map file name
	FString TimelineFileName;

	// Semantic map file handle to write the semantic map
	TSharedPtr<IFileHandle> TimelineFileHandle;
	
	// Property type definitions
	TArray<FString> PropertyDefinitions;

	// Class type definitions
	TArray<FString> ClassDefinitions;

	// Event individuals
	TArray<FOwlEvent> EventIndividuals;

	// Object individuals Class type and Instance
	TArray< TTuple<FString, FString> > ObjectIndividuals;

	// Timepoint individuals
	TArray<FString> TimepointIndividuals;

	// Metadata individual
	FOwlEvent* MetaDataIndividual;

	// Episode number
	int32 EpisodeNr;

	// Map of actor to its unique name
	TMap<AActor*, FString> ActorToUniqueName;

	// Map of actor to its class type, no namespace
	TMap<AActor*, FString> ActorToClassType;

	// Map of Open Touch Situations
	TMap<FString, FOwlEvent> UnfinishedTouchEvents;

	// Unifinished right grasp
	FOwlEvent* RightGraspEvent;

	// Unifinished left grasp
	FOwlEvent* LeftGraspEvent;

	// Island triger area
	ATriggerBase* IslandTriggerArea;

	// Sink triger area
	ATriggerBase* SinkTriggerArea;
};
