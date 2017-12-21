// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSBridgeHandler.h"
#include "ROSBridgePublisher.h"
#include "FROSStringSubScriber.h"
#include "std_msgs/String.h"
#include "StringPublisher.generated.h"

UCLASS()
class HELLOWORLD_API AStringPublisher : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AStringPublisher();

	UPROPERTY(EditAnywhere)
		FString WebsocketIPAddr;

	UPROPERTY(EditAnywhere)
		uint32 WebsocketPort;

	UPROPERTY(EditAnywhere)
		FString StringPubTopic;


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

	// Add a smart pointer to ROSBridgeHandler
	TSharedPtr<FROSBridgeHandler> Handler;

	// Add a ROSBridgePublisher smart pointer
	TSharedPtr<FROSBridgePublisher> Publisher;

	


	
	
};
