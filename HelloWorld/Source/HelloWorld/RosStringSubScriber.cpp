// Fill out your copyright notice in the Description page of Project Settings.

#include "RosStringSubScriber.h"


// Sets default values
ARosStringSubScriber::ARosStringSubScriber()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	WebsocketIPAddr = TEXT("192.168.101.171");

	// Set Port to 9090
	WebsocketPort = 9090;

	// Set rostopic name which publish strings
	StringSubTopic = TEXT("Ros_String");

}

// Called when the game starts or when spawned
void ARosStringSubScriber::BeginPlay()
{
	Super::BeginPlay();

	// Set websocket server address to ws ://192.168.101.171:9090
	Handlersub = MakeShareable<FROSBridgeHandler>(new FROSBridgeHandler(WebsocketIPAddr, WebsocketPort));

	Subscriber = MakeShareable<FROSStringSubScriber>(new FROSStringSubScriber(StringSubTopic));
	
	Handlersub->AddSubscriber(Subscriber);

	//Connect to ROSBridge Websocket server.
	Handlersub->Connect();
	
}

// Called every frame
void ARosStringSubScriber::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	Handlersub->Render();



}

void ARosStringSubScriber::EndPlay(const EEndPlayReason::Type Reason)
{
	
	// Disconnect the handler before parent ends
	Handlersub->Disconnect();
	Super::EndPlay(Reason);
}

