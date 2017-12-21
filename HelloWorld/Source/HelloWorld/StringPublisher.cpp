// Fill out your copyright notice in the Description page of Project Settings.

#include "StringPublisher.h"


// Sets default values
AStringPublisher::AStringPublisher()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Set websocket server address to the ros IP address
	WebsocketIPAddr = TEXT("192.168.101.171");

	// Set Port to 9090
	WebsocketPort = 9090;
	
	// Set rostopic name which publish strings
	StringPubTopic = TEXT("Unreal_String");




}

// Called when the game starts or when spawned
void AStringPublisher::BeginPlay()
{
	Super::BeginPlay();

	// Set websocket server address to ws ://192.168.101.171:9090
	Handler = MakeShareable<FROSBridgeHandler>(new FROSBridgeHandler(WebsocketIPAddr, WebsocketPort));

	// **** Create publishers here ****
	Publisher = MakeShareable<FROSBridgePublisher>(new FROSBridgePublisher(TEXT("std_msgs/String"), StringPubTopic));
	Handler->AddPublisher(Publisher);
	
	

	//Connect to ROSBridge Websocket server.
	Handler->Connect();
	
}

// Called every frame
void AStringPublisher::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	Handler->Render();
	

	FString Textdata = "Hello World";

	TSharedPtr<FROSBridgeMsgStdmsgsString> UnrealString = MakeShareable(new FROSBridgeMsgStdmsgsString());

	UnrealString->SetData(Textdata);

	Handler->PublishMsg(StringPubTopic, UnrealString);

	UE_LOG(LogTemp, Log, TEXT("StringMessage = %s"), *UnrealString->ToString());
	

}

void AStringPublisher::EndPlay(const EEndPlayReason::Type Reason)
{
	
	// Disconnect the handler before parent ends
	Handler->Disconnect();
	Super::EndPlay(Reason);
}

