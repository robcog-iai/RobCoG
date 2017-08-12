// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#include "RobCoGGameModeBase.h"

#include "ROSBridgeHandler.h"
#include "ROSBridgePublisher.h"
#include "rosgraph_msgs/Clock.h"

ARobCoGGameModeBase::ARobCoGGameModeBase()
	: Super()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ARobCoGGameModeBase::BeginPlay()
{
    Handler = MakeShareable<FROSBridgeHandler>(new FROSBridgeHandler(TEXT("127.0.0.1"), 9001));
    TimePublisher = MakeShareable<FROSBridgePublisher>(new FROSBridgePublisher(TEXT("rosgraph_msgs/Clock"), TEXT("clock")));
    Handler->AddPublisher(TimePublisher);
    Handler->Connect();
    UE_LOG(LogTemp, Log, TEXT("[ARobCoGGameModeBase::BeginPlay()] Websocket server connected."));
}

void ARobCoGGameModeBase::Tick(float DeltaSeconds)
{
    float GameTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    uint64 GameSeconds = (int)GameTime;
    uint64 GameUseconds = (GameTime - GameSeconds) * 1000000000;
    TSharedPtr<FROSBridgeMsgRosgraphmsgsClock> Clock = MakeShareable
            (new FROSBridgeMsgRosgraphmsgsClock(FROSTime(GameSeconds, GameUseconds)));
    Handler->PublishMsg("clock", Clock);
}

void ARobCoGGameModeBase::Logout(AController *Exiting)
{
    Handler->Disconnect();
    AGameModeBase::Logout(Exiting);
}
