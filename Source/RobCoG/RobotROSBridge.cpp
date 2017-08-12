// Fill out your copyright notice in the Description page of Project Settings.

#include "RobotROSBridge.h"
#include "RobCoG.h"
#include "sensor_msgs/JointState.h"

// Sets default values
ARobotROSBridge::ARobotROSBridge()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PrePhysics;

    WebsocketIPAddr = TEXT("127.0.0.1");
    WebsocketPort = 9001;
    RobotName = TEXT("pr2_base");
    RobotJointStateTopic = TEXT("unreal_joint_state");
    RobotEffortTopic = TEXT("unreal_force");

    TickCount = 0;
}

// Called when the game starts or when spawned
void ARobotROSBridge::BeginPlay()
{
	Super::BeginPlay();

    bool bFoundActor = false;
    Robot = nullptr;
    for (TActorIterator<ARRobot> ActorItr(GetWorld()); ActorItr; ++ActorItr)
    {
        UE_LOG(LogTemp, Log, TEXT("Actor Label = [%s]"), *ActorItr->GetActorLabel());
        if (ActorItr->GetActorLabel() == RobotName)
        {
            Robot = *ActorItr;
            bFoundActor = true;
            break;
        }
    }

    checkf(bFoundActor, TEXT("Robot Actor with Name [%s] not found!"), *RobotName);

    Handler = MakeShareable<FROSBridgeHandler>(new FROSBridgeHandler(WebsocketIPAddr, WebsocketPort));
    RobotJointStatePublisher = MakeShareable<FROSBridgePublisher>
            (new FROSBridgePublisher(TEXT("sensor_msgs/JointState"), RobotJointStateTopic));
    RobotForceSubscriber = MakeShareable<FROSBridgeForceSubScriber>
            (new FROSBridgeForceSubScriber(RobotEffortTopic, Robot));
    Handler->AddPublisher(RobotJointStatePublisher);
    Handler->AddSubscriber(RobotForceSubscriber);
    Handler->Connect();
}

// Called every frame
void ARobotROSBridge::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

    Handler->Render();

    TArray<FString> ListJointName;
    TArray<double> ListJointPosition, ListJointVelocity;

    for (auto &JointElement : Robot->JointComponents)
    {
        FString JointName = JointElement.Value->GetName();
        float JointPosition = Robot->GetJointPosition(JointElement.Key);
        float JointVelocity = Robot->GetJointVelocity(JointElement.Key);\

        ListJointName.Add(JointName);
        ListJointPosition.Add(JointPosition);
        ListJointVelocity.Add(JointVelocity);
    }
    
    TSharedPtr<FROSBridgeMsgSensormsgsJointState> JointState =
            MakeShareable(new FROSBridgeMsgSensormsgsJointState());
    JointState->SetHeader(FROSBridgeMsgStdmsgsHeader(++TickCount, FROSTime(), TEXT("0")));
    JointState->SetName(ListJointName);
    JointState->SetPosition(ListJointPosition);
    JointState->SetVelocity(ListJointVelocity);

    Handler->PublishMsg(RobotJointStateTopic, JointState);

    UE_LOG(LogTemp, Log, TEXT("JointState = %s"), *JointState->ToString());
}

void ARobotROSBridge::EndPlay(const EEndPlayReason::Type Reason)
{
    Handler->Disconnect();

    Super::EndPlay(Reason);
}

