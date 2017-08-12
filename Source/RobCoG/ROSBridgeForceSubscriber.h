// Fout your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RRobot.h"
#include "ROSBridgeHandler.h"
#include "ROSBridgeSubscriber.h"
#include "sensor_msgs/JointState.h"

class FROSBridgeForceSubScriber : public FROSBridgeSubscriber {

public:
    ARRobot* Robot;

    FROSBridgeForceSubScriber(FString Topic_, ARRobot* Robot_) :
        FROSBridgeSubscriber(TEXT("sensor_msgs/JointState"), Topic_)
    {
        Robot = Robot_;
    }

    ~FROSBridgeForceSubScriber() override {};

    TSharedPtr<FROSBridgeMsg> ParseMessage(TSharedPtr<FJsonObject> JsonObject) const override
    {
        TSharedPtr<FROSBridgeMsgSensormsgsJointState> JointStateMessage =
            MakeShareable<FROSBridgeMsgSensormsgsJointState>(new FROSBridgeMsgSensormsgsJointState());
        JointStateMessage->FromJson(JsonObject);

        return StaticCastSharedPtr<FROSBridgeMsg>(JointStateMessage);
    }

    void CallBack(TSharedPtr<FROSBridgeMsg> msg) const override
    {
        TSharedPtr<FROSBridgeMsgSensormsgsJointState> JointStateMessage = StaticCastSharedPtr<FROSBridgeMsgSensormsgsJointState>(msg);

        TArray<FString> ListJointName = JointStateMessage->GetName();
        TArray<double> ListJointForce = JointStateMessage->GetEffort();
        UE_LOG(LogTemp, Warning, TEXT("ListJointName.Num() = %d"), ListJointName.Num());

        checkf(ListJointName.Num() == ListJointForce.Num(), TEXT("Error: Length of JointName and JointForce aren't equal."));

        for (int i = 0; i < ListJointName.Num(); i++)
        {
            FString JointName = ListJointName[i];
            double JointEffort = ListJointForce[i];
            Robot->AddForceToJoint(JointName, JointEffort * 10000);
        }
        return;
    }

};
