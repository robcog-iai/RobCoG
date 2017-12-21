// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSBridgeHandler.h"
#include "ROSBridgeSubscriber.h"
#include "std_msgs/String.h"

/**
 * 
 */
class HELLOWORLD_API FROSStringSubScriber : public FROSBridgeSubscriber {
public:
	FROSStringSubScriber(FString Topic_);
	~FROSStringSubScriber() override;
	TSharedPtr<FROSBridgeMsg> ParseMessage(TSharedPtr<FJsonObject> JsonObject) const override;
	void CallBack(TSharedPtr<FROSBridgeMsg> msg) const override;
};
