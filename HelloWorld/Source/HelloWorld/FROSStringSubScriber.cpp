// Fill out your copyright notice in the Description page of Project Settings.

#include "FROSStringSubScriber.h"

FROSStringSubScriber::FROSStringSubScriber(FString Topic_) :
	FROSBridgeSubscriber(TEXT("std_msgs/String"), Topic_) {
}

FROSStringSubScriber::~FROSStringSubScriber()
{
}

TSharedPtr<FROSBridgeMsg> FROSStringSubScriber::ParseMessage
(TSharedPtr<FJsonObject> JsonObject) const
{
	TSharedPtr<FROSBridgeMsgStdmsgsString> StringMessage =
		MakeShareable<FROSBridgeMsgStdmsgsString>(new FROSBridgeMsgStdmsgsString());
	StringMessage->FromJson(JsonObject);
	return StaticCastSharedPtr<FROSBridgeMsg>(StringMessage);
}

void FROSStringSubScriber::CallBack(TSharedPtr<FROSBridgeMsg> msg) const
{
	TSharedPtr<FROSBridgeMsgStdmsgsString> StringMessage = StaticCastSharedPtr<FROSBridgeMsgStdmsgsString>(msg);
	// downcast to subclass using StaticCastSharedPtr function

	UE_LOG(LogTemp, Log, TEXT("Message received! Content: %s"), *StringMessage->GetData());
	// do something with the message

	return;
}
