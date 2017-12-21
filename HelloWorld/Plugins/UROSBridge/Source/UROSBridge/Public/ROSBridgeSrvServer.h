// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "Core.h"
#include "Json.h"

#include "ROSBridgeSrv.h"

class UROSBRIDGE_API FROSBridgeSrvServer 
{
protected:
    FString Name; 
    FString Type; 

public:
    FROSBridgeSrvServer() {}

	virtual ~FROSBridgeSrvServer() {}

    FROSBridgeSrvServer(FString Name_, FString Type_): Name(Name_), Type(Type_) {}

    FString GetName() const { return Name; }

    FString GetType() const { return Type; }

    virtual TSharedPtr<FROSBridgeSrv::SrvRequest> FromJson(TSharedPtr<FJsonObject> JsonObject) const = 0;
    
    virtual TSharedPtr<FROSBridgeSrv::SrvResponse> CallBack(TSharedPtr<FROSBridgeSrv::SrvRequest> Request) const = 0;
}; 
