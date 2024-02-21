// Copyright 2017-2021, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "Runtime/Engine/Classes/Engine/DataTable.h"
#include "TrialStruct.generated.h"


USTRUCT(BlueprintType)
struct FTrialStruct : public FTableRowBase
{
    GENERATED_USTRUCT_BODY()
    public:
        FTrialStruct(){}

        UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TrialSetup")
        float RotationSpeed;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TrialSetup")
        float PouringAngle;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TrialSetup")
        int TrialAmount;
};