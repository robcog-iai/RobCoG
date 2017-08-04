// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/TriggerBox.h"

#include "GraspingGame.generated.h"

UCLASS()
class ROBCOG_API AGraspingGame : public AActor
{
	GENERATED_BODY()
	
public:	
	UPROPERTY(EditAnywhere)
		ATriggerBox* SpwaningBox;

	UPROPERTY(EditAnywhere)
		ATriggerBox* TargetBox;


	// Sets default values for this actor's properties
	AGraspingGame();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};
