// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "RCloseDrawers.generated.h"

UCLASS()
class ROBCOG_API ARCloseDrawers : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARCloseDrawers();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
};
