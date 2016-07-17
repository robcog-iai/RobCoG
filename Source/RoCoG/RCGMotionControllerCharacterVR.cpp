// Fill out your copyright notice in the Description page of Project Settings.

#include "RoCoG.h"
#include "RCGMotionControllerCharacterVR.h"


// Sets default values
ARCGMotionControllerCharacterVR::ARCGMotionControllerCharacterVR()
{
 	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ARCGMotionControllerCharacterVR::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ARCGMotionControllerCharacterVR::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

// Called to bind functionality to input
void ARCGMotionControllerCharacterVR::SetupPlayerInputComponent(class UInputComponent* InputComponent)
{
	Super::SetupPlayerInputComponent(InputComponent);

}

