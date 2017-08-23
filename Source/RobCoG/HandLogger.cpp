// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "HandLogger.h"
#include "GameFramework/Actor.h"


// Sets default values
HandLogger::HandLogger()
{
}

void HandLogger::UpdateTimer()
{
	UE_LOG(LogTemp, Warning, TEXT("UpdateTimer"));

		// We're done counting down, so stop running the timer.
		//GetWorldTimerManager().ClearTimer(StartTimerHandle);

}

void HandLogger::TimerHasFinished()
{
	UE_LOG(LogTemp, Warning, TEXT("TimerHasFinished"));

	//GetWorldTimerManager().SetTimer(GameTimerHandle, this, &AGraspingGame::UpdateGameTimer, 1.0f, true);

}
