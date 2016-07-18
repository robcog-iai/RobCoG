#pragma once
#include "RPid.generated.h"

/* 
PID controller implementation

Error: where you want to be vs. where you are
This is certainly a big factor. If you are at point A and 
your target is at point B, then the vector from A to B 
tells you a lot about how you need to steer, but it isn't the only factor.

Derivative: how fast you are approaching
If you are approaching the target quickly and you are close to it,
you need to slow down. The derivative helps take that into consideration.

Integral: alignment error
Your robot may not actually do exactly what you tell it to do. 
The integral helps determine how much you need to compensate for that.
*/
USTRUCT()
struct FRPid
{
	GENERATED_USTRUCT_BODY()

	// Constructor
	FRPid(float _P = 0.0f, float _I = 0.0f, float _D = 0.0f,
		float _OutMax = 0.0, float _OutMin = 0.0f);

	// Destructor
	~FRPid();

	// Initialise all PID values
	void Init(float _P = 0.0f, float _I = 0.0f, float _D = 0.0f,
		float _OutMax = 0.0, float _OutMin = 0.0f);

	// Update the PID loop, 
	// Error is the difference between the current state and the target
	// DeltaT is the change in time since the last update
	float Update(const float Error, const float DeltaT);

	// Reset all values of the PID
	void Reset();

	// Proportional gain
	float P;

	// Integral gain
	float I;

	// Derivative gain
	float D;

	// Output maximul clamping value
	float OutMax;

	// Output minimum claming value
	float OutMin;

	// Previous step error value
	float PrevErr;

	// Integral error
	float IErr;
};