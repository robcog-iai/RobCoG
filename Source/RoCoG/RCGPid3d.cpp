#include "RoCoG.h"
#include "RCGPid3d.h"

// Constructor
FRCGPid3d::FRCGPid3d(float _P, float _I, float _D,
	float _OutMax, float _OutMin) :
	P(_P), I(_I), D(_D), OutMax(_OutMax), OutMin(_OutMin)
{
	IErr = FVector(0.0f, 0.0f, 0.0f);
	PrevErr = FVector(0.0f, 0.0f, 0.0f);
}

// Destructor
FRCGPid3d::~FRCGPid3d()
{
}

// Init
void FRCGPid3d::Init(float _P, float _I, float _D,
	float _OutMax, float _OutMin)
{
	P = _P;
	I = _I;
	D = _D;
	OutMax = _OutMax;
	OutMin = _OutMin;

	FRCGPid3d::Reset();
}

// Update the PID loop
FVector FRCGPid3d::Update(const FVector Error, const float DeltaT)
{
	if (DeltaT == 0.0f)
	{
		return FVector(0.0f, 0.0f, 0.0f);
	}

	// Calculate proportional output
	const FVector POut = P * Error;

	// Calculate integral error / output
	IErr += DeltaT * Error;
	const FVector IOut = I * IErr;

	// Calculate the derivative error / output
	const FVector DErr = (Error - PrevErr) / DeltaT;
	const FVector DOut = D * DErr;

	// Set previous error
	PrevErr = Error;
	
	// Calculate Output
	const FVector Out = POut + IOut + DOut;

	// If Out>Outmax --> return Outmax, else if Out<Outmin --> return Outmin, else return Out
	const FVector ClampedOut = FVector(
		(Out.X > OutMax) ? (OutMax) : (Out.X < OutMin) ? (OutMin) : (Out.X),
		(Out.Y > OutMax) ? (OutMax) : (Out.X < OutMin) ? (OutMin) : (Out.Y),
		(Out.Z > OutMax) ? (OutMax) : (Out.X < OutMin) ? (OutMin) : (Out.Z)
	);

	return ClampedOut;
}

// Reset
void FRCGPid3d::Reset()
{
	IErr = FVector(0.0f, 0.0f, 0.0f);
	PrevErr = FVector(0.0f, 0.0f, 0.0f);
}