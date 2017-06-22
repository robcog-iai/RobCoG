// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#include "PIDController.h"

// Default constructor
PIDController::PIDController()
{

}

// Constructor
PIDController::PIDController(float ProportionalVal, float IntegralVal, float DerivativeVal,
	float OutMaxVal, float OutMinVal) :
	P(ProportionalVal),
	I(IntegralVal),
	D(DerivativeVal),
	OutMax(OutMaxVal),
	OutMin(OutMinVal)
{
	IErr = 0.f;
	PrevErr = 0.f;
}

// Destructor
PIDController::~PIDController()
{
}

// Set all PID values
void PIDController::SetValues(float ProportionalVal, float IntegralVal, float DerivativeVal,
	float OutMaxVal, float OutMinVal)
{
	P = ProportionalVal;
	I = IntegralVal;
	D = DerivativeVal;
	OutMax = OutMaxVal;
	OutMin = OutMinVal;

	PIDController::Reset();
};

// Update the PID loop
float PIDController::Update(const float Error, const float DeltaTime)
{
	if (DeltaTime == 0.0f || FMath::IsNaN(Error))
	{
		return 0.0f;
	}

	// Calculate proportional output
	const float POut = P * Error;

	// Calculate integral error / output
	IErr += DeltaTime * Error;
	const float IOut = I * IErr;

	// Calculate the derivative error / output
	const float DErr = (Error - PrevErr) / DeltaTime;
	const float DOut = D * DErr;

	// Set previous error
	PrevErr = Error;

	// Calculate the output
	const float Out = POut + IOut + DOut;

	// Clamp output to max/min values
	if (OutMax > 0.f || OutMin < 0.f)
	{
		if (Out > OutMax)
			return OutMax;
		else if (Out < OutMin)
			return OutMin;
	}

	return Out;
};

// Update only P
float PIDController::UpdateAsP(const float Error, const float DeltaTime)
{
	if (DeltaTime == 0.0f || FMath::IsNaN(Error))
	{
		return 0.0f;
	}

	// Calculate proportional output
	const float Out = P * Error;

	// Clamp output to max/min values
	if (OutMax > 0.f || OutMin < 0.f)
	{
		if (Out > OutMax)
			return OutMax;
		else if (Out < OutMin)
			return OutMin;
	}
	return Out;
};

// Update only PD
float PIDController::UpdateAsPD(const float Error, const float DeltaTime)
{
	if (DeltaTime == 0.0f || FMath::IsNaN(Error))
	{
		return 0.0f;
	}

	// Calculate proportional output
	const float POut = P * Error;

	// Calculate the derivative error / output
	const float DErr = (Error - PrevErr) / DeltaTime;
	const float DOut = D * DErr;

	// Set previous error
	PrevErr = Error;

	// Calculate the output
	const float Out = POut + DOut;

	// Clamp output to max/min values
	if (OutMax > 0.f || OutMin < 0.f)
	{
		if (Out > OutMax)
			return OutMax;
		else if (Out < OutMin)
			return OutMin;
	}

	return Out;
};

// Update only PI
float PIDController::UpdateAsPI(const float Error, const float DeltaTime)
{
	if (DeltaTime == 0.0f || FMath::IsNaN(Error))
	{
		return 0.0f;
	}

	// Calculate proportional output
	const float POut = P * Error;

	// Calculate integral error / output
	IErr += DeltaTime * Error;
	const float IOut = I * IErr;
	
	// Calculate the output
	const float Out = POut + IOut;

	// Clamp output to max/min values
	if (OutMax > 0.f || OutMin < 0.f)
	{
		if (Out > OutMax)
			return OutMax;
		else if (Out < OutMin)
			return OutMin;
	}
	return Out;
};

// Reset error values of the PID
void PIDController::Reset()
{
	PrevErr = 0.0f;
	IErr = 0.0f;
};