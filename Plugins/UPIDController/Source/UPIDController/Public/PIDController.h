// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

/**
PID Controller
Error: where you are vs where you want to be 
Derivative: how fast you are approaching, dampening
Integral: alignment error
 */
class UPIDCONTROLLER_API PIDController
{
public:
	// Default constructor
	PIDController();

	// Constructor
	PIDController(float ProportionalVal, float IntegralVal, float DerivativeVal,
		float OutMaxVal = 0.f, float OutMinVal = 0.f);

	// Destructor
	~PIDController();

	// Set all PID values
	void SetValues(float ProportionalVal = 0.f, float IntegralVal = 0.f, float DerivativeVal = 0.f,
		float OutMaxVal = 0.f, float OutMinVal = 0.f);

	// Update the PID loop
	float Update(const float Error, const float DeltaTime);

	// Update only P
	float UpdateAsP(const float Error, const float DeltaTime);

	// Update only PD
	float UpdateAsPD(const float Error, const float DeltaTime);

	// Update only PÍ
	float UpdateAsPI(const float Error, const float DeltaTime);

	// Reset error values of the PID
	void Reset();

private:
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
