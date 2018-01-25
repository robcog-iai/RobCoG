// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "Enums/GraspType.h"
#include "Structs/HandOrientation.h"
#include "Structs/HandVelocity.h"


/**
 * This class parses the HandOrientation for all Grasp types out of ini files
 */
class UFORCEBASEDGRASPING_API HandInformationParser
{

public:
	//Constuctor
	HandInformationParser();
	
	// Destructor
	~HandInformationParser();

	// Reads the initial and the closed hand orientation out of the ini file
	void GetHandInformationForGraspType(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrietation, FHandVelocity & HandVelocity, const FString ConfigPath);

	// This funtion is able to create an ini file for a new grasp type
	void SetHandInformationForGraspType(const FHandOrientation & InitialHandOrientation, const FHandOrientation & ClosedHandOrietation, const FHandVelocity & HandVelocity, const FString ConfigPath);

private:
	// The name of the init section
	const FString InitOrientationSection = "InitialHandOrientation";

	// The name of the closed section
	const FString ClosedOrientationSection = "ClosedHandOrientation";

	// The name of the Velocity section
	const FString VelocitySection = "HandVelocity";

	// This shared pointer contains the config file
	TSharedPtr<FConfigCacheIni> ConfigFileHandler;

	// Writes an ini file for a grasptype
	void WriteGraspTypeIni(const FHandOrientation & InitialHandOrientation, const FHandOrientation & ClosedHandOrientation, const FHandVelocity & HandVelocity, const FString ConfigPath);

	// Reads the initial and the closed hand orientation out of the ini file
	void ReadGraspTypeIni(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrientation, FHandVelocity & HandVelocity, const FString ConfigPath);
};
