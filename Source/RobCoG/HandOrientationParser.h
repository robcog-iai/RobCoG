// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "GraspType.h"
#include "Structs/HandOrientation.h"


/**
 * This class parses the HandOrientation for all Grasp types out of ini files
 */
class ROBCOG_API HandOrientationParser
{

public:
	HandOrientationParser();
	~HandOrientationParser();

	// Reads the initial and the closed hand orientation out of the ini file
	void GetHandOrientationsForGraspType(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrietation, EGraspType GraspType);



private:
	// This shared pointer contains the config file
	TSharedPtr<FConfigCacheIni> ConfigFileHandler;

	// Writes an ini file for a grasptype
	void WriteGraspTypeIni(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrientation, EGraspType GraspType);
	// Reads the initial and the closed hand orientation out of the ini file
	void ReadGraspTypeIni(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrientation, EGraspType GraspType);
};
