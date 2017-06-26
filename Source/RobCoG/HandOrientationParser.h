// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

#include "GraspType.h"
#include "Structs/HandOrientation.h"


/**
 * 
 */
class ROBCOG_API HandOrientationParser
{

public:
	HandOrientationParser();
	~HandOrientationParser();

	void GetHandOrientationsForGraspType(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrietation, EGraspType GraspType);

	void WriteGraspTypeIni(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrientation, EGraspType GraspType);
	void ReadGraspTypeIni(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrientation, EGraspType GraspType);


private:
		TSharedPtr<FConfigCacheIni> ConfigFileHandler;
	
};
