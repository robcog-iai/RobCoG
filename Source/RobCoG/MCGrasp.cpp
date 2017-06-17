// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#include "MCGrasp.h"

// Contructor
MCGrasp::MCGrasp()
{
}

// Destructor
MCGrasp::~MCGrasp()
{
}

//// Set grasp fingers
//void MCGrasp::SetFingers(
//	const FFinger& InThumb,
//	const FFinger& InIndex,
//	const FFinger& InMiddle,
//	const FFinger& InRing,
//	const FFinger& InPinky)
//{
//	Thumb = InThumb;
//	Index = InIndex;
//	Middle = InMiddle;
//	Ring = InRing;
//	Pinky = InPinky;
//}

// Set grasp type
void  MCGrasp::SetGraspType(const EGraspType InGraspType)
{
	GraspType = InGraspType;
}

// Update grasp
void MCGrasp::Update(const float Goal)
{
	UE_LOG(LogTemp, Warning, TEXT("Grasp update: %f"), Goal);
}
