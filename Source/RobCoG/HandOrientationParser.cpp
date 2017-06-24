// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "HandOrientationParser.h"

UHandOrientationParser::UHandOrientationParser()
{
}

//FHandOrientation UHandOrientationParser::GetHandOrientationForGraspType_Implementation(EGraspType GraspType)

FHandOrientation UHandOrientationParser::GetInitialHandOrientationForGraspType(EGraspType GraspType)
{

	// TODO: Parse the Initial HandOrientation

	FHandOrientation HandOrientation;

	if (GraspType == EGraspType::FullGrasp)
	{
		HandOrientation.IndexOrientation.DistalOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.IndexOrientation.IntermediateOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.IndexOrientation.ProximalOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.IndexOrientation.MetacarpalOrientation.Orientation = FRotator(-50, 0, 0);

		HandOrientation.MiddleOrientation.DistalOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.MiddleOrientation.IntermediateOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.MiddleOrientation.ProximalOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation = FRotator(-50, 0, 0);

		HandOrientation.RingOrientation.DistalOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.RingOrientation.IntermediateOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.RingOrientation.ProximalOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.RingOrientation.MetacarpalOrientation.Orientation = FRotator(-50, 0, 0);

		HandOrientation.PinkyOrientation.DistalOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.PinkyOrientation.IntermediateOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.PinkyOrientation.ProximalOrientation.Orientation = FRotator(-50, 0, 0);
		HandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation = FRotator(-50, 0, 0);

		HandOrientation.ThumbOrientation.DistalOrientation.Orientation = FRotator(-30, 0, 0);
		HandOrientation.ThumbOrientation.IntermediateOrientation.Orientation = FRotator(-30, 0, 0);
		HandOrientation.ThumbOrientation.ProximalOrientation.Orientation = FRotator(30, 50, 5);
		HandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation = FRotator(0, 0, 0);
	}
	else if (GraspType == EGraspType::PinchGrasp)
	{

		HandOrientation.IndexOrientation.DistalOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.IndexOrientation.IntermediateOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.IndexOrientation.ProximalOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.IndexOrientation.MetacarpalOrientation.Orientation = FRotator(50, 0, 0);

		HandOrientation.MiddleOrientation.DistalOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.MiddleOrientation.IntermediateOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.MiddleOrientation.ProximalOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation = FRotator(50, 0, 0);

		HandOrientation.RingOrientation.DistalOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.RingOrientation.IntermediateOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.RingOrientation.ProximalOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.RingOrientation.MetacarpalOrientation.Orientation = FRotator(50, 0, 0);

		HandOrientation.PinkyOrientation.DistalOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.PinkyOrientation.IntermediateOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.PinkyOrientation.ProximalOrientation.Orientation = FRotator(50, 0, 0);
		HandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation = FRotator(50, 0, 0);

		HandOrientation.ThumbOrientation.DistalOrientation.Orientation = FRotator(0, 50, 0);
		HandOrientation.ThumbOrientation.IntermediateOrientation.Orientation = FRotator(0, 50, 0);
		HandOrientation.ThumbOrientation.ProximalOrientation.Orientation = FRotator(0, 50, 0);
		HandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation = FRotator(0, 50, 0);
	}
	else if (GraspType == EGraspType::PinchThreeGrasp)
	{
		HandOrientation.IndexOrientation.DistalOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.IndexOrientation.IntermediateOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.IndexOrientation.ProximalOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.IndexOrientation.MetacarpalOrientation.Orientation = FRotator(0, 0, 0);

		HandOrientation.MiddleOrientation.DistalOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.MiddleOrientation.IntermediateOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.MiddleOrientation.ProximalOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation = FRotator(0, 0, 0);

		HandOrientation.RingOrientation.DistalOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.RingOrientation.IntermediateOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.RingOrientation.ProximalOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.RingOrientation.MetacarpalOrientation.Orientation = FRotator(0, 0, 0);

		HandOrientation.PinkyOrientation.DistalOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.PinkyOrientation.IntermediateOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.PinkyOrientation.ProximalOrientation.Orientation = FRotator(45, 0, 0);
		HandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation = FRotator(0, 0, 0);

		HandOrientation.ThumbOrientation.DistalOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.ThumbOrientation.IntermediateOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.ThumbOrientation.ProximalOrientation.Orientation = FRotator(0, 0, 0);
		HandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation = FRotator(0, 0, 0);
	}

	return HandOrientation;
}

FHandOrientation UHandOrientationParser::GetClosedHandOrientationForGraspType(EGraspType GraspType)
{

	// TODO: Parse the closed HandOrientation

	FHandOrientation HandOrientation;


	HandOrientation.IndexOrientation.DistalOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.IndexOrientation.IntermediateOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.IndexOrientation.ProximalOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.IndexOrientation.MetacarpalOrientation.Orientation = FRotator(50, 0, 0);

	HandOrientation.MiddleOrientation.DistalOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.MiddleOrientation.IntermediateOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.MiddleOrientation.ProximalOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation = FRotator(50, 0, 0);

	HandOrientation.RingOrientation.DistalOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.RingOrientation.IntermediateOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.RingOrientation.ProximalOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.RingOrientation.MetacarpalOrientation.Orientation = FRotator(50, 0, 0);

	HandOrientation.PinkyOrientation.DistalOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.PinkyOrientation.IntermediateOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.PinkyOrientation.ProximalOrientation.Orientation = FRotator(50, 0, 0);
	HandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation = FRotator(50, 0, 0);

	HandOrientation.ThumbOrientation.DistalOrientation.Orientation = FRotator(0, -50, 0);
	HandOrientation.ThumbOrientation.IntermediateOrientation.Orientation = FRotator(0, -50, 0);
	HandOrientation.ThumbOrientation.ProximalOrientation.Orientation = FRotator(0, -50, 0);
	HandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation = FRotator(0, -50, 0);


	return HandOrientation;
}
