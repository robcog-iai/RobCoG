// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "HandInformationParser.h"
#include "Paths.h"
#include "ConfigCacheIni.h"

HandInformationParser::HandInformationParser()
{
	ConfigFileHandler = MakeShareable(new FConfigCacheIni(EConfigCacheType::DiskBacked));
}

HandInformationParser::~HandInformationParser()
{
}

void HandInformationParser::GetHandInformationForGraspType(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrietation, FHandVelocity & HandVelocity, const FString ConfigPath)
{
	ReadGraspTypeIni(InitialHandOrientation, ClosedHandOrietation, HandVelocity, ConfigPath);
}

void HandInformationParser::SetHandInformationForGraspType(const FHandOrientation & InitialHandOrientation, const FHandOrientation & ClosedHandOrietation, const FHandVelocity & HandVelocity, const FString ConfigPath)
{
	WriteGraspTypeIni(InitialHandOrientation, ClosedHandOrietation, HandVelocity, ConfigPath);
}

void HandInformationParser::WriteGraspTypeIni(const FHandOrientation & InitialHandOrientation, const FHandOrientation & ClosedHandOrientation, const FHandVelocity & HandVelocity, const FString ConfigPath)
{
	if (!ConfigFileHandler.IsValid()) return;

	UE_LOG(LogTemp, Warning, TEXT("ConfigPath: %s"), *ConfigPath);

	// Write Initial Hand Orientation into ini file
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("ThumbDistalOrientation"), InitialHandOrientation.ThumbOrientation.DistalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("ThumbItermediateOrientation"), InitialHandOrientation.ThumbOrientation.IntermediateOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("ThumbProximalOrientation"), InitialHandOrientation.ThumbOrientation.ProximalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("ThumbMetacarpalOrientation"), InitialHandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation, ConfigPath);

	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("IndexDistalOrientation"), InitialHandOrientation.IndexOrientation.DistalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("IndexItermediateOrientation"), InitialHandOrientation.IndexOrientation.IntermediateOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("IndexProximalOrientation"), InitialHandOrientation.IndexOrientation.ProximalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("IndexMetacarpalOrientation"), InitialHandOrientation.IndexOrientation.MetacarpalOrientation.Orientation, ConfigPath);

	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("MiddleDistalOrientation"), InitialHandOrientation.MiddleOrientation.DistalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("MiddleItermediateOrientation"), InitialHandOrientation.MiddleOrientation.IntermediateOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("MiddleProximalOrientation"), InitialHandOrientation.MiddleOrientation.ProximalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("MiddleMetacarpalOrientation"), InitialHandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation, ConfigPath);

	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("RingDistalOrientation"), InitialHandOrientation.RingOrientation.DistalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("RingItermediateOrientation"), InitialHandOrientation.RingOrientation.IntermediateOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("RingProximalOrientation"), InitialHandOrientation.RingOrientation.ProximalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("RingMetacarpalOrientation"), InitialHandOrientation.RingOrientation.MetacarpalOrientation.Orientation, ConfigPath);

	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("PinkyDistalOrientation"), InitialHandOrientation.PinkyOrientation.DistalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("PinkyItermediateOrientation"), InitialHandOrientation.PinkyOrientation.IntermediateOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("PinkyProximalOrientation"), InitialHandOrientation.PinkyOrientation.ProximalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*InitOrientationSection, TEXT("PinkyMetacarpalOrientation"), InitialHandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation, ConfigPath);

	//Write Closed Hand Orientation into ini file
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("ThumbDistalOrientation"), ClosedHandOrientation.ThumbOrientation.DistalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("ThumbItermediateOrientation"), ClosedHandOrientation.ThumbOrientation.IntermediateOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("ThumbProximalOrientation"), ClosedHandOrientation.ThumbOrientation.ProximalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("ThumbMetacarpalOrientation"), ClosedHandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation, ConfigPath);

	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("IndexDistalOrientation"), ClosedHandOrientation.IndexOrientation.DistalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("IndexItermediateOrientation"), ClosedHandOrientation.IndexOrientation.IntermediateOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("IndexProximalOrientation"), ClosedHandOrientation.IndexOrientation.ProximalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("IndexMetacarpalOrientation"), ClosedHandOrientation.IndexOrientation.MetacarpalOrientation.Orientation, ConfigPath);

	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("MiddleDistalOrientation"), ClosedHandOrientation.MiddleOrientation.DistalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("MiddleItermediateOrientation"), ClosedHandOrientation.MiddleOrientation.IntermediateOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("MiddleProximalOrientation"), ClosedHandOrientation.MiddleOrientation.ProximalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("MiddleMetacarpalOrientation"), ClosedHandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation, ConfigPath);

	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("RingDistalOrientation"), ClosedHandOrientation.RingOrientation.DistalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("RingItermediateOrientation"), ClosedHandOrientation.RingOrientation.IntermediateOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("RingProximalOrientation"), ClosedHandOrientation.RingOrientation.ProximalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("RingMetacarpalOrientation"), ClosedHandOrientation.RingOrientation.MetacarpalOrientation.Orientation, ConfigPath);

	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("PinkyDistalOrientation"), ClosedHandOrientation.PinkyOrientation.DistalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("PinkyItermediateOrientation"), ClosedHandOrientation.PinkyOrientation.IntermediateOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("PinkyProximalOrientation"), ClosedHandOrientation.PinkyOrientation.ProximalOrientation.Orientation, ConfigPath);
	ConfigFileHandler->SetRotator(*ClosedOrientationSection, TEXT("PinkyMetacarpalOrientation"), ClosedHandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation, ConfigPath);

	//Write Velocity into ini file
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("ThumbDistalVelocity"), HandVelocity.ThumbVelocity.DistalVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("ThumbItermediateVelocity"), HandVelocity.ThumbVelocity.IntermediateVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("ThumbProximalVelocity"), HandVelocity.ThumbVelocity.ProximalVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("ThumbMetacarpalVelocity"), HandVelocity.ThumbVelocity.MetacarpalVelocity.Velocity, ConfigPath);

	ConfigFileHandler->SetVector(*VelocitySection, TEXT("IndexDistalVelocity"), HandVelocity.IndexVelocity.DistalVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("IndexItermediateVelocity"), HandVelocity.IndexVelocity.IntermediateVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("IndexProximalVelocity"), HandVelocity.IndexVelocity.ProximalVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("IndexMetacarpalVelocity"), HandVelocity.IndexVelocity.MetacarpalVelocity.Velocity, ConfigPath);

	ConfigFileHandler->SetVector(*VelocitySection, TEXT("MiddleDistalVelocity"), HandVelocity.MiddleVelocity.DistalVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("MiddleItermediateVelocity"), HandVelocity.MiddleVelocity.IntermediateVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("MiddleProximalVelocity"), HandVelocity.MiddleVelocity.ProximalVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("MiddleMetacarpalVelocity"), HandVelocity.MiddleVelocity.MetacarpalVelocity.Velocity, ConfigPath);

	ConfigFileHandler->SetVector(*VelocitySection, TEXT("RingDistalVelocity"), HandVelocity.RingVelocity.DistalVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("RingItermediateVelocity"), HandVelocity.RingVelocity.IntermediateVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("RingProximalVelocity"), HandVelocity.RingVelocity.ProximalVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("RingMetacarpalVelocity"), HandVelocity.RingVelocity.MetacarpalVelocity.Velocity, ConfigPath);

	ConfigFileHandler->SetVector(*VelocitySection, TEXT("PinkyDistalVelocity"), HandVelocity.PinkyVelocity.DistalVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("PinkyItermediateVelocity"), HandVelocity.PinkyVelocity.IntermediateVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("PinkyProximalVelocity"), HandVelocity.PinkyVelocity.ProximalVelocity.Velocity, ConfigPath);
	ConfigFileHandler->SetVector(*VelocitySection, TEXT("PinkyMetacarpalVelocity"), HandVelocity.PinkyVelocity.MetacarpalVelocity.Velocity, ConfigPath);

	ConfigFileHandler->Flush(true, ConfigPath);
}


void HandInformationParser::ReadGraspTypeIni(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrientation, FHandVelocity & HandVelocity, const FString ConfigPath)
{
	if (!ConfigFileHandler.IsValid()) return;

	// InitOrientation
	bool bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("ThumbDistalOrientation"), InitialHandOrientation.ThumbOrientation.DistalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("ThumbItermediateOrientation"), InitialHandOrientation.ThumbOrientation.IntermediateOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("ThumbProximalOrientation"), InitialHandOrientation.ThumbOrientation.ProximalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("ThumbMetacarpalOrientation"), InitialHandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("IndexDistalOrientation"), InitialHandOrientation.IndexOrientation.DistalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("IndexItermediateOrientation"), InitialHandOrientation.IndexOrientation.IntermediateOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("IndexProximalOrientation"), InitialHandOrientation.IndexOrientation.ProximalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("IndexMetacarpalOrientation"), InitialHandOrientation.IndexOrientation.MetacarpalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("MiddleDistalOrientation"), InitialHandOrientation.MiddleOrientation.DistalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("MiddleItermediateOrientation"), InitialHandOrientation.MiddleOrientation.IntermediateOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("MiddleProximalOrientation"), InitialHandOrientation.MiddleOrientation.ProximalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("MiddleMetacarpalOrientation"), InitialHandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("RingDistalOrientation"), InitialHandOrientation.RingOrientation.DistalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("RingItermediateOrientation"), InitialHandOrientation.RingOrientation.IntermediateOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("RingProximalOrientation"), InitialHandOrientation.RingOrientation.ProximalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("RingMetacarpalOrientation"), InitialHandOrientation.RingOrientation.MetacarpalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("PinkyDistalOrientation"), InitialHandOrientation.PinkyOrientation.DistalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("PinkyItermediateOrientation"), InitialHandOrientation.PinkyOrientation.IntermediateOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("PinkyProximalOrientation"), InitialHandOrientation.PinkyOrientation.ProximalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*InitOrientationSection, TEXT("PinkyMetacarpalOrientation"), InitialHandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	
	//ClosedOrientation
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("ThumbDistalOrientation"), ClosedHandOrientation.ThumbOrientation.DistalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("ThumbItermediateOrientation"), ClosedHandOrientation.ThumbOrientation.IntermediateOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("ThumbProximalOrientation"), ClosedHandOrientation.ThumbOrientation.ProximalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("ThumbMetacarpalOrientation"), ClosedHandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;

	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("IndexDistalOrientation"), ClosedHandOrientation.IndexOrientation.DistalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("IndexItermediateOrientation"), ClosedHandOrientation.IndexOrientation.IntermediateOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("IndexProximalOrientation"), ClosedHandOrientation.IndexOrientation.ProximalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("IndexMetacarpalOrientation"), ClosedHandOrientation.IndexOrientation.MetacarpalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;

	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("MiddleDistalOrientation"), ClosedHandOrientation.MiddleOrientation.DistalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("MiddleItermediateOrientation"), ClosedHandOrientation.MiddleOrientation.IntermediateOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("MiddleProximalOrientation"), ClosedHandOrientation.MiddleOrientation.ProximalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("MiddleMetacarpalOrientation"), ClosedHandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;

	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("RingDistalOrientation"), ClosedHandOrientation.RingOrientation.DistalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("RingItermediateOrientation"), ClosedHandOrientation.RingOrientation.IntermediateOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("RingProximalOrientation"), ClosedHandOrientation.RingOrientation.ProximalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("RingMetacarpalOrientation"), ClosedHandOrientation.RingOrientation.MetacarpalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;

	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("PinkyDistalOrientation"), ClosedHandOrientation.PinkyOrientation.DistalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("PinkyItermediateOrientation"), ClosedHandOrientation.PinkyOrientation.IntermediateOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("PinkyProximalOrientation"), ClosedHandOrientation.PinkyOrientation.ProximalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetRotator(*ClosedOrientationSection, TEXT("PinkyMetacarpalOrientation"), ClosedHandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation, ConfigPath);
	if (!bSuccess) return;

	//Velocity
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("ThumbDistalVelocity"), HandVelocity.ThumbVelocity.DistalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("ThumbItermediateVelocity"), HandVelocity.ThumbVelocity.IntermediateVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("ThumbProximalVelocity"), HandVelocity.ThumbVelocity.ProximalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("ThumbMetacarpalVelocity"), HandVelocity.ThumbVelocity.MetacarpalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;

	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("IndexDistalVelocity"), HandVelocity.IndexVelocity.DistalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("IndexItermediateVelocity"), HandVelocity.IndexVelocity.IntermediateVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("IndexProximalVelocity"), HandVelocity.IndexVelocity.ProximalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("IndexMetacarpalVelocity"), HandVelocity.IndexVelocity.MetacarpalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;

	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("MiddleDistalVelocity"), HandVelocity.MiddleVelocity.DistalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("MiddleItermediateVelocity"), HandVelocity.MiddleVelocity.IntermediateVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("MiddleProximalVelocity"), HandVelocity.MiddleVelocity.ProximalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("MiddleMetacarpalVelocity"), HandVelocity.MiddleVelocity.MetacarpalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;

	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("RingDistalVelocity"), HandVelocity.RingVelocity.DistalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("RingItermediateVelocity"), HandVelocity.RingVelocity.IntermediateVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("RingProximalVelocity"), HandVelocity.RingVelocity.ProximalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("RingMetacarpalVelocity"), HandVelocity.RingVelocity.MetacarpalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;

	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("PinkyDistalVelocity"), HandVelocity.PinkyVelocity.DistalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("PinkyItermediateVelocity"), HandVelocity.PinkyVelocity.IntermediateVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("PinkyProximalVelocity"), HandVelocity.PinkyVelocity.ProximalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
	bSuccess = ConfigFileHandler->GetVector(*VelocitySection, TEXT("PinkyMetacarpalVelocity"), HandVelocity.PinkyVelocity.MetacarpalVelocity.Velocity, ConfigPath);
	if (!bSuccess) return;
}