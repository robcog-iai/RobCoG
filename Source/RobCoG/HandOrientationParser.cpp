// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "HandOrientationParser.h"
#include "Paths.h"
#include "ConfigCacheIni.h"

HandOrientationParser::HandOrientationParser()
{
	ConfigFileHandler = MakeShareable(new FConfigCacheIni(EConfigCacheType::DiskBacked));
}

HandOrientationParser::~HandOrientationParser()
{
}

void HandOrientationParser::GetHandOrientationsForGraspType(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrietation, EGraspType GraspType)
{
	ReadGraspTypeIni(InitialHandOrientation, ClosedHandOrietation, GraspType);
}

void HandOrientationParser::WriteGraspTypeIni(FHandOrientation & InitialHandOrientation, FHandOrientation & ClosedHandOrientation, EGraspType GraspType)
{
	if (!ConfigFileHandler.IsValid()) return;

	const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EGraspType"), true);
	if (!EnumPtr) return;

	FString GraspTypeString = EnumPtr->GetDisplayNameTextByIndex((int32)GraspType).ToString();
	FString ConfigDir = FPaths::GameConfigDir();
	FString ConfigName = ConfigDir + GraspTypeString + ".ini";

	UE_LOG(LogTemp, Warning, TEXT("ConfigName: %s"), *ConfigName);

	FString InitSection = "InitialHandOrientation";
	FString ClosedSection = "ClosedHandOrientation";

	ConfigFileHandler->SetRotator(*InitSection, TEXT("ThumbDistalOrientation"), InitialHandOrientation.ThumbOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("ThumbItermediateOrientation"), InitialHandOrientation.ThumbOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("ThumbProximalOrientation"), InitialHandOrientation.ThumbOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("ThumbMetacarpalOrientation"), InitialHandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->SetRotator(*InitSection, TEXT("IndexDistalOrientation"), InitialHandOrientation.IndexOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("IndexItermediateOrientation"), InitialHandOrientation.IndexOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("IndexProximalOrientation"), InitialHandOrientation.IndexOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("IndexMetacarpalOrientation"), InitialHandOrientation.IndexOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->SetRotator(*InitSection, TEXT("MiddleDistalOrientation"), InitialHandOrientation.MiddleOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("MiddleItermediateOrientation"), InitialHandOrientation.MiddleOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("MiddleProximalOrientation"), InitialHandOrientation.MiddleOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("MiddleMetacarpalOrientation"), InitialHandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->SetRotator(*InitSection, TEXT("RingDistalOrientation"), InitialHandOrientation.RingOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("RingItermediateOrientation"), InitialHandOrientation.RingOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("RingProximalOrientation"), InitialHandOrientation.RingOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("RingMetacarpalOrientation"), InitialHandOrientation.RingOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->SetRotator(*InitSection, TEXT("PinkyDistalOrientation"), InitialHandOrientation.PinkyOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("PinkyItermediateOrientation"), InitialHandOrientation.PinkyOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("PinkyProximalOrientation"), InitialHandOrientation.PinkyOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*InitSection, TEXT("PinkyMetacarpalOrientation"), InitialHandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation, ConfigName);


	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("ThumbDistalOrientation"), ClosedHandOrientation.ThumbOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("ThumbItermediateOrientation"), ClosedHandOrientation.ThumbOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("ThumbProximalOrientation"), ClosedHandOrientation.ThumbOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("ThumbMetacarpalOrientation"), ClosedHandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("IndexDistalOrientation"), ClosedHandOrientation.IndexOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("IndexItermediateOrientation"), ClosedHandOrientation.IndexOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("IndexProximalOrientation"), ClosedHandOrientation.IndexOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("IndexMetacarpalOrientation"), ClosedHandOrientation.IndexOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("MiddleDistalOrientation"), ClosedHandOrientation.MiddleOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("MiddleItermediateOrientation"), ClosedHandOrientation.MiddleOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("MiddleProximalOrientation"), ClosedHandOrientation.MiddleOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("MiddleMetacarpalOrientation"), ClosedHandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("RingDistalOrientation"), ClosedHandOrientation.RingOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("RingItermediateOrientation"), ClosedHandOrientation.RingOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("RingProximalOrientation"), ClosedHandOrientation.RingOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("RingMetacarpalOrientation"), ClosedHandOrientation.RingOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("PinkyDistalOrientation"), ClosedHandOrientation.PinkyOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("PinkyItermediateOrientation"), ClosedHandOrientation.PinkyOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("PinkyProximalOrientation"), ClosedHandOrientation.PinkyOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->SetRotator(*ClosedSection, TEXT("PinkyMetacarpalOrientation"), ClosedHandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->Flush(true, ConfigName);
}


void HandOrientationParser::ReadGraspTypeIni(FHandOrientation & InitialHandOrientation,FHandOrientation & ClosedHandOrientation, EGraspType GraspType)
{

	if (!ConfigFileHandler.IsValid()) return;

	const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EGraspType"), true);
	if (!EnumPtr) return;

	FString GraspTypeString = EnumPtr->GetDisplayNameTextByIndex((int32)GraspType).ToString();
	FString ConfigDir = FPaths::GameConfigDir();
	FString ConfigName = ConfigDir + GraspTypeString + ".ini";

	UE_LOG(LogTemp, Warning, TEXT("ConfigName: %s"), *ConfigName);

	FString InitSection = "InitialHandOrientation";
	FString ClosedSection = "ClosedHandOrientation";

	ConfigFileHandler->GetRotator(*InitSection, TEXT("ThumbDistalOrientation"), InitialHandOrientation.ThumbOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("ThumbItermediateOrientation"), InitialHandOrientation.ThumbOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("ThumbProximalOrientation"), InitialHandOrientation.ThumbOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("ThumbMetacarpalOrientation"), InitialHandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->GetRotator(*InitSection, TEXT("IndexDistalOrientation"), InitialHandOrientation.IndexOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("IndexItermediateOrientation"), InitialHandOrientation.IndexOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("IndexProximalOrientation"), InitialHandOrientation.IndexOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("IndexMetacarpalOrientation"), InitialHandOrientation.IndexOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->GetRotator(*InitSection, TEXT("MiddleDistalOrientation"), InitialHandOrientation.MiddleOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("MiddleItermediateOrientation"), InitialHandOrientation.MiddleOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("MiddleProximalOrientation"), InitialHandOrientation.MiddleOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("MiddleMetacarpalOrientation"), InitialHandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->GetRotator(*InitSection, TEXT("RingDistalOrientation"), InitialHandOrientation.RingOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("RingItermediateOrientation"), InitialHandOrientation.RingOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("RingProximalOrientation"), InitialHandOrientation.RingOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("RingMetacarpalOrientation"), InitialHandOrientation.RingOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->GetRotator(*InitSection, TEXT("PinkyDistalOrientation"), InitialHandOrientation.PinkyOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("PinkyItermediateOrientation"), InitialHandOrientation.PinkyOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("PinkyProximalOrientation"), InitialHandOrientation.PinkyOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*InitSection, TEXT("PinkyMetacarpalOrientation"), InitialHandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation, ConfigName);


	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("ThumbDistalOrientation"), ClosedHandOrientation.ThumbOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("ThumbItermediateOrientation"), ClosedHandOrientation.ThumbOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("ThumbProximalOrientation"), ClosedHandOrientation.ThumbOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("ThumbMetacarpalOrientation"), ClosedHandOrientation.ThumbOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("IndexDistalOrientation"), ClosedHandOrientation.IndexOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("IndexItermediateOrientation"), ClosedHandOrientation.IndexOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("IndexProximalOrientation"), ClosedHandOrientation.IndexOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("IndexMetacarpalOrientation"), ClosedHandOrientation.IndexOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("MiddleDistalOrientation"), ClosedHandOrientation.MiddleOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("MiddleItermediateOrientation"), ClosedHandOrientation.MiddleOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("MiddleProximalOrientation"), ClosedHandOrientation.MiddleOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("MiddleMetacarpalOrientation"), ClosedHandOrientation.MiddleOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("RingDistalOrientation"), ClosedHandOrientation.RingOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("RingItermediateOrientation"), ClosedHandOrientation.RingOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("RingProximalOrientation"), ClosedHandOrientation.RingOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("RingMetacarpalOrientation"), ClosedHandOrientation.RingOrientation.MetacarpalOrientation.Orientation, ConfigName);

	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("PinkyDistalOrientation"), ClosedHandOrientation.PinkyOrientation.DistalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("PinkyItermediateOrientation"), ClosedHandOrientation.PinkyOrientation.IntermediateOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("PinkyProximalOrientation"), ClosedHandOrientation.PinkyOrientation.ProximalOrientation.Orientation, ConfigName);
	ConfigFileHandler->GetRotator(*ClosedSection, TEXT("PinkyMetacarpalOrientation"), ClosedHandOrientation.PinkyOrientation.MetacarpalOrientation.Orientation, ConfigName);
}