// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "UElectricalDevicesEd.h"
#include "UElectricalDevicesEdStyle.h"
#include "UElectricalDevicesEdCommands.h"
#include "Misc/MessageDialog.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"

#include "LevelEditor.h"

//Utils
#include "Tags.h"
static const FName UElectricalDevicesEdTabName("UElectricalDevicesEd");

#define LOCTEXT_NAMESPACE "FUElectricalDevicesEdModule"

void FUElectricalDevicesEdModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
	
	FUElectricalDevicesEdStyle::Initialize();
	FUElectricalDevicesEdStyle::ReloadTextures();

	FUElectricalDevicesEdCommands::Register();
	
	PluginCommands = MakeShareable(new FUICommandList);

	PluginCommands->MapAction(
		FUElectricalDevicesEdCommands::Get().PluginAction,
		FExecuteAction::CreateRaw(this, &FUElectricalDevicesEdModule::PluginButtonClicked),
		FCanExecuteAction());
		
	FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	
	{
		TSharedPtr<FExtender> MenuExtender = MakeShareable(new FExtender());
		MenuExtender->AddMenuExtension("WindowLayout", EExtensionHook::After, PluginCommands, FMenuExtensionDelegate::CreateRaw(this, &FUElectricalDevicesEdModule::AddMenuExtension));

		LevelEditorModule.GetMenuExtensibilityManager()->AddExtender(MenuExtender);
	}
	
	{
		TSharedPtr<FExtender> ToolbarExtender = MakeShareable(new FExtender);
		ToolbarExtender->AddToolBarExtension("Settings", EExtensionHook::After, PluginCommands, FToolBarExtensionDelegate::CreateRaw(this, &FUElectricalDevicesEdModule::AddToolbarExtension));
		
		LevelEditorModule.GetToolBarExtensibilityManager()->AddExtender(ToolbarExtender);
	}
}

void FUElectricalDevicesEdModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
	FUElectricalDevicesEdStyle::Shutdown();

	FUElectricalDevicesEdCommands::Unregister();
}

void FUElectricalDevicesEdModule::PluginButtonClicked()
{
	LoopActor();
}

FReply FUElectricalDevicesEdModule::LoopActor()
{
	for (TActorIterator<AActor> ActItr(GEditor->GetEditorWorldContext().World()); ActItr; ++ActItr)
	{
		int32 TagIndex = FTags::GetTagTypeIndex(*ActItr, "EDCookTopButtonController");
		if (TagIndex != INDEX_NONE)
		{
			UE_LOG(LogTemp, Warning, TEXT("%d"),TagIndex);
		}
	}
	return FReply::Handled();
}

void FUElectricalDevicesEdModule::AddMenuExtension(FMenuBuilder& Builder)
{
	Builder.AddMenuEntry(FUElectricalDevicesEdCommands::Get().PluginAction);
}

void FUElectricalDevicesEdModule::AddToolbarExtension(FToolBarBuilder& Builder)
{
	Builder.AddToolBarButton(FUElectricalDevicesEdCommands::Get().PluginAction);
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FUElectricalDevicesEdModule, UElectricalDevicesEd)