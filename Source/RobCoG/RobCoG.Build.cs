// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;

public class RobCoG : ModuleRules
{
	public RobCoG(TargetInfo Target)
	{
		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			//"HydraPlugin",
			//"HeadMountedDisplay",
			//"XmlParser"
			//"PhysX",
			//"APEX",
			//"Json",
			//"JsonUtilities"
        });

		PrivateDependencyModuleNames.AddRange(new string[] {
			"HeadMountedDisplay",
			"SteamVR"
		 });

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");
		// if ((Target.Platform == UnrealTargetPlatform.Win32) || (Target.Platform == UnrealTargetPlatform.Win64))
		// {
		//		if (UEBuildConfiguration.bCompileSteamOSS == true)
		//		{
		//			DynamicallyLoadedModuleNames.Add("OnlineSubsystemSteam");
		//		}
		// }
	}
}
