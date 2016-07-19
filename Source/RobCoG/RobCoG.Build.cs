// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;
using System.IO;

public class RobCoG : ModuleRules
{
	// PATH HELPERS
    private string ThirdPartyPath
    {
        get { return Path.Combine( ModuleDirectory, "../../ThirdParty/" ); }
    }

	public RobCoG(TargetInfo Target)
	{
		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"Json",
			"JsonUtilities",
			//"HydraPlugin",
			//"HeadMountedDisplay",
			//"XmlParser"
			//"PhysX",
			//"APEX",
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

		// THIRD PARTY
        PublicIncludePaths.Add(Path.Combine(ThirdPartyPath, "RapidJson", "Includes"));
        PublicIncludePaths.Add(Path.Combine(ThirdPartyPath, "RapidXml", "Includes"));
	}
}
