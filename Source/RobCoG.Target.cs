// Copyright 2017-2020, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

using UnrealBuildTool;
using System.Collections.Generic;

public class RobCoGTarget : TargetRules
{
	public RobCoGTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;

		ExtraModuleNames.AddRange( new string[] { "RobCoG" } );
	}
}
