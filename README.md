# RobCoG - **Rob**ot **Co**mmonsense **G**ames 

## Required plugins 
UPIDController (https://github.com/robcog-iai/UPIDController)


# Google Summer of Code 2017
*Author: Marcel Meier*

## Description

The task for the Google Summer of Code was to create a grasping behavior for a currently developed hand simulation. 

![alt text][logo]

A robot has to decide where to place the fingers on the objects to be grasped, so there are many grasp types with different forces to be chosen. 
To be able to perform a realistic hand manipulation, it is necessary to observe humans performing hand manipulation tasks in virtual environments.

## Used Software

Unreal Engine 4.18.3
Visual Studio 17

## Requirements of the Projects

* Fully rigged five finger hand
* Force controlled joints
* Dynamically choosing between various grasping styles
* VR environmet for grasping items
* Log forces of grasps to analyze data

## Installation

### Setting up the Character

1. Set PlugIn-Content visible

2. Add the `MCCharacter` (C++ Class) to the environment

3. Add one Hand (C++ Class) representing the right and one for the left hand to the environment

4. Set the `Mesh/SkeletalMesh` to the `SK_RightMannequinHand` in the content folder

   4.1. Left Hand: set `RotationX: 180` and `ScaleZ: -1`

5. Set the left and right hand references to the MCCharacters `MC/Hands/LeftSkelActor` and `MC/Hands/LeftSkelActor` settings

6. Check if the `MC/Hand/HandType` and bone names in the `MC/Hand/<Finger>/FingerPartToBoneName`  settings are correct

7. You are now able to play the character in VR 

Optional:
6. If logging is needed add the `GraspLogger` to the environment
  6.1 Set the `GraspLogger/Hand` to the hand reference to be logged.

  ​


### Adding a new Grasptype to the Code

1. Add the new GraspType name to the EGraspType enum in GraspType.h

```c++
UENUM(BlueprintType)
enum class EGraspType : uint8
{
LargeDiameter		UMETA(DisplayName = "LargeDiameter"),
Ring				UMETA(DisplayName = "Ring"),
PalmarPinch			UMETA(DisplayName = "PalmarPinch"),
ParallelExtension	UMETA(DisplayName = "ParallelExtension"),
Lateral				UMETA(DisplayName = "Lateral"),
Tripod				UMETA(DisplayName = "Tripod"),
Example				UMETA(DisplayName = "Example"),
};
```

2. Add a new file `Example.ini` ind the plugins config folder which is called like the enum DisplayName and add the rotator values of each grasp:

   ```ini
   [InitialHandOrientation]
   ThumbDistalOrientation=P=0.000000 Y=30.000000 R=0.000000
   ThumbItermediateOrientation=P=0.000000 Y=10.000000 R=0.000000
   ThumbProximalOrientation=P=30.000000 Y=10.000000 R=10.000000
   ThumbMetacarpalOrientation=P=0.000000 Y=0.000000 R=0.000000
   IndexDistalOrientation=P=-10.000000 Y=0.000000 R=0.000000
   IndexItermediateOrientation=P=-50.000000 Y=0.000000 R=0.000000
   IndexProximalOrientation=P=-50.000000 Y=0.000000 R=0.000000
   IndexMetacarpalOrientation=P=0.000000 Y=0.000000 R=0.000000
   MiddleDistalOrientation=P=-10.000000 Y=0.000000 R=0.000000
   MiddleItermediateOrientation=P=-50.000000 Y=0.000000 R=0.000000
   MiddleProximalOrientation=P=-50.000000 Y=0.000000 R=0.000000
   MiddleMetacarpalOrientation=P=0.000000 Y=0.000000 R=0.000000
   RingDistalOrientation=P=-10.000000 Y=0.000000 R=0.000000
   RingItermediateOrientation=P=-50.000000 Y=0.000000 R=0.000000
   RingProximalOrientation=P=-50.000000 Y=0.000000 R=0.000000
   RingMetacarpalOrientation=P=0.000000 Y=0.000000 R=0.000000
   PinkyDistalOrientation=P=-10.000000 Y=0.000000 R=0.000000
   PinkyItermediateOrientation=P=-50.000000 Y=0.000000 R=0.000000
   PinkyProximalOrientation=P=-50.000000 Y=0.000000 R=0.000000
   PinkyMetacarpalOrientation=P=0.000000 Y=0.000000 R=0.000000

   [ClosedHandOrientation]
   ThumbDistalOrientation=P=0.000000 Y=-30.000000 R=0.000000
   ThumbItermediateOrientation=P=0.000000 Y=-30.000000 R=0.000000
   ThumbProximalOrientation=P=30.000000 Y=10.000000 R=10.000000
   ThumbMetacarpalOrientation=P=0.000000 Y=0.000000 R=0.000000
   IndexDistalOrientation=P=12.000000 Y=0.000000 R=0.000000
   IndexItermediateOrientation=P=12.000000 Y=0.000000 R=0.000000
   IndexProximalOrientation=P=12.000000 Y=0.000000 R=0.000000
   IndexMetacarpalOrientation=P=0.000000 Y=0.000000 R=0.000000
   MiddleDistalOrientation=P=12.000000 Y=0.000000 R=0.000000
   MiddleItermediateOrientation=P=12.000000 Y=0.000000 R=0.000000
   MiddleProximalOrientation=P=12.000000 Y=0.000000 R=0.000000
   MiddleMetacarpalOrientation=P=0.000000 Y=0.000000 R=0.000000
   RingDistalOrientation=P=12.000000 Y=0.000000 R=0.000000
   RingItermediateOrientation=P=12.000000 Y=0.000000 R=0.000000
   RingProximalOrientation=P=12.000000 Y=0.000000 R=0.000000
   RingMetacarpalOrientation=P=0.000000 Y=0.000000 R=0.000000
   PinkyDistalOrientation=P=12.000000 Y=0.000000 R=0.000000
   PinkyItermediateOrientation=P=12.000000 Y=0.000000 R=0.000000
   PinkyProximalOrientation=P=12.000000 Y=0.000000 R=0.000000
   PinkyMetacarpalOrientation=P=0.000000 Y=0.000000 R=0.000000
   ```

3. Use your new GraspType

   ![Szenario4-2][scenario]

## Future Tasks

* Better define grasp types
* Add more grasp types
* Add an arm to the model for more realistic movement
* Minimizing the MassRatio problem
* Limit torque to a maximum value
* Add the sem logger


[logo]: Documentation/Img/HandMesh.png "HandMesh"
[scenario]: Documentation/Img/Szenario4-2.png "Scenario4-2"