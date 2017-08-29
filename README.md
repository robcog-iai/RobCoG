# RobCoG - **Rob**ot **Co**mmonsense **G**ames 

# Required plugins (available as submodule)
UPIDController (https://github.com/robcog-iai/UPIDController)

	git submodule update --init --recursive

# Google Summer of Code 2017
*Author: Marcel Meier*

## Description

The task for the Google Summer of Code was to create a grasping behavior for a currently developed hand simulation. 

![alt text][logo]

A robot has to decide where to place the fingers on the objects to be grasped, so there are many grasp types with different forces to be chosen. 
To be able to perform a realistic hand manipulation, it is necessary to observe humans performing hand manipulation tasks in virtual environments.

## Used Software

Unreal Engine 4.17.2
Visual Studio 17

## Requirements of the Projects

* Fully rigged five finger hand
* Force controlled joints
* Dynamically choosing between various grasping styles
* VR environmet for grasping items
* Log forces of grasps to analyze data

## Instructions for playing

1. Press space to spawn an item
2. Choose the grasp type with which youo would grasp that sort of item
3. Try to carry the item to the light blue cube
4. When finished, restart by pressing space again

## Tutorial

### For Playing

1. Start the Unreal Editor
2. Change the map to TestChamber
3. Press Play

### For Setting up the level

1. Create an environment with a spawn and a target position (e.g. two cubes)
2. Add two triggerboxes on the spawn and a target position
3. Add the GraspingGame to the environment
3.1. Set the Triggerboxes reference to the spawn and target of the GraspingGame
4. Add two hands to the environment and configure it to be the left/right one
5. Add the MCCharacter to the environment
5.1. Set the left and right hand references to the MCCharacter

Optional:
6. If logging is needed add the GraspLogger to the environment
6.1 Set the GraspingGame reference and the hand reference to be logged.


## Future Tasks

* Mix orientation and velocity based motors on grasp
* Better defined grasping types
* Add more items to be grasped
* Variably apply force on item


[logo]: Images/GraspMilk.png "Grasp Milk Screenshot"
