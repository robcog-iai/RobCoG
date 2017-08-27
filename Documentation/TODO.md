Todo, Bugs and Feature Requests
========

Todo
------

### [Urgent] Debug PR2 Unreal Node

Currently though the unreal node and unreal game could compile, but the movement of robot is completely erratic. A very very large force was added to the robot by the controller. The forces should be limited and in the beginning they were limited, but after hundreds of frames they just got out of constraint. (unit: N.m, the forces on wheels should be no more than 7.0)

```
name: ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', 'br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'torso_lift_motor_screw_joint']
position: []
velocity: []
effort: [-156., -3549, -3535, -1.89, 3632., 3651., 147.3, 3600., 3586., 388.0, -3632, -3596, 0.000, 3.183] 
```

Possible reasons for this bug: 

- Bugs occur in GetVelocity or GetAngle
- Bugs in setting damping force
    - I just found that I didn't convert the velocity from deg/s to rad/s when calculating damping force, which is the most probable reason for this bug. I have already submitted a patch in my repository but I am not sure if it works. 

Debugging is not easy for the simulation, since the Unreal side sends the wall clock time to ROS clock and uses wall clock time in physics simulation, so simple debugging it using step over or breakpoints will make the result completely different (time differential _dt_ will be too large between frames). 

### [Urgent] Add prismatic joint support 

Now in URoboSim we only implemented `GetPosition`, `GetVelocity` and `AddForce` for revolute (hinge) joints. For prismatic joints it will prompt an error message.

But in PR2, the torso and gripper joints are prismatic joints, in order to control them we need to modify functions above to support prismatic joints. 

### Add Timeout Callback for `libwebsockets`

Now in UROSBridge we are using `WebSockets.cpp` in `HTML5Networking` package as the websockets client. This client is a wrapper of `libwebsockets` library. When websockets server exists it works well, but if it cannot connect to the server (server not exists, timeout, etc. ) the whole game will stuck and user has to restart the unreal editor. This is because in the `Connect` function in `WebSockets.cpp`, it calls the blocking connection function defined in libwebsockets. 

We need to modify the WebSocket wrapper class so that when it succeed to connect, it sets a "Connection Success" flag; when it fails to connect, it automatically retries for several times (can be set by user) and if it still fails the game ends with prompting an error message. `libwebsockets` supports timeout callback, making it possible for a more robust WebSocket client wrapper class.  

### Add more common msg types 

Currently the message types in UROSBridge is enough for the PR2 simulator, but for other applications, other common message types may be used, including messages for: 

* actions (actionlib_msgs)
* diagnostics (diagnostic_msgs)
* geometric primitives (geometry_msgs)
* robot navigation (nav_msgs)
* shapes (shape_msgs)
* stereo processing (stereo_msgs)
* robot trajectories (trajectory_msgs)
* messages used by higher level packages, such as rviz, that deal in visualization-specific data (visualization_msgs)
* common sensors (sensor_msgs), such as laser range finders, cameras, point clouds

And currently the message types are named like `FROSBridgeMsgStdmsgsString`, which is too long for a class name. Maybe we can move them into a separated namespace like `ROSBridgeMsg::Std_msgs::String`. 

According to the [coding standard](https://docs.unrealengine.com/latest/INT/Programming/Development/CodingStandard/index.html#namespaces),  
> You can use namespaces to organize your classes, functions and variables where appropriate, as long as you follow the rules below.
> 
> Namespaces are not supported by UnrealHeaderTool, so should not be used when defining UCLASSes, USTRUCTs etc.

since they are non-UObject classes, using namespaces is possible. 


### Write a script to automatically generate msg headers

It is possible to write a python script to automatically generate msg C++ headers from its `.msg` definition. We can even merge it into the ROS packages like the message builder used in catkin packages. 

Bugs
------

### Unable to include ROS libraries

  Before using ROSBridge, I have tried to include ROS headers and link ROS libaries directly to the Unreal Engine, but it all fails. 
  
  See detailed bug report at [my github gist](https://gist.github.com/gnoliyil/a824904a0f96c8e02ab284b47c5326cc) .  

### GetTwist / Swing cannot get correct value in PR2 Robot

  In Unreal / PhySX there are functions to get the 6-DOF Physics Constraint's relation rotation around its Z, X and Y axis, named `GetCurrentSwing1`, `GetCurrentSwing2`, and `GetCurrentTwist`. 
  
  These functions works in my self-built hinged doors or pendulums, but when it comes to PR2, for a hinge joint with 1 DoF, when we try to turn the caster, only one value from swing1, swing2 and twist are supposed to change, but all the three values of `GetCurrentSwing1`, `GetCurrentSwing2`, and `GetCurrentTwist` change, which makes no sense. 
  
  This bug does only appears in my PR2 robot and I can not find a way to replicate on other robots or other physics constraints between rigid bodies. 

Feature Requests
------

### Support for Hinge Joints, Prismatics etc.

  Currently, in order to get joint information, we need to calculate them using the joint bodies' init position and current relative position by ourselves. 
  
  For hinge joints, we can create a joint class which extends the normal D6Joint, but with only X axis unlocked so that the child can only rotate around the parent. And we can add methods like Get Angle, Get Angular Velocity etc to this class. We can also create a prismatic joint class with only one linear degree of freedom (move around its X axis), which implements Get Position and Get Angular Velocity methods. Then it will be much easier for us to migrate programs using other physics engine (like gazebo simulator with ode, bullet engine) to Unreal with PhysX. 

