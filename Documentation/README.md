RoboSim in Unreal Engine
=====

RoboSim is a robot simulator for URDF based robot models (like Willow Garage PR2) in Unreal Engine 4. It utilizes Unreal Engine's built-in PhysX physics engine and can be controlled by either external controller, like PR2's ROS control stack, or built-in controller from Unreal Engine. 

Table of Contents
------

- [Components](#components)
    - [URoboSim](#urobosim)
    - [UROSBridge](#urosbridge)
    - [PR2 ROS Package For Kinetic](#pr2-ros-package-for-kinetic)
- [Installation](#installation)
    - [URoboSim and UROSBridge](#urobosim-and-urosbridge)
    - [PR2 ROS Package For Kinetic](#pr2-ros-package-for-kinetic-1)
- [Usage](#usage)
    - [URoboSim](#urobosim-1)
    - [UROSBridge](#urosbridge-1)
    - [PR2 ROS Package For Kinetic](#pr2-ros-package-for-kinetic-2)
- [Extra](#extra)
    - [Simulate Other Robots in Unreal](#simulate-other-robots-in-unreal)

Components
------
RoboSim consists of three main parts: two Unreal Engine plugins, URoboSim ad UROSBridge, and a set of PR2 packages which can be run in ROS Kinetic Kame. 

### URoboSim

**Repository**: https://github.com/robcog-iai/URoboSim

URoboSim is an Unreal plugin to import URDF robot models. It contains the following C++ classes: 

- **RURDFData**: When XML/URDF files are imported into Unreal Editor, it will be loaded into an RURDFData class instance which saves and secures the information for robot construction. 
- **FURDFParser**: Parser who gets the information from the XML for the robot.
- **RRobot**: Robot class who saves robot information (links and joints) interpreted from XML description files, and creates Unreal Static Mesh Component / Shape Component classes and Unreal Joints (Physics Constraint Components). Besides it extends the original Physics Constraint to add support for rotating its hinge joints or moving its prismatic joints to a given position / translation; it also supports get joints' position, velocity or add force / torque to joints, which has only limited support in Unreal / PhysX. 
- **RFactoryRRobot and RFactoryRURDFData**: Factory classes who supports importing XML/URDF files to Unreal Contents Browser and drag-and-drop of RRobot actors to Unreal Editor. 

### UROSBridge

**Repository**: https://github.com/robcog-iai/UROSBridge

UROSBridge is an Unreal plugin to communicate with rosbridge WebSocket server so that it can receive and send messages, send and process ROS service requests. The plugin contains: 

- **FWebSocket**: This is a class used in Unreal HTML5Networking plugin which implements a WebSocket client with callback function on messages using the libwebsocket library. 
- **ROSBridgeHandler**: The class who manages all the subscription and advertisement of message topics and services. It binds topics / service names with corresponding callback functions, which will be called to process incoming messages when they come. 

### PR2 ROS Package For Kinetic

**Repository**: https://github.com/gnoliyil/pr2_kinetic_packages

This is a ROS workspace to run PR2 gazebo simulator and unreal simulator in ROS Kinetic. It includes pr2 description files, pr2 controllers (including controller definitions) and controller manager, pr2 actions, pr2 apps and their dependencies: 

- `pr2_robot` (Source: https://github.com/PR2/pr2_robot)
  - `imu_monitor` 
  - `pr2_bringup`
  - `pr2_camera_synchronizer` 
  - `pr2_computer_monitor`
  - `pr2_controller_configuration`
  - `pr2_ethercat`
  - `pr2_run_stop_auto_restart`
- `pr2_controllers`: (Source: https://github.com/pr2/pr2_controllers)
  - `control_toolbox`
  - `ethercat_trigger_controllers`
  - `joint_trajectory_action`
  - `pr2_calibration_controllers`
  - `pr2_controllers_msgs`
  - `pr2_gripper_action`
  - `pr2_head_action`
  - `pr2_mechanism_controllers`
  - `robot_mechanism_controllers`
  - `single_joint_position_action`
- `pr2_ethercat_drivers`: (Source: https://github.com/PR2/pr2_ethercat_drivers)
  - `ethercat_hardware`
  - `fingertip_pressure`
- `pr2_common_actions`: (Source: https://github.com/pr2/pr2_common_actions)
  - `joint_trajectory_action_tools`
  - `joint_trajectory_generator`
  - `pr2_arm_move_ik`
  - `pr2_common_action_msgs`
  - `pr2_tilt_laser_interface`
  - `pr2_tuck_arms_action`
- `pr2_power_drivers`: (Source: https://github.com/pr2/pr2_power_drivers)
  - `ocean_battery_driver`
  - `power_monitor`
  - `pr2_power_board`
- `pr2_apps`: (Source: https://github.com/pr2/pr2_apps)
  - `pr2_app_manager`
  - `pr2_mannequin_mode`
  - `pr2_position_scripts`
  - `pr2_teleop`
  - `pr2_teleop_general`
  - `pr2_tuckarm`
- `pr2_simulator`: (Source: https://github.com/PR2/pr2_simulator)
  - `pr2_controller_configuration_gazebo`
  - `pr2_gazebo`
  - `pr2_gazebo_plugins`
- `pr2_mechanism`: (Source: https://github.com/pr2/pr2_mechanism)
  - `pr2_controller_interface`
  - `pr2_controller_manager`
  - `pr2_hardware_interface`
  - `pr2_mechanism_diagnostics`
  - `pr2_mechanism_model`
- `app_manager` (Source: https://github.com/pr2/app_manager) 
- `moveit_msgs` (Source: https://github.com/ros-planning/moveit_msgs)
- `willow_maps` (Source: https://github.com/pr2/willow_maps)
- `pr2_description` (Source: https://github.com/pr2/pr2_common)

Installation
----

### URoboSim and UROSBridge

Copy these two plugins to Unreal project's `Plugins` directory, and add `"UROSBridge", "URoboSim", "Json", "JsonUtilities"` to the project's `Build.cs` file: 

```
	public RobCoG(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "UROSBridge", "URoboSim", "Json", "JsonUtilities" });

		PrivateDependencyModuleNames.AddRange(new string[] {});
```

Notice: `Json` and `JsonUtilities` are required since we need to implement our ROSBridgeSubscriber classes, where we need to write json-parsing functions. 

### PR2 ROS Package For Kinetic

__Step 0.__ Clone this repository and make sure you are in the `master` branch: 

```
git clone https://github.com/gnoliyil/pr2_kinetic_packages.git
git checkout master
```

__Step 1.__ Install dependency packages: `ros-kinetic-pr2-common` `ros-kinetic-pr2-description` `ros-kinetic-pr2-machine` `ros-kinetic-pr2-msgs` and `ros-kinetic-moveit` are required; Download EML(EtherCAT Master for Linux) from [this repository](https://github.com/ros-gbp/eml-release/tree/release/hydro/eml/eml-svn) (in release/hydro/eml branch) and run (do not run in the catkin workspace)

```
$ mkdir build
$ cd build
$ cmake ..; make; sudo make install 
```
to install the EML library. 

__Step 2.__ Build all packages.

```
$ catkin_make
```

It may prompt that some service headers are __not found__, if this occurs, please use the following command to compile only the required package, then run `catkin_make` again.

For example, if it prompts  

```
/home/gnoliyil/Work/catkin-ws/src/pr2_teleop/src/teleop_gripper.cpp:32:52: fatal error: pr2_controllers_msgs/Pr2GripperCommand.h: No such file or directory
/home/gnoliyil/Work/catkin-ws/src/pr2_teleop/src/teleop_pr2.cpp:44:65: fatal error: pr2_controllers_msgs/JointTrajectoryControllerState.h: No such file or directory
compilation terminated.
/home/gnoliyil/Work/catkin-ws/src/pr2_run_stop_auto_restart/src/run_stop_auto_restart.cpp:33:47: fatal error: pr2_power_board/PowerBoardCommand.h: No such file or directory
```
It means `pr2_controllers_msgs` and `pr2_power_board` needs to be manually compiled. Just run
 
```
$ catkin_make --pkg pr2_controllers_msgs pr2_power_board
```

Then run `$ catkin_make` again. 

If you see error messages like 

```
CMake Error at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "moveit_msgs" with
  any of the following names:

    moveit_msgsConfig.cmake
    moveit_msgs-config.cmake
```
It means you need to install other dependency packages from `apt-get` (`ros-kinetic-moveit-msgs` in the example above)

__Step 3.__ Set environment variables using 

```
source PATH_TO_PACKAGES/devel/setup.sh
```

Please add it in your `.bashrc` file. 


Usage
----

### URoboSim 

#### Get Current Joint Position

Use function 

```
float ARRobot::GetJointPosition(FString JointName);
```
to get the current position of the robot. 

For hinge (rotational) joints, the return value is the relative rotation angle between the children link's initial state and current state in the parent link's coordination system, the unit of which is **degree**. Since Unreal uses left-hand coordinate system, positive angel means **counter-clockwise rotation**, while negative means clockwise rotation. For prismatic joints, the return value is the linear distance of the children link moved along the direction of the axis. 

Only joints with one degree of freedom (i.e. hinge joints or prismatic joints) are supported. 


#### Get Current Joint Velocity

Use function 

```
float ARRobot::GetJointVelocity(FString JointName);
```
to get the current velocity of the robot. 

For hinge (rotational) joints, the return value is the angular velocity of the child link relative to the parent link, the unit of which is **deg/s**. Since Unreal uses left-hand coordinate system, positive velocity means **counter-clockwise rotation**, while negative means clockwise rotation. For prismatic joints, the return value is the relative linear velocity of the child link relative to the parent, in the direction of the axis. 

Only joints with one degree of freedom (i.e. hinge joints or prismatic joints) are supported. 

#### Add Force / Torque to Joint

Use function 

```
void ARRobot::ddForceToJoint(FString JointName, float Force);
```
to add force or torque to the joint. 

For hinge (rotational) joints, the torque is in **kg cm/s^2 = N.m / 10000**. Add a torque to the hinge means to add a torque in the direction of the rotation axis to the child, and add a torque in the opposite direction to the parent; positive force will make the child do ccw rotation. 

For prismatic joints, the force is in **kg m/s^2 = N / 100**. Add a torque to a prismatic joint means to add a positive force in child's moving direction to child and add a negative force to the parent. 

All of these three functions are Blueprint callable. 

### UROSBridge

#### Connection and disconnection

UROSBridge could be used in Unreal Actors or in timers. To use it in actors, we need to add a smart pointer to ROSBridgeHandler first: 

```
UCLASS()
class PROJECT_API AROSActor : public AActor
{
    GENERATED_BODY()

public:
    TSharedPtr<FROSBridgeHandler> Handler; 
    ...
}
```

In Actor's `BeginPlay()` function, create handler instance and connect to the websocket server:

```
void AROSActor::BeginPlay()
{
    Super::BeginPlay();
    
    // Set websocket server address to ws://127.0.0.1:9001
    Handler = MakeShareable<FROSBridgeHandler>(new FROSBridgeHandler(TEXT("127.0.0.1"), 9001));
    
    // Add topic subscribers and publishers
    // Add service clients and servers
    
    // Connect to ROSBridge Websocket server.
    Handler->Connect();
}
```

In Actor's `Tick(float)` function, add `Handler->Render()` function to let handler process incoming messages in the message queue. 

```
void AROSActor::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    // Do something
    
    Handler->Render();
}
```

In Actor's `Logout` or `EndPlay` function, add `Handler->Disconnect()` function before the parent class ends. 

```
void AROSActor::EndPlay(const EEndPlayReason::Type Reason)
{
    Handler->Disconnect(); 
    // Disconnect the handler before parent ends

    Super::EndPlay(Reason);
}
```

#### Publish Message 

To publish message to a topic, we need to first advertise the topic in ROS bridge. In AROSActor's class definition, Add a ROSBridgePublisher smart pointer. 

```
UCLASS()
class PROJECT_API AROSActor : public AActor
{
    GENERATED_BODY()

public:
    TSharedPtr<FROSBridgeHandler> Handler; 
    TSharedPtr<FROSBridgePublisher> Publisher; // This
    ...
}
```

In AROSActor's `BeginPlay()` function, create the publisher using its **type** and **topic**, and then register it to the ROSBridgeHandler. 

```
void AROSActor::BeginPlay()
{
    Super::BeginPlay();
    
    // Set websocket server address to ws://127.0.0.1:9001
    Handler = MakeShareable<FROSBridgeHandler>(new FROSBridgeHandler(TEXT("127.0.0.1"), 9001));
    
    // **** Create publishers here ****
    Publisher = MakeShareable<FROSBridgePublisher>(new FROSBridgePublisher(TEXT("sensor_msgs/JointState"), TEXT("/talker")));
    Handler->AddPublisher(Publisher); 
    
    // Connect to ROSBridge Websocket server.
    Handler->Connect();
}
```

#### Subscribe to Topics

This plugin uses `FROSBridgeSubscriber` class interface to subscribe to topics. We need to extend a `FROSBridgeSubscriber` subclass for each topic we would like to subscribe to, implementing the constructor, destructor, `ParseMessage` function and `CallBack` function. 

##### Include Messages

In class header, the UROSBridge Message class header should be included. 

```
#include "ROSBridgeSubscriber.h"
#include "std_msgs/String.h"
#include "Core.h"
class FROSStringSubScriber : public FROSBridgeSubscriber {
   FROSStringSubScriber(FString Topic_); 
   ~FROSStringSubScriber() override; 
   TSharedPtr<FROSBridgeMsg> ParseMessage(TSharedPtr<FJsonObject> JsonObject) const override; 
   void CallBack(TSharedPtr<FROSBridgeMsg> msg) const override; 
}
```

##### Constructor 

In class constructor, we need to call the parent class constructor to set type and topic for this subscriber. 

```
FROSStringSubScriber::FROSStringSubScriber(FString Topic_):
    FROSBridgeSubscriber(TEXT("std_msgs/String"), Topic_) {}
``` 

##### Destructor 

Class destructors will be required if you need to do some cleaning work after the ROS bridge client disconnects. 

```
FROSStringSubScriber::~FROSStringSubScriber() {};
```

##### ParseMessage

`ParseMessage` function is used by ROSBridgeHandler to convert a `JSONObject` to `FROSBridgeMsg` instance. Create a ROSBridgeMessage class with specified message type (e.g. `FROSBridgeMsgStdmsgsString`) and call its `FromJson` method to parse the JSON message. Finally convert the pointer to a `FROSBridgeMsg` pointer. 

```
TSharedPtr<FROSBridgeMsg> FROSStringSubScriber::ParseMessage
(TSharedPtr<FJsonObject> JsonObject) const
{
    TSharedPtr<FROSBridgeMsgStdmsgsString> StringMessage =
        MakeShareable<FROSBridgeMsgStdmsgsString>(new FROSBridgeMsgStdmsgsString());
    StringMessage->FromJson(JsonObject);
    return StaticCastSharedPtr<FROSBridgeMsg>(StringMessage);
}
```

##### CallBack

`CallBack` is the callback function called when a new message comes and is successfully parsed to a `ROSBridgeMsg` instance. In this function, we need to first down-cast the `FROSBridgeMsg` pointer to a pointer of its subclass. 

```
void CallBack(TSharedPtr<FROSBridgeMsg> msg) const 
{
    TSharedPtr<FROSBridgeMsgStdmsgsString> StringMessage = StaticCastSharedPtr<FROSBridgeMsgStdmsgsString>(msg);
    // downcast to subclass using StaticCastSharedPtr function
    
    UE_LOG(LogTemp, Log, TEXT("Message received! Content: %s"), *StringMessage->GetData());
    // do something with the message

    return;
}
```

##### In Unreal Actor

In Unreal Actors, before the ROS Bridge Handler connects to the server, we need to add pointer to subscriber to the subscriber list first. 

```
void AROSActor::BeginPlay()
{
    Super::BeginPlay();
    
    // Set websocket server address to ws://127.0.0.1:9001
    Handler = MakeShareable<FROSBridgeHandler>(new FROSBridgeHandler(TEXT("127.0.0.1"), 9001));
    
    // Add topic subscribers and publishers
    TSharedPtr<FROSStringSubScriber> Subscriber = 
        MakeShareable<FROSStringSubScriber>(new FROSStringSubScriber(TEXT("/chatter")));
    Handler->AddSubscriber(Subscriber);
    
    // Connect to ROSBridge Websocket server.
    Handler->Connect();
}
```

When the ROS Bridge Handler disconnects to server, it automatically destroys all subscriber instances. 

#### Request Service

A service consists of two parts: Request and Response. Clients send out requests, and then get response from server. Servers process received requests and send out response. 

To send service requests in UROSBridge, we need to create a service client class first. This class should extend the FROSBridgeSrvClient and implement the constructor and a callback function. Below is an example of service "AddTwoInts" client.  

```
#pragma once
#include "ROSBridgeSrvClient.h"
#include "tutorial_srvs/AddTwoInts.h"
class FROSAddTwoIntsClient final : public FROSBridgeSrvClient
{
public:
    FROSAddTwoIntsClient(FString Name):
        FROSBridgeSrvClient(Name, TEXT("beginner_tutorials/AddTwoInts")) {}
    void CallBack(TSharedPtr<FROSBridgeSrv::SrvRequest> Request, TSharedPtr<FROSBridgeSrv::SrvResponse> Response) const override
    {
        TSharedPtr<FROSBridgeSrvRospytutorialsAddTwoInts::Request> Request_ =
            StaticCastSharedPtr<FROSBridgeSrvRospytutorialsAddTwoInts::Request>(Request);
        TSharedPtr<FROSBridgeSrvRospytutorialsAddTwoInts::Response> Response_=
            StaticCastSharedPtr<FROSBridgeSrvRospytutorialsAddTwoInts::Response>(Response);
        // Do downcast to convert Request and Response to corresponding types
        //        
        UE_LOG(LogTemp, Log, TEXT("Add Two Ints: %d + %d = %d"), Request_->GetA(), Request_->GetB(), Response_->GetSum());
    }
};
```

Then in ROSActor, we can send service requests using the following function: 

```
TSharedPtr<FROSAddTwoIntsClient> ServiceClient = 
    MakeShareable<FROSAddTwoIntsClient>(new FROSAddTwoIntsClient(TEXT("add_two_ints")));

int NumA = FMath::RandRange(1, 10000);
int NumB = FMath::RandRange(1, 10000);
TSharedPtr<FROSBridgeSrv::SrvRequest> Request = MakeShareable(new FROSBridgeSrvRospytutorialsAddTwoInts::Request(NumA, NumB));
// Create a request instance with request parameters
TSharedPtr<FROSBridgeSrv::SrvResponse> Response = MakeShareable(new FROSBridgeSrvRospytutorialsAddTwoInts::Response());
// Create an empty response instance 
Handler->CallService(ServiceClient, Request, Response);
```

Notice: The `CallService` is not a blocking function, i.e. it will not block the main actor thread to wait for service response but it will call the callback function once it receives the response in following ticks. 

#### Send Response to Service Requests

The plugin can also works as a "server" side who receives ros service requests from and returns response to the clients. 

To process service requests in UROSBridge, we need to create a service server class first. This class should extend the FROSBridgeSrvServer and implement the constructor, `FromJson`, and `CallBack` function. It is very similar to ROS Bridge Subscriber classes but the only difference is that the return type of `CallBack` is `TSharedPtr<FROSBridgeSrv::SrvResponse>` rather than `void`. Below is an example of service AddTwoInts server. 

```
 #pragma once
 
 #include "ROSBridgeSrvServer.h"
 #include "tutorial_srvs/AddTwoInts.h"
 
 class FROSAddTwoIntsServer final : public FROSBridgeSrvServer
 {
 public:
     FROSAddTwoIntsServer(FString Name):
         FROSBridgeSrvServer(Name, TEXT("beginner_tutorials/AddTwoInts")) {}
          
     TSharedPtr<FROSBridgeSrv::SrvRequest> FromJson(TSharedPtr<FJsonObject> JsonObject) const override
     {
         TSharedPtr<FROSBridgeSrvRospytutorialsAddTwoInts::Request> Request_ =
             MakeShareable(new FROSBridgeSrvRospytutorialsAddTwoInts::Request());
         Request_->FromJson(JsonObject);
         return TSharedPtr<FROSBridgeSrv::SrvRequest>(Request_);
     } 
     
     TSharedPtr<FROSBridgeSrv::SrvResponse> CallBack(TSharedPtr<FROSBridgeSrv::SrvRequest> Request) const override
     {
         TSharedPtr<FROSBridgeSrvRospytutorialsAddTwoInts::Request> Request_ =
             StaticCastSharedPtr<FROSBridgeSrvRospytutorialsAddTwoInts::Request>(Request);
 
         int64 Sum = Request_->GetA() + Request_->GetB();
         UE_LOG(LogTemp, Log, TEXT("Service [%s] Server: Add Two Ints: %d + %d = %d"), *Name, Request_->GetA(), Request_->GetB(), Sum);
         return MakeShareable<FROSBridgeSrv::SrvResponse>
                   (new FROSBridgeSrvRospytutorialsAddTwoInts::Response(Sum));
     }

 }
```

In Actor, before connection, first register the server to ROS bridge, then it will process incoming service requests automatically. After disconnection, the server will be automatically destroyed. 

```
void AROSActor::BeginPlay()
{
    Super::BeginPlay();
    
    // Set websocket server address to ws://127.0.0.1:9001
    Handler = MakeShareable<FROSBridgeHandler>(new FROSBridgeHandler(TEXT("127.0.0.1"), 9001));
    
    // Add service clients and servers
    TSharedPtr<FROSAddTwoIntsServer> ServiceServer = MakeShareable<FROSAddTwoIntsServer>(new FROSAddTwoIntsServer(TEXT("add_two_ints_2")));
    Handler->AddServiceServer(ServiceServer);
    
    // Connect to ROSBridge Websocket server.
    Handler->Connect();
}
```

#### Add More Message / Service Types

This plugin already has support for `std_msgs`, `geometry_msgs` and `std_srvs`, but sometimes other types of message / service will be required. We can add new message classes to the plugin or directly to the project source folder.   

##### Message / Topic 

Messages should extend the `FROSBridgeMsg` class, and implement the following functions: 

- Constructor & Destructor
- `void FromJson(TSharedPtr<FJsonObject> JsonObject)`
    - set all the properties from the JsonObject
    - For numbers, use `JsonObject->GetNumberField(FieldName)`
    - For strings, use `JsonObject->GetStringField(FieldName)`
    - For other message types or ros time, use `ClassType::GetFromJson(JsonObject->GetObjectField(FieldName))`    
    - For array, first get JsonValue array using `TArray<TSharedPtr<FJsonValue>> PointsPtrArray = JsonObject->GetArrayField(TEXT("points"))`, then for each element `ptr` in the array, call `ptr->AsObject()`, `ptr->AsNumber()` or `ptr->AsString()` function to get its value. 
    - It is recommended to always do type checking using Unreal `check(...)` macro
- `static CLASS_NAME GetFromJson(TSharedPtr<FJsonObject> JsonObject)`
    - the static version of FromJson()
    - Create a new message class and use `FromJson(JsonObject)` to set its properties
- `virtual FString ToString () const`
    - Create a string with all of the properties of this class for printing. 
- `virtual TSharedPtr<FJsonObject> ToJsonObject() const`
    - Create an FJsonObject and save all of the properties of this message instance to the JsonObject. 
    - For numbers, use `JsonObject->SetNumberField(FieldName, Value)`
    - For strings, use `JsonObject->SetStringField(FieldName, Value)`
    - For other message types or ros time, use `JsonObject->SetObjectField(FieldName, Field.ToJsonObject())`    
    - For arrays, we first create a JsonValue array using `TArray<TSharedPtr<FJsonValue>> PtrArray`, then add shared pointer to new created FJsonValueObject / FJsonValueNumber / FJsonValueString to the array, and finally set the field value to this array using `Object->SetArrayField(FieldName, PtrArray);`
    
There are several good examples to follow when writing message classes. `geometry_msgs/Vector3` is a message class with only built-in types; `geometry_msgs/Accel` is a message class which includes other messages; `geometry_msgs/Polygon` is a message class with arrays.

##### Services

Services consists of two parts, **request** and **response**, each of which is a ROS message class. In the plugin,  for each service we define a class extending to the `FROSBridgeSrv` base class, inside which we define `Request` and `Response` classes respectively extending to the `SrvRequest` and `SrvResponse` class. Like messages, in each class we need to implement `FromJson`, `ToString`, `ToJsonObject` and static `GetFromJson` method functions. 

In `tutorial_srvs/AddTwoInts.h` there is a service class implementing `rospy_tutorials/AddTwoInts` service. 


### PR2 ROS Package For Kinetic

#### Run PR2 Unreal ROS Node 

After installation, run the following command in terminal to start a roscore server:

```
$ roscore
```

In a new terminal session, run ROSBridge server, you can also modify the port number to other numbers.  

```
$ roslaunch rosbridge_server websocket_server.launch port:=9001
```

In a new terminal session, run the following command to start the ROS Unreal Node. (Notice: this is a simplified pr2 model with base only)

```
$ roslaunch pr2_unreal pr2_simple.launch
```

In a new terminal session, run external PR2 controller, like `pr2_teleop`

```
$ roslaunch pr2_teleop teleop_keyboard.launch
```

Launch URoboSim project, in Unreal Editor Preferences's General - Miscellaneous tab, untick the "Use Less CPU When in Background" option. 

![image](images/urobo-2.png)

Then run game. 


#### How PR2 ROS Control Stack Works

##### Gazebo Node

In gazebo, ROS node is a plugin which integrates into the gazebo simulator. When simulator starts, 
it creates a robot model using given urdf description file, creates a controller manager which 
manages all the controllers, then adds controllers of base, arms, etc. to the manager. 

![image](images/gazebo-1.png)

External applications are connected to robot controllers. For example, the `pr2_teleop` reads 
keyboard input, then it send target velocity / rotation to the base controller. 

![image](images/gazebo-2.png)

The controller manager works in a loop in every frame. 

1. It first propagate (forward) **actuator positions** to **joint positions** (including position, velocity and measured effort).
   ![image](images/gazebo-3.png) 

2. Then it updates the controller to set the “**effort**” property in joint states. 
   ![image](images/gazebo-4.png) 

3. Controllers get current state (including position, velocity, previous efforts), and calculate the efforts so 
that they can update the efforts in robot model. 
   ![image](images/gazebo-4-2.png) 

4. Finally the controller manager propagates (forward) joint efforts to the actuator. The actuator can 
be connected to the EtherCAT devices to control a real PR2 robot, or to a **fake state** to 
synchronize robot state with simulators. 
   ![image](images/gazebo-5.png) 

In each gazebo frame, the gazebo 3d model will communicate with the ros plugin. 

1. The plugin gets all joint position and velocity into the fake joint state, 
   ![image](images/gazebo-6.png) 

   then backward-propagates them to actuator position.
   ![image](images/gazebo-7.png) 

2. Then the controller manager updates (executes the loop), reading actuator positions and returning actuator efforts. 
   ![image](images/gazebo-8.png) 

3. The plugin reverses the jointEffort->actuatorEffort transmission to get **joint efforts** in **fake joint states**. 
   ![image](images/gazebo-9.png) 

4. Finally the plugin uses the joint efforts to **add force / torque** to corresponding joint in gazebo.
   ![image](images/gazebo-10.png) 

#### Unreal Node

![image](images/urobo-1.png)

The Unreal Node is similar to the gazebo plugin in PR2 gazebo simulator. The only difference is that now unreal robot model and ros node are in different nodes, different processes so they cannot directly communicate with each other to send forces or get status. We need to use topics / messages to get joint states / joint forces at each frame. 

Now this node just subscribes `unreal_joint_state` as its joint state topic and publishes joint forces to `unreal_force` topic. 

![image](images/urobo-3.png)


Extra
-------

### Simulate Other Robots in Unreal

To simulate other robots in Unreal, we need a URDF model to import to URoboSim and definition of ROS controllers. 

#### URDF Model

For macros in `xacro` format, please run `rosrun xacro xacro.py new_robot.xacro` to convert xacro files to urdf xml files. 

#### ROS Controllers

ROS has provided a lot of common robot controllers in package `robot_mechanism_controllers` and PR2's ROS package `pr2_mechanism_controllers`, including: 

- robot_mechanism_controllers
  - JointTrajectoryActionController
  - JointEffortController
  - JointPositionController 
  - JointVelocityController
- pr2_mechanism_controllers
  - Pr2BaseController
  - Pr2GripperController
  - LaserScannerTrajController
  - Pr2Odometry
  
To define controllers, we need to create new controller configuration files in `pr2_controller_configuration_gazebo/config` directory. In this directory, create a new `yaml` file as controller definition file, like `new_robot.yaml`. 

In this yaml, we define each controller's name, type, joint that it works on and other parameters. 

For example, for `JointEffortController`, we only need its name, type and joint name, since the effort controller does only adds the force given by user to the joint. 

```
torso_lift_effort_controller: # name 
  type: robot_mechanism_controllers/JointEffortController 
  joint: torso_lift_joint
```

For `JointVelocityController` and `JointPositionController`, since they are PID controllers, apart from name, type and joint name, we also need the controller's PID parameters. 

```
r_shoulder_pan_velocity_controller:
  type: robot_mechanism_controllers/JointVelocityController
  joint: r_shoulder_pan_joint
  pid: &shoulder_pan_velocity_gains
    p: 30
    i: 4.67
    d: 0.0
    i_clamp: 100.0
```


