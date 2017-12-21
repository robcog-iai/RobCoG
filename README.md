
# [UROSBridge](https://github.com/robcog-iai/UROSBridge) - Hello World

An example of using [UROSBridge](https://github.com/robcog-iai/UROSBridge) to communicate between Unreal and Ros with std-msgs/String. 

# Quick Start

### 1. Install [rosbridge_suite ](http://wiki.ros.org/rosbridge_suite) package in linux system.
* After installation, set up your environment for ROS and rosbridge:

   ```source /opt/ros/<rosdistro>/setup.bash```
   
* Run the rosbridge to create a [WebSocket](https://www.websocket.org/) on port 9090 by default.

   ```roslaunch rosbridge_server rosbridge_websocket.launch```
   
### 2. Drag and drop the ```StringPublisher``` and ```RosStringSubscriber``` to your level Editor.

* ```StringPublisher``` -- A Actor to publish FString Message to ROS topic. 

* ```RosStringSubscriber```  -- A Actor to subscribe String Message from another ROS topic.
   Your need to set _WebsocketIPAddr_ to your linux IP in both Actor. You can get more details about how to compile publisher and subscriber in Unreal from this [Tutorial](https://github.com/robcog-iai/UROSBridge/blob/master/Documentation/Examples.md).

