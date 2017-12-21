
# [UROSBridge](https://github.com/robcog-iai/UROSBridge) - Hello World

An example of using [UROSBridge](https://github.com/robcog-iai/UROSBridge) to communicate between Unreal and Ros with std-msgs/String. 

# Quick Start

### 1. Install [rosbridge_suite ](http://wiki.ros.org/rosbridge_suite) package in Ubuntu system.

* After installation, set up your environment for ROS and rosbridge:

   ```source /opt/ros/<rosdistro>/setup.bash```
   
* Run the rosbridge to create a [WebSocket](https://www.websocket.org/) on port 9090 by default.

   ```roslaunch rosbridge_server rosbridge_websocket.launch```
   
* Run the following command to publish a String Message 'Hello World' to a ROS topic.

   ``` rostopic pub Ros_String std_msgs/String "Hello World"```
   
   
   
### 2. Drag and drop the ```StringPublisher``` and ```RosStringSubscriber``` to your Level Editor.

* **```StringPublisher```** -- A Actor to publish FString Message to ROS topic ```Unreal_String```. 

* **```RosStringSubscriber```**  -- A Actor to subscribe String Message from another ROS topic ```Ros_String```.

   Your need to set _WebsocketIPAddr_ to your linux IP in both Actor. You can get more details about how to compile publisher and subscriber in Unreal from this [Tutorial](https://github.com/robcog-iai/UROSBridge/blob/master/Documentation/Examples.md).
   
   

### 3. Now you can compile and play in Level Editor.

* Make Sure to run the rosbridge in Ubuntu and then press play button in Unreal.




### Congratulation! Now you should able to see the Message published in ```Unreal_String``` topic and Message subscribed from topic ```Ros_String``` in Output Log in Unreal. 
