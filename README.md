 
# [UROSBridge](https://github.com/robcog-iai/UROSBridge) - Hello World

An example of using [UROSBridge](https://github.com/robcog-iai/UROSBridge) to communicate between Unreal and Ros with std-msgs/String. 

# Quick Start

See repository branches for various projects/versions:

### 1. Install [rosbridge_suite ](http://wiki.ros.org/rosbridge_suite) package in linux system.
* After installation, set up your environment for ROS and rosbridge:

   ```source /opt/ros/<rosdistro>/setup.bash```
   
* Run the rosbridge to create a [WebSocket](https://www.websocket.org/) on port 9090 by default.

   ```roslaunch rosbridge_server rosbridge_websocket.launch```

