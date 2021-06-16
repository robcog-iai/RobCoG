## Test CloudSim queries in knowrob_ameva

### Setup and testing

* make sure k8s and the launcher is installed and setup: [k8s](CloudSim_k8sSetup.md), [cloudsim_k8s_launcher](CloudSim_k8sLauncher.md)

* make sure mongodb is running

  * you can use mongodb installed in your PC
 
    * Edit /etc/mongod.conf. Change ` bindIp: 127.0.0.1 ` to ` bindIp: 0.0.0.0 `

    ` $  sudo systemctl start mongod ` 
    
  * or you can use dockerized version of mongodb
   
    ` $ docker run --name mongo-server -d -p 27017:27017 mongo:latest ` This will start a new container for the first time. 
    
    ` $ docker container start mongo-server ` This will restart the container after reboot. You still have the data

* make sure ROS is sourced (`$ source ~/catkin_ws/devel/setup.bash`) 

* launch knowrob

 `$ roslaunch knowrob knowrob.launch` 
 
* in a new terminal start rosprolog

  `$ rosrun rosprolog rosprolog_commandline.py`
  
  `?- register_ros_package('knowrob_ameva').`
  
  `?- ue_start_srv`
  
* Test queries:
  
    ```
    am_get_drawer_stack_max(Num, 'StackCupsInIAI', 'utJDwYBP8CA', 'http://knowrob.org/kb/knowrob.owl#Cup', 'http://knowrob.org/kb/ameva_log.owl#zVBHGrf9n0qEVqc8aDbF-w', 23, 0, 0, 'http://knowrob.org/kb/ameva_log.owl#zoAfiCIbi0KJP-fecLJkoQ', 'http://knowrob.org/kb/ameva_log.owl#61nqVeqryECsFlyF5fjcBQ', 'http://knowrob.org/kb/ameva_log.owl#Dkkpmdu-L0Ssc3R5YJTftw', 20, 0, 0, 2)
    ```
  
  * test stacking plates in the drawer in IAI Kitchen
  
    ```
    am_get_drawer_stack_max(Num, 'StackPlatesInIAIKitchen', 'utJDwYBP8CA', 'http://knowrob.org/kb/knowrob.owl#MPPlate01', 'http://knowrob.org/kb/ameva_log.owl#zVBHGrf9n0qEVqc8aDbF-w', 23, 0, 0, 'http://knowrob.org/kb/ameva_log.owl#zoAfiCIbi0KJP-fecLJkoQ', 'http://knowrob.org/kb/ameva_log.owl#61nqVeqryECsFlyF5fjcBQ', 'http://knowrob.org/kb/ameva_log.owl#Dkkpmdu-L0Ssc3R5YJTftw', 20, 0, 0, 2) 
    
    ```
  
  * test stacking cups in the drawer in PK Kitchen
  
    ```
    am_get_drawer_stack_max(Num, 'StackCupsInPKKitchen', 'GttFys24Hbn', 'http://knowrob.org/kb/knowrob.owl#Cup', 'http://knowrob.org/kb/ameva_log.owl#dgw8m90FnkuHUnOaFpBe2A', 0, 35, 0, 'http://knowrob.org/kb/ameva_log.owl#ys6UK2Uy8k2YbsE_z0EJ3w', 'http://knowrob.org/kb/ameva_log.owl#OYcKhdsnpkmy0uZX203YGA', 'http://knowrob.org/kb/ameva_log.owl#mWSNVVYsTEy7Xqo-qnrriQ', 0, 30, 0, 1) 
    ```
  
    

## Visualize CloudSim world state data in RobCoG

### Commands

* *check mongodb data*
* *open the corresponding Map in RobCoG project and set up the KnowrobManager*
* am_semantic_map:am_load_semantic_map('utJDwYBP8CA', MapInst)
* am_semantic_map:am_get_individual_list('http://knowrob.org/kb/knowrob.owl#Cup', MapInst, IndividualList)
* ue_set_task(ClienId, Task)
* ue_set_episode(ClienId, Episode)
* ue_draw_marker(ClientId, CupId, 0, 100, 'sphere', 'red', 0.02, 'Translucent')

## Trouble shooting

* check if unreal engine running in k8s, make sure Pod ue-gs-xxxxx(xxx is random) is running
 
  `kubectl get pods`
 
* print unreal engine log in the Pod
  
  `kubectl logs ue-gs-xxxxx(xxx is random) ue-app`
