## Test CloudSim queries in knowrob_ameva

### Commands

* terminal one
  * sudo systemctl start mongod  or  docker run --name mongo-server -d -p 27017:27017 mongo:latest  (You can skip this if you mongodb is already running in the PC. Or you can use docker container run mongo-server in haidu-vr)
  * source devel/setup.bash (You can skip this, if you have added `source devel/setup.bash` to `~/.basrhc` )
  * roslaunch knowrob knowrob.launch
* terminal two
  * rosrun rosprolog rosprolog_commandline.py
  
  * register_ros_package('knowrob_ameva').
  
  * ue_start_srv
  
  * test stacking cups in the drawer in IAI Kitchen
  
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

