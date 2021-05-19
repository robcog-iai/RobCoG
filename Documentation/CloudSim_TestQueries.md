## Test CloudSim queries in knowrob_ameva

### Commands

* terminal one
  * sudo systemctl start mongod
  * source devel/setup.bash
  * roslaunch knowrob knowrob.launch
* terminal two
  * rosrun rosprolog rosprolog_commandline.py
  * register_ros_package('knowrob_ameva').
  * ue_start_srv
  * test stacking cups in the drawer in IAI Kitchen
  * am_get_drawer_stack_max(Num, 'StackCupsInIAI', 'utJDwYBP8CA', 'http://knowrob.org/kb/knowrob.owl#Cup', 'http://knowrob.org/kb/ameva_log.owl#zVBHGrf9n0qEVqc8aDbF-w', 23, 0, 0, 'http://knowrob.org/kb/ameva_log.owl#zoAfiCIbi0KJP-fecLJkoQ', 'http://knowrob.org/kb/ameva_log.owl#61nqVeqryECsFly1F5fjcBQ', 'http://knowrob.org/kb/ameva_log.owl#Dkkpmdu-L0S01s0c3R5YJTftw"', -20, 0, 0, 2)
  * test stacking plates in the drawer in IAI Kitchen
  * test stacking cups in the drawer in PK Kitchen



## Visualize CloudSim world state data in RobCoG

### Commands

* *check mongodb data*
* *open the corresponding Map in RobCoG project and set up the KnowrobManager*
* am_semantic_map:am_load_semantic_map('utJDwYBP8CA', MapInst)
* am_semantic_map:am_get_individual_list('http://knowrob.org/kb/knowrob.owl#Cup', MapInst, IndividualList)
* ue_set_task(ClienId, Task)
* ue_set_episode(ClienId, Episode)
* ue_draw_marker(ClientId, CupId, 0, 100, 'sphere', 'red', 0.02, 'Translucent')

