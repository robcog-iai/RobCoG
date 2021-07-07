# MMM Project

## Getting started

* clone https://github.com/robcog-iai/RobCoG with the following submodules:
  * `$ git clone -b master https://github.com/robcog-iai/RobCoG.git`
  * `$ git submodule update --init Plugins/UPhysicsBasedMC`
  * `$ git submodule update --init Plugins/USemLog`
  * `$ git submodule update --init Plugins/UMongoC`
* make sure the following plugins were cloned and are up to date  
  * `RobCoG/Plugins/UPhysicsBasedMC` - VR-enabled hand movements
  * `RobCoG/Plugins/USemLog` - semantic logger module for writing the episodic memories
  * `RobCoG/Plugins/UMongoC` - module for writing episodic memories to MongoDB
* install [MongoDB](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-windows/)
* install Unreal Engine 4.23

## Project structure

* mmm main map location `Content/Maps/MMM/MMM_IAIKitchen_VR.umap`
* main map is always empty and only includes sublevels (see [robcog doc](Sublevels.md) and [ue doc](https://docs.unrealengine.com/4.26/en-US/Basics/Levels/LevelsWindow/))
* sublevels are prefixed with `SL_`:
  * `SL_MMM_IAIKitchen_MC` - contains the VR enabled hands
  * `SL_MMM_IAIKitchen_Furniture` - contains the furniture meshes
  * `SL_MMM_IAIKitchen_SemLog` - contains the semantic logger manager
  * `SL_MMM_IAIKitchen_Items_VR` - contains the active items used for a given scenario
