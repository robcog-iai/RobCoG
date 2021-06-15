### Package CloudSim Level in a Docker Image in Linux

Notice: Compiling in Linux is more strict than in Windows. Some error will not occur in Windows. To compile the project we need to use the tool `ue4-docker` to build Unreal Engine in Linux and use the image to compile the project

* Dependencies

```
$ sudo apt-get install python3-setuptools
$ pip3 install -U pip
```

* Install ue4-docker. https://pypi.org/project/ue4-docker/

  ```
  pip install ue4-docker
  ```

* Build container images for the Unreal Engine 4.23 version Pixel Streaming for Linux. https://docs.adamrehn.com/ue4-docker/commands/build

  ```
  ue4-docker build custom:4.23.1-pixelstreaming -repo=https://github.com/adamrehn/UnrealEngine.git -branch=4.23.1-pixelstreaming --no-engine
  
  ue4-docker build \
    custom:4.23.1-pixelstreaming \                        # Tag the image as adamrehn/ue4-full:4.23.1-pixelstreaming
    -repo=https://github.com/adamrehn/UnrealEngine.git \  # Use Adam's fork of the Unreal Engine
    -branch=4.23.1-pixelstreaming \                       # Use the branch for the Engine version we are targeting
    --no-engine                                           # Don't build the ue4-engine image, just source, minimal and full
    
  ```

* goto `/path_to_proj/RobCoG/`
* `RobCoG/Dockerfile` will build the image with the RobCoG project (under linux with nvidia gpu)
* make sure`docker_map_name` and the `LevelName` tag in the semantic map is the same

  ```
  docker build -t robcog/docker_map_name .
  ```

* login to docker (user robcog) with `$ docker login`
* upload the image to docker hub, `cloudsim_k8s_launcher` will pull the image from docker hub


  ```
  docker push robcog/docker_map_name
  ```

# Issues

* in case docker-eu is not recognized:

```
$ pip uninstall ue4-docker
reboot PC
$ pip install ue4-docker
```

