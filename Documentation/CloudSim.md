# CloudSim

## Introduction to CloudSim

CloudSim is a robot simulation system that is deployed on the Kubernetes cluster. In CloudSim, an Unreal Engine Pixel Streaming application represents a virtual environment for executing the simulation. CloudSim allows running multiple independent simulation simtanously by creating multiple Pixel Streaming applications in the clusters. Each Pixel Streaming application can be considered as a simulation instance. User can create the simualtion instances via HTTP service on demand.

The following image shows how CloudSim works. A GameServer Launcher is reponsible for creating and closing the Unreal Eninge Pixel Streaming applications in the cluster. Each Unreal Eninge Pixel Streaming application is wrap as an Agones GameServer. A Pixel Streaming application is composed of three components, Unreal Engine application, WebRTC Proxy Server,  Signalling and Web Server.

- ![cloudsim](/Users/lanxiaojun/Desktop/RobCoG/Documentation/Img/CloudSim.jpg)



## CloudSim Tutorial

1. Install required softwares for CloudSim [here](./CloudSim_Installation.md)
2. Setup k8s cluster for CloudSim [here](./CloudSim_k8sSetup.md)
3. Deploy cloudsim_k8s_launcher on the cluster [here](./CloudSim_k8sLauncher.md)
4. Test example queries in knowrob_ameva [here](./CloudSim_TestQueries.md)



## Custom CloudSim Map

1. CloudSim Level development with RobCoG project in Windows [here](./CustomCSMap_Development.md)
2. Package CloudSim Level in a Docker Image in Linux [here](./CustomCSMap_Package.md)