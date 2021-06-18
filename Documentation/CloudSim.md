# CloudSim

## Introduction to CloudSim

CloudSim is a robot simulation system that is deployed on the Kubernetes cluster. In CloudSim, an Unreal Engine Pixel Streaming application represents a virtual environment for executing the simulation. CloudSim allows running multiple independent simulation simtanously by creating multiple Pixel Streaming applications in the clusters. Each Pixel Streaming application can be considered as a simulation instance. User can create the simualtion instances via HTTP service on demand.

The following image shows how CloudSim works. A GameServer Launcher is reponsible for creating and closing the Unreal Eninge Pixel Streaming applications in the cluster. Each Unreal Eninge Pixel Streaming application is wrap as an Agones GameServer. A Pixel Streaming application is composed of three components, Unreal Engine application, WebRTC Proxy Server,  Signalling and Web Server.

- ![cloudsim](/Users/lanxiaojun/Desktop/RobCoG/Documentation/Img/CloudSim.jpg)



## CloudSim Tutorial

1. Install required softwares for CloudSim (This step is only required when working with new PC) [here](./CloudSim_Installation.md)
2. Setup k8s cluster for CloudSim (This step is required after reboot) [here](./CloudSim_k8sSetup.md)
3. Deploy cloudsim_k8s_launcher on the cluster (This step is required after reboot) [here](./CloudSim_k8sLauncher.md)
4. Test example queries in knowrob_ameva [here](./CloudSim_TestQueries.md)



## Custom CloudSim Map

1. CloudSim Level development with RobCoG project in Windows [here](./CustomCSMap_Development.md)
2. Package CloudSim Level in a Docker Image in Linux [here](./CustomCSMap_Package.md)

### Soft reboot

```
kubectl delete gameservers --all \ 
&& kubectl delete pods --all \
&& cd ~/cloudsim_k8s_launcher && kubectl apply -f ./cloudsim_k8s_launcher.yaml \
&& mongo roslog --eval "printjson(db.dropDatabase())"
```

### Hard reboot batch cmd

```
yes | sudo kubeadm reset \
&& sudo swapoff -a \
&& sudo kubeadm init --pod-network-cidr=10.244.0.0/16 \
&& mkdir -p $HOME/.kube \
&& yes | sudo cp -i /etc/kubernetes/admin.conf $HOME/.kube/config \
&& sudo chown $(id -u):$(id -g) $HOME/.kube/config \
&& kubectl apply -f https://raw.githubusercontent.com/coreos/flannel/master/Documentation/kube-flannel.yml \
&& kubectl taint nodes --all node-role.kubernetes.io/master- && kubectl create namespace agones-system \
&& kubectl apply -f https://raw.githubusercontent.com/googleforgames/agones/release-1.10.0/install/yaml/install.yaml \
&& kubectl create clusterrolebinding default-view --clusterrole=view --serviceaccount=default:default \
&& kubectl create clusterrolebinding serviceaccounts-cluster-admin --clusterrole=cluster-admin --group=system:serviceaccounts \
&& cd ~/cloudsim_k8s_launcher \
&& kubectl apply -f ./cloudsim_k8s_launcher.yaml \
&& sudo systemctl start mongod
```

### Troubleshooting

* if simulation loops endlessly, remove `rosprolog` database from mongo and start again.
