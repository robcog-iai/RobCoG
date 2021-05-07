# Prerequisites:

* ubuntu 18.04
* docker
* kubernetes
* agones
* mongodb
* ROS
* swipl
* knowrob
* knowrob_ameva

## Docker

* from https://docs.docker.com/engine/install/ubuntu/

 
  * `$ sudo apt-get update && sudo apt-get install apt-transport-https ca-certificates curl gnupg lsb-release`
  * `$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg`
  * `$ echo \
  "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null`
  * `$ sudo apt-get update && sudo apt-get install docker-ce docker-ce-cli containerd.io` 
  
## Kubernetes

### kubeadm, kubelet and kubectl

* from https://kubernetes.io/docs/setup/production-environment/tools/kubeadm/install-kubeadm/
 
  * `$ sudo apt-get update`
  * `$ sudo apt-get install -y apt-transport-https ca-certificates curl`
  * `$ sudo curl -fsSLo /usr/share/keyrings/kubernetes-archive-keyring.gpg https://packages.cloud.google.com/apt/doc/apt-key.gpg`
  * `$ echo "deb [signed-by=/usr/share/keyrings/kubernetes-archive-keyring.gpg] https://apt.kubernetes.io/ kubernetes-xenial main" | sudo tee /etc/apt/sources.list.d/kubernetes.list`
  * `$ sudo apt-get update`
  * `$ sudo apt-get install -y kubelet=1.16.15-00 kubeadm=1.16.15-00 kubectl=1.19.2-00`
  * `$ sudo apt-mark hold kubelet kubeadm kubectl`
 
* init:

 * `$ sudo swapoff -a`
 * `$ sudo kubeadm init --pod-network-cidr=10.244.0.0/16`
 * `$ kubectl apply -f https://raw.githubusercontent.com/coreos/flannel/master/Documentation/kube-flannel.yml`

 * To start using your cluster, you need to run the following as a regular user:

  * `$ mkdir -p $HOME/.kube`
  * `$ sudo cp -i /etc/kubernetes/admin.conf $HOME/.kube/config`
  * `$ sudo chown $(id -u):$(id -g) $HOME/.kube/config`

*  fix "master node doesnít schedule pod"
 * `$ kubectl taint nodes --all node-role.kubernetes.io/master-`


### gpu (nvidia) support  

* from https://kubernetes.io/docs/tasks/manage-gpus/scheduling-gpus/

  * `$ nvidia-smi`
  * `$ kubectl create -f https://raw.githubusercontent.com/NVIDIA/k8s-device-plugin/1.0.0-beta4/nvidia-device-plugin.yml`

## Agones

* from https://agones.dev/site/docs/installation/install-agones/yaml/
  
  * `$ kubectl create namespace agones-system`
  * `$ kubectl apply -f https://raw.githubusercontent.com/googleforgames/agones/release-1.10.0/install/yaml/install.yaml`

### cloudsim_k8s_launcher
* k8s access authority
 
  * `$ kubectl create clusterrolebinding default-view --clusterrole=view --serviceaccount=default:default`
  * `$ kubectl create clusterrolebinding serviceaccounts-cluster-admin --clusterrole=cluster-admin  --group=system:serviceaccounts`

* from https://github.com/robcog-iai/cloudsim_k8s_launcher
  * `$ git clone ttps://github.com/robcog-iai/cloudsim_k8s_launcher && cd loudsim_k8s_launcher`
  * Edit Dockefile
  
   * PORT - the port the launcher listen to(no need to change)
   * HOST - the ip address of the host running the k8s cluster. The host runs the cloudsim_k8s_launcher
   * MONGO_IP - ip address of the mongodb for keeping world state data, usually the same host
   * MONGO_PORT - port of mongodb
   * IMAGE_REPO - when launcher create the resource, it will pull images from the docker hub. This specify where to pull the image.
  
 * `$ docker build -t robcog/cloudsim_k8s_launcher .`
 * `$ docker push robcog/cloudsim_k8s_launcher .`
 * `$ kubectl apply -f ./cloudsim_k8s_launcher.yaml`

## MongoDB

* from https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/:

### libmongoc:

* from http://mongoc.org/libmongoc/current/installing.html

  * `$ sudo apt-get install libmongoc-1.0-0`

## ROS

* from https://wiki.ros.org/melodic/Installation/Ubuntu

 * `$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
 * `$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`
 * `$ sudo apt update && sudo apt install ros-melodic-desktop`
 * `$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`
 * `$ source ~/.bashrc`



## Prolog

* from https://www.swi-prolog.org/build/PPA.html:

  * `$ sudo apt-add-repository ppa:swi-prolog/stable`
  * `$ sudo apt-get update && sudo apt-get install swi-prolog`

  
## KnowRob

* from https://github.com/knowrob/knowrob:

  * `$ sudo apt install python-rosdep2 python-wstool rosbash`
  * `$ cd ~ && mkdir catkin_ws && cd catkin_ws && rosdep update`
  * `$ wstool init src`
  * `$ cd src`
  * `$ wstool merge https://raw.github.com/knowrob/knowrob/master/rosinstall/knowrob-base.rosinstall`
  * `$ wstool update`
  * `$ rosdep install --ignore-src --from-paths .`
  * `$ cd ~/catkin_ws && catkin_make`
  * `$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`  
  * `$ echo "export SWI_HOME_DIR=/usr/lib/swi-prolog" >> ~/.bashrc`  
  * `$ source ~/.bashrc`


### knowrob_ameva

* websockets:

  * `$ sudo apt-get install libwebsockets-dev`

* protobuf (https://github.com/protocolbuffers/protobuf/blob/master/src/README.md):

  * `$ sudo apt-get install autoconf automake libtool curl make g++ unzip`
  * `$ cd ~ && mkdir thirdparty && cd thirdparty`
  * `$ git clone https://github.com/protocolbuffers/protobuf.git`
  * `$ cd protobuf`
  * `$ git submodule update --init --recursive`
  * `$ ./autogen.sh`
  * `$ ./configure`
  * `$ make`
  * `$ make check`
  * `$ sudo make install`
  * `$ sudo ldconfig`


* knowrob_ameva:

  * `$ sudo apt install libcurlpp-dev`
  * `$ cd ~/catkin_ws/src`
  * `$ git clone https://github.com/robcog-iai/knowrob_ameva.git`
  * `$ cd ~/catkin_ws && catkin_make knowrob_ameva`

