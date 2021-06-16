## Setup k8s cluster for CloudSim

#### Video

Not available at the moment

### commands

* Reset k8s cluster after reboot the PC:
* 
  `sudo kubeadm reset`
  
* Init k8s cluster with kubeadm

  `sudo swapoff -a`
  
  `sudo kubeadm init --pod-network-cidr=10.244.0.0/16`
  
* To start using your cluster, you need to run the following as a regular user: (This is required each time you reset the k8s. This file keeps the randomly generated authentication token, therefore you need to replace the old config file if exists)

  `mkdir -p $HOME/.kube`
  
  `sudo cp -i /etc/kubernetes/admin.conf $HOME/.kube/config`
  
  `sudo chown $(id -u):$(id -g) $HOME/.kube/config`
  
* Install Pod network

  `kubectl apply -f https://raw.githubusercontent.com/coreos/flannel/master/Documentation/kube-flannel.yml`
  
* GPU(Nvidia) support (from https://kubernetes.io/docs/tasks/manage-gpus/scheduling-gpus/)

  `nvidia-smi`
  
  `kubectl create -f https://raw.githubusercontent.com/NVIDIA/k8s-device-plugin/1.0.0-beta4/nvidia-device-plugin.yml`
  
* fix "master node doesnít schedule pod"
* 
  `kubectl taint nodes --all node-role.kubernetes.io/master-`

* Install Agones

  `kubectl create namespace agones-system`
  
  `kubectl apply -f https://raw.githubusercontent.com/googleforgames/agones/release-1.10.0/install/yaml/install.yaml`

