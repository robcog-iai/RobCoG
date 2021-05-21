## Deploy cloudsim_k8s_launcher on the cluster

#### Video

Not available at the moment

### cloudsim_k8s_launcher repo

https://github.com/robcog-iai/cloudsim_k8s_launcher 

###  Commands

* Authentication to allocate Pod in k8s
  * kubectl create clusterrolebinding default-view --clusterrole=view --serviceaccount=default:default
  * kubectl create clusterrolebinding serviceaccounts-cluster-admin --clusterrole=cluster-admin --group=system:serviceaccounts
* Build cloudsim_k8s_launcher image. (You can skip cloning the project if you have already it. If the image in  docker hub is up-to-date, then you don't need to build the image again )
  * git clone https://github.com/robcog-iai/cloudsim_k8s_launcher 
  * cd cloudsim_k8s_launcher
  * *EDIT DOCKERFILE* 
  * docker build -t robcog/cloudsim_k8s_launcher .
  * docker push robcog/cloudsim_k8s_launcher 
* Deploy cloudsim_k8s_launcher in k8s
  * kubectl apply -f ./cloudsim_k8s_launcher.yaml

