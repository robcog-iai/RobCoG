## Deploy cloudsim_k8s_launcher on the cluster

#### Video

Not available at the moment

### cloudsim_k8s_launcher repo

https://github.com/robcog-iai/cloudsim_k8s_launcher 

###  Commands

* kubectl create clusterrolebinding default-view --clusterrole=view --serviceaccount=default:default
* kubectl create clusterrolebinding serviceaccounts-cluster-admin --clusterrole=cluster-admin --group=system:serviceaccounts
* git clone https://github.com/robcog-iai/cloudsim_k8s_launcher 
* cd cloudsim_k8s_launcher

* *EDIT DOCKERFILE* 
* docker build -t robcog/cloudsim_k8s_launcher .
* docker push robcog/cloudsim_k8s_launcher 
* kubectl apply -f ./cloudsim_k8s_launcher.yaml

