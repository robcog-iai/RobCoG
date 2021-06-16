## Deploy cloudsim_k8s_launcher on the cluster

#### Video

Not available at the moment

### cloudsim_k8s_launcher repo

https://github.com/robcog-iai/cloudsim_k8s_launcher 

###  Commands

* Authentication to allocate Pod in k8s
  * kubectl create clusterrolebinding default-view --clusterrole=view --serviceaccount=default:default
  * kubectl create clusterrolebinding serviceaccounts-cluster-admin --clusterrole=cluster-admin --group=system:serviceaccounts
* Build cloudsim_k8s_launcher image. (You can skip cloning the project if you have already the project in your PC)
  * git clone https://github.com/robcog-iai/cloudsim_k8s_launcher 
  * cd cloudsim_k8s_launcher (following commands execute under cloudsim_k8s_launcher project)
  * *EDIT DOCKERFILE* 
  ```
  ENV PORT=9090 \                  
    HOST=192.168.102.25 \
    MONGO_IP=192.168.102.25 \
    MONGO_PORT=27017 \
    IMAGE_REPO=robcog
  ```
  * docker build -t robcog/cloudsim_k8s_launcher .
  * docker push robcog/cloudsim_k8s_launcher 
* Deploy cloudsim_k8s_launcher in k8s
  * kubectl apply -f ./cloudsim_k8s_launcher.yaml

