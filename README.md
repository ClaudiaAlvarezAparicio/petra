# PeTra (People Tracking)

PeTra is a tool which allows detecting and tracking people. The system is based on a CNN that uses an occupancy map constructed from the readings of a LIDAR sensor.

* Dataset: https://www.frontiersin.org/articles/10.3389/fnbot.2017.00072/full
* PeTra:
  * https://www.frontiersin.org/articles/10.3389/fnbot.2018.00085/full
  * https://www.mdpi.com/2218-6581/8/3/75

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

* ROS KINECTIC
* ROS TensorFlow installation
```
$ cd ~/catkin_ws/src/  
$ git clone https://github.com/ethz-asl/tensorflow_catkin.git  
$ git clone https://github.com/catkin/catkin_simple.git
```
To compile tensorflow_catkin package with CPU optimization. Open the CMakeLists.txt file and change to ON the flag: "-Dtensorflow_OPTIMIZE_FOR_NATIVE_ARCH=OFF"

```
$ cd ~/catkin_ws/
$ catkin_make  
$ cd ~/catkin_ws/src/
$ git clone https://github.com/tradr-project/tensorflow_ros_cpp.git
$ cd ..
$ catkin_make
```

### PeTra Installation

 ```
 $ cd ~/catkin_ws/src/  
 $ git clone https://github.com/ClaudiaAlvarezAparicio/petra.git
 $ cd ..  
 $ catkin_make  

 ```
### Configuration
Edit the different parameters of petra/config/parameters.yaml


### Execution

Execution with a rosbag:  
```
roslaunch petra petra_rosbag.launch rosbag_file:=absolute_path_to_bag_file
```
Execution in real time:  
```
roslaunch petra petra.launch
```

## Docker Image   
To test PeTra we have create a docker image, steps to test it:  
```  
$ docker pull claudiaalvarezaparicio/petra:kinetic  
$ docker run -d -p 6901:6901 -e VNC_PW=petra --name=petra claudiaalvarezaparicio/petra:kinetic  
```  
In the browser: http://localhost:6901/  
Password: petra  
  
### Execute PeTra in docker  
1.- Open terminal:   
```
$ roscore  
```
2.- Open terminal:  
```
$ rosrun rviz rviz  
```  
3.- Open terminal  
```
$ roslaunch petra petra_rosbag.launch rosbag_file:=/home/student/rosbags/kitchen1.bag  
```

## Hardware Specifications
The tool has been test with a Hokuyo URG-04LX-UG01 (https://www.hokuyo-aut.jp/search/single.php?serial=166).

With another LIDAR it can work but is better train a new neural network model.

### Data Labelig for Training

To train a new neural network model is necessary get the LIDAR data and label it. The data_labeling folder has a README to label data with a KIO RTLS system.


### Network Model Training

Once the data to train the network is ready, is time to train the model. The neural_network folder contains the train file and the directories tree to easily train the model. First put the .npy files created in the petra/neural_network/tf_keras_folder/input/ folder.

#### Singularity installation

```
$ VERSION=2.5.2  
$ wget https://github.com/singularityware/singularity/releases/download/$VERSION/singularity-$VERSION.tar.gz  
$ tar xvf singularity-$VERSION.tar.gz  
$ cd singularity-$VERSION  
$ sudo apt-get install libarchive-dev  
$ ./configure --prefix=/usr/local  
$ make  
$ sudo make install  

```

#### Build a Singularity Container
```
$ cd petra/neural_network/
$ git clone https://github.com/amir-abdi/keras_to_tensorflow.git  
$ singularity image.create -s 2600 tf_keras_container.img  
$ sudo singularity build tf_keras_container.img tf_keras_build_container  
$ sudo singularity shell -B ../tf_keras_folder/:/root/tf_keras_folder tf_keras_container.img  
$ Singularity tf_keras_container.img:~> python3.5 train_neural_network.py
$ Singularity tf_keras_container.img:~> python3.5 tf_keras_folder.py --input_model="/root/tf_keras_folder/model/model.h5" --output_model="/root/tf_keras_folder/model/model.pb"  
$ Singularity tf_keras_container.img:~> exit  
```

Once the model is trained and in a TensorFlow format.

```
cp petra/neural_network/tf_keras_folder/model/model.pb /petra/petra/model/
```