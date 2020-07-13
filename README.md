# PeTra (People Tracking)

PeTra is a tool which allows detecting and tracking people. The system is based on a CNN that uses an occupancy map constructed from the readings of a LIDAR sensor.

* Dataset: https://www.frontiersin.org/articles/10.3389/fnbot.2017.00072/full
* PeTra:
  * https://www.frontiersin.org/articles/10.3389/fnbot.2018.00085/full
  * https://www.mdpi.com/2218-6581/8/3/75

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

* ROS KINECTIC or ROS MELODIC
* ROS TensorFlow installation
```
$ cd ~/catkin_ws/src/  
&& git clone https://github.com/ethz-asl/tensorflow_catkin.git  
&& cd tensorflow_catkin/  
&& git checkout bcd27b06e65dd88c3f2f2875ff498b3c2bddcff3  
&& cd ../  
& git clone https://github.com/catkin/catkin_simple.git  
&& cd catkin_simple/  
&& git checkout 0e62848b12da76c8cc58a1add42b4f894d1ac21e  
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
To test PeTra we have create tow docker images, steps to test it:  
* ROS Kinetic
```  
$ docker pull claudiaalvarezaparicio/petra:kinetic  
$ docker run -d -p 6901:6901 -e VNC_PW=petra --name=petra claudiaalvarezaparicio/petra:kinetic  
```  
* ROS Melodic
```  
$ docker pull claudiaalvarezaparicio/petra:melodic  
$ docker run -d -p 6901:6901 -e VNC_PW=petra --name=petra claudiaalvarezaparicio/petra:melodic  
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

## Stop and Remove Docker Container   
```  
$ docker stop petra
$ docker rm petra 
```  

## Hardware Specifications
The tool has been test with a Hokuyo URG-04LX-UG01 (https://www.hokuyo-aut.jp/search/single.php?serial=166).

With another LIDAR it can work but is better train a new neural network model.

### Data Labelig for Training

To train a new neural network model is necessary to get the LIDAR data and label it. The data_labeling folder has a README to label data with a KIO RTLS system or with the own PeTra.


### Network Model Training

Once the data to train the network is ready, its time to train the model. The neural_network folder contains a README with the necessary steps to train the model. 

