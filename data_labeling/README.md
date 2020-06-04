# Data Labeling

This node (data_labeling_one_person) allow label the data to train the neural network. It creates for each laser scanning 2 black and
white images. The first contains all the points of the laser and the second contains this points labeled (only contains the points
of the laser that form part of a person).



## Execution with KIO

### Requirements

To labeling the data we use as groundh-truth data the KIO RTLS System. To get the information from KIO RTLS in ROS we need the package
kio_rtls available in github:
```
$ cd ~/catkin_ws
$ git clone https://github.com/am-guerrero/kio_rtls.git
$ cd ..
$ catkin_make
```

### For launch it
1.- Change the name of the topic where kio_rtls publish

2.- roslaunch data_labeling data_labeling_one_person.launch rosbag_file:=<absolute_path_to_bag_file>

This launcher run the node data_labeling_one_person and the script gather_npy_data_bw.py

## Execution with PeTra
In this case, the labeling of the information to train the neural network used by PeTra, is carried out using the data provided by itself. It is necessary configure the topic of the LIDAR in the PeTra's parameter.yaml file

roslaunch data_labeling data_labeling_with_petra.launch rosbag_file:=<absolute_path_to_bag_file> npy_directory:=<absolute_path_to_directory_where_save_npys>

## Scripts explanation

There are 3 types of scripts:

1.- gather_npy_data_bw.py: this script create 2 npy files (one for raw data an other for label data). This script creates
an structure of directories in the folder of the .bag file received as parameter. In this directories were saved the npy files
with raw and label info, these files has the same name as the bag file. The .npy that creates has the length (x, 1, 256, 256).
The structure is cretaed is:
<directory of bag file>
	npy_files
		raw
			*.npy
		labels
			*.npy

2.- join_npy_files_bw.py: this script create 2 npy files with the information of all the npy files that were in the directory
that receives as parameter. For each element in the raw folder it search the same name file in the label folder and if exist
it can be add to the global npy, if one of them doesn't exist, neither file is add to the global npy. To execute it:
rosrun data_labeling join_npy_files_bw.py <absolute_path>/npy_files

3.- npy_to_images_bw.py: this script create the images containing in the .npy file that receives as parameter. To execute it:
rosrun data_labeling npy_to_images_bw.py <absolute_path_npy_file> <name_of_imagenes> <absolute_path_directory_to_save_the_images>.
