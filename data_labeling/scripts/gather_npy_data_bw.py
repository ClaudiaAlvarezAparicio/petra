#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import os
from data_labeling.msg import DataLabelingBW

vector_data_raw = []
vector_data_label = []
length_matrix = 256

# Callback of data_labeling
def callbackData(data):
    vector_data_raw.append(vector_to_matrix(data.data_raw))
    vector_data_label.append(vector_to_matrix(data.data_label))

# Method that creates a npy file with the information recived of the data_labeling topic
def generate_npy(ruta_bag):
    npy_data_raw = np.ndarray((len(vector_data_raw), 1, length_matrix, length_matrix), dtype=np.uint8)
    npy_data_label = np.ndarray((len(vector_data_raw), 1, length_matrix, length_matrix), dtype=np.uint8)

    for i in range(0, len(vector_data_raw)):
        npy_data_raw[i] = np.array([vector_data_raw[i]])
        npy_data_label[i] = np.array([vector_data_label[i]])

    directory, name = get_npy_directory_and_name(ruta_bag)

    file_raw = directory + "/npy_files/raw/" + name +'.npy'
    file_label = directory + "/npy_files/label/" + name +'.npy'

    np.save(file_raw, npy_data_raw)
    np.save(file_label, npy_data_label)

    print("Files created:")
    print("%s.  Size:  %s" % (file_raw, npy_data_raw.shape))
    print("%s.  Size:  %s" % (file_label, npy_data_label.shape))

# Method that creates the structure of directories to save the npy files
def generate_structure_directories(ruta_bag):
    directory, name = get_npy_directory_and_name(ruta_bag)
    directory = directory + "/npy_files"

    if not os.path.isdir(directory):
        os.makedirs(directory)
        subdirectory = directory + "/raw"
        os.makedirs(subdirectory)
        subdirectory = directory + "/label"
        os.makedirs(subdirectory)
        print("Directories structure created:")
        print(directory)
        print(directory + "/raw")
        print(directory + "/label")

# Method that receives the path to .bag file and return the directory and the name of the file without extension
def get_npy_directory_and_name(ruta):
    division = ruta.split("/")
    directory = ""
    for i in range(1, len(division) - 1):
        directory = directory + "/" +division[i]

    name = division[len(division) - 1].split(".bag")
    name = name[0]

    return directory, name

# Method that receives a vector and return a matrix with its values
def vector_to_matrix(vector):
    i = 0
    j = 0
    matriz = np.zeros((length_matrix,length_matrix), dtype=int)
    for item in vector:

        matriz[i][j] = item

        if j == length_matrix -1:
            i = i+1
            j = 0
        else:
            j = j+1

    return matriz


def start():
    rospy.init_node('gather_npy_data_bw', anonymous=True)
    #Get the rosbag_file param
    ruta_bag = rospy.get_param('gather_npy_data_bw/rosbag_file','rosbag_file')

    data_sub = rospy.Subscriber('/data_labeling', DataLabelingBW, callbackData)

    rospy.spin()

    # When the ros execution is finished if the structure of directories don't exist it is
    # created and the npy files are saved in it
    print("="*100)
    generate_structure_directories(ruta_bag)
    generate_npy(ruta_bag)
    print("="*100)

if __name__ == '__main__':
    start()
