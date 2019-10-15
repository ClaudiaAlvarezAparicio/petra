#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
from os import listdir
from os.path import isfile, join
import numpy as np
import cv2

image_rows = 256
image_cols = 256

# Returns true if the directory exist, if not return false
def is_directory_correct(directory):
    return os.path.isdir(directory) 

# Returns two list with the files .npy containing in the directory received
def get_files_npy(directory):
    listNPY_raw = []
    listNPY_label = []

    directory_raw = directory + "/raw/"
    # For each item in the directory /raw
    for item in listdir(directory_raw):
        # Test if item is a file 
        if isfile(directory_raw+item):
            # Test if the extension is .npy
            (nombreFichero, extension) = os.path.splitext(directory_raw+item)
            if(extension == ".npy"):
                # test if the file is in the label directory 
                if isfile(directory+"/label/"+item):
                    # Add both files to the lists
                    listNPY_raw.append(directory+"/raw/"+item)
                    listNPY_label.append(directory+"/label/"+item)   

    return listNPY_raw, listNPY_label

# Method that join all the npy files received in the list to an two unique npy list
def join_data_npy(listNPY_raw, listNPY_label, directory):
    npy_raw_global = []
    npy_label_global = []
    posicion = 0

    for i in range(0, len(listNPY_raw)):
        # Open .npy files
        print(listNPY_raw[i])
        print(listNPY_label[i])
        data_raw = np.load(listNPY_raw[i])
        data_label = np.load(listNPY_label[i])
        # print its shape
        print(data_raw.shape)
        print(data_label.shape)

        for j in range( 0, data_raw.shape[0]):
            npy_raw_global.append(data_raw[j])
            npy_label_global.append(data_label[j])
 
    save_npy_global(npy_raw_global, npy_label_global, directory)


# Methot that creates two npy files with the information that receives
def save_npy_global(npy_raw_global, npy_label_global, directory):
    
    npy_raw = np.ndarray((len(npy_raw_global), 1, image_rows, image_cols), dtype=np.uint8)
    npy_label = np.ndarray((len(npy_label_global),1, image_rows, image_cols), dtype=np.uint8)
    
    for i in range(0, len(npy_raw_global)):
        npy_raw[i] = np.array([npy_raw_global[i]])
        npy_label[i] = np.array([npy_label_global[i]])


    np.save(directory +'/raw/npy_total.npy', npy_raw)
    np.save(directory +'/label/npy_total.npy', npy_label)

    print("="*100)
    print("Files created:")
    print("%s/raw/npy_total.npy. Size:  %s" % (directory, npy_raw.shape))
    print("%s/label/npy_total.npy. Size:  %s" % (directory, npy_label.shape))
    print("="*100)


def start(args):
    
    directory = args[1]

    if not is_directory_correct(directory):
        print("The directory doesn't exists")
        sys.exit(1)

    listNPY_raw, listNPY_label = get_files_npy(directory)

    if len(listNPY_raw) == 0:
        print("The directory doesn't contains any .npy file")
    else:
        join_data_npy(listNPY_raw, listNPY_label, directory)


if __name__ == '__main__':
    start(sys.argv)