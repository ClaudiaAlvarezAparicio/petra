#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np
import cv2

image_rows = 256
image_cols = 256

def start(args):
    npy_file = args[1]
    name_images = args[2]
    directory = os.path.abspath(os.path.join(os.getcwd(), os.pardir)) + "/images_npy/"

    if (len(args) == 4):
        directory = args[3]
    
    data = np.load(npy_file)
    print(data.shape)

    for i in range(0, data.shape[0]):
        item = data[i]
        item = item.reshape(image_rows,image_cols)
        img1=cv2.normalize(item, item,0,255,cv2.NORM_MINMAX)
        cv2.imwrite(directory+"/"+name_images+'_'+ str(i)+ '.png',img1)

    print("Number of imagenes: "+str(data.shape[0])+" .The images were written in: "+directory)


if __name__ == '__main__':

    if (len(sys.argv) < 3 or len(sys.argv) > 4):
        print("Usage: python npy_to_images_bw.py <absolute_path_npy_file> <name_of_imagenes> <absolute_path_directory_to_save>")
        print("<absolute_path_directory_to_save> is OPTIONAL. If not specify, the imagenes are saved in the images_npy directory of the proyect")
    
    start(sys.argv)