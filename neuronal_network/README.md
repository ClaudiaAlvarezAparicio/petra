#############################
##  DIRECTORY EXPLANATION  ##
#############################
Inside of tf_keras_folder we have:
- input folder to include the .npy files to train the neural network
- model folder where the train model will be saved
- output folder where the output of the prediction of the network will be saved

#############################
## SINGULARITY INSTALATION ##
#############################

$ VERSION=2.5.2  
$ wget https://github.com/singularityware/singularity/releases/download/$VERSION/singularity-$VERSION.tar.gz  
$ tar xvf singularity-$VERSION.tar.gz  
$ cd singularity-$VERSION  
$ sudo apt-get install libarchive-dev  
$ ./configure --prefix=/usr/local  
$ make  
$ sudo make install  

#######################################################
## TRANSFORM .h5 model of Keras to .pb of TENSORFLOW ##
#######################################################

$ cd petra/neural_network/tf_keras_folder
$ git clone https://github.com/amir-abdi/keras_to_tensorflow  


#############################
##     BUILD CONTAINER     ##
#############################
$ cd neural_network
$ singularity image.create -s 2600 tf_keras_container.img
$ sudo singularity build tf_keras_container.img tf_keras_build_container
$ sudo singularity shell -B ./tf_keras_folder/:/root/tf_keras_folder tf_keras_container.img
Singularity tf_keras_container.img:~> mkdir /root/tf_keras_folder/
Singularity tf_keras_container.img:~> exit
$ sudo singularity shell -B ./tf_keras_folder/:/root/tf_keras_folder tf_keras_container.img
Singularity tf_keras_container.img:~> cd tf_keras_folder/
Singularity tf_keras_container.img:~/tf_keras_folder> ./execute.sh
