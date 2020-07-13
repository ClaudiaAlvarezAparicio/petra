## Prerequisites
* Docker installation

## Training Folder Explanation
* input folder: to include the .npy files to train the neural network
* model folder: where the trained model will be saved
* output folder: where the output of the test prediction will be saved

## Execution
First put the .npy files created in the petra/neural_network/training/input/ folder.

```
$ docker pull tensorflow/tensorflow:1.13.1-py3
$ docker run -it -v <absolute_path_training_folder>:/PeTra --rm tensorflow/tensorflow:1.13.1-py3 bash
```
Inside the docker container:
```
$ cd /PeTra/
$ ./prerequisites.sh 
$ ./train_export_model.sh 
```
The project https://github.com/amir-abdi/keras_to_tensorflow allow transforming .h5 model of Keras to .pb of Tensorflow. The execution of this script is included in the ./train_export_model.sh script. It reports some important data:
```
"Converted output node names are: ['conv2d_19/Sigmoid']"
This value "conv2d_19/Sigmoid" is necessary to specify in the  petra/petra/config/parameters.yaml file.

```
Once the model is trained and in a TensorFlow format.

```
cp petra/neural_network/training/model/model.pb petra/petra/model/
```