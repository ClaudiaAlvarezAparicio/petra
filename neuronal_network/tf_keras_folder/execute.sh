#!/bin/bash

python train_neural_network.py
python keras_to_tensorflow-master/keras_to_tensorflow.py --input_model="model/modelo.h5" --output_model="model/modelo.pb"
