#!/bin/bash
python train_neural_network.py
python keras_to_tensorflow/keras_to_tensorflow.py --input_model="model/model.h5" --output_model="model/model.pb"