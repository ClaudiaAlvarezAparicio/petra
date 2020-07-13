#!/bin/bash
apt-get update
#apt-get -y upgrade
apt-get install -y git
pip install keras==2.3.1
pip install pathlib
pip install absl-py
git clone https://github.com/amir-abdi/keras_to_tensorflow.git
