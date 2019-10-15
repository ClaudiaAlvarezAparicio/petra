#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import cv2
import numpy as np
from keras.models import Model, load_model
from keras.layers import Input, concatenate, Conv2D, MaxPooling2D, UpSampling2D
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint, LearningRateScheduler
from keras import backend as K
import sys
K.set_image_data_format('channels_last')

# Shape of images
img_rows = 256
img_cols = 256
smooth = 1.

# Metrics
def dice_coef(y_true, y_pred):
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)
    return (2. * intersection + smooth) / (K.sum(y_true_f) + K.sum(y_pred_f) + smooth)


def dice_coef_loss(y_true, y_pred):
    return -dice_coef(y_true, y_pred)

# Build model
def get_unet():

    inputs = Input((img_rows, img_cols, 1))
    conv1 = Conv2D(8, (3, 3), activation='relu', padding='same')(inputs)
    conv1 = Conv2D(8, (3, 3), activation='relu', padding='same')(conv1)
    pool1 = MaxPooling2D(pool_size=(4, 4))(conv1)

    conv2 = Conv2D(16, (3, 3), activation='relu', padding='same')(pool1)
    conv2 = Conv2D(16, (3, 3), activation='relu', padding='same')(conv2)
    pool2 = MaxPooling2D(pool_size=(4, 4))(conv2)

    conv3 = Conv2D(32, (3, 3), activation='relu', padding='same')(pool2)
    conv3 = Conv2D(32, (3, 3), activation='relu', padding='same')(conv3)
    pool3 = MaxPooling2D(pool_size=(4, 4))(conv3)

    conv4 = Conv2D(64, (3, 3), activation='relu', padding='same')(pool3)
    conv4 = Conv2D(64, (3, 3), activation='relu', padding='same')(conv4)
    pool4 = MaxPooling2D(pool_size=(4, 4))(conv4)

    conv5 = Conv2D(128, (3, 3), activation='relu', padding='same')(pool4)
    conv5 = Conv2D(128, (3, 3), activation='relu', padding='same')(conv5)

    up6 = concatenate([UpSampling2D(size=(4, 4))(conv5), conv4], axis=3)
    conv6 = Conv2D(64, (3, 3), activation='relu', padding='same')(up6)
    conv6 = Conv2D(64, (3, 3), activation='relu', padding='same')(conv6)

    up7 = concatenate([UpSampling2D(size=(4, 4))(conv6), conv3], axis=3)
    conv7 = Conv2D(32, (3, 3), activation='relu', padding='same')(up7)
    conv7 = Conv2D(32, (3, 3), activation='relu', padding='same')(conv7)

    up8 = concatenate([UpSampling2D(size=(4, 4))(conv7), conv2], axis=3)
    conv8 = Conv2D(16, (3, 3), activation='relu', padding='same')(up8)
    conv8 = Conv2D(16, (3, 3), activation='relu', padding='same')(conv8)

    up9 = concatenate([UpSampling2D(size=(4, 4))(conv8), conv1], axis=3)
    conv9 = Conv2D(8, (3, 3), activation='relu', padding='same')(up9)
    conv9 = Conv2D(8, (3, 3), activation='relu', padding='same')(conv9)

    conv10 = Conv2D(1, (1, 1), activation='sigmoid')(conv9)

    model = Model(inputs=inputs, outputs=conv10)

    model.compile(optimizer=Adam(lr=5e-4), loss=dice_coef_loss, metrics=[dice_coef])
    return model

# Preprocess images
def preprocess(imgs):
    imgs_p = np.swapaxes(imgs,1,2)
    imgs_p = np.swapaxes(imgs_p,2,3)
    return imgs_p

# Entrenamiento y test
def train_and_predict(args):
    print('-'*50)
    print('Load and Preprocess data')
    print('-'*50)

    train_points = np.load('./input/npy_total_points.npy')
    train_labels = np.load('./input/npy_total_labels.npy')

    train_points = preprocess(train_points)
    train_labels = preprocess(train_labels)

    print('-'*50)
    print('Shape of train data: ',train_points.shape)
    print('-'*50)

    print('-'*50)
    print('Building model')
    print('-'*50)
    model = get_unet()
    model_checkpoint = ModelCheckpoint('./model/model.h5', monitor='loss', save_best_only=True)

    print('-'*50)
    print('Training model')
    print('-'*50)
    model.fit(train_points, train_labels, batch_size=128, epochs=30, verbose=1, shuffle=True,
              callbacks=[model_checkpoint])

    print('-'*50)
    print('Load and Preprocess data test')
    print('-'*50)

    test_points = np.load('./input/npy_total_test_points.npy')
    test_points = preprocess(test_points)

    print('-'*50)
    print('Shape of test data: ', test_points.shape)
    print('-'*50)

    print('-'*50)
    print('Load weights of model')
    print('-'*50)
    model.load_weights('./model/model.h5')

    print('-'*50)
    print('Prediction test data')
    print('-'*50)
    test_labels_generated = model.predict(test_points, verbose=1)
    # Save output of prediction
    np.save('./output/test_labels' , test_labels_generated)
    model.save('./model/model.h5', include_optimizer=False)

if __name__ == '__main__':
    train_and_predict(sys.argv)
