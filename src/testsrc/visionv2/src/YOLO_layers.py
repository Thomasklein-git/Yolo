import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'
from tensorflow.python.client import device_lib
print(device_lib.list_local_devices())
import shutil
import numpy as np
import tensorflow as tf
#from tensorflow.keras.utils import plot_model
#from yolov3.dataset import Dataset
from yolov3.yolov3 import Create_Yolo, compute_loss
from yolov3.utils import load_yolo_weights
from yolov3.configs import *
#from evaluate_mAP import get_mAP

if YOLO_TYPE == "yolov3":
    Darknet_weights = YOLO_V3_WEIGHTS #ÆNDRET

if TRAIN_TRANSFER:
        Darknet = Create_Yolo(input_size=YOLO_INPUT_SIZE)
        load_yolo_weights(Darknet, Darknet_weights) # use darknet weights

yolo = Create_Yolo(input_size=YOLO_INPUT_SIZE, training=True, CLASSES=TRAIN_CLASSES)
if TRAIN_FROM_CHECKPOINT:
    try:
        yolo.load_weights(TRAIN_FROM_CHECKPOINT)
    except ValueError:
        print("Shapes are incompatible, transfering Darknet weights")
        TRAIN_FROM_CHECKPOINT = False

if TRAIN_TRANSFER and not TRAIN_FROM_CHECKPOINT:
    for i, l in enumerate(Darknet.layers):
        layer_weights = l.get_weights()
        if layer_weights != []:
            try:
                yolo.layers[i].set_weights(layer_weights)
            except:
                print("skipping", yolo.layers[i].name)

#yolo.summary()
#for layer in yolo.layers:
#    layer.trainable=False

for i in range(len(yolo.layers)):
    yolo.layers[i].trainable=False
    if yolo.layers[i].name in ["conv2d_74", "conv2d_66", "conv2d_58"]:
        print(yolo.layers[i].name,"trainable layer")
        yolo.layers[i].trainable=True
yolo.summary()
