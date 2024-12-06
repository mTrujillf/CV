
import os
import cv2
import random
import glob
import numpy as np
import tensorflow as tf
import keras.api._v2.keras as keras
from keras.preprocessing.image import ImageDataGenerator
from keras.utils import to_categorical
from keras.applications import MobileNetV2
from keras.models import Model
from keras.layers import Dense, GlobalAveragePooling2D
from keras.models import load_model
from keras.optimizers import Adam



def load_images_and_labels(labels_file):
    pitch_roll_yaw = []
    max_arr = [4.0, 2.0, 149.0]
    with open(labels_file, 'r') as file:
        for line in file:
            arr_datos = line.strip().split(',')
            float_datos = [float(dato) for dato in arr_datos[:3]]
            pitch_roll_yaw.append(float_datos)
    
    pitch_roll_yaw_normal = []
    
    for x in pitch_roll_yaw:
        res = []
        res.append(x[0]/max_arr[0])
        res.append(x[1]/max_arr[1])
        res.append(x[2]/max_arr[2])
        pitch_roll_yaw_normal.append(res)

   
    images = [cv2.imread(file, cv2.IMREAD_COLOR) for file in glob.glob('FotosOriginales/FotosYDatos2/Fotos/*.jpg')]
    images_final = [cv2.resize(img, (224, 224)) for img in images if img is not None]
    
    images_final = np.array(images_final, dtype=np.float32)
    images_final /= 255.0  # Normalize to [0, 1]
    
    return images_final, np.array(pitch_roll_yaw_normal)



#new_images, new_labels = load_images_and_labels('FotosSegundoFin/NuevasFotos/datos_vuelo.txt')
new_images, new_labels = load_images_and_labels('FotosOriginales/FotosYDatos2/datos_vuelo.txt')
#new_images, new_labels = load_images_and_labels('FotosSoloGiro/datos_vuelo.txt')
 
if len(new_images) == len(new_labels):
    #model = load_model('C:/NN/drone_path_following_model_normal.h5')
    model = load_model('C:/NN/giro_updated_drone_path_following_model_normal_2_2.keras')

    for layer in model.layers[:-4]:
        layer.trainable = False
    for layer in model.layers[-4:]:
        layer.trainable = True


    #model.fit(images, labels, epochs=5, batch_size=32, validation_split=0.2)

    optimizer = Adam(learning_rate=0.001)
    model.compile(optimizer=optimizer, loss='mse', metrics=['mae'])

    model.fit(new_images, new_labels, epochs=30, batch_size=32, validation_split=0.2)

    #loss, mae = model.evaluate(validation_images, validation_labels)
    #print("Validation Loss:", loss, "Validation MAE:", mae)

    model.save('giro_updated_drone_path_following_model_normal_2_3.keras')
else:
    print("Ver tamano de txt")