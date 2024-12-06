
import os
import cv2
import glob
import numpy as np
import tensorflow as tf
import keras.api._v2.keras as keras
from keras.preprocessing.image import ImageDataGenerator
from keras.utils import to_categorical
from keras.applications import MobileNetV2
from keras.models import Model
from keras.layers import Dense, GlobalAveragePooling2D
from keras.callbacks import EarlyStopping

def load_images_and_labels(labels_file):
    pitch_roll_yaw = []
    with open(labels_file, 'r') as file:
        for line in file:
            arr_datos = line.strip().split(',')
            float_datos = [float(dato) for dato in arr_datos[:3]]
            pitch_roll_yaw.append(float_datos)

    roll = []
    pitch = []
    yaw = []
    for x in pitch_roll_yaw:
        roll.append(x[1])
        pitch.append(x[0])
        yaw.append(x[2])

    max_arr = []
    max_arr.append(max(list(map(abs, pitch))))
    max_arr.append(max(list(map(abs, roll))))
    max_arr.append(max(list(map(abs, yaw))))

    pitch_roll_yaw_normal = []
    
    for x in pitch_roll_yaw:
        res = []
        res.append(x[0]/max_arr[0])
        res.append(x[1]/max_arr[1])
        res.append(x[2]/max_arr[2])
        pitch_roll_yaw_normal.append(res)


    images = [cv2.imread(file, cv2.IMREAD_COLOR) for file in glob.glob('FotosSegundoFin/Todo/Fotos/*.jpg')]
    images_final = [cv2.resize(img, (224, 224)) for img in images if img is not None]
    
    images_final = np.array(images_final, dtype=np.float32)
    images_final /= 255.0  # Normalize to [0, 1]
    
    return images_final, np.array(pitch_roll_yaw_normal),max_arr


images, labels,max_arr = load_images_and_labels('FotosSegundoFin/Todo/datos_vuelo.txt')
if len(images) == len(labels):
   
    base_model = MobileNetV2(weights='imagenet', include_top=False, input_shape=(224, 224, 3))

    
    base_model.trainable = False

    
    x = base_model.output
    x = GlobalAveragePooling2D()(x)
    x = Dense(512, activation='relu')(x)  # Dense layer 1
    x = Dense(128, activation='relu')(x)  # Dense layer 2
    predictions = Dense(3, activation='linear')(x)  # Output layer for pitch, roll, yaw

    
    model = Model(inputs=base_model.input, outputs=predictions)

    model.compile(optimizer='adam', loss='mse', metrics=['mae'])

    #early_stopping = EarlyStopping(monitor='val_loss', patience=5, restore_best_weights=True)
    #model.fit(images, labels, epochs=100, batch_size=64,validation_split=0.2, callbacks=[early_stopping])
    model.fit(images, labels, epochs=30, batch_size=32,validation_split=0.2)
    model.save('giro_todo_drone_path_following_model_normal.keras')

    loss, mae = model.evaluate(images, labels)
    print('Loss:', loss, 'MAE:', mae)

    print(max_arr)
else: 
    print("Error tamano")