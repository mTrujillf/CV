

#from djitellopy import Tello
import analisisImagenMenosVision as aI
from time import sleep
import cv2
#import detectorTeclas as dt
import time
import numpy as np
import os
#Método para detectar la entrada del teclado y detener el drone
def getKeyboradInput():
    if dt.getKey("k") :
        cv2.destroyAllWindows()
        tello.land()
        tello.streamoff()
        exit(1)
    
    if dt.getKey("i"):
        cv2.imshow("Img", matInicio)
        cv2.waitKey(3)
        return False
    
    return True
#Se refiere a la función que calcula el máximo de matrices. Si el valor de la 
#matriz de adelante es menor a 400, se revisa cuál de las matrices laterales es 
#la máxima y si es mayor a 200
#tope = [400,200]#caso 3x3
tope = [450,150]
todosRC = list()
dir_final= r'D:\Python\nuevaIdea\fotosVuelo2'
jpg = ".jpg"
#Control RC dependiendo de qué matriz fue máxima.
matRc = [[[0,15,7],[0,25,7],[0,33,7]],
         [[0,40,5],[0,45,5]],
         [[0,55,0],[0,60,0]],
         [[-10,67,0],[-10,74,0]],
         [[0,-15,-7],[0,-25,-7],[0,-33,-7]],
         [[0,-40,-5],[0,-45,-5]],
         [[0,-55,0],[0,-60,0]],
         [[-10,-67,0],[-10,-74,0]]]

#Crea el objeto del drone.
tello = Tello()
 
#Inicio de detección de teclado.                
dt.init()

#Inicio de instrucciones para el drone.
tello.connect()
tello.send_rc_control(0, 0, 0, 0)
tello.streamoff()
tello.streamon()
cont = 0
try:
    print(tello.get_battery())
    tello.takeoff()
    tello.send_command_with_return("downvision 0")
    tello.send_command_with_return("downvision 1")
    tello.set_video_fps(Tello.FPS_5)
    tello.set_video_fps(Tello.FPS_30)
    img = tello.get_frame_read().frame
    tello.move_up(20)
    tello.move_down(25)
    #sleep(2)
    sigue =True 
    
    matInicio = np.ones((240,320))

    while sigue:
        sigue = getKeyboradInput()
        img = tello.get_frame_read().frame
        bw = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        BW = aI.blackwhite(bw)
        der = sum(sum(BW[0:119,0:80]))
        izq = sum(sum(BW[120:239,0:90]))
        x = (der - izq)
        x = int(x/150)
        #print(x)
        tello.send_rc_control(x,-5, 0, 0)
        if abs(x) < 5 :
            sigue = False
        cv2.imshow("Img", img)
        cv2.waitKey(1)
     
        
    getKeyboradInput()#Detecta si se apretó alguna tecla para detener.
        
    tello.send_rc_control(0, 0, 0, 0)
    #arr = [0,0,0,0]
    #todosRC.append(arr)
    #Lectura y visualización de la imagen.
    img = tello.get_frame_read().frame
    img = img[39:199,59:299]
    bw = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    BW = aI.blackwhite(bw)
    cv2.imshow("Img", BW)
    cv2.waitKey(1)
        
    #Método que calcula el máximo de las matrices.
    i,k = aI.getTodo(BW,tope) 
    #start = time.time_ns()
    #os.chdir(dir_final) 
    #os.listdir(dir_final)
    
    while True:
        getKeyboradInput()#Detecta si se apretó alguna tecla para detener.
    
        while i == 8:#caso 4x3
            
            getKeyboradInput()
            img = tello.get_frame_read().frame
            #img = img[39:199,39:279]
            img = img[29:209,40:279]
            bw = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            BW = aI.blackwhite(bw)
            
            der = BW[49:89,0:70]
            izq = BW[90:130,0:70]
            centro = img[60:119,0:59]
            #Diferencia entre matrices para centrar la imagen.
            x = aI.centrar(BW)
        
            #yaw = int(x * 1.5 * signo)#Para subir un poco más la rotación de centrado.
            #if(abs(x[0])<10):
            tello.send_rc_control(x[0], 15, 0, x[1])
            arr = [x[0],15,0,[1]]
            #else:
            #    tello.send_rc_control(x[0], 15, 0, 0)
            #    arr = [x[0],15,0,0]
            #print(arr)
            todosRC.append(arr)
            #Mostrar imagen del drone.
            cv2.imshow("Img", BW)
            cv2.imshow("Der",der)
            cv2.imshow("Izq", izq)
            cv2.imshow("centro", centro)
            cv2.moveWindow("Der", 50, 400)
            cv2.moveWindow("Izq", 50, 520)
            cv2.moveWindow("centro", 50, 300)
            cv2.waitKey(1)
            i,k = aI.getTodo(BW,tope)
            #filename = "_________" + str(time.time()) + "________________"+ jpg
            #cv2.imwrite(filename, img)
    #        start = time.time_ns()
    #        cont = cont + 1
        
    #    while i < 6:#caso 3x3
        while i < 8:
    
            getKeyboradInput()
            #Dependiendo de qué matriz salió máxima, se guardan los valores de RC.
            arrRc = matRc[i] 
            rc = arrRc[k]
            tello.send_rc_control(0, rc[0], 0, rc[1])#Mandar los valores de RC al drone.
            arr = [rc[2],rc[0],0,rc[1]]
            #print(arr)
            todosRC.append(arr)
            #Nueva imagen
            img = tello.get_frame_read().frame
            img = img[29:209,40:279]
            bw = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            BW = aI.blackwhite(bw)
        
            cv2.imshow("Img", BW)
            cv2.waitKey(1)
            i,k = aI.getTodo(BW,tope)
            #filename = "_________" + str(time.time()) + "________________"+ jpg
            #cv2.imwrite(filename, img)
    #        start = time.time_ns()
    #        cont = cont + 1
except:
    tello.land()