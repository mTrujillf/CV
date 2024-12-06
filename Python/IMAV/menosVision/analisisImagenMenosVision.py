
import numpy as np
import cv2

def blackwhite(img):
  
    channel3Min = 50.1
    #107.1
    channel3Max = 255.000
    
    BW = (img[:, :] >= channel3Min) & (img[:, :] <= channel3Max)

    BW = BW.astype(float)
    return BW    

def centrar(img):
    res = [0,0]
    der = sum(sum(img[49:89,25:95]))
    izq = sum(sum(img[90:130,25:95]))
    
    der1 = sum(sum(img[59:89,45:74]))
    izq1 = sum(sum(img[90:120,45:74]))
    der2 = sum(sum(img[59:89,75:105])) 
    izq2 = sum(sum(img[90:120,75:105]))
    
    x = der - izq
    
    x1 = der1 - izq1
    x2 = der2 - izq2

    if abs(x1) > abs(x2):
        yaw = x1
    else:
        yaw = -x2
        
    res [0] = int( x/160)
    res[1] = int (yaw/50)
    #160
    #60
    return res
 
def getTodo(img,tope):
    i = 8
    k = 0
    
    #Pantallas central
    mat5 = img[60:119,0:59]
    
    sumMat5 = sum(sum(mat5))
    
    if sumMat5 < tope[0]:
        
        #Pantalla Der
        mat1 = img[0:59,0:59]
        mat2 = img[0:59,60:119]
        mat3 = img[0:59,120:179]
        mat4 = img[0:59,180:239]
        
        
        #Pantallas Izq
        mat6 = img[120:179,0:59]
        mat7 = img[120:179,60:119]
        mat8 = img[120:179,120:179]
        mat9 = img[120:179,180:239]
        
        sumMat1 = sum(sum(mat1))
        sumMat2 = sum(sum(mat2))
        sumMat3 = sum(sum(mat3))
        sumMat4 = sum(sum(mat4))
        
        sumMat6 = sum(sum(mat6))
        sumMat7 = sum(sum(mat7))
        sumMat8 = sum(sum(mat8))
        sumMat9 = sum(sum(mat9))
        
        todasSumMat = [sumMat1,sumMat2,sumMat3,sumMat4
                       ,sumMat6,sumMat7,sumMat8,sumMat9]
    
        sumMax = max(todasSumMat)

        j = np.array(todasSumMat).argmax()
        
        if sumMax >tope[1]:
            todasMat = [mat1,mat2,mat3,mat4,mat6,mat7,mat8,mat9]
            maxMat = todasMat[j]
            
            cv2.imshow("MaxMat", maxMat)
            cv2.moveWindow("MaxMat", 50, 700)
            
            if j == 0:
                ventana1 = maxMat[30:59,0:29]
                ventana2 = maxMat[0:29,0:29]
                ventana3 = maxMat[0:20,30:59]
            
                sumVentana1 = sum(sum(ventana1))
                sumVentana2 = sum(sum(ventana2))
                sumVentana3 = sum(sum(ventana3))
            
                subVentanas = [sumVentana1,sumVentana2,sumVentana3]
                k = np.array(subVentanas).argmax()
                
            else:
                if j == 4:
                    ventana1 = maxMat[0:29,0:29]
                    ventana2 = maxMat[30:59,0:29]
                    ventana3 = maxMat[30:59,30:59]
            
                    sumVentana1 = sum(sum(ventana1))
                    sumVentana2 = sum(sum(ventana2))
                    sumVentana3 = sum(sum(ventana3))
            
                    subVentanas = [sumVentana1,sumVentana2,sumVentana3]
                    k = np.array(subVentanas).argmax()
                else:
                    if j < 4:
                    
                        ventana1 = maxMat[29:59,0:29]
                        ventana2 = maxMat[29:59,30:59]
            
                        sumVentana1 = sum(sum(ventana1))
                        sumVentana2 = sum(sum(ventana2))
            
                        subVentanas = [sumVentana1,sumVentana2]
                        k = np.array(subVentanas).argmax()
                    else:
                        ventana1 = maxMat[0:29,0:29]
                        ventana2 = maxMat[0:29,30:59]
            
                        sumVentana1 = sum(sum(ventana1))
                        sumVentana2 = sum(sum(ventana2))
            
                        subVentanas = [sumVentana1,sumVentana2]
                        k = np.array(subVentanas).argmax()
                        
            i = j 
        else:
            i = 9
    return i, k
    





















