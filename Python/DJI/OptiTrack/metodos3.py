# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 08:44:43 2024

@author: Drones01
"""
import numpy as np
import math

def path_following (t,yaw,x,y,z,dwh,desfase,posLider,dist):
    #waypoints
    x1d = 1.0
    y1d = -1.0
    
    x2d = 1.0
    y2d = 1.0
    
    x3d = -1.0
    y3d = 1.0
    
    x4d = -1.0
    y4d = -1.0
    
    #seleccion del waypoint
    wp = dwh[4]
    
    if wp == 4:
        wp = 0
        
    if wp == 0:
        xdi = x1d
        ydi = y1d
    
        xdi1 = x2d
        ydi1 = y2d
    elif wp == 1:
        xdi = x2d
        ydi = y2d
    
        xdi1 = x3d
        ydi1 = y3d
    elif wp == 2:
        xdi = x3d
        ydi = y3d
        
        xdi1 = x4d
        ydi1 = y4d
    elif wp == 3:
        xdi = x4d
        ydi = y4d
        
        xdi1 = x1d
        ydi1 = y1d
        
    #Forma Vectorial
    Xdi = np.array([xdi,ydi])
    
    #Derivada de el deseado (cero para waypoints)
    xdi_dot = 0.0
    ydi_dot = 0.0
    
    #distancia del dron al waypoint
    d = math.sqrt((x-xdi1)*(x-xdi1) + (y-ydi1)*(y-ydi1))
    
    ds = 0.15
    
    if d <= ds:
        wp = wp + 1
    
    #Entrada de control de velocidad longitudinal
    u = 3
    
    #Angulo de orientacion de la linea
    psid = math.atan2(ydi1-ydi,xdi1-xdi)
    
    s_psid = math.sin(psid)
    c_psid = math.cos(psid)
    
    Rd = np.array([[c_psid,-s_psid],[s_psid,c_psid]])
    RdT = Rd.transpose()
    
    Rd_dot = np.array([[0,0],[0,0]]) #Si es una linea, la derivada del angulo de orientacion deseado es 0
    
    #Ganancias del sistema
    k = 0.4 #0.4
    kR = 210 #210 funciona bien
    k1 = 0 #- 0.15
    
    #Estados del sistema
    r = dwh[3]
    
    s_psi = math.sin(yaw)
    c_psi = math.cos(yaw)
    
    psi_dot = r
    
    R = np.array([[c_psi,-s_psi],[s_psi,c_psi]])
    
    X = np.array([x,y])
    
    V = np.array([u,0])
    
    X_dot = np.dot(R,V)
    
    x_dot = X_dot[0]
    y_dot = X_dot[1]
    
    
    Dr_hat = dwh[5]
    
    #Error lateral
    Xr = np.dot(RdT,X) #Posicion en el nuevo sistema de ejes
    Xrt = Xr - np.dot(RdT,Xdi) #Error en el nuevo sistema de ejes
    
    yt = Xrt[1] #Error lateral
    yt_dot = - x_dot*s_psid + y_dot*c_psid + xdi_dot*s_psid - ydi_dot*c_psid + psi_dot*(- x*c_psid - y*s_psid + xdi*c_psid + ydi*s_psid)
    
    #Matriz de correccion
    Rc = np.array([[math.sqrt(1-math.tanh(k*yt)*math.tanh(k*yt)),math.tanh(k*yt)],[-math.tanh(k*yt),math.sqrt(1-math.tanh(k*yt)*math.tanh(k*yt))]])
    
    r11_dot = -math.sqrt(1-math.tanh(k*yt)*math.tanh(k*yt))*math.tanh(k*yt)
    r12_dot = 1-math.tanh(k*yt)*math.tanh(k*yt)
    
    Rc_dot =k*yt_dot*np.array([[r11_dot,r12_dot],[-r12_dot,r11_dot]])
    
    #Matriz de rotacion (multiplicacion de matriz de orientacion deseada con matriz de correccion)
    Rr = np.dot(Rd,Rc)
    Rr_dot = np.dot(Rd_dot,Rc) + np.dot(Rd,Rc_dot)
    
    rd_wedge = np.dot(Rr.transpose(),Rr_dot)
    
    rd = rd_wedge[1][0]
    
    Rt = np.dot(Rr.transpose(),R)
    
    Pa = 0.5*(Rt - Rt.transpose())
    Pa_21 = Pa[1][0]
    
    beta1 = k1*yaw
    
    r = rd - kR*Pa_21 - 0*Dr_hat + beta1   
    
    v = 0.0
    
    #Control de altitud
    ka = 10

    zDes = -1.2
    #zDes = -1
    zDesdot = 0
    #zDesdot = 0
    ze   = z - zDes
    w = -ka * (ze + zDesdot)
    w = -w
    
    #Estimador de orientacion
    dDr_hat = Pa_21 #k1*(rd - kR*Pa_21)
    
    Dr_hat = Dr_hat + 1/40 *dDr_hat
    
    dwh = [u,v,w,r,wp,Dr_hat,0,0,0]
    
    return int(v),int(u),int(w),int(r),Pa_21,yt,zDes,psid,dwh,posLider,0

def circulo_con_rotacion_1 (t,yaw,x,y,z,dwh,desfase,posLider,dist):
    #Se lee el contador
    cont = dwh[6]
    xm = dwh[7]
    ym = dwh[8]
    
    if cont == 0:
        xi = x
        yi = y
    else:
        xi = xm
        yi = ym
    
    #Circulo de un metro
    a = 0.75
        
    f = 1/45
    w1 = 2 * math.pi * f
    
    #Meterlas como matriz diagonal de ganancias
    k1 = -np.array([[0.65,0],[0,0.55]]) #0.4
    k2 = -0.5
    
    kp = np.array([[9.5,0],[0,7.5]])
    #Esta ganancia de rotacion debe ser muy baja
    kr = 220
    
    
    #Valores de la tangente
    at = 16
    bt = 0.25
    
    #Posiciones deseadas
    xd = a * math.sin(w1*t) #-0.75
    yd = a * math.cos(w1*t)
    
    #Posicion en forma matricial
    X = np.array([x,y])
    
    #Valores deseados en forma matricial
    XDes = np.array([xd,yd])
    dXDes = np.array([-a*w1*math.cos(w1*t),-a*w1*math.sin(w1*t)])
    
    #Errores
    Xt = X - XDes
    
    #Campos vectoriales de repulsion
    dist12 = dist[0]
    dist13 = dist[1]
    
    distSeg = dist[3]
    
    gan = 20
    
    #posLider[1][0] = 0
    #posLider[1][1] = 0
    #posLider[1][2] = -0.9
    
    bDrone12U = 0*gan * ((posLider[0][0] - posLider[1][0]) - (posLider[0][1] - posLider[1][1]) - (posLider[0][2] - posLider[1][2]))
    bDrone12V = 0*gan * ((posLider[0][0] - posLider[1][0]) + (posLider[0][1] - posLider[1][1]) - (posLider[0][2] - posLider[1][2]))
    bDrone12W = 0*gan * ((posLider[0][0] - posLider[1][0]) + (posLider[0][1] - posLider[1][1]) + (posLider[0][2] - posLider[1][2]))
    
    bDrone13U = 0*gan * ((posLider[0][0] - posLider[2][0]) - (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
    bDrone13V = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
    bDrone13W = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) + (posLider[0][2] - posLider[2][2]))
    
    multSeg12 = -(math.tanh(100*(dist12-0.73))-1)/2
    multSeg13 = 0
    
    if(dist13 < distSeg):
        multSeg13 = 1
    
    #print(dist12)
    
    #if dist12 < distSeg or dist13 < distSeg:
    #    psid = multSeg12*math.atan2(bDrone12V,bDrone12U) + multSeg13*math.atan2(bDrone13V,bDrone13U)
    
    
    #Orientacion deseada
    psid = math.atan2(yd,xd+0.75)+math.pi/2
    
    psid_dot = (dXDes[1]*xd - dXDes[0]*yd)/(xd*xd + yd*yd)
    
    #Matriz de orientacion actual
    R = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    
    #Matriz de orientacion deseada
    Rd = np.array([[math.cos(psid),-math.sin(psid)],[math.sin(psid),math.cos(psid)]])
    
    #Error de orientacion
    Rt = np.dot(Rd.transpose(),R)
    
    #Parte antisimetrica
    Pa = 0.5*(Rt-Rt.transpose())
    
    
    #Sacamos los estimadores del buffer
    hat_deltaxy = np.array([dwh[0],dwh[1]])
    hat_deltar = dwh[2]
    
    #Estimadores
    Xalt = np.array([math.tanh(Xt[0]*6),math.tanh(Xt[1]*6)])
    dothat_deltaxy = Xt - np.dot(kp,np.dot(k1,Xalt)) + np.dot(k1,multSeg12*np.array([bDrone12U,bDrone12V]))
    
    dothat_deltar = k2*(psid_dot - kr*Pa[1,0])
    
    
    beta1 = np.dot(k1,X)
    beta2 = k2*yaw
    
    
    U = np.dot(R.transpose(),-np.dot(kp,Xalt) + dXDes - hat_deltaxy + beta1 + multSeg12*np.array([bDrone12U,bDrone12V]))
    
    r = - hat_deltar + beta2 - kr*Pa[1][0] + psid_dot
    u = U[0]
    v = U[1]
    
    deltaxy = hat_deltaxy - beta1
    deltar = hat_deltar - beta2
    
    #Control de altitud
    a2 = 0.2
    f2 = 0.032
    w2 = 2 * math.pi * f2
    ka = 10

    zDes = -1.2
    #zDes = -1
    zDesdot = 0
    #zDesdot = 0
    ze   = z - zDes
    w = -ka * (ze + zDesdot) + 2.5*multSeg12*bDrone12W
    w = -w
    
    #Integraciones
    hat_deltaxy = hat_deltaxy + 1/120 * dothat_deltaxy
    hat_deltar = hat_deltar + 1/120 * dothat_deltar
    
    #if dothat_deltaxy[0] > 0.2:
     #   dothat_deltaxy[0] = 0.2
        
    #if dothat_deltaxy[0] < -0.2:
     #   dothat_deltaxy[0] = -0.2
    
    #if dothat_deltaxy[1] > 0.2:
     #   dothat_deltaxy[1] = 0.2
        
    #if dothat_deltaxy[1] < -0.2:
      #  dothat_deltaxy[1] = -0.2
    
    dwh = [hat_deltaxy[0],hat_deltaxy[1],hat_deltar,dothat_deltaxy[0],deltaxy[1],deltar,cont+1,xi,yi]
    
    return int(v),int(u),int(w),int(r),xd,yd,zDes,psid,dwh,posLider,0

def circulo_con_rotacion_2 (t,yaw,x,y,z,dwh,desfase,posLider,dist):
    #Se lee el contador
    cont = dwh[6]
    xm = dwh[7]
    ym = dwh[8]
    
    if cont == 0:
        xi = x
        yi = y
    else:
        xi = xm
        yi = ym
    
    #Circulo de un metro
    a = 0.75
        
    f = 1/45
    w1 = 2 * math.pi * f
    
    #Meterlas como matriz diagonal de ganancias
    k1 = -np.array([[0.65,0],[0,0.55]]) #0.4
    k2 = -0.5
    
    kp = np.array([[9.5,0],[0,7.5]])
    #Esta ganancia de rotacion debe ser muy baja
    kr = 220
    
    
    #Valores de la tangente
    at = 16
    bt = 0.25
    
    #Posiciones deseadas
    xd = 0.75 - a * math.sin(w1*t + math.pi)
    yd = a * math.cos(w1*t + math.pi)
    
    #Posicion en forma matricial
    X = np.array([x,y])
    
    #Valores deseados en forma matricial
    XDes = np.array([xd,yd])
    dXDes = np.array([-a*w1*math.cos(w1*t),-a*w1*math.sin(w1*t)])
    
    #Errores
    Xt = X - XDes
    
    #Campos vectoriales de repulsion
    dist12 = dist[0]
    dist13 = dist[1]
    
    distSeg = dist[3]
    
    gan = 20
    
    #posLider[1][0] = 0
    #posLider[1][1] = 0
    #posLider[1][2] = -0.9
    
    bDrone21U = gan * ((posLider[1][0] - posLider[0][0]) - (posLider[1][1] - posLider[0][1]) - (posLider[1][2] - posLider[0][2]))
    bDrone21V = gan * ((posLider[1][0] - posLider[0][0]) + (posLider[1][1] - posLider[0][1]) - (posLider[1][2] - posLider[0][2]))
    bDrone21W = gan * ((posLider[1][0] - posLider[0][0]) + (posLider[1][1] - posLider[0][1]) + (posLider[1][2] - posLider[0][2]))
    
    bDrone13U = 0*gan * ((posLider[0][0] - posLider[2][0]) - (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
    bDrone13V = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
    bDrone13W = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) + (posLider[0][2] - posLider[2][2]))
    
    multSeg12 = -(math.tanh(100*(dist12-0.73))-1)/2
    multSeg13 = 0
    
    if(dist13 < distSeg):
        multSeg13 = 1
    
    #print(dist12)
    
    #if dist12 < distSeg or dist13 < distSeg:
     #   psid = multSeg12*math.atan2(bDrone21V,bDrone21U) + multSeg13*math.atan2(bDrone13V,bDrone13U)
    
    
    #Orientacion deseada
    psid = math.atan2(yd,xd-0.75) + math.pi/2
    
    psid_dot = (dXDes[1]*xd - dXDes[0]*yd)/(xd*xd + yd*yd)
    
    #Matriz de orientacion actual
    R = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    
    #Matriz de orientacion deseada
    Rd = np.array([[math.cos(psid),-math.sin(psid)],[math.sin(psid),math.cos(psid)]])
    
    #Error de orientacion
    Rt = np.dot(Rd.transpose(),R)
    
    #Parte antisimetrica
    Pa = 0.5*(Rt-Rt.transpose())
    
    
    #Sacamos los estimadores del buffer
    hat_deltaxy = np.array([dwh[0],dwh[1]])
    hat_deltar = dwh[2]
    
    #Estimadores
    Xalt = np.array([math.tanh(Xt[0]*6),math.tanh(Xt[1]*6)])
    dothat_deltaxy = Xt - np.dot(kp,np.dot(k1,Xalt)) + np.dot(k1,multSeg12*np.array([bDrone21U,bDrone21V]))
    
    dothat_deltar = k2*(psid_dot - kr*Pa[1,0])
    
    
    beta1 = np.dot(k1,X)
    beta2 = k2*yaw
    
    
    U = np.dot(R.transpose(),-np.dot(kp,Xalt) + dXDes - hat_deltaxy + beta1 + multSeg12*np.array([bDrone21U,bDrone21V]))
    
    r = - hat_deltar + beta2 - kr*Pa[1][0] + psid_dot
    u = U[0]
    v = U[1]
    
    deltaxy = hat_deltaxy - beta1
    deltar = hat_deltar - beta2
    
    #Control de altitud
    a2 = 0.2
    f2 = 0.032
    w2 = 2 * math.pi * f2
    ka = 10

    zDes = -1.2
    #zDes = -1
    zDesdot = 0
    #zDesdot = 0
    ze   = z - zDes
    w = -ka * (ze + zDesdot) + 2.5*multSeg12*bDrone21W
    w = -w
    
    #Integraciones
    hat_deltaxy = hat_deltaxy + 1/120 * dothat_deltaxy
    hat_deltar = hat_deltar + 1/120 * dothat_deltar
    
    #if dothat_deltaxy[0] > 0.2:
     #   dothat_deltaxy[0] = 0.2
        
    #if dothat_deltaxy[0] < -0.2:
     #   dothat_deltaxy[0] = -0.2
    
    #if dothat_deltaxy[1] > 0.2:
     #   dothat_deltaxy[1] = 0.2
        
    #if dothat_deltaxy[1] < -0.2:
      #  dothat_deltaxy[1] = -0.2
    
    dwh = [hat_deltaxy[0],hat_deltaxy[1],hat_deltar,dothat_deltaxy[0],deltaxy[1],deltar,cont+1,xi,yi]
    
    return int(v),int(u),int(w),int(r),xd,yd,zDes,psid,dwh,posLider,0

def lemniscata (t,yaw,x,y,z,dwh,desfase,posLider,dist):
    #Se lee el contador
    cont = dwh[6]
    xm = dwh[7]
    ym = dwh[8]
    
    if cont == 0:
        xi = x
        yi = y
    else:
        xi = xm
        yi = ym
    
    #Circulo de un metro
    a = 1.0
    d = 1.0    
    
    f = 1/60
    w1 = 2 * math.pi * f
    
    #Meterlas como matriz diagonal de ganancias
    k1 = -np.array([[0.65,0],[0,0.55]]) #0.4
    k2 = -0.5
    
    kp = np.array([[5.5,0],[0,3.5]])
    #Esta ganancia de rotacion debe ser muy baja
    kr = 110
    
    
    #Valores de la tangente
    at = 16
    bt = 0.25
    
    desfase = 0
    
    #Posiciones deseadas
    xd = d * math.sqrt(2) * ((math.sin((w1*t) + desfase))/(1+(math.cos((w1*t) + desfase))*math.cos((w1*t) + desfase)))
    yd = d * math.sqrt(2) * ((math.sin((w1*t) + desfase) * math.cos((w1*t) + desfase))/(1+(math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase))))
    
    xd_dot = math.sqrt(2) * d *((w1 * math.cos((w1*t) + desfase) * (1+(math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase))))+2*w1 * math.cos((w1*t) + desfase)*math.sin((w1*t) + desfase)*math.sin((w1*t) + desfase))/((1+math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase))*(1+math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase)))
    yd_dot = math.sqrt(2) * d *(((math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase)-math.sin((w1*t) + desfase)*math.sin((w1*t) + desfase)) * (1+(math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase)))) + 2 * w1 * math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase)*math.sin((w1*t) + desfase)*math.sin((w1*t) + desfase))/((1+math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase))*(1+math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase)))
    #Posicion en forma matricial
    X = np.array([x,y])
    
    #Valores deseados en forma matricial
    XDes = np.array([xd,yd])
    dXDes = np.array([xd_dot,yd_dot])
    
    #Errores
    Xt = X - XDes
    
    #Campos vectoriales de repulsion
    dist12 = dist[0]
    dist13 = dist[1]
    
    distSeg = dist[3]
    
    gan = 20
    
    #posLider[1][0] = 0
    #posLider[1][1] = 0
    #posLider[1][2] = -0.9
    
    bDrone12U = gan * ((posLider[0][0] - posLider[1][0]) - (posLider[0][1] - posLider[1][1]) - (posLider[0][2] - posLider[1][2]))
    bDrone12V = gan * ((posLider[0][0] - posLider[1][0]) + (posLider[0][1] - posLider[1][1]) - (posLider[0][2] - posLider[1][2]))
    bDrone12W = gan * ((posLider[0][0] - posLider[1][0]) + (posLider[0][1] - posLider[1][1]) + (posLider[0][2] - posLider[1][2]))
    
    bDrone13U = 0*gan * ((posLider[0][0] - posLider[2][0]) - (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
    bDrone13V = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
    bDrone13W = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) + (posLider[0][2] - posLider[2][2]))
    
    multSeg12 = 0
    multSeg13 = 0
    if(dist12 < distSeg):
        multSeg12 = 1
    
    if(dist13 < distSeg):
        multSeg13 = 1
    
    print(dist12)
    
    if dist12 < distSeg or dist13 < distSeg:
        psid = multSeg12*math.atan2(bDrone12V,bDrone12U) + multSeg13*math.atan2(bDrone13V,bDrone13U)
    
    
    #Orientacion deseada
    z12 = -4*xd*((xd*xd)+(yd*yd)-(d*d))
    z22 = 4*yd*((xd*xd)+(yd*yd)+(d*d))
    
    
    if xd > 0:
        psid = math.atan2(z12,z22)
    else:
        psid = math.atan2(-z12, -z22) 
    
    z12dot = -4 * xd_dot *((xd*xd)+(yd*yd)+(d*d)) - 4 * xd * (2 * xd * xd_dot + 2 * yd * yd_dot)
    z22dot =  4 * yd_dot *((xd*xd)+(yd*yd)+(d*d)) + 4 * yd * (2 * xd * xd_dot + 2 * yd * yd_dot)
    
    psid_dot = (z12dot * z22 - z12 * z22dot)/(z22 * z22 + z12 * z12)
    
    #Matriz de orientacion actual
    R = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    
    #Matriz de orientacion deseada
    Rd = np.array([[math.cos(psid),-math.sin(psid)],[math.sin(psid),math.cos(psid)]])
    
    #Error de orientacion
    Rt = np.dot(Rd.transpose(),R)
    
    #Parte antisimetrica
    Pa = 0.5*(Rt-Rt.transpose())
    
    
    #Sacamos los estimadores del buffer
    hat_deltaxy = np.array([dwh[0],dwh[1]])
    hat_deltar = dwh[2]
    
    #Estimadores
    Xalt = np.array([math.tanh(Xt[0]*6),math.tanh(Xt[1]*6)])
    dothat_deltaxy = Xt - np.dot(kp,np.dot(k1,Xalt)) + np.dot(k1,multSeg12*np.array([bDrone12U,bDrone12V]))
    
    dothat_deltar = k2*(psid_dot - kr*Pa[1,0])
    
    
    beta1 = np.dot(k1,X)
    beta2 = k2*yaw
    
    
    U = np.dot(R.transpose(),-np.dot(kp,Xalt) + dXDes - hat_deltaxy + beta1 + multSeg12*np.array([bDrone12U,bDrone12V]))
    
    r = - hat_deltar + beta2 - kr*Pa[1][0] + psid_dot
    u = U[0]
    v = U[1]
    
    deltaxy = hat_deltaxy - beta1
    deltar = hat_deltar - beta2
    
    #Control de altitud
    a2 = 0.2
    f2 = 0.032
    w2 = 2 * math.pi * f2
    ka = 10

    zDes = -1.2
    #zDes = -1
    zDesdot = 0
    #zDesdot = 0
    ze   = z - zDes
    w = -ka * (ze + zDesdot) + 2.5*multSeg12*bDrone12W
    w = -w
    
    #Integraciones
    hat_deltaxy = hat_deltaxy + 0.05 * dothat_deltaxy
    hat_deltar = hat_deltar + 0.05 * dothat_deltar
    
    #if dothat_deltaxy[0] > 0.2:
     #   dothat_deltaxy[0] = 0.2
        
    #if dothat_deltaxy[0] < -0.2:
     #   dothat_deltaxy[0] = -0.2
    
    #if dothat_deltaxy[1] > 0.2:
     #   dothat_deltaxy[1] = 0.2
        
    #if dothat_deltaxy[1] < -0.2:
      #  dothat_deltaxy[1] = -0.2
    
    dwh = [hat_deltaxy[0],hat_deltaxy[1],hat_deltar,dothat_deltaxy[0],deltaxy[1],deltar,cont+1,xi,yi]
    
    return int(v),int(u),int(w),int(r),xd,yd,zDes,psid,dwh,posLider,0


def infinito (t,yaw,x,y,z,dwh,desfase,posLider,dist):
    
    a = 1.0
    f = 0.032
    w1 = 2 * math.pi * f
    kp = 30.0
    kr = 110.0
    k1 = 0.05    
    f2 = 0.025
    w3 = 2 * math.pi * f2
    
    desfase = 0
    
    d = 1
    
    hat_deltaxy = np.array([dwh[0],dwh[1]])
    hat_deltaz = dwh[2]
    
    xDes = d * math.sqrt(2) * ((math.sin((w1*t) + desfase))/(1+(math.cos((w1*t) + desfase))*math.cos((w1*t) + desfase)))
    
    xe = x - xDes
    xDesdot = math.sqrt(2) * d *((w1 * math.cos((w1*t) + desfase) * (1+(math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase))))+2*w1 * math.cos((w1*t) + desfase)*math.sin((w1*t) + desfase)*math.sin((w1*t) + desfase))/((1+math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase))*(1+math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase)))
    
    
    yDes = d * math.sqrt(2) * ((math.sin((w1*t) + desfase) * math.cos((w1*t) + desfase))/(1+(math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase))))
    ye = y - yDes
    yDesdot = math.sqrt(2) * d *(((math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase)-math.sin((w1*t) + desfase)*math.sin((w1*t) + desfase)) * (1+(math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase)))) + 2 * w1 * math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase)*math.sin((w1*t) + desfase)*math.sin((w1*t) + desfase))/((1+math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase))*(1+math.cos((w1*t) + desfase)*math.cos((w1*t) + desfase)))
    
    
    z12 = -4*xDes*((xDes*xDes)+(yDes*yDes)-(d*d))
    z22 = 4*yDes*((xDes*xDes)+(yDes*yDes)+(d*d))
    
    
    if xDes > 0:
        
        psiDes = math.atan2(z12,z22)
    else:
        psiDes = math.atan2(-z12, -z22) 
    #else:
    #psiDes = math.atan2(z12, z22)
    
    z12dot = -4 * xDesdot *((xDes*xDes)+(yDes*yDes)+(d*d)) - 4 * xDes * (2 * xDes * xDesdot + 2 * yDes * yDesdot)
    z22dot =  4 * yDesdot *((xDes*xDes)+(yDes*yDes)+(d*d)) + 4 * yDes * (2 * xDes * xDesdot + 2 * yDes * yDesdot)
    
    psiDesdot = (z12dot * z22 - z12 * z22dot)/(z22 * z22 + z12 * z12)
    #psiDesdot = 0
    xMatrix = np.array([x,y])
    xDesMatrix = np.array([xDes,yDes])
    
    Rpsi = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    RpsiDes = np.array([[math.cos(psiDes),-math.sin(psiDes)],[math.sin(psiDes),math.cos(psiDes)]])
    RpsiT = Rpsi.transpose()
    
    xeMatrix = xMatrix - xDesMatrix
    
    xDesdotMatrix = np.array([xDesdot,yDesdot])
    
    deltaxy = np.array([hat_deltaxy[0]+k1*x,hat_deltaxy[1]+k1*y])
    
    vMatrix = np.dot(RpsiT ,(xDesdotMatrix - (kp*xeMatrix) - deltaxy))
    
    
    RpsiDesdot = np.array([[(-math.sin(psiDes)*psiDesdot),(-math.cos(psiDes)*psiDesdot)],
                  [(math.cos(psiDes)*psiDesdot),(-math.sin(psiDes)*psiDesdot)]])
    
    RpsiDesT = RpsiDes.transpose()
    
    RpsiDesRpsiDesdot = np.dot(RpsiDesT,RpsiDesdot)
    
    
    Rgorro = np.dot(RpsiDesT,Rpsi)
    RgorroT = Rgorro.transpose()
    Pa = (1/2) *  (Rgorro - RgorroT)
    paElemento = Pa[1][0]
    
    rd = RpsiDesRpsiDesdot[1][0]
    
    
    r = rd - kr * paElemento
    u = int(vMatrix[0])
    v = int(vMatrix[1])
    
    gamma = 0
    
    a2 = 0.2
    f2 = 0.032
    w2 = 2 * math.pi * f2
    ka = 30
    
    zDes = -1.2 + a2 * math.sin(w2*t)
    zDesdot = a2*w2*math.cos(w2*t)
    ze   = z - zDes
    w = -ka * (ze + zDesdot)
    w = -w
    
    U = np.array([u,v])
    barX = np.array([x,y])
    dothat_deltaxy = -k1*(np.dot(Rpsi,U) + hat_deltaxy + k1*barX)
    hat_deltaxy = hat_deltaxy + 0.05 * dothat_deltaxy
    
    dothat_deltaz = gamma*(w + hat_deltaz - gamma*z)
    hat_deltaz = hat_deltaz + 0.05 * dothat_deltaz
    
    dwh = [hat_deltaxy[0],hat_deltaxy[1],hat_deltaz]
    
    return int(v),int(u),int(w),int(r),xDes,yDes,zDes,psiDes,dwh,posLider,0

def pos_estatica_drone1 (t,yaw,x,y,z,dwh,desfase,posLider,dist):
    
    #Se lee el contador
    cont = dwh[6]
    xm = dwh[7]
    ym = dwh[8]
    
    if cont == 0:
        xi = x
        yi = y
    else:
        xi = xm
        yi = ym
    
    
    #Meterlas como matriz diagonal de ganancias
    k1 = -np.array([[0.4,0],[0,0.4]]) #0.4
    k2 = -0.015
    
    kp = 2.5
    #Esta ganancia de rotacion debe ser muy baja
    kr = 1
    
    #Puntos finales deseados
    xf = 0.75
    yf = 0
    
    #Distancia de la trayectoria
    xdis = xf-xi
    ydis = yf-yi
    
    #Valores de la tangente
    at = 16
    bt = 0.25
    
    #Posiciones deseadas
    xd = 0.75 #xdis*(math.tanh(bt*(t-at))-1)/2 + xf
    yd = 0 #ydis*(math.tanh(bt*(t-at))-1)/2 + yf
    
    #Posicion en forma matricial
    X = np.array([x,y])
    
    #Valores deseados en forma matricial
    XDes = np.array([xd,yd])
    dXDes = np.array([0,0]) #np.array([xdis*(1-math.tanh(bt*(t-at))*math.tanh(bt*(t-at)))*bt/2,ydis*(1-math.tanh(bt*(t-at))*math.tanh(bt*(t-at)))*bt/2])
    
    #Errores
    Xt = X - XDes
    
    #Campos vectoriales de repulsion
    dist12 = dist[0]
    dist13 = dist[1]
    
    distSeg = dist[3]
    
    gan = 20
    
    posLider[1][0] = 0
    posLider[1][1] = 0
    posLider[1][2] = -0.9
    
    bDrone12U = gan * ((posLider[0][0] - posLider[1][0]) - (posLider[0][1] - posLider[1][1]) - (posLider[0][2] - posLider[1][2]))
    bDrone12V = gan * ((posLider[0][0] - posLider[1][0]) + (posLider[0][1] - posLider[1][1]) - (posLider[0][2] - posLider[1][2]))
    bDrone12W = gan * ((posLider[0][0] - posLider[1][0]) + (posLider[0][1] - posLider[1][1]) + (posLider[0][2] - posLider[1][2]))
    
    bDrone13U = 0*gan * ((posLider[0][0] - posLider[2][0]) - (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
    bDrone13V = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
    bDrone13W = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) + (posLider[0][2] - posLider[2][2]))
    
    multSeg12 = 0
    multSeg13 = 0
    if(dist12 < distSeg):
        multSeg12 = 1
    
    if(dist13 < distSeg):
        multSeg13 = 1
    
    print(dist12)
    
    if dist12 < distSeg or dist13 < distSeg:
        psid = multSeg12*math.atan2(bDrone12V,bDrone12U) + multSeg13*math.atan2(bDrone13V,bDrone13U)
    
    
    #Orientacion deseada
    psid = 0 #math.atan2(yd-yi, xd-xi)
    
    #Matriz de orientacion actual
    R = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    
    #Matriz de orientacion deseada
    Rd = np.array([[math.cos(psid),-math.sin(psid)],[math.sin(psid),math.cos(psid)]])
    
    #Error de orientacion
    Rt = np.dot(Rd.transpose(),R)
    
    #Parte antisimetrica
    Pa = 0.5*(Rt-Rt.transpose())
    
    Dpsi = np.array([0,0])
    
    #Sacamos los estimadores del buffer
    hat_deltaxy = np.array([dwh[0],dwh[1]])
    hat_deltar = dwh[2]
    
    #Estimadores
    Xalt = np.array([math.tanh(Xt[0]*6),math.tanh(Xt[1]*6)])
    dothat_deltaxy = Xt - kp*np.dot(k1,Xalt) + np.dot(k1,multSeg12*np.array([bDrone12U,bDrone12V]))
    
    dothat_deltar = Pa[1][0] - k2*kp*np.dot(Dpsi,Xt) - kr*Pa[1,0]
    
    
    beta1 = np.dot(k1,X)
    beta2 = k2*yaw
    
    
    U = np.dot(R.transpose(),-kp*Xalt + dXDes - hat_deltaxy + beta1 + multSeg12*np.array([bDrone12U,bDrone12V]))
    
    r = - hat_deltar + beta2 - kp*np.dot(Dpsi,Xt) - kr*Pa[1][0]
    u = U[0]
    v = U[1]
    
    deltaxy = hat_deltaxy - beta1
    deltar = hat_deltar - beta2
    
    #Control de altitud
    a2 = 0.2
    f2 = 0.032
    w2 = 2 * math.pi * f2
    ka = 10

    zDes = -1.2
    #zDes = -1
    zDesdot = 0
    #zDesdot = 0
    ze   = z - zDes
    w = -ka * (ze + zDesdot) + 2.5*multSeg12*bDrone12W
    w = -w
    
    #Integraciones
    hat_deltaxy = hat_deltaxy + 0.05 * dothat_deltaxy
    hat_deltar = hat_deltar + 0.05 * dothat_deltar
    
    #if dothat_deltaxy[0] > 0.2:
     #   dothat_deltaxy[0] = 0.2
        
    #if dothat_deltaxy[0] < -0.2:
     #   dothat_deltaxy[0] = -0.2
    
    #if dothat_deltaxy[1] > 0.2:
     #   dothat_deltaxy[1] = 0.2
        
    #if dothat_deltaxy[1] < -0.2:
      #  dothat_deltaxy[1] = -0.2
    
    dwh = [hat_deltaxy[0],hat_deltaxy[1],hat_deltar,dothat_deltaxy[0],deltaxy[1],deltar,cont+1,xi,yi]
    
    return int(v),int(u),int(w),int(r),xd,yd,zDes,psid,dwh,posLider,0

def pos_estatica_drone2 (t,yaw,x,y,z,dwh,desfase,posLider,dist):
    
    a = 1.0
    f = 1/30
    w1 = 2 * math.pi * f
    #kp = 30.0
    kp = 4
    kr = 110.0
        
    f2 = 0.025
    w3 = 2 * math.pi * f2
    
    xDes = 1.2
    xe = x - xDes
    xDesdot = 0.0
    
    yDes = 1.2
    ye = y - yDes
    yDesdot = 0.0
    dist12 = dist[0]
    dist23 = dist[2]
    distSeg = dist[3]
    gan = 10
    
    bDrone21U = gan * ((posLider[1][0] - posLider[0][0]) - (posLider[1][1] - posLider[0][1]) - (posLider[1][2] - posLider[0][2]))
    bDrone21V = gan * ((posLider[1][0] - posLider[0][0]) + (posLider[1][1] - posLider[0][1]) - (posLider[1][2] - posLider[0][2]))
    bDrone21W = gan * ((posLider[1][0] - posLider[0][0]) + (posLider[1][1] - posLider[0][1]) + (posLider[1][2] - posLider[0][2]))
    
    bDrone23U = gan * ((posLider[1][0] - posLider[2][0]) - (posLider[1][1] - posLider[2][1]) - (posLider[1][2] - posLider[2][2]))
    bDrone23V = gan * ((posLider[1][0] - posLider[2][0]) + (posLider[1][1] - posLider[2][1]) - (posLider[1][2] - posLider[2][2]))
    bDrone23W = gan * ((posLider[1][0] - posLider[2][0]) + (posLider[1][1] - posLider[2][1]) + (posLider[1][2] - posLider[2][2]))
    
    multSeg12 = 0
    multSeg23 = 0
    if(dist12 < distSeg):
        multSeg12 = 1
    
    if(dist23 < distSeg):
        multSeg23 = 1
        
    psiDes = math.atan2(-ye,-xe)
    if dist12 < distSeg or dist23 < distSeg:
        psiDes = multSeg12*math.atan2(bDrone21V,bDrone21U) + multSeg23*math.atan2(bDrone23V,bDrone23U)
    
    xMatrix = np.array([x,y])
    xDesMatrix = np.array([xDes,yDes])
    
    Rpsi = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    #psiDes= math.atan2(yDes,xDes)-math.pi/2
    #psiDes = 0
    RpsiDes = np.array([[math.cos(psiDes),-math.sin(psiDes)],[math.sin(psiDes),math.cos(psiDes)]])
    RpsiT = Rpsi.transpose()
    
    xeMatrix = xMatrix - xDesMatrix
    
    xDesdotMatrix = np.array([xDesdot,yDesdot])
    
    vMatrix = np.dot(RpsiT ,(xDesdotMatrix - (kp*xeMatrix)))
    
    #psiDesdot = ((xDes*yDesdot) - (yDes*xDesdot))/(a*a)
    psiDesdot = 0
    
    RpsiDesdot = np.array([[(-math.sin(psiDes)*psiDesdot),(-math.cos(psiDes)*psiDesdot)],
                  [(math.cos(psiDes)*psiDesdot),(-math.sin(psiDes)*psiDesdot)]])
    
    RpsiDesT = RpsiDes.transpose()
    
    RpsiDesRpsiDesdot = np.dot(RpsiDesT,RpsiDesdot)
    
    
    Rgorro = np.dot(RpsiDesT,Rpsi)
    RgorroT = Rgorro.transpose()
    Pa = (1/2) *  (Rgorro - RgorroT)
    paElemento = Pa[1][0]
    
    rd = RpsiDesRpsiDesdot[1][0]
    
    
    r = rd - kr * paElemento
    print(vMatrix[0])
    
    u = int(vMatrix[0] + multSeg12*bDrone21U + multSeg23*bDrone23U)
    v = int(vMatrix[1] + multSeg12*bDrone21V + multSeg23*bDrone23V)
    
    #vecTemp = [u,v]
    #xPunto =  np.dot(Rpsi,vecTemp)

    #posLider[4] = xPunto[0]
    #posLider[5] = xPunto[1]
    
    gamma = 0
    
    a2 = 0.2
    f2 = 0.032
    w2 = 2 * math.pi * f2
    ka = 10

    zDes = -1.2
    #zDes = -1
    zDesdot = 0
    #zDesdot = 0
    ze   = z - zDes
    w = -ka * (ze + zDesdot) + 3.5*multSeg12*bDrone21W + 3.5*multSeg23*bDrone23W
    w = -w
    
    dwhdot = gamma*(w + 0 - gamma*z)
    dwh = [0,0,0]
    
    return int(v),int(u),int(w),int(r),xDes,yDes,zDes,psiDes,dwh,posLider,0

def pos_estatica_drone3 (t,yaw,x,y,z,dwh,desfase,posLider,dist):
    
    a = 1.0
    f = 1/30
    w1 = 2 * math.pi * f
    #kp = 30.0
    kp = 4
    kr = 110.0
        
    f2 = 0.025
    w3 = 2 * math.pi * f2
    
    xDes = -1.2
    xe = x - xDes
    xDesdot = 0.0
    
    yDes = 1.2
    ye = y - yDes
    yDesdot = 0.0
    dist13 = dist[1]
    dist23 = dist[2]
    distSeg = dist[3]
    gan = 10
    
    bDrone31U = gan * ((posLider[2][0] - posLider[0][0]) - (posLider[2][1] - posLider[0][1]) - (posLider[2][2] - posLider[0][2]))
    bDrone31V = gan * ((posLider[2][0] - posLider[0][0]) + (posLider[2][1] - posLider[0][1]) - (posLider[2][2] - posLider[0][2]))
    bDrone31W = gan * ((posLider[2][0] - posLider[0][0]) + (posLider[2][1] - posLider[0][1]) + (posLider[2][2] - posLider[0][2]))
    
    bDrone32U = gan * ((posLider[2][0] - posLider[1][0]) - (posLider[2][1] - posLider[1][1]) - (posLider[2][2] - posLider[1][2]))
    bDrone32V = gan * ((posLider[2][0] - posLider[1][0]) + (posLider[2][1] - posLider[1][1]) - (posLider[2][2] - posLider[1][2]))
    bDrone32W = gan * ((posLider[2][0] - posLider[1][0]) + (posLider[2][1] - posLider[1][1]) + (posLider[2][2] - posLider[1][2]))
    
    multSeg13 = 0
    multSeg23 = 0
    if(dist13 < distSeg):
        multSeg13 = 1
    
    if(dist23 < distSeg):
        multSeg23 = 1
        
    psiDes = math.atan2(-ye,-xe)
    if dist23 < distSeg or dist13 < distSeg:
        psiDes = multSeg23*math.atan2(bDrone32V,bDrone32U) + multSeg13*math.atan2(bDrone31V,bDrone31U)
    
    xMatrix = np.array([x,y])
    xDesMatrix = np.array([xDes,yDes])
    
    Rpsi = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    #psiDes= math.atan2(yDes,xDes)-math.pi/2
    #psiDes = 0
    RpsiDes = np.array([[math.cos(psiDes),-math.sin(psiDes)],[math.sin(psiDes),math.cos(psiDes)]])
    RpsiT = Rpsi.transpose()
    
    xeMatrix = xMatrix - xDesMatrix
    
    xDesdotMatrix = np.array([xDesdot,yDesdot])
    
    vMatrix = np.dot(RpsiT ,(xDesdotMatrix - (kp*xeMatrix)))
    
    #psiDesdot = ((xDes*yDesdot) - (yDes*xDesdot))/(a*a)
    psiDesdot = 0
    
    RpsiDesdot = np.array([[(-math.sin(psiDes)*psiDesdot),(-math.cos(psiDes)*psiDesdot)],
                  [(math.cos(psiDes)*psiDesdot),(-math.sin(psiDes)*psiDesdot)]])
    
    RpsiDesT = RpsiDes.transpose()
    
    RpsiDesRpsiDesdot = np.dot(RpsiDesT,RpsiDesdot)
    
    
    Rgorro = np.dot(RpsiDesT,Rpsi)
    RgorroT = Rgorro.transpose()
    Pa = (1/2) *  (Rgorro - RgorroT)
    paElemento = Pa[1][0]
    
    rd = RpsiDesRpsiDesdot[1][0]
    
    
    r = rd - kr * paElemento
    print(vMatrix[0])
    
    u = int(vMatrix[0] + multSeg13*bDrone31U + multSeg23*bDrone32U)
    v = int(vMatrix[1] + multSeg13*bDrone31V + multSeg23*bDrone32V)
    
    #vecTemp = [u,v]
    #xPunto =  np.dot(Rpsi,vecTemp)

    #posLider[4] = xPunto[0]
    #posLider[5] = xPunto[1]
    
    gamma = 0
    
    a2 = 0.2
    f2 = 0.032
    w2 = 2 * math.pi * f2
    ka = 10

    zDes = -1.2
    #zDes = -1
    zDesdot = 0
    #zDesdot = 0
    ze   = z - zDes
    w = -ka * (ze + zDesdot) + 3.5*multSeg13*bDrone31W + 3.5*multSeg23*bDrone32W
    w = -w
    
    dwhdot = gamma*(w + 0 - gamma*z)
    dwh = [0,0,0]
    
    return int(v),int(u),int(w),int(r),xDes,yDes,zDes,psiDes,dwh,posLider,0

def lider_seguidor_estatico1(t,yaw,x,y,z,dwh,desfase,posLider,dist):
    
    a = 1.0
    f = 1/32
    w1 = 2 * math.pi * f
    kp = 30.0
    kr = 110.0

    f2 = 0.025
    w3 = 2 * math.pi * f2
  
    RpsiLider = np.array([[math.cos(posLider[0][3]),-math.sin(posLider[0][3])],[math.sin(posLider[0][3]),math.cos(posLider[0][3])]])  
    
    
    #Ecuaciones para el vecTemp
    vec = np.array([-1,0.6])
    
    
    xDes = posLider[0][0] + vec[0]
    xe = x - xDes
    xDesdot = 0
    #xDesdot = 0

    
    yDes = posLider[0][1] + vec[1]
    ye = y - yDes
    yDesdot = 0
    #yDesdot = 0
    
    xMatrix = np.array([x,y])
    xDesMatrix = np.array([xDes,yDes])
    
    Rpsi = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])

    psiDes= posLider[0][3]
    #psiDes = 0
    RpsiDes = np.array([[math.cos(psiDes),-math.sin(psiDes)],[math.sin(psiDes),math.cos(psiDes)]])
    RpsiT = Rpsi.transpose()
    
    xeMatrix = xMatrix - xDesMatrix
    
    xDesdotMatrix = np.array([xDesdot,yDesdot])
    
    vMatrix = np.dot(RpsiT ,(xDesdotMatrix - (kp*xeMatrix)))
    
    psiDesdot = 0
    #psiDesdot = 0
    
    RpsiDesdot = np.array([[(-math.sin(psiDes)*psiDesdot),(-math.cos(psiDes)*psiDesdot)],
                  [(math.cos(psiDes)*psiDesdot),(-math.sin(psiDes)*psiDesdot)]])
    
    RpsiDesT = RpsiDes.transpose()
    
    RpsiDesRpsiDesdot = np.dot(RpsiDesT,RpsiDesdot)
    
    
    Rgorro = np.dot(RpsiDesT,Rpsi)
    RgorroT = Rgorro.transpose()
    Pa = (1/2) *  (Rgorro - RgorroT)
    paElemento = Pa[1][0]
    
    rd = RpsiDesRpsiDesdot[1][0]
    
    
    r = rd - kr * paElemento
    
    u = int(vMatrix[0])
    v = int(vMatrix[1])
    
    gamma = 0
    
    a2 = 0.2
    f2 = 0.032
    w2 = 2 * math.pi * f2
    ka = 30
    
    #zDes = -1.2 + a2 * math.sin(w2*t)
    
    zDes = posLider[0][2]
    #zDesdot = a2*w2*math.cos(w2*t)
    zDesdot = 0
    ze   = z - zDes
    w = -ka * (ze + zDesdot)
    w = -w
    
    dwhdot = gamma*(w + 0 - gamma*z)
    dwh = [0,0,0]
    
    
    return int(v),int(u),int(w),int(r),xDes,yDes,zDes,psiDes,dwh,posLider,desfase

def lider_seguidor_estatico2(t,yaw,x,y,z,dwh,desfase,posLider,dist):
    
    a = 1.0
    f = 1/32
    w1 = 2 * math.pi * f
    kp = 30.0
    kr = 110.0

    f2 = 0.025
    w3 = 2 * math.pi * f2
  
    RpsiLider = np.array([[math.cos(posLider[0][3]),-math.sin(posLider[0][3])],[math.sin(posLider[0][3]),math.cos(posLider[0][3])]])  
    
    
    #Ecuaciones para el vecTemp
    vec = np.array([-1,-0.6])
    
    
    xDes = posLider[0][0] + vec[0]
    xe = x - xDes
    xDesdot = 0
    #xDesdot = 0

    
    yDes = posLider[0][1] + vec[1]
    ye = y - yDes
    yDesdot = 0
    #yDesdot = 0
    
    xMatrix = np.array([x,y])
    xDesMatrix = np.array([xDes,yDes])
    
    Rpsi = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])

    psiDes= posLider[0][3]
    #psiDes = 0
    RpsiDes = np.array([[math.cos(psiDes),-math.sin(psiDes)],[math.sin(psiDes),math.cos(psiDes)]])
    RpsiT = Rpsi.transpose()
    
    xeMatrix = xMatrix - xDesMatrix
    
    xDesdotMatrix = np.array([xDesdot,yDesdot])
    
    vMatrix = np.dot(RpsiT ,(xDesdotMatrix - (kp*xeMatrix)))
    
    psiDesdot = 0
    #psiDesdot = 0
    
    RpsiDesdot = np.array([[(-math.sin(psiDes)*psiDesdot),(-math.cos(psiDes)*psiDesdot)],
                  [(math.cos(psiDes)*psiDesdot),(-math.sin(psiDes)*psiDesdot)]])
    
    RpsiDesT = RpsiDes.transpose()
    
    RpsiDesRpsiDesdot = np.dot(RpsiDesT,RpsiDesdot)
    
    
    Rgorro = np.dot(RpsiDesT,Rpsi)
    RgorroT = Rgorro.transpose()
    Pa = (1/2) *  (Rgorro - RgorroT)
    paElemento = Pa[1][0]
    
    rd = RpsiDesRpsiDesdot[1][0]
    
    
    r = rd - kr * paElemento
    
    u = int(vMatrix[0])
    v = int(vMatrix[1])
    
    gamma = 0
    
    a2 = 0.2
    f2 = 0.032
    w2 = 2 * math.pi * f2
    ka = 30
    
    #zDes = -1.2 + a2 * math.sin(w2*t)
    
    zDes = posLider[0][2]
    #zDesdot = a2*w2*math.cos(w2*t)
    zDesdot = 0
    ze   = z - zDes
    w = -ka * (ze + zDesdot)
    w = -w
    
    dwhdot = gamma*(w + 0 - gamma*z)
    dwh = [0,0,0]
    
    
    return int(v),int(u),int(w),int(r),xDes,yDes,zDes,psiDes,dwh,posLider,desfase

    def pos_estatica_drone_bailarina (t,yaw,x,y,z,dwh,desfase,posLider,dist):
        
        a = 1.0
        f = 1/30
        w1 = 2 * math.pi * f
        #kp = 30.0
        
        #Meterlas como matriz diagonal de ganancias
        k1 = np.array([[0.5,0],[0,0.5]]) #0.4
        
        hat_deltaxy = np.array([dwh[0],dwh[1]])
        hat_deltar = dwh[2]
        
        up = dwh[3]
        vp = dwh[4]
        Up = [up,vp]
        
        kp = 4
        kr = 220.0
            
        f2 = 0.025
        w3 = 2 * math.pi * f2
        
        xDes = 0.0
        xe = x - xDes
        xDesdot = 0.0
        
        yDes = 0 #-1.5
        ye = y - yDes
        yDesdot = 0.0
        
        dist12 = dist[0]
        dist13 = dist[1]
        
        distSeg = dist[3]
        
        gan = 10
        
        bDrone12U = 0*gan * ((posLider[0][0] - posLider[1][0]) - (posLider[0][1] - posLider[1][1]) - (posLider[0][2] - posLider[1][2]))
        bDrone12V = 0*gan * ((posLider[0][0] - posLider[1][0]) + (posLider[0][1] - posLider[1][1]) - (posLider[0][2] - posLider[1][2]))
        bDrone12W = 0*gan * ((posLider[0][0] - posLider[1][0]) + (posLider[0][1] - posLider[1][1]) + (posLider[0][2] - posLider[1][2]))
        
        bDrone13U = 0*gan * ((posLider[0][0] - posLider[2][0]) - (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
        bDrone13V = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
        bDrone13W = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) + (posLider[0][2] - posLider[2][2]))
        
        multSeg12 = 0
        multSeg13 = 0
        if(dist12 < distSeg):
            multSeg12 = 1
        
        if(dist13 < distSeg):
            multSeg13 = 1
        
        if dist12 < distSeg or dist13 < distSeg:
            psiDes = multSeg12*math.atan2(bDrone12V,bDrone12U) + multSeg13*math.atan2(bDrone13V,bDrone13U)
        
        psiDes = math.atan2(-ye,-xe)
        
        
        xMatrix = np.array([x,y])
        xDesMatrix = np.array([xDes,yDes])
        
        Rpsi = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        #psiDes= math.atan2(yDes,xDes)-math.pi/2
        #psiDes = 0
        RpsiDes = np.array([[math.cos(psiDes),-math.sin(psiDes)],[math.sin(psiDes),math.cos(psiDes)]])
        RpsiT = Rpsi.transpose()
        
        xeMatrix = xMatrix - xDesMatrix
        
        xDesdotMatrix = np.array([xDesdot,yDesdot])
        
        deltaxy = np.array([hat_deltaxy[0],hat_deltaxy[1]]) + np.dot(k1,xMatrix)
        
        #Control
        vMatrix = np.dot(RpsiT ,(xDesdotMatrix - (kp*xeMatrix) - deltaxy))
        
        #psiDesdot = ((xDes*yDesdot) - (yDes*xDesdot))/(a*a)
        psiDesdot = 0
        
        RpsiDesdot = np.array([[(-math.sin(psiDes)*psiDesdot),(-math.cos(psiDes)*psiDesdot)],
                      [(math.cos(psiDes)*psiDesdot),(-math.sin(psiDes)*psiDesdot)]])
        
        RpsiDesT = RpsiDes.transpose()
        
        RpsiDesRpsiDesdot = np.dot(RpsiDesT,RpsiDesdot)
        
        
        Rgorro = np.dot(RpsiDesT,Rpsi)
        RgorroT = Rgorro.transpose()
        Pa = (1/2) *  (Rgorro - RgorroT)
        paElemento = Pa[1][0]
        
        rd = RpsiDesRpsiDesdot[1][0]
        
        k2 = 0.5
        deltar = hat_deltar + k2*yaw
        
        deltapsid = np.array([-ye/(xe*xe + ye*ye),xe/(xe*xe + ye*ye)])
        
        r = - deltar - kp*np.dot(deltapsid.transpose(),xeMatrix) - kr*paElemento
        u = int(vMatrix[0] + multSeg12*bDrone12U + multSeg13*bDrone13U)
        v = int(vMatrix[1] + multSeg12*bDrone12V + multSeg13*bDrone13V)
        
        #vecTemp = [u,v]
        #xPunto =  np.dot(Rpsi,vecTemp)

        #posLider[4] = xPunto[0]
        #posLider[5] = xPunto[1]
        
        gamma = 0.0 #0.1
        
        a2 = 0.2
        f2 = 0.032
        w2 = 2 * math.pi * f2
        ka = 10

        zDes = -1.2
        #zDes = -1
        zDesdot = 0
        #zDesdot = 0
        ze   = z - zDes
        w = -ka * (ze + zDesdot) + 3.5*multSeg12*bDrone12W + 3.5*multSeg13*bDrone13W
        w = -w
        
        U=np.array([u,v])
        
        dothat_deltaxy = xeMatrix + kp*np.dot(k1,xeMatrix) - paElemento*deltapsid
        hat_deltaxy = hat_deltaxy + 1/60 * dothat_deltaxy
        
        dothat_deltar = paElemento + gamma*kp*np.dot(deltapsid,xeMatrix) - kr*paElemento
        hat_deltar = hat_deltar + 1/60 * dothat_deltar
        
        dwh = [hat_deltaxy[0],hat_deltaxy[1],hat_deltar,deltaxy[0],deltaxy[1],deltar]
        
        return int(v),int(u),int(w),int(r),xDes,yDes,zDes,psiDes,dwh,posLider,0
    
    def prueba_orientación (t,yaw,x,y,z,dwh,desfase,posLider,dist):
        
        #Se lee el contador
        cont = dwh[6]
        xm = dwh[7]
        ym = dwh[8]
        
        if cont == 0:
            xi = x
            yi = y
        else:
            xi = xm
            yi = ym
        
        
        #Meterlas como matriz diagonal de ganancias
        k1 = -np.array([[0.35,0],[0,0.35]]) #0.4
        k2 = -0.015
        
        kp = 20
        #Esta ganancia de rotacion debe ser muy baja
        kr = 1
        
        
        #Posiciones deseadas
        xd = 0
        yd = 0
        
        #Posicion en forma matricial
        X = np.array([x,y])
        
        #Valores deseados en forma matricial
        XDes = np.array([xd,yd])
        dXDes = np.array([0,0])
        
        #Errores
        Xt = X - XDes
        
        #Campos vectoriales de repulsion
        dist12 = dist[0]
        dist13 = dist[1]
        
        distSeg = dist[3]
        
        gan = 10
        
        bDrone12U = 0*gan * ((posLider[0][0] - posLider[1][0]) - (posLider[0][1] - posLider[1][1]) - (posLider[0][2] - posLider[1][2]))
        bDrone12V = 0*gan * ((posLider[0][0] - posLider[1][0]) + (posLider[0][1] - posLider[1][1]) - (posLider[0][2] - posLider[1][2]))
        bDrone12W = 0*gan * ((posLider[0][0] - posLider[1][0]) + (posLider[0][1] - posLider[1][1]) + (posLider[0][2] - posLider[1][2]))
        
        bDrone13U = 0*gan * ((posLider[0][0] - posLider[2][0]) - (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
        bDrone13V = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) - (posLider[0][2] - posLider[2][2]))
        bDrone13W = 0*gan * ((posLider[0][0] - posLider[2][0]) + (posLider[0][1] - posLider[2][1]) + (posLider[0][2] - posLider[2][2]))
        
        multSeg12 = 0
        multSeg13 = 0
        if(dist12 < distSeg):
            multSeg12 = 1
        
        if(dist13 < distSeg):
            multSeg13 = 1
        
        if dist12 < distSeg or dist13 < distSeg:
            psid = multSeg12*math.atan2(bDrone12V,bDrone12U) + multSeg13*math.atan2(bDrone13V,bDrone13U)
        
        
        #Orientacion deseada
        psid = 0 #math.atan2(yd-yi, xd-xi)
        
        rd = 0;
        
        #Matriz de orientacion actual
        R = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        
        #Matriz de orientacion deseada
        Rd = np.array([[math.cos(psid),-math.sin(psid)],[math.sin(psid),math.cos(psid)]])
        
        #Error de orientacion
        Rt = np.dot(Rd.transpose(),R)
        
        #Parte antisimetrica
        Pa = 0.5*(Rt-Rt.transpose())
        
        Dpsi = np.array([0,0])
        
        #Sacamos los estimadores del buffer
        hat_deltaxy = np.array([dwh[0],dwh[1]])
        hat_deltar = dwh[2]
        
        #Estimadores
        Xalt = np.array([math.tanh(Xt[0]*6),math.tanh(Xt[1]*2)])
        dothat_deltaxy = Xt - kp*np.dot(k1,Xalt) - Pa[1][0]*Dpsi
        
        dothat_deltar = k2*(rd - kr*Pa[1][0])
        
        
        beta1 = np.dot(k1,X)
        beta2 = k2*yaw 
        
        
        U = np.dot(R.transpose(),-kp*Xalt + dXDes - hat_deltaxy + beta1)
        
        r = rd - kr * Pa[1][0] - hat_deltar + beta2
        u = 0
        v = 0
        
        deltaxy = hat_deltaxy - beta1
        deltar = hat_deltar - beta2
        
        #Control de altitud
        a2 = 0.2
        f2 = 0.032
        w2 = 2 * math.pi * f2
        ka = 10

        zDes = -1.2
        #zDes = -1
        zDesdot = 0
        #zDesdot = 0
        ze   = z - zDes
        w = -ka * (ze + zDesdot) + 3.5*multSeg12*bDrone12W + 3.5*multSeg13*bDrone13W
        w = 0
        
        #Integraciones
        hat_deltaxy = hat_deltaxy + 0.05 * dothat_deltaxy
        hat_deltar = hat_deltar + 0.05 * dothat_deltar
        
        #if dothat_deltaxy[0] > 0.5:
        #    dothat_deltaxy[0] = 0.5
            
        #if dothat_deltaxy[0] < -0.5:
        #    dothat_deltaxy[0] = -0.5
        
        #if dothat_deltaxy[1] > 0.5:
        #    dothat_deltaxy[1] = 0.5
            
        #if dothat_deltaxy[1] < -0.5:
        #    dothat_deltaxy[1] = -0.5
        
        dwh = [hat_deltaxy[0],hat_deltaxy[1],hat_deltar,dothat_deltaxy[0],deltaxy[1],deltar,cont+1,xi,yi]
        
        return int(v),int(u),int(w),int(r),xd,yd,zDes,psid,dwh,posLider,0