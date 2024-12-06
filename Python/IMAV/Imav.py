import cv2
import cv2.aruco as aruco
import numpy as  np
import menosVision.metodosVision as mv
import metodos
import grpc
import drones_pb2
import drones_pb2_grpc
import os
from scipy.ndimage import center_of_mass
from time import sleep
import socket


class Cliente:
    def __init__(self, hostPort):
        self.hostPort = hostPort
        self.channel = grpc.insecure_channel(self.hostPort)
        self.stub = drones_pb2_grpc.ManejoDroneStub(self.channel)

    def llamarServer(self,uP,vP,wP,rP):
        velocidades = drones_pb2.Velocidades(u=uP,
                                            v=vP,
                                            w=wP,
                                            r=rP) 
        
        return self.stub.mandaVelocidades(velocidades)
  
    def llamaServerStr(self,strMsg):
        msgJava = drones_pb2.VelocidadesStr(msg = strMsg)


        return self.stub.mandaVelocidadesStr(msgJava)


def connect_rtmp_stream(stream_url):
    # Create a VideoCapture object
    cap = cv2.VideoCapture(stream_url)

    if not cap.isOpened():
        print("Error: Unable to open stream.")
        return
    cont_ids = 0
    todos_ids = [100,105,200,302,400,405,100]
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    parameters = aruco.DetectorParameters()
    sigue_camino = True
    verifica_qr = True
    descartar_frames= True
    #arr_2 = [3,1]
    arr_2 = [1,3]
    arr_alturas_2 = [75,100]
    cont_tot_1 = 0
    cont_tot_2 = 0
    cont_tot_2_1 = 0
    cont_tot_3 = 0
    cont_tot_4 = 0
    cont_tot_5 = 0
    cont_tot_5_1 = 0
    ventana_bool = False
    obj_bool = False
    centrado_obj = False
    centrar_obj_verde = False
    mass_center_anterior_x = 0
    mass_center_anterior_y = 0
    cont_diff_mass = 0

    seguimiento_camino = False
    cont_tot_6 = 0

    indx_velocidad = 0
    vec_velocidad = [12,16,6]

    lower_green = np.array([40, 40, 40])
    upper_green = np.array([100, 255, 255])
    channel3Min = 150.0
    channel3Max = 255.000

    #cliente.llamaServerStr("Take")
    ret, frame = cap.read()
    
    cont_1 = 0
    while cont_1 < 300:
        ret, frame = cap.read()
        print("a")
        cont_1 += 1
        if not ret:
            print("Error: Unable to fetch frame.")
            break

        cv2.imshow('RTMP Stream', frame)
        # Press 'q' on the keyboard to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            descartar_frames = False

    vec = [0,0,0,0]
    while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            if not ret:
                print("Error: Unable to fetch frame.")
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detectar los marcadores
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            try:
                if ids is not None:
                    frame = aruco.drawDetectedMarkers(frame, corners, ids)
                    s = frame.shape
                    indice_id = np.where(ids == todos_ids[cont_ids])[0]
                    if cont_ids == 0:
                        if indice_id == 0:
                            vec,c = metodos.alinea_qr_alt(corners,s,2,75)
                            cv2.circle(frame, c, 5, (0, 255, 0), -1)
                            tot = abs(vec[0]) + abs(vec[1]) + abs(vec[2]) + abs(vec[3])
                            print("centrando",todos_ids[cont_ids],"suma:",cont_tot_1)
                            if tot <= 2:
                                cont_tot_1 += 1
                                if cont_tot_1 == 10:
                                    cont_ids += 1
                            else:
                                cont_tot_1 = 0
                    elif cont_ids == 1:
                        if indice_id == 0:
                            vec,c = metodos.alinea_qr_alt(corners,s,arr_2[cont_tot_2_1],arr_alturas_2[cont_tot_2_1])
                            cv2.circle(frame, c, 5, (0, 255, 0), -1)
                            #tot = abs(vec[0]) + abs(vec[1]) + abs(vec[2]) + abs(vec[3])
                            tot = abs(vec[3])
                            print("centrando",todos_ids[cont_ids],"suma:",cont_tot_2)
                            if tot <= 1:
                                cont_tot_2 += 1
                                if cont_tot_2 == 10:
                                    cont_tot_2_1 += 1
                                    cont_tot_2 = 0
                                    if cont_tot_2_1 == 1:
                                        cont_r = 0
                                        print("Ver imagen")
                                        while cont_r < 1000:
                                            ret, frame = cap.read()
                                            cliente.llamarServer(0,0,0,100)
                                            cv2.imshow('RTMP Stream', frame)
                                            cv2.waitKey(1)
                                            cont_r+=1
                                        cont_r = 0
                                        while cont_r < 100:
                                            ret, frame = cap.read()
                                            cliente.llamarServer(0,0,0,100)
                                            cv2.imshow('RTMP Stream', frame)
                                            cv2.waitKey(1)
                                            cont_r+=1
                                    elif cont_tot_2_1 == 2:
                                        cont_ids += 1
                            else:
                                cont_tot_2=0

                        else:
                            vec = [0,vec_velocidad[indx_velocidad],0,0]

                    elif cont_ids == 2:
                        if indice_id == 0:
                            if not ventana_bool:
                                vec,c = metodos.alinea_qr_alt(corners,s,3,180)
                                cv2.circle(frame, c, 5, (0, 255, 0), -1)
                                tot = abs(vec[0]) + abs(vec[1]) + abs(vec[2]) + abs(vec[3])
                                print("centrando",todos_ids[cont_ids],"suma:",cont_tot_3)
                                if tot <= 1:
                                    cont_tot_3 += 1
                                    if cont_tot_3 == 20:
                                        ventana_bool = True
                                else:
                                    cont_tot_3=0
                            else:
                                ventanaPasada = False
                                umbralBool = False
                                qr_encontrado = False
                                centrado = False


                                print("Mover camara ver marco")
                                cont = 0
                                while cont<300:
                                    cont += 1
                                    ret, frame = cap.read()
                                    cv2.imshow('RTMP Stream', frame)
                                    cv2.waitKey(1)

                                while not umbralBool:
                                    ret, frame = cap.read()
                                    cliente.llamarServer(0,0,5,0)
                                    umbralBool,tot = metodos.calcula_umbral2(frame)
                                    
                                cliente.llamarServer(0,0,0,0)

                                print("Mover camara ver QR despues del marco")
                                cont = 0
                                while cont<300:
                                    cont += 1
                                    ret, frame = cap.read()
                                    cv2.imshow('RTMP Stream', frame)
                                    cv2.waitKey(1)

                                #cont_frames = 0

                                #while not qr_encontrado:
                                #    ret, frame = cap.read()
                                #    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                                #    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                                #    s = frame.shape
                                #    centrado,vec,frame = metodos.qr_205(s,centrado,ids,corners,frame)
                                #    cliente.llamarServer(vec[0],vec[1],vec[2],vec[3])
                                #    tot = abs(vec[0]) + abs(vec[1]) + abs(vec[2]) + abs(vec[3])
                                #    if tot <= 1 :
                                #        if cont_frames == 25:
                                #            qr_encontrado = True
                                #        cont_frames +=1
                                #    else:
                                #        cont_frames = 0
                                #    cv2.imshow('RTMP Stream', frame)

                                #    if cv2.waitKey(1) & 0xFF == ord('q'):
                                #        break
                                cont_ids +=1
                                indx_velocidad = 1
                                
                        
                        else:
                            vec = [0,8,0,0]
                    elif cont_ids == 3:
                        if indice_id == 0:
                            des = [0,40]
                            vec,c = metodos.alinea_qr_alt_des(corners,s,3,75,des)
                            cv2.circle(frame, c, 5, (0, 255, 0), -1)
                            tot = abs(vec[0]) + abs(vec[1]) + abs(vec[2]) + abs(vec[3])
                            print("centrando",todos_ids[cont_ids],"suma:",cont_tot_1)
                            if tot <= 2:
                                cont_tot_1 += 1
                                if cont_tot_1 == 60:
                                    cont_ids += 1
                                    #cliente.llamaServerStr("Land")
                                    cont_land = 0
                                    while(cont_land < 300):
                                        ret, frame = cap.read()
                                        cv2.imshow('RTMP Stream', frame)
                                        cv2.waitKey(1)
                                        cont_land += 1
                                    #cliente.llamaServerStr("Take")
                                    cont_take = 0
                                    while(cont_take < 300):
                                        ret, frame = cap.read()
                                        cv2.imshow('RTMP Stream', frame)
                                        cv2.waitKey(1)
                                        cont_take += 1
                                    
                            else:
                                cont_tot_1 = 0
                        else:
                            vec = [0,vec_velocidad[indx_velocidad],0,0]
                    elif cont_ids == 4:
                        if not obj_bool:
                            if indice_id == 0:
                                des = [0,0]
                                vec,c = metodos.alinea_qr_alt_des(corners,s,0,158,des)
                                cv2.circle(frame, c, 5, (0, 255, 0), -1)
                                tot = abs(vec[2])
                                print("Suma:",cont_tot_4)
                                if tot <= 1:
                                    cont_tot_5 += 1
                                    if cont_tot_5 == 20:

                                        obj_bool = True
                                        centrar_obj_verde = True
                                        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                                        cliente.llamarServer(0,0,0,0)
                                        cont_ids += 1
                                        print("Sali de conts",cont_ids)
                                        try:
                                            sock.connect((arduino_ip, arduino_port))
                                            command = "0"
                                            sock.sendall(command.encode())
                                            response = sock.recv(1024)
                                        except Exception as e:
                                            print(f"Error al conectar arduino: {e}")
                                            cont_ids += 1
                                            seguimiento_camino = True
                                            cont_no_vio = 0
                                            while cont_no_vio < 5000:
                                                ret, frame = cap.read()
                                                cont_no_vio += 1
                                            
                                        
                                        sock.close()
                                else:
                                    cont_tot_5=0
                            else:
                                indx_velocidad = 2
                                vec = [0,6,0,0]
                    elif cont_ids == 5:
                        if indice_id == 0:
                            des = [0,-40]
                            vec,c = metodos.alinea_qr_alt_des(corners,s,3,150,des)
                            cv2.circle(frame, c, 5, (0, 255, 0), -1)
                            tot = abs(vec[0]) + abs(vec[1])
                            print("Suma:",cont_tot_5_1)
                            if tot <= 1:
                                cont_tot_5_1 += 1
                                if cont_tot_5_1 == 20:
                                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                                    cliente.llamarServer(0,0,0,0)
                                    try:
                                        sock.connect((arduino_ip, arduino_port))
                                        command = "1"
                                        sock.sendall(command.encode())
                                        response = sock.recv(1024)
                                        cont_ids += 1
                                    except Exception as e:
                                        print(f"Error al conectar arduino: {e}")
                                    
                                    sock.close()

                                    seguimiento_camino = True
                            else:
                                cont_tot_5_1=0
                        else:
                            indx_velocidad = 2
                            vec = [0,6,0,0]
                    else:
                        if indice_id == 0:
                            
                            vec,c = metodos.alinea_qr_alt(corners,s,1,75)
                            cv2.circle(frame, c, 5, (0, 255, 0), -1)
                            tot = abs(vec[0]) + abs(vec[1]) + abs(vec[2]) + abs(vec[3])
                            print("centrando",todos_ids[cont_ids],"suma:",cont_tot_6)
                            if tot <= 1:
                                cont_tot_6 += 1
                                if cont_tot_6 == 10:
                                    cliente.llamaServerStr("Land")
                            else:
                                cont_tot_6=0
                        else:
                            todoCentro = frame[96:287,338:506]
                            todoCentro = mv.blanco_negro_nuevo(todoCentro)
                            img2 = todoCentro[0:95,:]
                            
                            sumImg2 = sum(sum(img2))

                            if(sumImg2 > 1000):

                                x = mv.centrar(todoCentro)
                                vec = [x[0],2,0,x[1]]

                            else:

                                decideLado = frame[96:383,169:675]
                                decideLado = mv.blanco_negro_nuevo(decideLado)

                                yaw = mv.no_derecho(decideLado)
                                yaw = int(yaw)
                                
                                vec = [0,0,0,yaw]
                            
                elif centrar_obj_verde:
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    print("Entroooooooo")
                    # Definir el rango de color verde en HSV
                    lower_green = np.array([40, 40, 40])  # Mínimo del color verde
                    upper_green = np.array([90, 255, 255])  # Máximo del color verde

                    # Crear una máscara que capture solo el color verde
                    mask = cv2.inRange(hsv, lower_green, upper_green)

                    # Aplicar la máscara a la imagen original
                    result = cv2.bitwise_and(frame, frame, mask=mask)

                    bw = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)

                    channel3Min = 100.0
                    #107.1
                    channel3Max = 255.000

                    BW = (bw[:, :] >= channel3Min) & (bw[:, :] <= channel3Max)

                    BW = BW.astype(float)

                    com = center_of_mass(BW)

                    try:
                        center_x = int(com[1])
                        center_y = int(com[0])
                        s = frame.shape
                        c_x = int((s[1] / 2))
                        c_y = int((s[0] / 2) + 70)
                        xDif = (center_x - c_x) / 30
                        yDif = (c_y - center_y) / 30
                        vec = [int(xDif), int(yDif),0,0]

                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

                        diff_mass_x = mass_center_anterior_x - center_x
                        diff_mass_y = mass_center_anterior_y - center_y
                        diff_mass = abs(diff_mass_x) + abs(diff_mass_y)
                        mass_center_anterior_x = center_x
                        mass_center_anterior_y = center_y
                        print("suma mass:",cont_diff_mass)
                        if diff_mass < 2:
                            cont_diff_mass += 1
                            if cont_diff_mass == 15:
                                centrar_obj_verde = False
                                cont_subir = 0
                                #cont_ids +=1
                                print("print antes de subir: ",cont_ids)
                                cliente.llamarServer(0,0,8,0)
                                while cont_subir < 100:
                                    ret, frame = cap.read()
                                    cont_subir+=1                                  
                                    print("subiendo")
                                    cv2.imshow('RTMP Stream', frame)
                                    cv2.waitKey(1)

                        else:
                            cont_diff_mass = 0
                    except:
                        print("No verde ")
                        vec = [0,6,0,0]
                elif seguimiento_camino:
                    todoCentro = frame[96:287,338:506]
                    todoCentro = mv.blanco_negro_nuevo(todoCentro)
                    img2 = todoCentro[0:95,:]
                        
                    sumImg2 = sum(sum(img2))

                    if(sumImg2 > 1000):

                        x = mv.centrar(todoCentro)
                        vec = [x[0],2,0,x[1]]

                    else:

                        decideLado = frame[96:383,169:675]
                        decideLado = mv.blanco_negro_nuevo(decideLado)

                        yaw = mv.no_derecho(decideLado)
                        yaw = int(yaw)
                            
                        vec = [0,0,0,yaw]
            
                else:
                    vec = [0,vec_velocidad[indx_velocidad],0,0]
            except:
                vec = [0,vec_velocidad[indx_velocidad],0,0]

            cliente.llamarServer(vec[0],vec[1],vec[2],vec[3])
            cv2.imshow('RTMP Stream', frame)
            # Press 'q' on the keyboard to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

# Example usage
arduino_ip = "192.168.1.136"
arduino_port = 80
datos = list()
cliente = Cliente('localhost:50051') 
#stream_url = 'rtmp://192.168.1.122/live/sample1'
stream_url = 'rtmp://10.217.24.213/live/sample1'
connect_rtmp_stream(stream_url)