import menosVision.metodosVision as mv
import numpy as np
import cv2
import cv2.aruco as aruco
import math

def seguimineto_de_camino(frame):

    muestra_img = mv.blanco_negro_nuevo(frame)
    
    todoCentro = frame[96:287,338:506]
    todoCentro = mv.blanco_negro_nuevo(todoCentro)

    img2 = todoCentro[0:95,:]
    
    sumImg2 = sum(sum(img2))

    if(sumImg2 > 1000):

        x = mv.centrar(todoCentro)

        vec = x[0],2,0,x[1]
    else:

        decideLado = frame[96:383,169:675]
        decideLado = mv.blanco_negro_nuevo(decideLado)

        yaw = mv.no_derecho(decideLado)
        yaw = int(yaw)

        vec = 0,0,0,yaw

    return vec

def alinea_qr(corners,s,alineado):
    corners = np.array(corners)
    corner = corners.reshape((4, 2))
    center_x = int(corner[:, 0].mean())
    center_y = int(corner[:, 1].mean())


    a11 = corners[0][0][0+alineado][1]
    a12 = corners[0][0][1+alineado][1]
    
    dif = (a12 - a11)/1
    #dif = 0

    c_x = int((s[1] / 2))
    c_y = int((s[0] / 2))

    xDif = (center_x - c_x) / 30
    yDif = (c_y - center_y) / 30

    vec = [int(xDif), int(yDif),0,int(dif)]
    c = (c_x,c_y)

    return vec,c

def alinea_qr_alt(corners,s,alineado,altura):

    corners = np.array(corners)
    corner = corners.reshape((4, 2))
    center_x = int(corner[:, 0].mean())
    center_y = int(corner[:, 1].mean())

    a11 = corners[0][0][0][0]
    a12 = corners[0][0][0][1]
                    
    b11 = corners[0][0][1][0]
    b12 = corners[0][0][1][1]

    dist = math.sqrt((b11 - a11)**2 + (b12 - a12)**2)

    if alineado < 3:
        c11 = corners[0][0][0+alineado][1]
        c12 = corners[0][0][1+alineado][1]
    else:
        c11 = corners[0][0][3][1]
        c12 = corners[0][0][0][1]
    
    dif = (c12 - c11)/1

    c_x = int((s[1] / 2))
    c_y = int((s[0] / 2))


    w = (dist - altura)/10

    xDif = (center_x - c_x) / 30
    yDif = (c_y - center_y) / 30

    vec = [int(xDif), int(yDif),int(w),int(dif)]
    c = (c_x,c_y)

    return vec,c

def alinea_qr_alt_des(corners,s,alineado,altura,des):

    corners = np.array(corners)
    corner = corners.reshape((4, 2))
    center_x = int(corner[:, 0].mean())
    center_y = int(corner[:, 1].mean())

    a11 = corners[0][0][0][0]
    a12 = corners[0][0][0][1]
                    
    b11 = corners[0][0][1][0]
    b12 = corners[0][0][1][1]

    dist = math.sqrt((b11 - a11)**2 + (b12 - a12)**2)

    if alineado < 3:
        c11 = corners[0][0][0+alineado][1]
        c12 = corners[0][0][1+alineado][1]
    else:
        c11 = corners[0][0][3][1]
        c12 = corners[0][0][0][1]
    
    dif = (c12 - c11)/1

    c_x = int((s[1] / 2))
    c_y = int((s[0] / 2))
    c_x = c_x + des[0]
    c_y = c_y + des[1]


    w = (dist - altura)/10

    xDif = (center_x - c_x) / 25
    yDif = (c_y - center_y) / 25

    vec = [int(xDif), int(yDif),int(w),int(dif)]
    c = (c_x,c_y)

    return vec,c

def centrar_ventana(frame):
    cYc = 240
    cXc = 424
    center_y = 240
    resp = False

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definir el rango para el color rojo en HSV
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])

    # Crear una máscara que capture solo el color rojo
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Aplicar la máscara a la imagen original
    red_filtered = cv2.bitwise_and(frame, frame, mask=mask)

    # Convertir la imagen filtrada a escala de grises
    gray = cv2.cvtColor(red_filtered, cv2.COLOR_BGR2GRAY)

    # Usar detección de bordes para encontrar las líneas
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    # Usar la transformada de Hough para encontrar líneas
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)

    # Almacenar las coordenadas y calcular el centro entre las líneas horizontales
    if lines is not None:
        horizontal_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if abs(y1 - y2) < 10:  # Consideramos solo las líneas casi horizontales
                horizontal_lines.append((y1, y2))

        if len(horizontal_lines) >= 2:
            y_coords = [y for y1, y2 in horizontal_lines for y in [y1, y2]]
            y_coords.sort()
            top_line_y = y_coords[0]
            bottom_line_y = y_coords[-1]

            center_y = (top_line_y + bottom_line_y) // 2
            center_x = frame.shape[1] // 2  # Asumimos el centro en el eje X como la mitad de la imagen

            # Dibujar las líneas detectadas y el centro en la imagen
            for y1, y2 in horizontal_lines:
                cv2.line(frame, (0, y1), (frame.shape[1], y2), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
            cv2.putText(frame, "Centro", (center_x - 25, center_y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    #xDif = (center_x - cXc)/20
    yDif = (cYc - center_y)/10

    if yDif<2:
        resp = True

    cv2.imshow('RTMP Stream', frame)
    cv2.waitKey(1)

    return resp,yDif

def calcula_umbral2(frame):

    resp = False
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definir el rango para el color rojo en HSV
    lower_red1 = np.array([0, 40, 40])
    upper_red1 = np.array([15, 255, 255])
    lower_red2 = np.array([160, 40, 40])
    upper_red2 = np.array([180, 255, 255])

    # Crear una máscara que capture solo el color rojo
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Aplicar la máscara a la imagen original
    red_filtered = cv2.bitwise_and(frame, frame, mask=mask)

    gray = cv2.cvtColor(red_filtered, cv2.COLOR_BGR2GRAY)

    channel3Min = 35.0
    #107.1
    channel3Max = 260.000
    
    BW = (gray[:, :] >= channel3Min) & (gray[:, :] <= channel3Max)

    BW_superior = BW[:159,:]
    BW_inferior = BW[320:,:]

    tot_superior = sum(sum(BW_superior))
    tot_inferior = sum(sum(BW_inferior))

    if tot_superior > 10000 and tot_inferior > 10000:
        resp = True
    BW = BW.astype(float)
    cv2.imshow('RTMP Stream', BW)
    cv2.waitKey(1)

    return resp,tot_inferior

def qr_205(s,centrado,ids,corners,frame):

    if not centrado:

        try:
            a = np.where(ids == 205)[0]
            if a == 0:
                for corner in corners:
                    corner = np.array(corner)
                    corner = corner.reshape((4, 2))
                    center_x = int(corner[:, 0].mean())
                    center_y = int(corner[:, 1].mean())
                    center = (center_x, center_y)
                c_x = int((s[1] / 2))
                c_y = int((s[0] / 2))
                c_y = c_y + 20

                c = (c_x,c_y)
                cv2.circle(frame, center, 5, (255, 0, 0), -1)
                cv2.circle(frame, c, 5, (0, 255, 0), -1)
                vec = [0,7,0,0]
                if center_y > c_y:
                    centrado = True
            else:
                vec = [0,9,0,0]
        except:
            vec = [0,9,0,0]
    else:
        try:
            a = np.where(ids == 205)[0]
            if a == 0 and len(ids) == 1:
                for corner in corners:
                    corner = np.array(corner)
                    corner = corner.reshape((4, 2))
                    center_x = int(corner[:, 0].mean())
                    center_y = int(corner[:, 1].mean())

                a11 = corners[0][0][0][0]
                a12 = corners[0][0][0][1]
                        
                b11 = corners[0][0][1][0]
                b12 = corners[0][0][1][1]

                c11 = corners[0][0][1][1]
                c12 = corners[0][0][2][1]

                dist = math.sqrt((b11 - a11)**2 + (b12 - a12)**2)
                c_x = int((s[1] / 2))
                c_y = int((s[0] / 2))
                c_x = c_x + 200
                c_y = c_y + 20


                alt_des = 75

                w = (dist - alt_des)/10

                xDif = (center_x - c_x) / 25
                yDif = (c_y - center_y) / 25

                dif = c12 - c11
                
                vec = [int(xDif), int(yDif),int(w) , int(dif)]
            else:
                vec = [0,0,0,1]
        except:
            vec = [0,0,0,1]
    
    return centrado,vec,frame

def aterriza(s,corners):
    
    for corner in corners:
        corner = np.array(corner)
        corner = corner.reshape((4, 2))
        center_x = int(corner[:, 0].mean())
        center_y = int(corner[:, 1].mean())

    a11 = corners[0][0][0][0]
    a12 = corners[0][0][0][1]
            
    b11 = corners[0][0][1][0]
    b12 = corners[0][0][1][1]

    dist = math.sqrt((b11 - a11)**2 + (b12 - a12)**2)
    c_x = int((s[1] / 2))
    c_y = int((s[0] / 2))
    c_y = c_y + 100

    alt_des = 75

    w = (dist - alt_des)/10

    xDif = (center_x - c_x) / 25
    yDif = (c_y - center_y) / 25

    dif = b12 - a12
    
    vec = [int(xDif), int(yDif),int(w) , int(dif)]

    return vec