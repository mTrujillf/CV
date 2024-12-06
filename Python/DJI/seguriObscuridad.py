from scipy.ndimage import center_of_mass
import cv2
import numpy as  np
import menosVision.metodosVision as mv
import grpc
import drones_pb2
import drones_pb2_grpc
import os

class Cliente:
    def __init__(self, hostPort):
        self.hostPort = hostPort
        self.channel = grpc.insecure_channel(self.hostPort)
        self.stub = drones_pb2_grpc.ManejoDroneStub(self.channel)

    def llamarServer(self, uP, vP, wP, rP):
        velocidades = drones_pb2.Velocidades(u=uP, v=vP, w=wP, r=rP) 
        return self.stub.mandaVelocidades(velocidades)

def connect_rtmp_stream(stream_url, output_path, cliente):
    cap = cv2.VideoCapture(stream_url)

    # Verificar si el video se abrió correctamente
    if not cap.isOpened():
        print("Error al abrir el archivo de video")
        return

    # Obtener información del video (ancho, alto, fps)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    # Definir el codec y crear el objeto VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height), isColor=False)
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("No se puede leer el frame. Fin del video o error de lectura.")
                break

            binary_image = mv.blanco_negro_nuevo_inv(frame)
            com = center_of_mass(binary_image)

            s = frame.shape

            c_x = int((s[1] / 2))
            c_y = int((s[0] / 2))

            try:
                center_x = int(com[1])
                center_y = int(com[0])
                xDif = (center_x - c_x) / 20
                yDif = (c_y - center_y) / 5

                #cliente.llamarServer(int(xDif), 3, int(yDif), int(xDif*2))
                cliente.llamarServer(0, 3, int(yDif), int(xDif*5))
            except:
                center_x = c_x
                center_y = c_y 
                cliente.llamarServer(0, 0, 0, 0)

            # Dibujar un círculo en el centro de masa
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.circle(frame, (c_x, c_y), 5, (255, 0, 0), -1)
            
            # Guardar el frame en el archivo de video
            out.write(frame)

            # Mostrar el frame
            cv2.imshow('Frame', frame)
            
            # Esperar 25 ms para mostrar el siguiente frame. Presiona 'q' para salir.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cap.release()
                out.release()
                break
    except:
        cap.release()
        out.release()
        cv2.destroyAllWindows()

    # Liberar el objeto VideoCapture y VideoWriter, y cerrar todas las ventanas
    cap.release()
    out.release()
    cv2.destroyAllWindows()

cliente = Cliente('localhost:50051') 
stream_url = 'rtmp://10.10.28.26/live/sample1'
output_path = 'output_video.avi'
connect_rtmp_stream(stream_url, output_path, cliente)
