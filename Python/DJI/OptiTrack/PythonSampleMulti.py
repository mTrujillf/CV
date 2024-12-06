#Copyright © 2018 Naturalpoint
#
#Licensed under the Apache License, Version 2.0 (the "License")
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.


# OptiTrack NatNet direct depacketization sample for Python 3.x
#
# Uses the Python NatNetClient.py library to establish a connection (by creating a NatNetClient),
# and receive data via a NatNet connection and decode it using the NatNetClient library.

import sys
import time
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData
import math
import numpy as np
import grpc 
import drones_pb2
import drones_pb2_grpc
import metodos
import metodos3
from time import sleep

class Cliente():
    def __init__(self,hostPort):
        self.hostPort = hostPort
        self.channel = grpc.insecure_channel(self.hostPort)
        self.stub = drones_pb2_grpc.ManejoDroneStub(self.channel)
        
    def llamaServer(self,uP,vP,wP,rP):
        velocidades = drones_pb2.Velocidades(u=uP,
                                             v=vP,
                                             w=wP,
                                             r=rP)
        return self.stub.mandaVelocidades(velocidades)
    
        
# This is a callback function that gets connected to the NatNet client
# and called once per mocap frame.
def receive_new_frame(data_dict):
    order_list=[ "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
                "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
    dump_args = False
    if dump_args == True:
        out_string = "    "
        for key in data_dict:
            out_string += key + "="
            if key in data_dict :
                out_string += data_dict[key] + " "
            out_string+="/"
        print(out_string)

def generador_velocidades(velocidad_u,velocidad_v,velocidad_w,velocidad_r):
    yield drones_pb2.Velocidades(u = velocidad_u,
                                 v = velocidad_v,
                                 w = velocidad_w,
                                 r = velocidad_r)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame( new_id, position, rotation,frame_actual,dwh,signo,cont,va,vm,contd,arrDist,desfase):

    if frame_actual%3 == 0:
        

        t = time.time() - timeStart 
        datos_vuelta = list()
        pos_actual = position
        rot_actual = rotation
        t_nano = time.time_ns() - t_nano_start 
        x = -pos_actual[0]
        z = -pos_actual[1]
        y = -pos_actual[2]
        
        x = np.float32(x)
        z = np.float32(z)
        y = np.float32(y)
        
        q11,q31,q21,q01 = rot_actual
        
        roll = math.atan(((2 * q21*q31) + (2 * q01*q11))/((2 * q01*q01) + (2 * q31*q31) - 1 ))
        pitch = (-math.asin((2 * q11*q31) - (2 * q01*q21)))
        yaw = math.atan2(((2 * q11*q21) + (2*q01*q31)), ((2 * q01*q01) + (2 * q11*q11) - 1)) 
        yaw = -yaw
        yaw =  np.float32(yaw)
        
        posArr = new_id - 1
        print(arrDist)
        arrDist[posArr] = [x,y,z,yaw,0,0,0]
        #Se debe eliminar - dron virtual
        #arrDist[1][0] = 0
        #arrDist[1][1] = 0
        #arrDist[1][2] = -0.9
        #Distancia en el plano entre dron 1 y 2
        xDist12 = arrDist[0][0] - arrDist[1][0]
        xDist12 = xDist12 * xDist12
        yDist12 = arrDist[0][1] - arrDist[1][1]
        yDist12 = yDist12 * yDist12
        zDist12 = arrDist[0][2] - arrDist[1][2]
        zDist12 = zDist12 * zDist12
        #Distancia en el plano entre dron 1 y 3
        xDist13 = arrDist[0][0] - arrDist[2][0]
        xDist13 = xDist13 * xDist13
        yDist13 = arrDist[0][1] - arrDist[2][1]
        yDist13 = yDist13 * yDist13
        zDist13 = arrDist[0][2] - arrDist[2][2]
        zDist13 = zDist13 * zDist13
        #Distancia en el plano entre dron 2 y 3
        xDist23 = arrDist[1][0] - arrDist[2][0]
        xDist23 = xDist23 * xDist23
        yDist23 = arrDist[1][1] - arrDist[2][1]
        yDist23 = yDist23 * yDist23
        zDist23 = arrDist[1][2] - arrDist[2][2]
        zDist23 = zDist23 * zDist23
        #zDist = arrDist[0][2] - arrDist[1][2]
        #zDist = zDist * zDist
        #dist = xDist + yDist + zDist
        dist12 = xDist12 + yDist12 + zDist12
        dist12 = math.sqrt(dist12)
        
        dist13 = xDist13 + yDist13 + zDist13
        dist13 = math.sqrt(dist13)
        
        dist23 = xDist23 + yDist23 + zDist23
        dist23 = math.sqrt(dist23)
        
        distSeg = 0.75
        
        dist = np.array([dist12,dist13,dist23,distSeg])

        metodo = arrMetodos[posArr]
        #desfase = arrDesfase[posArr]
        velocidad_u,velocidad_v,velocidad_w,velocidad_r,xDes,yDes,zDes,psiDes,dwh,arrDist,desfase = metodo(t,yaw,x,y,z,dwh,desfase,arrDist,dist)
        
        
        print("---------------------------")
        print(new_id)
        print(psiDes)
        print(velocidad_r)
        print("---------------------------")
        datos_vuelta.append(x)
        datos_vuelta.append(xDes)
        datos_vuelta.append(y)
        datos_vuelta.append(yDes)
        datos_vuelta.append(z)
        datos_vuelta.append(zDes)
        datos_vuelta.append(yaw)
        datos_vuelta.append(velocidad_u)
        datos_vuelta.append(velocidad_v)
        datos_vuelta.append(velocidad_w)
        datos_vuelta.append(velocidad_r)
        #datos_vuelta.append(desfase[0])
        #datos_vuelta.append(desfase[1])
        #datos_vuelta.append(desfase[2])
        datos_vuelta.append(psiDes)
        datos_vuelta.append(t)
        datos_vuelta.append(frame_actual)
        datos_vuelta.append(arrDist[0][6])
        datos_vuelta.append(dwh[0])
        datos_vuelta.append(dwh[1])
        datos_vuelta.append(dwh[2])
        datos_vuelta.append(dwh[3])
        datos_vuelta.append(dwh[4])
        datos_vuelta.append(dwh[5])
                
        datos[posArr].append(datos_vuelta)
            
            
        
        cli = arrClientes[posArr]
        
        cli.llamaServer(velocidad_u,velocidad_v,velocidad_w,velocidad_r)
            
    return dwh,signo,cont,va,contd,arrDist,desfase


def add_lists(totals, totals_tmp):
    totals[0]+=totals_tmp[0]
    totals[1]+=totals_tmp[1]
    totals[2]+=totals_tmp[2]
    return totals

def print_configuration(natnet_client):
    natnet_client.refresh_configuration()
    print("Connection Configuration:")
    print("  Client:          %s"% natnet_client.local_ip_address)
    print("  Server:          %s"% natnet_client.server_ip_address)
    print("  Command Port:    %d"% natnet_client.command_port)
    print("  Data Port:       %d"% natnet_client.data_port)

    if natnet_client.use_multicast:
        print("  Using Multicast")
        print("  Multicast Group: %s"% natnet_client.multicast_address)
    else:
        print("  Using Unicast")

    #NatNet Server Info
    application_name = natnet_client.get_application_name()
    nat_net_requested_version = natnet_client.get_nat_net_requested_version()
    nat_net_version_server = natnet_client.get_nat_net_version_server()
    server_version = natnet_client.get_server_version()

    print("  NatNet Server Info")
    print("    Application Name %s" %(application_name))
    print("    NatNetVersion  %d %d %d %d"% (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))
    print("    ServerVersion  %d %d %d %d"% (server_version[0], server_version[1], server_version[2], server_version[3]))
    print("  NatNet Bitstream Requested")
    print("    NatNetVersion  %d %d %d %d"% (nat_net_requested_version[0], nat_net_requested_version[1],\
       nat_net_requested_version[2], nat_net_requested_version[3]))
    #print("command_socket = %s"%(str(natnet_client.command_socket)))
    #print("data_socket    = %s"%(str(natnet_client.data_socket)))


def print_commands(can_change_bitstream):
    outstring = "Commands:\n"
    outstring += "Return Data from Motive\n"
    outstring += "  s  send data descriptions\n"
    outstring += "  r  resume/start frame playback\n"
    outstring += "  p  pause frame playback\n"
    outstring += "     pause may require several seconds\n"
    outstring += "     depending on the frame data size\n"
    outstring += "Change Working Range\n"
    outstring += "  o  reset Working Range to: start/current/end frame = 0/0/end of take\n"
    outstring += "  w  set Working Range to: start/current/end frame = 1/100/1500\n"
    outstring += "Return Data Display Modes\n"
    outstring += "  j  print_level = 0 supress data description and mocap frame data\n"
    outstring += "  k  print_level = 1 show data description and mocap frame data\n"
    outstring += "  l  print_level = 20 show data description and every 20th mocap frame data\n"
    outstring += "Change NatNet data stream version (Unicast only)\n"
    outstring += "  3  Request 3.1 data stream (Unicast only)\n"
    outstring += "  4  Request 4.1 data stream (Unicast only)\n"
    outstring += "t  data structures self test (no motive/server interaction)\n"
    outstring += "c  show configuration\n"
    outstring += "h  print commands\n"
    outstring += "q  quit\n"
    outstring += "\n"
    outstring += "NOTE: Motive frame playback will respond differently in\n"
    outstring += "       Endpoint, Loop, and Bounce playback modes.\n"
    outstring += "\n"
    outstring += "EXAMPLE: PacketClient [serverIP [ clientIP [ Multicast/Unicast]]]\n"
    outstring += "         PacketClient \"192.168.10.14\" \"192.168.10.14\" Multicast\n"
    outstring += "         PacketClient \"127.0.0.1\" \"127.0.0.1\" u\n"
    outstring += "\n"
    print(outstring)

def request_data_descriptions(s_client):
    # Request the model definitions
    s_client.send_request(s_client.command_socket, s_client.NAT_REQUEST_MODELDEF,    "",  (s_client.server_ip_address, s_client.command_port) )

def test_classes():
    totals = [0,0,0]
    print("Test Data Description Classes")
    totals_tmp = DataDescriptions.test_all()
    totals=add_lists(totals, totals_tmp)
    print("")
    print("Test MoCap Frame Classes")
    totals_tmp = MoCapData.test_all()
    totals=add_lists(totals, totals_tmp)
    print("")
    print("All Tests totals")
    print("--------------------")
    print("[PASS] Count = %3.1d"%totals[0])
    print("[FAIL] Count = %3.1d"%totals[1])
    print("[SKIP] Count = %3.1d"%totals[2])

def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict


if __name__ == "__main__":

    optionsDict = {}
    optionsDict["clientAddress"] = "192.168.1.127" #PC Local
    optionsDict["serverAddress"] = "192.168.1.132" #PC Opti
    optionsDict["use_multicast"] = True

    # This will create a new NatNet client
    optionsDict = my_parse_args(sys.argv, optionsDict)
    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])
    

    t_nano_start = time.time_ns()
    t_start = time.time()

    numMetodo = 50
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.new_frame_listener = receive_new_frame
    streaming_client.rigid_body_listener = receive_rigid_body_frame
    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    datos = [list(),list(),list()]
    timeStart = time.time()
    
    try:
        cli1 = Cliente("localhost:50051")
        cli2 = Cliente("localhost:50052")
        cli3 = Cliente("localhost:50053")
    except:
        print("server grpc inactivo")
    
    #metodo1 = metodos.circulo_con_rotacion
    #metodo2 = metodos.lider_seguidor_estatico
    
    #metodo1 = metodos3.pos_estatica_drone1
    #metodo2 = metodos3.pos_estatica_drone2
    #metodo3 = metodos3.pos_estatica_drone3
    
    metodo1 = metodos.seguimiento
    metodo2 = metodos3.circulo_con_rotacion_2
    metodo3 = metodos3.lider_seguidor_estatico2
    
    arrClientes = [cli1,cli2,cli3]
    
    arrMetodos = [metodo1,metodo2,metodo3]
    
    arrDesfase = [0,0,0]
    
    is_running = streaming_client.run()
    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    is_looping = True
    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
        try:
            sys.exit(2)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    print_configuration(streaming_client)
    print("\n")
    print_commands(streaming_client.can_change_bitstream_version())

    while is_looping:
        inchars = input('Enter command or (\'h\' for list of commands)\n')
        
        if len(inchars)>0:
            c1 = inchars[0].lower()
            if c1 == 'h' :
                print_commands(streaming_client.can_change_bitstream_version())
            elif c1 == 'c' :
                print_configuration(streaming_client)
            #elif c1 == 's':
            #    request_data_descriptions(streaming_client)
            #    time.sleep(1)
            elif c1 == 'y':
                
                arrMetodos [0] = metodos.pos_estatica_drone1
                arrMetodos [1] = metodos.pos_estatica_drone2
                
            elif c1 == 'm':
                numMetodo = numMetodo+1
            elif c1 == 'n':
                numMetodo = numMetodo-1

            elif c1 == 'w':
                arrMetodos[0] = metodos.adelante
                arrMetodos[1] = metodos.adelante
                
            elif c1 == 'a':
                arrMetodos[0] = metodos.izquierda
                arrMetodos[1] = metodos.izquierda
                
            elif c1 == 'd':
                arrMetodos[0] = metodos.derecha
                arrMetodos[1] = metodos.derecha
                
            elif c1 == 's':
                arrMetodos[0] = metodos.atras
                arrMetodos[1] = metodos.atras
                
            elif c1 == 'q':
                arrMetodos[0] = metodos.detener
                arrMetodos[1] = metodos.detener               
                
            elif c1 == 'u':

                arrMetodos[0] = metodos.detener
                arrMetodos[1] = metodos.detener
                arrMetodos[2] = metodos.detener
                    
                metodos.escribe_final_multi(datos)
                sleep(1)
                is_looping = False
                streaming_client.shutdown()
                break
            elif (c1 == '3') or (c1 == '4'):
                if streaming_client.can_change_bitstream_version():
                    tmp_major = 4
                    tmp_minor = 1
                    if(c1 == '3'):
                        tmp_major = 3
                        tmp_minor = 1
                    return_code = streaming_client.set_nat_net_version(tmp_major,tmp_minor)
                    time.sleep(1)
                    if return_code == -1:
                        print("Could not change bitstream version to %d.%d"%(tmp_major,tmp_minor))
                    else:
                        print("Bitstream version at %d.%d"%(tmp_major,tmp_minor))
                else:
                    print("Can only change bitstream in Unicast Mode")

            elif c1 == 'p':
                sz_command="TimelineStop"
                return_code = streaming_client.send_command(sz_command)
                time.sleep(1)
                print("Command: %s - return_code: %d"% (sz_command, return_code) )
            elif c1 == 'r':
                sz_command="TimelinePlay"
                return_code = streaming_client.send_command(sz_command)
                print("Command: %s - return_code: %d"% (sz_command, return_code) )
            elif c1 == 'o':
                tmpCommands=["TimelinePlay",
                            "TimelineStop",
                            "SetPlaybackStartFrame,0",
                            "SetPlaybackStopFrame,1000000",
                            "SetPlaybackLooping,0",
                            "SetPlaybackCurrentFrame,0",
                            "TimelineStop"]
                for sz_command in tmpCommands:
                    return_code = streaming_client.send_command(sz_command)
                    print("Command: %s - return_code: %d"% (sz_command, return_code) )
                time.sleep(1)
            elif c1 == 'w':
                tmp_commands=["TimelinePlay",
                            "TimelineStop",
                            "SetPlaybackStartFrame,10",
                            "SetPlaybackStopFrame,1500",
                            "SetPlaybackLooping,0",
                            "SetPlaybackCurrentFrame,100",
                            "TimelineStop"]
                for sz_command in tmp_commands:
                    return_code = streaming_client.send_command(sz_command)
                    print("Command: %s - return_code: %d"% (sz_command, return_code) )
                time.sleep(1)
            elif c1 == 't':
                test_classes()

            elif c1 == 'j':
                streaming_client.set_print_level(0)
                print("Showing only received frame numbers and supressing data descriptions")
            elif c1 == 'k':
                streaming_client.set_print_level(1)
                print("Showing every received frame")

            elif c1 == 'l':
                print_level = streaming_client.set_print_level(20)
                print_level_mod = print_level % 100
                if(print_level == 0):
                    print("Showing only received frame numbers and supressing data descriptions")
                elif (print_level == 1):
                    print("Showing every frame")
                elif (print_level_mod == 1):
                    print("Showing every %dst frame"%print_level)
                elif (print_level_mod == 2):
                    print("Showing every %dnd frame"%print_level)
                elif (print_level == 3):
                    print("Showing every %drd frame"%print_level)
                else:
                    print("Showing every %dth frame"%print_level)

            elif c1 == 'q':
                is_looping = False
                metodos.escribe_final(datos)
                
                break
            else:
                print("Error: Command %s not recognized"%c1)
            print("Ready...\n")
    print("exiting")
