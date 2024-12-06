package participants;

import java.net.*;
import java.io.*;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;

public class TCPServer {

}

class Connection extends Thread {
    private DataInputStream in;
    private Socket clientSocket;
    private InetAddress group;
    private MulticastSocket socket;
    private boolean paso;
    private String user;
    private String data;
    private Character chr;
    private DataOutputStream out;

    private Integer nHit;
    private Integer mHit;
    Dictionary<String, Integer> dictionary ;
    private int puntaje;

    int [] monstruo;
    public Connection(Socket aClientSocket,InetAddress group,MulticastSocket socket, int [] monstruo,Dictionary<String, Integer> dictionary ) {
        this.group = group;
        this.socket = socket;
        this.monstruo = monstruo;
        this.dictionary = dictionary;
        try {
            clientSocket = aClientSocket;
            in = new DataInputStream(clientSocket.getInputStream());
            out = new DataOutputStream(clientSocket.getOutputStream());

            out.writeUTF("228.5.194.7");
            out.writeUTF("49155");

            user = in.readUTF();
            System.out.println("El usuario " + user + " entro al juego");
            dictionary.put(user,0);
        } catch (IOException e) {
            System.out.println("Connection:" + e.getMessage());
        }
    }

    @Override
    public void run() {
        try {

            while(true) {
                // an echo server
                user = in.readUTF();
                paso = false;
                while(!paso) {
                    try {
                        data = in.readUTF();

                        chr = data.charAt(0);
                        mHit = Integer.parseInt(chr.toString());

                        chr = data.charAt(2);
                        nHit = Integer.parseInt(chr.toString());

                        if(nHit == monstruo[0] && mHit == monstruo[1]){
                            puntaje = dictionary.get(user);

                            puntaje = puntaje+1;
                            if(puntaje == 5){
                                String myMessage = "El ganador de la ronda es: " + user;
                                byte[] men = myMessage.getBytes();
                                DatagramPacket messageOut = new DatagramPacket(men, men.length, group, 49155);
                                socket.send(messageOut);
                                Enumeration<String> k = dictionary.keys();
                                while (k.hasMoreElements()) {
                                    String key = k.nextElement();
                                    dictionary.put(key,0);
                                }
                            }else{
                                dictionary.put(user,puntaje);
                            }
                            monstruo[0] = (int) (Math.random()*(4)+0);
                            monstruo[1] = (int) (Math.random()*(4)+0);
                            paso = true;
                        }else{
                            data = in.readUTF();
                        }

                    }catch(Exception e){
                        data = in.readUTF();
                    }
                }
            }
            //}// envio respuesta

        } catch (EOFException e) {
            System.out.println("EOF:" + e.getMessage());
        } catch (IOException e) {
            System.out.println("IO:" + e.getMessage());
        } finally {
            try {
                clientSocket.close();
            } catch (IOException e) {
                System.out.println(e);
            }
        }
    }
}