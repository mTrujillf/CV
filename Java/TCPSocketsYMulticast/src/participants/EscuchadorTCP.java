package participants;

import java.io.IOException;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Dictionary;

public class EscuchadorTCP extends Thread {

    private int serverPort;
    private InetAddress group;
    private MulticastSocket socket;
    private int [] monstruo;
    private Dictionary<String, Integer> dictionary;
    ServerSocket listenSocket ;
    public EscuchadorTCP (int serverPort, InetAddress group, MulticastSocket socket, int [] monstruo, Dictionary<String, Integer> dictionary ){
        this.serverPort = serverPort;
        this.group = group;
        this.socket = socket;
        this.monstruo = monstruo;
        this.dictionary = dictionary;
        try {
            listenSocket = new ServerSocket(this.serverPort);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void run() {
        while(true) {
            try {


                Socket clientSocket = listenSocket.accept();  // Listens for a connection to be made to this socket and accepts it. The method blocks until a connection is made.
                Connection c = new Connection(clientSocket, group, socket, monstruo, dictionary);
                c.start();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }

}
