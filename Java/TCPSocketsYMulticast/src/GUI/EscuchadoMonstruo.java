package GUI;

import javax.swing.*;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;

public class EscuchadoMonstruo extends Thread {

    private Integer n;
    private Integer m;
    private InetAddress group;
    private MulticastSocket socket;
    private String data;
    private byte[] buffer;
    private DatagramPacket messageIn;
    private Character chr1;
    private Character chr2;
    private int []monstruo;
    private JCheckBox [] arrCheck;
    private int pos;
    public EscuchadoMonstruo(int [] monstruo,JCheckBox [] arrCheck,MulticastSocket socket) {
        this.monstruo = monstruo;
        this.arrCheck = arrCheck;

        this.socket = socket;
        buffer = new byte[1000];

    }
    public void run() {
        try {

            while (true) {
                buffer = new byte[1000];
                messageIn = new DatagramPacket(buffer, buffer.length);
                socket.receive(messageIn);
                data = new String(messageIn.getData()).trim();
                //System.out.println(data);
                try{
                    chr1 = data.charAt(0);

                    chr2 = data.charAt(2);

                    m = Integer.parseInt(chr1.toString());
                    n = Integer.parseInt(chr2.toString());

                    if(!(monstruo[0] == m && monstruo[1]==n)){
                        pos = monstruo[0]*4 + monstruo[1];
                        arrCheck[pos].setSelected(false);
                    }
                    monstruo[0] = m;
                    monstruo[1] = n;
                    pos = monstruo[0]*4 + monstruo[1];
                    arrCheck[pos].setSelected(true);
                }catch(Exception e){
                    System.out.println(data);
                }
            }
        }catch(Exception e){

        }
    }


}
