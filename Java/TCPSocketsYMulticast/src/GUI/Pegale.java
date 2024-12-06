package GUI;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.*;

public class Pegale {
    String user;
    private JPanel panelMain;private JCheckBox a00CheckBox;private JCheckBox a01CheckBox;private JCheckBox a02CheckBox;private JCheckBox a03CheckBox;
    private JCheckBox a10CheckBox;private JCheckBox a11CheckBox;private JCheckBox a12CheckBox;private JCheckBox a13CheckBox;private JCheckBox a20CheckBox;private JCheckBox a21CheckBox;
    private JCheckBox a22CheckBox;private JCheckBox a23CheckBox;private JCheckBox a30CheckBox;private JCheckBox a31CheckBox;private JCheckBox a32CheckBox;private JCheckBox a33CheckBox;private JTextField textField1;

    private int m;
    private int n;
    private int [] monstruo;
    private Socket s = null;
    private String data;
    private DataInputStream in;
    private DataOutputStream out;
    private byte[] buffer;
    private DatagramPacket messageIn;
    private EscuchadoMonstruo em;
    private int serverPort = 49152;
    MulticastSocket socket;

    public Pegale(String user,Socket s,MulticastSocket socket) {
        monstruo = new int[2];
        this.user = user;
        textField1.setVisible(false);
        this.socket = socket;

        buffer = new byte[1000];
        JCheckBox [] arrCheck = {a00CheckBox,a01CheckBox,a02CheckBox,a03CheckBox,
                                a10CheckBox,a11CheckBox,a12CheckBox,a13CheckBox,
                                a20CheckBox,a21CheckBox,a22CheckBox,a23CheckBox,
                                a30CheckBox,a31CheckBox,a32CheckBox,a33CheckBox};
        em = new EscuchadoMonstruo(monstruo,arrCheck,socket);
        em.start();
        messageIn = new DatagramPacket(buffer, buffer.length);


        try {
            this.s = s;
            in = new DataInputStream(s.getInputStream());
            out = new DataOutputStream(s.getOutputStream());
            out.writeUTF(user);

        } catch (UnknownHostException e) {

        } catch (IOException e) {

        }

        a00CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 0;
                n = 0;
                a00CheckBox.setSelected(false);
                validaHit();
            }

        });
        a01CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 0;
                n = 1;
                a01CheckBox.setSelected(false);
                validaHit();
            }
        });
        a02CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 0;
                n = 2;
                a02CheckBox.setSelected(false);
                validaHit();
            }
        });
        a03CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 0;
                n = 3;
                a03CheckBox.setSelected(false);
                validaHit();
            }
        });
        a10CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 1;
                n = 0;
                a10CheckBox.setSelected(false);
                validaHit();
            }
        });
        a11CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 1;
                n = 1;
                a11CheckBox.setSelected(false);
                validaHit();
            }
        });
        a12CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 1;
                n = 2;
                a12CheckBox.setSelected(false);
                validaHit();
            }
        });
        a13CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 1;
                n = 3;
                a13CheckBox.setSelected(false);
                validaHit();
            }
        });
        a20CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 2;
                n = 0;
                a20CheckBox.setSelected(false);
                validaHit();

            }
        });
        a21CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 2;
                n = 1;
                a21CheckBox.setSelected(false);
                validaHit();
            }
        });
        a22CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 2;
                n = 2;
                a22CheckBox.setSelected(false);
                validaHit();
            }
        });
        a23CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 2;
                n = 3;
                a23CheckBox.setSelected(false);
                validaHit();
            }
        });
        a30CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 3;
                n = 0;
                a30CheckBox.setSelected(false);
                validaHit();
            }
        });
        a31CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 3;
                n = 1;
                a31CheckBox.setSelected(false);
                validaHit();
            }
        });
        a32CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 3;
                n = 2;
                a32CheckBox.setSelected(false);
                validaHit();
            }
        });
        a33CheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                m = 3;
                n = 3;
                a33CheckBox.setSelected(false);
                validaHit();
            }
        });

    }

    public void validaHit(){
        if(n == monstruo[1] && m == monstruo[0]){
            try {
                out = new DataOutputStream(s.getOutputStream());

                out.writeUTF(user);
                out.writeUTF(monstruo[0] + "," + monstruo[1]);
            } catch (IOException e) {

            }

        }
    }


    public JPanel getPanelMain(){
        return panelMain;
    }

}
