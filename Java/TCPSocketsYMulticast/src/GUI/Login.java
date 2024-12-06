package GUI;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.Socket;

public class Login {
    MulticastSocket socket = null;
    Socket s = null;
    int serverPort;
    private JButton loginButton;
    private JPanel panelLogin;
    private JTextPane textPane1;
    private JLabel userName;
    private JTextArea textArea1;
    private JLabel severPortLabel;
    private JButton serverButton;
    private JLabel serverIpLabel;
    private JTextArea textArea2;
    private JFrame framePegale ;
    private DataInputStream in;
    private DataOutputStream out;
    private int val;
    private String strGroup;
    private String strIPGroup;
    private InetAddress group;
    private int groupPort;
    public Login(int val){
        this.val = val;
        loginButton.setEnabled(false);
        System.out.println("localhost  ,   49152");
        //try {
        //    s = new Socket("localhost", serverPort);
        //} catch (IOException e) {

        //}

        loginButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                String user = textPane1.getText();
                if (! user.isEmpty()){
                    framePegale = new JFrame("Pegale");
                    framePegale.setContentPane(new Pegale(user,s,socket).getPanelMain());
                    framePegale.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
                    framePegale.pack();
                    framePegale.setLocation(50 + (275*val),350);
                    framePegale.setVisible(true);
                }

            }
        });
        serverButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                String strServerPort = textArea1.getText();
                String strServerIP = textArea2.getText();
                if(!strServerPort.isEmpty() && !strServerIP.isEmpty()){
                    try{
                        serverPort = Integer.parseInt(strServerPort);
                        s = new Socket(strServerIP, serverPort);

                        in = new DataInputStream(s.getInputStream());
                        strIPGroup = in.readUTF();
                        strGroup = in.readUTF();

                        groupPort = Integer.parseInt(strGroup);
                        group = InetAddress.getByName(strIPGroup); // destination multicast group

                        socket = new MulticastSocket(groupPort);
                        socket.joinGroup(group);

                        serverButton.setEnabled(false);
                        loginButton.setEnabled(true);
                    }catch (Exception ex){

                    }
                }

            }
        });
    }

    public JPanel getPanelLogin(){
        return panelLogin;
    }


    public static void main(String[] args){

        JFrame frame = new JFrame("Login");
        frame.setContentPane(new Login(0).panelLogin);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.setLocation(700,350);
        frame.setVisible(true);

    }
}
