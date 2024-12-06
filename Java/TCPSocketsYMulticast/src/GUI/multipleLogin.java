package GUI;

import javax.swing.*;

public class multipleLogin {
    public static void main(String[] args){
        JFrame [] arrFrame = new JFrame[5];

        for (int i = 0; i<2; i++) {
            arrFrame[i] = new JFrame("Login" + i);
            arrFrame[i].setContentPane(new Login(i).getPanelLogin());
            arrFrame[i].setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            arrFrame[i].pack();
            arrFrame[i].setLocation(50 + (275*i), 350);
            arrFrame[i].setVisible(true);
        }

    }
}
