package org.firstinspires.ftc.teamcode.subsystem.calculations;

public class Basic {
    public boolean withinDeadzone(double L_Stick_x, double L_Stick_y, double R_Stick_x, double deadzone){
        return L_Stick_x * L_Stick_x + L_Stick_y * L_Stick_y >= deadzone * deadzone || Math.abs(R_Stick_x) >= deadzone;
    }
}
