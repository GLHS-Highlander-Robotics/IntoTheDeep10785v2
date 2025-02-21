package org.firstinspires.ftc.teamcode.subsystem.calculations;

public class Vector2D {
    public double x, y;
    // **NOTE THAT 0 IS STRAIGHT AHEAD!!!!!!!!!!!**

    public Vector2D(double x_i, double y_i){
        x = x_i;
        y  = y_i;
    }

    public Vector2D(double unitAngleinRad){
        x = Math.cos(unitAngleinRad);
        y  = Math.sin(unitAngleinRad);
    }

    public void rotateVector (double angleInRad){
        x = Math.cos(angleInRad)*x-Math.sin(angleInRad)*y;
        y = Math.sin(angleInRad)*x+Math.cos(angleInRad)*y;
    }

}
