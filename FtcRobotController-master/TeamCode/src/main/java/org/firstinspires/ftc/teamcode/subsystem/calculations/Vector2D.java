package org.firstinspires.ftc.teamcode.subsystem.calculations;

public class Vector2D {
    public double x, y, mag, dir;

    // **NOTE THAT 0 IS STRAIGHT AHEAD!!!!!!!!!!!**
    public Vector2D(double magnitude, double direction){
        mag = magnitude;
        dir = direction;
        x = Math.sin(direction);
        y = Math.cos(direction);
    }

    public Vector2D(int xy, double x_i, double y_i){
        x = x_i;
        y  = y_i;
        mag = Math.sqrt(x_i * x_i + y_i * y_i);
        dir = Math.atan2(x_i, y_i);
    }

    public Vector2D rotateVector (double angle){
        return new Vector2D(mag, dir + angle);
    }
}
